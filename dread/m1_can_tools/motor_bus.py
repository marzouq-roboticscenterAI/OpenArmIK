"""Maintenance-mode CAN bus owner for the M1 arms + lift.

``MotorBus`` is the single owner of the CAN bus while the robot is in
*maintenance* mode (ros2_control is down). It is what the ``m1_hwconfig`` web
page drives: scan/enumerate motors, enable/disable, jog (clamped), and read
telemetry. Firmware set-zero is deliberately blocked. It speaks the
:mod:`dm_protocol` codec over a :mod:`transport`
backend, so it is fully exercisable on a :class:`~m1_can_tools.transport.FakeTransport`
with no hardware.

Bus-ownership is mutually exclusive with the live ros2_control stack (see the
deployment design's safety section): in *run* mode this owner is NOT
constructed; the config page reads ``/joint_states`` instead.

The motor map (ID -> logical joint) is persisted to YAML. Schema, per joint::

    joint_name:
      id:          <int>          # CAN slave id
      master_id:   <int>          # host/master id (feedback arb id; = id + 0x10)
      model:       <str>          # DM model -> per-model [P,V,T]MAX
      soft_limits: {pos: [lo, hi], vel: <float>, effort: <float>}
      dir:         +1 | -1        # joint-direction sign vs. motor
      scale:       <positive float> # joint units per motor radian
      offset:      <float>        # joint = dir*scale*motor + offset
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional

from m1_can_tools import dm_protocol as dm
from m1_can_tools.transport import Transport

# Default jog gains (gentle impedance for a maintenance nudge).
DEFAULT_JOG_KP = 10.0
DEFAULT_JOG_KD = 1.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class MotorBus:
    """Owns the CAN bus in maintenance mode; per-joint enable/jog/zero/telemetry."""

    def __init__(self, transport: Transport, motor_map: Dict[str, dict]) -> None:
        self.transport = transport
        self.motor_map = motor_map

    # --- map helpers -------------------------------------------------------
    def _info(self, joint: str) -> dict:
        try:
            return self.motor_map[joint]
        except KeyError as exc:  # noqa: BLE001
            raise KeyError(
                f"joint {joint!r} not in motor map; "
                f"known: {sorted(self.motor_map)}") from exc

    def joints(self) -> List[str]:
        """Logical joint names known to this bus, in map order."""
        return list(self.motor_map.keys())

    # --- enable / disable --------------------------------------------------
    def _special(self, joint: str, kind: str) -> None:
        info = self._info(joint)
        self.transport.send(dm.arb_id(info["id"], "mit"), dm.special_frame(kind))

    def enable(self, joint: str) -> None:
        """Enable (energize) one motor."""
        self._special(joint, "enable")

    def disable(self, joint: str) -> None:
        """Disable (de-energize) one motor."""
        self._special(joint, "disable")

    def set_zero(self, joint: str) -> None:
        """Firmware zero writes are intentionally unavailable on M1."""
        raise PermissionError(
            "firmware set-zero is disabled; use m1_calibrate to set a map offset")

    def clear_error(self, joint: str) -> None:
        """Clear a latched error on one motor."""
        self._special(joint, "clear_error")

    def enable_all(self) -> None:
        """Enable every mapped motor."""
        for joint in self.motor_map:
            self.enable(joint)

    def disable_all(self) -> None:
        """Disable every mapped motor."""
        for joint in self.motor_map:
            self.disable(joint)

    # --- jog (clamped) -----------------------------------------------------
    def jog(
        self,
        joint: str,
        pos: float,
        vel: float = 0.0,
        kp: float = DEFAULT_JOG_KP,
        kd: float = DEFAULT_JOG_KD,
        tau: float = 0.0,
    ) -> None:
        """Send one MIT command, clamped to the model AND the soft limits.

        The position/velocity/torque are clamped first to the configured
        ``soft_limits`` and then to the per-model ``[P,V,T]MAX`` (the encoder
        clamps to the model range too, but we clamp explicitly so the commanded
        value is observable). A jog is a single frame; the web page's deadman
        decides whether to keep sending it.

        NB (frame convention): ``jog`` commands in the **motor** frame -- the
        ``dir``/``offset`` calibration is NOT applied here, whereas
        :meth:`telemetry` reports ``pos`` in the **joint** frame (dir/offset
        applied). This is deliberate for raw maintenance nudges; an operator near
        a limit should read the motor-frame setpoint accordingly. (The live
        ros2_control path applies dir/offset in C++.)
        """
        # Reject non-finite fields up front with a clear error: a NaN/Inf would
        # otherwise either be silently coerced by _clamp's max/min ordering or
        # raise an opaque error deep in the codec. A jog drives a real motor, so
        # a garbage setpoint must fail loudly, not emit a full-scale command.
        for nm, val in (("pos", pos), ("vel", vel), ("kp", kp),
                        ("kd", kd), ("tau", tau)):
            if not math.isfinite(float(val)):
                raise ValueError(f"jog {nm!r} must be finite, got {val!r}")

        info = self._info(joint)
        p_max, v_max, t_max = dm.limits(info["model"])
        soft = info.get("soft_limits", {})

        pos_lo, pos_hi = soft.get("pos", [-p_max, p_max])
        v_soft = float(soft.get("vel", v_max))
        t_soft = float(soft.get("effort", t_max))

        p = _clamp(float(pos), max(pos_lo, -p_max), min(pos_hi, p_max))
        v = _clamp(float(vel), -min(v_soft, v_max), min(v_soft, v_max))
        tq = _clamp(float(tau), -min(t_soft, t_max), min(t_soft, t_max))
        # Clamp the gains to the encoder's valid range too (was passed raw), so
        # the maintenance jog path can never emit an out-of-range stiffness.
        kp = _clamp(float(kp), dm.KP_MIN, dm.KP_MAX)
        kd = _clamp(float(kd), dm.KD_MIN, dm.KD_MAX)

        data = dm.encode_mit(p, v, kp, kd, tq, info["model"])
        self.transport.send(dm.arb_id(info["id"], "mit"), data)

    # --- telemetry / scan --------------------------------------------------
    def telemetry(self, joint: str, timeout: float = 0.01,
                  poll: bool = True) -> Optional[dict]:
        """Read & decode the most recent feedback frame for *joint*.

        Returns the decoded ``{id, err, pos, vel, torque, t_mos, t_rotor}``
        reported in the **joint** frame (``dir`` applied to pos/vel/torque and
        ``offset`` to pos, matching the C++ plugin's convention), or ``None``
        if no frame arrived. When ``poll`` (default), first sends a
        non-energizing **refresh** request (``0xCC`` -> ``0x7FF``) so a motor
        that isn't streaming still replies -- the documented way to poll DM
        state, and what makes passive maintenance telemetry work without
        enabling the motor.

        NB: frames belonging to OTHER joints are discarded by this single-joint
        read; to refresh the whole table (the config page), use
        :meth:`telemetry_all`, which dispatches every reply instead.
        """
        info = self._info(joint)
        want = info.get("master_id", dm.master_id(info["id"]))
        if poll:
            self.transport.send(dm.PARAM_ARB_ID, dm.refresh_frame(info["id"]))
        frame = self.transport.recv(timeout=timeout)
        while frame is not None:
            arb, data = frame
            if arb == want and len(data) >= 8:
                fb = self._to_joint_frame(dm.decode_feedback(data, info["model"]),
                                          joint, info)
                return fb
            frame = self.transport.recv(timeout=timeout)
        return None

    @staticmethod
    def _to_joint_frame(fb: dict, joint: str, info: dict) -> dict:
        """Motor-frame feedback -> joint frame.

        ``scale`` is separate from ``dir`` because the gripper motor is rotary
        while the URDF finger joint has a different angular travel.  Position
        and velocity scale; torque uses the inverse scale so power is preserved.
        This is the same convention as the C++ hardware plugin.
        """
        d = info.get("dir", 1)
        s = float(info.get("scale", 1.0))
        if not math.isfinite(s) or s <= 0.0:
            raise ValueError(f"{joint}: scale must be finite and > 0")
        fb["pos"] = d * s * fb["pos"] + info.get("offset", 0.0)
        fb["vel"] = d * s * fb["vel"]
        fb["torque"] = d * fb["torque"] / s
        fb["joint"] = joint
        return fb

    def telemetry_all_raw(self, timeout: float = 0.01) -> Dict[str, dict]:
        """Non-energizing refresh of every motor, without map transforms."""
        by_master = {
            info.get("master_id", dm.master_id(info["id"])): (joint, info)
            for joint, info in self.motor_map.items()
        }
        for info in self.motor_map.values():
            self.transport.send(dm.PARAM_ARB_ID, dm.refresh_frame(info["id"]))
        out: Dict[str, dict] = {}
        frame = self.transport.recv(timeout=timeout)
        while frame is not None:
            arb, data = frame
            named = by_master.get(arb)
            if named is not None and len(data) >= 8:
                joint, info = named
                fb = dm.decode_feedback(data, info["model"])
                fb["joint"] = joint
                out[joint] = fb
            frame = self.transport.recv(timeout=timeout)
        return out

    def telemetry_all(self, timeout: float = 0.01) -> Dict[str, dict]:
        """Refresh-poll EVERY mapped motor and decode all replies in one drain.

        Sends one non-energizing refresh per mapped id, then drains the
        receive queue once, dispatching each reply to its joint by master id.
        A per-joint :meth:`telemetry` loop instead *discards* every frame that
        belongs to a different joint, so on a real (asynchronous) bus joint
        N's reply is typically eaten by joint N+1's poll and the table
        starves; here nothing is thrown away. Returns ``{joint: fb}`` for the
        motors that replied (joint-frame values, like :meth:`telemetry`).
        """
        by_master = {
            info.get("master_id", dm.master_id(info["id"])): (joint, info)
            for joint, info in self.motor_map.items()
        }
        for info in self.motor_map.values():
            self.transport.send(dm.PARAM_ARB_ID, dm.refresh_frame(info["id"]))
        out: Dict[str, dict] = {}
        frame = self.transport.recv(timeout=timeout)
        while frame is not None:
            arb, data = frame
            named = by_master.get(arb)
            if named is not None and len(data) >= 8:
                joint, info = named
                fb = self._to_joint_frame(dm.decode_feedback(data, info["model"]),
                                          joint, info)
                out[joint] = fb
            frame = self.transport.recv(timeout=timeout)
        return out

    def scan(self, ids: Iterable[int]) -> List[dict]:
        """Poll each id in *ids* (non-energizing) and list the motors that replied.

        Used by the config page's inventory: pings the candidate slave ids with a
        **state-refresh** frame (opcode ``0xCC`` to ``0x7FF``) -- the motor replies
        with a feedback frame on its master id WITHOUT being enabled/powered, so an
        inventory scan never energizes the arm. Returns one dict per responder
        ``{id, master_id, model, joint, ...fb}``.
        """
        # Index the known map by master id so a responder can be named.
        by_master = {
            info.get("master_id", dm.master_id(info["id"])): (joint, info)
            for joint, info in self.motor_map.items()
        }
        # Poll every candidate id with the non-energizing refresh request.
        for sid in ids:
            self.transport.send(dm.PARAM_ARB_ID, dm.refresh_frame(sid))

        # Collect responders keyed by their arbitration id (== master id), so a
        # motor that emits more than one feedback frame in the poll window appears
        # exactly once (last frame wins = freshest telemetry).
        by_arb: Dict[int, dict] = {}
        frame = self.transport.recv(timeout=0.01)
        while frame is not None:
            arb, data = frame
            named = by_master.get(arb)
            model = named[1]["model"] if named else "DM4310"
            if len(data) >= 8:
                fb = dm.decode_feedback(data, model)
                joint = named[0] if named else None
                # The feedback frame's id byte carries only the LOW 4 BITS of the
                # slave id, so it mis-identifies any id > 0x0F (e.g. 0x11 reads as
                # 1). The reply's arbitration id (== master id) is the only
                # unambiguous identifier, so derive the slave id from it -- or use
                # the authoritative mapped id when the responder is named.
                slave_id = (named[1]["id"] if named is not None
                            else arb - dm.MASTER_ID_OFFSET)
                entry = {
                    "id": slave_id,
                    "master_id": arb,
                    "model": model,
                    "joint": joint,
                    "pos": fb["pos"],
                    "vel": fb["vel"],
                    "torque": fb["torque"],
                    "t_mos": fb["t_mos"],
                    "t_rotor": fb["t_rotor"],
                    "err": fb["err"],
                }
                by_arb[arb] = entry
            frame = self.transport.recv(timeout=0.01)
        # Sort by id for a stable inventory listing.
        return sorted(by_arb.values(), key=lambda m: m["id"])

    def close(self) -> None:
        """Disable everything and release the transport."""
        try:
            self.disable_all()
        finally:
            self.transport.close()


# --- YAML map persistence ---------------------------------------------------
def load_map(path: str) -> Dict[str, dict]:
    """Load an ID->joint motor map from a YAML file."""
    import yaml  # ament_python dep; available for the ROS interpreter
    with open(path, "r") as fh:
        data = yaml.safe_load(fh)
    return data or {}


def save_map(path: str, m: Dict[str, dict]) -> None:
    """Atomically persist an ID->joint motor map to a YAML file."""
    import yaml
    import os
    import tempfile
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".motor-map-", suffix=".tmp", dir=parent)
    try:
        with os.fdopen(fd, "w") as fh:
            yaml.safe_dump(m, fh, sort_keys=False, default_flow_style=False)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        raise
