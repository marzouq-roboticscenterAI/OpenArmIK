"""m1_calibrate_live -- guided live OpenArm v1.0 encoder calibration.

Why this exists
---------------
Getting RViz to match the real arms kept failing for two reasons, both now
designed out:

1. **Disabled Damiao motors report a FROZEN encoder value** (proven: hundreds of
   identical reads while a joint was hand-bent). Every older capture path
   (``m1_calibrate --capture-dropped`` and the removed standalone companion)
   polls *disabled* motors, so its "reference" raws never matched the physical pose and the solved
   offsets were built on dead data. This wizard **compliant-enables** the arm at
   strictly zero torque (kp=kd=tau=0 -> freely back-drivable, no holding force,
   no snap) so the encoder streams the TRUE live position the whole time, and
   RViz tracks the real arm as you move it.
2. **Direction judged by eye in RViz is unreliable** (roll joints look flipped
   from the wrong camera side). Here every ``offset`` is derived OBJECTIVELY from
   mechanical hardstops (viewpoint-free), and every ``dir`` is confirmed in a
   live verify loop with a one-key, exact, reversible flip.

Flow (single command, one arm at a time)::

    ros2 run m1_can_tools m1_calibrate_live left      # or right / both
    # (run the RViz side once, in another terminal:)
    ros2 launch m1_bringup live_calibration.launch.py

  1. resolve the side's bus by stable USB path; confirm; compliant-enable at
     ZERO torque (RViz now tracks the real arm live).
  2. per arm joint J1..J7: hand-move to BOTH hardstops (J4: the one safe stop) and
     hold; the wizard captures the raw at each and solves offset + a span check.
  3. gripper: capture fully-closed then fully-open -> sign + scale.
  4. atomically write both motor-map copies with a timestamped backup + a printed
     rollback command.
  5. LIVE VERIFY: keep streaming with the new map; move each joint and confirm
     RViz matches. ``flip``/``nudge``/``stops`` hot-fix the map with no restart.

Safety: the ONLY CAN frames sent are enable, a zero-torque MIT keepalive, and
disable. EVERY exit path (Enter-done, Ctrl-C, SIGTERM, exception) disables all
motors. Keep the hardware e-stop in reach. Bus ownership is exclusive -- stop
``m1_hwconfig`` / ros2_control / the passive viz first.
"""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import re
import select
import signal
import sys
import time
from pathlib import Path

try:
    import termios
except ImportError:  # non-POSIX; flush becomes a no-op
    termios = None

# CSI / escape sequences (e.g. arrow keys "\x1b[D") that a terminal may leave in
# the stdin buffer and which would otherwise corrupt a typed answer.
_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]|\x1b.")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from m1_can_tools import calibrate_live_solve as solve
from m1_can_tools.calibrate import (
    ADAPTER_PATH_HINTS,
    BusLocks,
    GRIPPER_TRAVEL,
    SIDES,
    URDF_LIMITS,
    _can_socket_preflight,
    _conflicting_processes,
    _iface_preflight,
    commit_maps,
    current_gripper_mapping,
    joint_name,
    side_map,
    validate_map_shape,
)
from m1_can_tools.motor_bus import MotorBus, load_map, save_map
from m1_can_tools.transport import SimTransport, SocketCanTransport

import os as _os

# Both map copies are env-overridable (M1_MOTOR_MAP / M1_REPO_MAP) so a sim
# rehearsal or test can write to throwaway files instead of the deployed map.
# The repo copy defaults cwd-relative, like m1_calibrate (the deploy flow runs
# from the repo root).
MAP_PATH = Path(_os.environ.get(
    "M1_MOTOR_MAP", Path.home() / ".config/m1/motor_map.yaml"))
REPO_MAP = Path(_os.environ.get(
    "M1_REPO_MAP",
    Path.cwd() / "dread/config/motor_map.openarm_v10.yaml"))
BACKUP_ROOT = Path(_os.environ.get(
    "M1_BACKUP_ROOT", Path.home() / ".config/m1/calibration-backups"))
LIFT_HEIGHT = 0.62865

MAX_STILL_VEL = 0.15   # rad/s; a captured stop must be held roughly still
CAPTURE_MEDIAN_N = 8   # samples the held-stop value is a median over

# Short physical descriptions so the operator knows which joint is which without
# reading the URDF. Direction words are avoided; a hardstop is a hardstop.
JOINT_DESC = {
    1: "J1 shoulder rotation (upper arm swivels)",
    2: "J2 shoulder pitch (upper arm up/down)",
    3: "J3 upper-arm roll (rotate about the upper-arm length)",
    4: "J4 elbow bend",
    5: "J5 forearm roll (rotate about the forearm length)",
    6: "J6 wrist pitch (hand up/down)",
    7: "J7 wrist roll (rotate the hand)",
    8: "gripper",
}


class LiveCalibrator(Node):
    def __init__(self, sides, transport="socketcan"):
        super().__init__("m1_calibrate_live")
        self.sides = sides
        self.transport_kind = transport
        self.sim = transport == "sim"
        self._stop = False

        self.full_map = load_map(str(MAP_PATH))
        validate_map_shape(self.full_map)
        if transport == "socketcan":
            for s in sides:
                _iface_preflight(SIDES[s], ADAPTER_PATH_HINTS[s])
            _can_socket_preflight([SIDES[s] for s in sides])
            conflicts = _conflicting_processes([SIDES[s] for s in sides])
            if conflicts:
                raise RuntimeError(
                    "exclusive bus ownership failed:\n  " + "\n  ".join(conflicts))

        self._locks = BusLocks([SIDES[s] for s in sides]) if not self.sim else None
        if self._locks:
            self._locks.__enter__()
        self.sm = {s: side_map(self.full_map, s) for s in sides}
        self.tx = {s: self._make_tx(s) for s in sides}
        self.bus = {s: MotorBus(self.tx[s], self.sm[s]) for s in sides}
        self.raw_latest = {s: {} for s in sides}
        self.joint_latest = {s: {} for s in sides}
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._enabled = False

    # --- transport / enable ------------------------------------------------
    def _make_tx(self, side):
        if self.sim:
            motors = {int(v["id"]): {
                "master_id": int(v.get("master_id", int(v["id"]) + 0x10)),
                "model": v["model"], "pos": 0.1 * int(v["id"]),
            } for v in self.sm[side].values()}
            return SimTransport(motors)
        return SocketCanTransport(SIDES[side], fd=False)

    def enable(self):
        # Prove all eight IDs on every selected bus before energizing anything.
        # A partial arm is a wiring fault, not a calibration session.
        for side in self.sides:
            replies = self._read_complete(side, deadline=0.5)
            missing = sorted(set(self.sm[side]) - set(replies))
            if missing:
                raise RuntimeError(
                    f"{side}: refusing to enable; no status reply from "
                    + ", ".join(missing))

        # Prime a zero-gain MIT frame while disabled, then repeat it immediately
        # after enable. This avoids relying on whatever target/gains a motor may
        # have retained from an earlier program. kp=kd=tau=0 commands no motion
        # or holding torque; q and dq are consequently inert.
        for side in self.sides:
            for joint in self.sm[side]:
                self.bus[side].jog(
                    joint, pos=0.0, vel=0.0, kp=0.0, kd=0.0, tau=0.0)
        # Arm the disable guard BEFORE sending any enable frame: if a CAN write
        # raises partway through, shutdown() must still de-energize the motors
        # that did get enabled (the "every exit path disables" guarantee).
        self._enabled = True
        for s in self.sides:
            for j in self.sm[s]:
                self.bus[s].enable(j)
                self.bus[s].jog(
                    j, pos=0.0, vel=0.0, kp=0.0, kd=0.0, tau=0.0)
        time.sleep(0.02)
        self.pump()
        for side in self.sides:
            missing = sorted(set(self.sm[side]) - set(self.raw_latest[side]))
            errors = sorted(
                name for name, feedback in self.raw_latest[side].items()
                if int(feedback.get("err", 0)) != 0)
            if missing or errors:
                raise RuntimeError(
                    f"{side}: enabled-state telemetry failed; "
                    f"missing={missing}, motor_errors={errors}")

    def disable_all(self):
        for _ in range(3):
            for s in self.sides:
                for j in self.sm[s]:
                    try:
                        self.bus[s].disable(j)
                    except Exception:
                        pass
            time.sleep(0.02)
        self._enabled = False

    def shutdown(self):
        try:
            if self._enabled:
                self.disable_all()
        finally:
            for tx in self.tx.values():
                try:
                    tx.close()
                except Exception:
                    pass
            if self._locks:
                self._locks.__exit__(None, None, None)
                self._locks = None
        self.get_logger().warning("DISABLED all motors; bus released")

    # --- live pump ---------------------------------------------------------
    def _read_complete(self, side, deadline=0.15):
        want = set(self.sm[side])
        got = {}
        end = time.monotonic() + deadline
        while True:
            got.update(self.bus[side].telemetry_all_raw(timeout=0.02))
            if want.issubset(got) or time.monotonic() >= end:
                return got

    def pump(self):
        """One live cycle: zero-torque keepalive, read encoders, publish RViz."""
        for s in self.sides:
            for j in self.sm[s]:  # kp=kd=tau=0 -> no force, motor stays enabled
                try:
                    self.bus[s].jog(j, pos=0.0, vel=0.0, kp=0.0, kd=0.0, tau=0.0)
                except Exception:
                    pass
            for name, raw in self._read_complete(s).items():
                self.raw_latest[s][name] = raw
                self.joint_latest[s][name] = MotorBus._to_joint_frame(
                    dict(raw), name, self.sm[s][name])
        self._publish()

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["lift_joint"]
        msg.position = [LIFT_HEIGHT]
        for s in ("left", "right"):
            for mid in range(1, 9):
                suf = f"joint{mid}" if mid <= 7 else "finger_joint1"
                name = f"openarm_{s}_{suf}"
                fb = self.joint_latest.get(s, {}).get(name)
                pos = float(fb["pos"]) if fb else 0.0
                msg.name.append(name)
                msg.position.append(pos)
                if mid == 8:
                    msg.name.append(name.replace("finger_joint1", "finger_joint2"))
                    msg.position.append(pos)
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        self._pub.publish(msg)

    # --- interaction helpers ----------------------------------------------
    def _pump_until_line(self, prompt):
        """Print prompt; keep the live stream going until the operator hits Enter.
        Returns the typed line (stripped). In sim, returns "" immediately."""
        sys.stdout.write("\n" + prompt + " ")
        sys.stdout.flush()
        if self.sim:
            self.pump()
            return ""
        # Discard any bytes already in the tty input buffer (stray arrow-key
        # escape sequences typed before the prompt) so they can't be prepended
        # to the operator's answer. Best-effort; a no-op if stdin isn't a tty.
        if termios is not None:
            try:
                termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
            except (OSError, ValueError):
                pass
        while not self._stop:
            self.pump()
            r, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r:
                line = sys.stdin.readline()
                # Strip any residual escape sequences + non-printables so
                # terminal noise never corrupts the answer.
                line = _ANSI_RE.sub("", line)
                line = "".join(c for c in line if c.isprintable())
                return line.strip()
        return ""

    def _raw_of(self, side, mid):
        name = joint_name(side, mid)
        fb = self.raw_latest[side].get(name)
        return float(fb["pos"]) if fb else float("nan")

    def _vel_of(self, side, mid):
        name = joint_name(side, mid)
        fb = self.raw_latest[side].get(name)
        return abs(float(fb["vel"])) if fb else 0.0

    def _sim_stop(self, side, mid, label):
        """Deterministic synthetic stop raw for an offline (sim) rehearsal:
        distinct min/max per arm joint (span == URDF span, with a fake motor
        offset so the offset math is exercised) and a plausible gripper delta."""
        lo, hi = URDF_LIMITS[side][mid - 1]
        if mid == 8:
            return -1.10 if "CLOSED" in label else -0.01
        base = 0.30  # pretend the motor zero is offset from the joint zero
        return (lo + base) if ("#1" in label or "SAFE" in label) else (hi + base)

    def _capture_stop(self, side, mid, label):
        """Hold-a-stop capture: median raw over the last CAPTURE_MEDIAN_N pumps,
        gated on stillness. Returns the captured raw (motor frame)."""
        if self.sim:
            self.pump()
            raw = self._sim_stop(side, mid, label)
            print(f"\n  [sim] {label}: raw = {raw:+.4f}")
            return raw
        while True:
            self._pump_until_line(
                f"  {label}: move {JOINT_DESC[mid]} there, HOLD, then press Enter.")
            # A signal (SIGTERM) unblocks _pump_until_line without a real line;
            # abort the whole run rather than capturing a bogus pose and later
            # committing a shape-valid but physically-meaningless map.
            if self._stop:
                raise KeyboardInterrupt("aborted by signal")
            samples = []
            for _ in range(CAPTURE_MEDIAN_N):
                self.pump()
                samples.append(self._raw_of(side, mid))
            samples = [x for x in samples if x == x]  # drop NaN
            if not samples:
                print("    no telemetry for this joint; check the bus. Retrying.")
                continue
            vel = self._vel_of(side, mid)
            # NaN vel (== itself is False) counts as "not still", not a bypass.
            if not self.sim and not (vel == vel and vel <= MAX_STILL_VEL):
                print("    joint still moving / bad vel -- hold it steady and retry.")
                continue
            samples.sort()
            raw = samples[len(samples) // 2]
            print(f"    captured raw = {raw:+.4f} rad")
            return raw

    # --- solve / write -----------------------------------------------------
    def calibrate_side(self, side):
        print(f"\n=== Calibrating {side.upper()} arm "
              f"({'sim' if self.sim else SIDES[side]}) ===")
        results = {}
        for mid in range(1, 8):
            lo, hi = URDF_LIMITS[side][mid - 1]
            entry = self.full_map[joint_name(side, mid)]
            seed_dir = int(entry.get("dir", 1))
            seed_scale = float(entry.get("scale", 1.0))
            print(f"\n-- {JOINT_DESC[mid]}   URDF range [{lo:+.3f}, {hi:+.3f}]")
            if mid in solve.SINGLE_STOP_JOINTS:
                end = solve.SINGLE_STOP_URDF_END[mid]
                which = "OPEN/straight" if end == "lo" else "fully bent"
                raw = self._capture_stop(side, mid, f"SAFE stop (elbow {which})")
                r = solve.solve_single_stop_joint(
                    seed_dir, lo, hi, raw, end, seed_scale)
                r["stops"] = {"lo": lo, "hi": hi, "raw_stop": raw,
                              "end": end, "scale": seed_scale}
                print("    (single safe stop; retained prior scale because the "
                      "front stop is not safely approached)")
            else:
                r1 = self._capture_stop(side, mid, "hardstop #1")
                r2 = self._capture_stop(side, mid, "hardstop #2 (opposite)")
                rmin, rmax = min(r1, r2), max(r1, r2)
                r = solve.solve_arm_joint(seed_dir, lo, hi, rmin, rmax)
                r["stops"] = {"lo": lo, "hi": hi, "r_min": rmin, "r_max": rmax}
                print(f"    measured encoder span {r['span_measured']:.4f} rad; "
                      f"full-range scale {r['scale']:.12f}")
                if r["span_mismatch"] > solve.SPAN_MISMATCH_TOL:
                    print(f"    WARNING: measured span {r['span_measured']:.3f} vs "
                          f"URDF {r['span_urdf']:.3f} (off by {r['span_mismatch']:.3f}). "
                          f"URDF limit may not be a true stop; verify this joint.")
            results[mid] = r

        # gripper (J8) -- the fingers may not back-drive by hand at zero torque
        # (unlike the rotary arm joints), so a bad closed/open reading must NOT
        # discard the whole (already-captured) arm session. Retry, or skip and
        # seed a sane default the operator can refine later.
        lo, hi = URDF_LIMITS[side][7]
        print(f"\n-- {JOINT_DESC[8]}   travel to URDF {GRIPPER_TRAVEL[side]:+.4f}")
        while True:
            raw_closed = self._capture_stop(side, 8, "gripper fully CLOSED")
            raw_open = self._capture_stop(side, 8, "gripper fully OPEN")
            try:
                g = solve.solve_gripper(raw_closed, raw_open, GRIPPER_TRAVEL[side])
                g["stops"] = {"lo": lo, "hi": hi, "raw_stop": raw_closed,
                              "end": "closed"}
                break
            except ValueError as exc:
                print(f"    {exc}")
                ans = self._pump_until_line(
                    "  gripper travel not measured -- the fingers may not "
                    "back-drive by hand. Type 'r' to re-capture, or 's' to SKIP "
                    "and seed a default (arm joints keep their calibration):"
                ).lower()
                if ans.startswith("s"):
                    previous = self.full_map[joint_name(side, 8)]
                    if not current_gripper_mapping(previous, side):
                        print("    cannot skip: the prior gripper map belongs to "
                              "the old rotary-gripper URDF, not the current "
                              "0..44 mm OpenArm v1.0 joint. Re-capture is required.")
                        continue
                    g = {"dir": int(previous["dir"]),
                         "scale": float(previous["scale"]),
                         "offset": float(previous["offset"]),
                         "stops": ((previous.get("calibration") or {}).get("stops")
                                   or {"lo": lo, "hi": hi,
                                       "raw_stop": raw_closed, "end": "closed",
                                       "scale": float(previous["scale"])}),
                         "preserved": True}
                    print("    gripper: preserved the prior mapping; arm joints "
                          "remain calibrated.")
                    break
        results[8] = g
        self._apply_results(side, results)
        return results

    def _apply_results(self, side, results):
        """Write solved dir/scale/offset into self.full_map + self.sm (in memory)."""
        now = dt.datetime.now(dt.timezone.utc).isoformat()
        for mid, r in results.items():
            name = joint_name(side, mid)
            e = self.full_map[name]
            e["dir"] = int(r["dir"])
            e["scale"] = float(r["scale"])
            e["offset"] = float(r["offset"])
            lo, hi = URDF_LIMITS[side][mid - 1]
            e["soft_limits"]["pos"] = [lo, hi]
            cal = e.get("calibration") or {}
            cal.update({
                "source": "live_hardstop_wizard",
                "captured_at": now,
                "firmware_zero_changed": False,
                "stops": r["stops"],
            })
            e["calibration"] = cal
        self.sm[side] = side_map(self.full_map, side)
        # rebind the bus map so telemetry uses the new transform immediately
        self.bus[side].motor_map = self.sm[side]

    def commit(self, dry_run=False):
        validate_map_shape(self.full_map)
        backup = commit_maps(self.full_map, MAP_PATH, REPO_MAP, BACKUP_ROOT,
                             dry_run=dry_run)
        if dry_run:
            print("DRY RUN: generated map validated; no file changed.")
        else:
            print(f"\nSAVED both maps atomically. Backup: {backup}")
            print(f"Rollback: ros2 run m1_can_tools m1_calibrate --rollback {backup}")
        return backup

    def _write_inplace(self):
        """Persist self.full_map to both copies WITHOUT a new backup (verify edits)."""
        validate_map_shape(self.full_map)
        save_map(str(MAP_PATH), self.full_map)
        save_map(str(REPO_MAP), self.full_map)

    # --- live verify loop --------------------------------------------------
    def _print_table(self, side):
        print(f"\n  {'joint':10s} {'raw':>9s} {'q(urdf)':>9s} {'range':>18s} "
              f"{'in?':>4s}")
        for mid in range(1, 9):
            name = joint_name(side, mid)
            raw = self._raw_of(side, mid)
            fb = self.joint_latest[side].get(name)
            q = float(fb["pos"]) if fb else float("nan")
            lo, hi = URDF_LIMITS[side][mid - 1]
            inrange = "yes" if lo - 1e-3 <= q <= hi + 1e-3 else "NO"
            print(f"  J{mid:<9d} {raw:+9.3f} {q:+9.3f} "
                  f"[{lo:+.2f},{hi:+.2f}]".ljust(48) + f"{inrange:>4s}")

    def _flip(self, side, mids):
        for mid in mids:
            if mid == 8:
                print("    J8 (gripper): sign+scale come from the closed/open "
                      "measurement, not a stored-stops flip — re-run the wizard "
                      "to recalibrate the gripper.")
                continue
            name = joint_name(side, mid)
            e = self.full_map[name]
            stops = (e.get("calibration") or {}).get("stops")
            if not stops:
                print(f"    J{mid}: no stored stops; cannot flip exactly.")
                continue
            e["dir"] = -int(e["dir"])
            e["offset"] = float(solve.recompute_offset(int(e["dir"]), stops))
            print(f"    J{mid}: dir -> {e['dir']:+d}, offset -> {e['offset']:+.4f}")
        self.sm[side] = side_map(self.full_map, side)
        self.bus[side].motor_map = self.sm[side]
        self._write_inplace()

    def _nudge(self, side, delta, mids):
        for mid in mids:
            e = self.full_map[joint_name(side, mid)]
            e["offset"] = float(e["offset"]) + delta
            print(f"    J{mid}: offset += {delta:+.4f} -> {e['offset']:+.4f}")
        self.sm[side] = side_map(self.full_map, side)
        self.bus[side].motor_map = self.sm[side]
        self._write_inplace()

    def verify_loop(self, side):
        print("\n=== LIVE VERIFY ===  Move each joint and watch RViz.")
        print("Commands:  t=table   flip J...   nudge <rad> J...   "
              "stops J   done")
        self.pump()  # refresh joint_latest under the just-applied map
        if self.sim:
            self._print_table(side)
            return
        while not self._stop:
            line = self._pump_until_line("verify>")
            parts = line.split()
            if not parts:
                continue
            cmd = parts[0].lower()
            if cmd in ("done", "q", "quit"):
                return
            if cmd == "t":
                self._print_table(side)
            elif cmd == "flip":
                self._flip(side, _mids(parts[1:]))
            elif cmd == "nudge" and len(parts) >= 3:
                self._nudge(side, float(parts[1]), _mids(parts[2:]))
            elif cmd == "stops" and len(parts) >= 2:
                mid = _mids(parts[1:])[0]
                lo, hi = URDF_LIMITS[side][mid - 1]
                if mid in solve.SINGLE_STOP_JOINTS or mid == 8:
                    print("    re-capture via a fresh run for single-stop/gripper.")
                    continue
                r1 = self._capture_stop(side, mid, "hardstop #1")
                r2 = self._capture_stop(side, mid, "hardstop #2 (opposite)")
                rmin, rmax = min(r1, r2), max(r1, r2)
                self._apply_results(side, {mid: {
                    **solve.solve_arm_joint(int(self.full_map[joint_name(side, mid)]["dir"]),
                                            lo, hi, rmin, rmax),
                    "stops": {"lo": lo, "hi": hi, "r_min": rmin, "r_max": rmax}}})
                self._write_inplace()
            else:
                print("    ? t | flip J... | nudge <rad> J... | stops J | done")


def _mids(tokens):
    out = []
    for tok in tokens:
        tok = tok.lower().lstrip("j")
        if tok.isdigit() and 1 <= int(tok) <= 8:
            out.append(int(tok))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("side", nargs="?", default="left",
                    choices=("left", "right", "both"))
    ap.add_argument("--transport", choices=("socketcan", "sim"),
                    default="socketcan")
    ap.add_argument("--dry-run", action="store_true",
                    help="solve + validate but do not write the map files")
    ap.add_argument("--yes", action="store_true",
                    help="skip the side-confirmation prompt (sim/CI)")
    args = ap.parse_args(argv)
    sides = ["left", "right"] if args.side == "both" else [args.side]

    rclpy.init()
    try:
        node = LiveCalibrator(sides, transport=args.transport)
    except Exception as exc:
        print(f"FAILED to start: {exc}\nNo motor was enabled; no map changed.",
              file=sys.stderr)
        rclpy.shutdown()
        return 2
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, "_stop", True))
    try:
        if args.transport == "socketcan":
            print("Resolved buses:  " + ", ".join(
                f"{s}={SIDES[s]}" for s in sides))
            print("Confirm this matches the PHYSICAL arm(s) you intend to "
                  "calibrate (side is resolved by stable USB path).")
            if not args.yes:
                if node._pump_until_line(
                        "Type the side name to proceed, anything else to abort:"
                ) not in sides + ["both"]:
                    print("Aborted; no motor was enabled.")
                    return 0
        print("\nCompliant-enabling at ZERO torque (arm goes limp, back-drivable, "
              "no snap). Keep the e-stop in reach.")
        node.enable()
        for side in sides:
            node.calibrate_side(side)
        node.commit(dry_run=args.dry_run)
        if not args.dry_run:
            for side in sides:
                node.verify_loop(side)
        return 0
    except KeyboardInterrupt:
        print("\nABORTED (Ctrl-C).")
        return 130
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
