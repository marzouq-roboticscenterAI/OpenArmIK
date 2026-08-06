"""Refresh-only CAN telemetry publisher for passive live RViz visualization.

This node cannot energize or command a motor: a transport firewall rejects
every transmitted frame except Damiao state-refresh requests (0xCC to 0x7ff).
It applies the deployed motor-map transforms and publishes only /joint_states.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from m1_can_tools import dm_protocol as dm
from m1_can_tools.calibrate import (
    ADAPTER_PATH_HINTS,
    BusLocks,
    SIDES,
    _can_socket_preflight,
    _conflicting_processes,
    _iface_preflight,
    side_map,
    validate_map_shape,
)
from m1_can_tools.motor_bus import MotorBus, load_map
from m1_can_tools.transport import SocketCanTransport, Transport


class RefreshOnlyTransport(Transport):
    """Runtime TX firewall: permit state refreshes, reject everything else."""

    def __init__(self, channel: str, motor_ids):
        self._inner = SocketCanTransport(channel, fd=False)
        self._allowed = {
            (dm.PARAM_ARB_ID, dm.refresh_frame(int(mid)))
            for mid in motor_ids
        }
        self.refresh_count = 0
        self.rejected_count = 0

    def send(self, arb_id: int, data: bytes) -> None:
        frame = (int(arb_id), bytes(data))
        if frame not in self._allowed:
            self.rejected_count += 1
            raise PermissionError(
                "passive CAN firewall rejected non-refresh TX frame")
        self.refresh_count += 1
        self._inner.send(*frame)

    def recv(self, timeout: float = 0.0):
        return self._inner.recv(timeout)

    def close(self) -> None:
        self._inner.close()


class PassiveJointState(Node):
    def __init__(self):
        super().__init__("m1_passive_joint_state")
        self.declare_parameter(
            "motor_map", str(Path.home() / ".config/m1/motor_map.yaml"))
        self.declare_parameter("lift_height", 0.62865)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("left_j1_visual_bias", 0.0)
        self.declare_parameter("right_j1_visual_bias", 0.0)
        self.declare_parameter("left_j5_visual_bias", 0.0)
        self.declare_parameter("right_j5_visual_bias", 0.0)
        map_path = Path(str(self.get_parameter("motor_map").value))
        self._lift = float(self.get_parameter("lift_height").value)
        rate = float(self.get_parameter("publish_rate").value)
        self._visual_bias = {
            ("left", 1): float(self.get_parameter("left_j1_visual_bias").value),
            ("right", 1): float(self.get_parameter("right_j1_visual_bias").value),
            ("left", 5): float(self.get_parameter("left_j5_visual_bias").value),
            ("right", 5): float(self.get_parameter("right_j5_visual_bias").value),
        }
        if not 0.0 <= self._lift <= 0.85:
            raise ValueError("lift_height outside URDF range [0, 0.85]")
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate must be positive and finite")

        motor_map = load_map(str(map_path))
        validate_map_shape(motor_map)
        for side, iface in SIDES.items():
            _iface_preflight(iface, ADAPTER_PATH_HINTS[side])
        _can_socket_preflight(SIDES.values())
        conflicts = _conflicting_processes(SIDES.values())
        if conflicts:
            raise RuntimeError("CAN owner conflict:\n  " + "\n  ".join(conflicts))

        self._locks = BusLocks(SIDES.values())
        self._locks.__enter__()
        self._buses = {}
        self._transports = {}
        self._maps = {}
        for side, iface in SIDES.items():
            sm = side_map(motor_map, side)
            self._maps[side] = sm
            tx = RefreshOnlyTransport(iface, [v["id"] for v in sm.values()])
            self._transports[side] = tx
            self._buses[side] = MotorBus(tx, sm)

        # Hot-reload the motor map when the file changes, so calibration edits
        # take effect within a tick without restarting this node.
        self._map_path = map_path
        try:
            self._map_mtime = map_path.stat().st_mtime
        except OSError:
            self._map_mtime = 0.0

        # Live range-of-motion tracking: record raw min/max per joint as the
        # operator sweeps each joint hardstop-to-hardstop, dumped to a JSON the
        # calibration solver reads. Objective (encoder-only) direction data --
        # no viewpoint dependence, unlike eyeballing rotation direction in RViz.
        self._range_path = map_path.parent / "passive_range.json"
        self._rmin = {}
        self._rmax = {}
        self._range_dump_mtime = 0.0

        self._latest = {"left": {}, "right": {}}
        self._raw_latest = {"left": {}, "right": {}}
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._raw_pub = self.create_publisher(
            JointState, "/m1_passive/raw_joint_states", 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warning(
            "PASSIVE LIVE RVIZ telemetry active: CAN TX firewall permits only "
            "0xCC state refreshes; publishing /joint_states; no motor commands")

    def _maybe_reload_map(self):
        try:
            mtime = self._map_path.stat().st_mtime
        except OSError:
            return
        if mtime == self._map_mtime:
            return
        try:
            motor_map = load_map(str(self._map_path))
            validate_map_shape(motor_map)
        except Exception as exc:  # keep serving the last-good map on a bad edit
            self.get_logger().warning(
                f"map reload skipped (invalid): {exc}",
                throttle_duration_sec=3.0)
            self._map_mtime = mtime
            return
        for side in SIDES:
            self._maps[side] = side_map(motor_map, side)
        self._map_mtime = mtime
        self.get_logger().warning("motor map hot-reloaded")

    def _dump_range(self):
        data = {n: {"raw_min": self._rmin[n], "raw_max": self._rmax[n],
                    "raw_span": self._rmax[n] - self._rmin[n]}
                for n in sorted(self._rmin)}
        tmp = self._range_path.with_suffix(".json.tmp")
        try:
            tmp.write_text(json.dumps(data, indent=2) + "\n")
            tmp.replace(self._range_path)
        except OSError:
            pass

    def _read_side_complete(self, side, deadline=0.15):
        """Collect a FRESH reply from every mapped motor on this bus, retrying
        the ones that miss a refresh burst until all answer or the deadline.
        Prevents a motor that skips a burst from publishing a stale (frozen)
        value -- the bug that made distal joints look motionless."""
        want = set(self._maps[side])
        got = {}
        end = time.monotonic() + deadline
        while True:
            for name, raw in self._buses[side].telemetry_all_raw(timeout=0.02).items():
                got[name] = raw
            if want.issubset(got) or time.monotonic() >= end:
                return got

    def _tick(self):
        self._maybe_reload_map()
        for side in ("right", "left"):
            got = self._read_side_complete(side)
            for name, raw in got.items():
                self._raw_latest[side][name] = raw
                rp = float(raw["pos"])
                self._rmin[name] = min(self._rmin.get(name, rp), rp)
                self._rmax[name] = max(self._rmax.get(name, rp), rp)
                fb = MotorBus._to_joint_frame(
                    dict(raw), name, self._maps[side][name])
                if int(fb.get("err", 0)) != 0:
                    self.get_logger().error(
                        f"{name}: motor error {fb['err']}",
                        throttle_duration_sec=2.0)
                self._latest[side][name] = fb
        self._dump_range()
        if any(len(self._latest[side]) != 8 for side in ("left", "right")):
            self.get_logger().warning(
                "waiting for all 16 arm motors",
                throttle_duration_sec=2.0)
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["lift_joint"]
        msg.position = [self._lift]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        for side in ("left", "right"):
            for name, fb in self._latest[side].items():
                msg.name.append(name)
                pos = float(fb["pos"])
                for joint_num in (1, 5):
                    if name == f"openarm_{side}_joint{joint_num}":
                        pos += self._visual_bias[(side, joint_num)]
                msg.position.append(pos)
                msg.velocity.append(float(fb["vel"]))
                msg.effort.append(float(fb["torque"]))
                if name.endswith("finger_joint1"):
                    # URDF mimic joint is state-visible and follows finger1 1:1.
                    msg.name.append(name.replace("finger_joint1", "finger_joint2"))
                    msg.position.append(float(fb["pos"]))
                    msg.velocity.append(float(fb["vel"]))
                    msg.effort.append(float(fb["torque"]))
        self._pub.publish(msg)

        raw_msg = JointState()
        raw_msg.header.stamp = msg.header.stamp
        for side in ("left", "right"):
            for name, fb in self._raw_latest[side].items():
                raw_msg.name.append(name)
                raw_msg.position.append(float(fb["pos"]))
                raw_msg.velocity.append(float(fb["vel"]))
                raw_msg.effort.append(float(fb["torque"]))
        self._raw_pub.publish(raw_msg)

    def destroy_node(self):
        # Never call MotorBus.close(): it sends disable frames. Close raw
        # transports only, preserving this node's refresh-only TX guarantee.
        for tx in getattr(self, "_transports", {}).values():
            tx.close()
        locks = getattr(self, "_locks", None)
        if locks is not None:
            locks.__exit__(None, None, None)
            self._locks = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PassiveJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
