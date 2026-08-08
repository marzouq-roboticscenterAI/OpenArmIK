"""Guarded, passive-first calibration support for the OpenArm v1.0 pair.

The default action is a read-only preflight.  This module has no set-zero path
and never sends enable, MIT, velocity, or position commands.  The only CAN
frames it emits are documented Damiao state-refresh requests (0xCC to 0x7ff).
"""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import fcntl
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path
from typing import Dict, Iterable, Tuple

import yaml

from m1_can_tools.motor_bus import MotorBus, load_map, save_map
from m1_can_tools.transport import SimTransport, SocketCanTransport

# Legacy USB-path side resolution. Adapters are matched on the final two
# octets* of their sysfs USB path (the parent hub port changes across replugs:
# seen at ...4.4.4.x, then ...4.3.4.x): `.4.3` and `.4.4`.
#
# The current dual-channel adapter cannot be separated by USB path. The
# repository launcher supplies the commissioned assignment explicitly:
# can0=robot-right and can1=robot-left, confirmed 2026-08-06 by moving each arm
# under the strictly read-only observer. These hints remain only for the older
# single-channel PCAN deployment from which DREAD was collected.
ADAPTER_PATH_HINTS = {"right": ".4.4", "left": ".4.3"}
# Fallback used only when no matching interface is found (sim / CI / no bus).
_SIDES_DEFAULT = {"right": "can0", "left": "can1"}
# The current dual-channel DM-USB2FDCAN uses `gs_usb`; the collected deployment
# used PCAN-USB/`peak_usb`. Keep both accepted for provenance and offline tests,
# while dread.sh supplies this machine's commissioned side map explicitly.
_ARM_CAN_DRIVERS = {"peak_usb", "gs_usb"}
_ARM_CAN_MODELS = {"PCAN-USB_FD", "DM-USB2FDCAN"}


_USB_PORT_RE = re.compile(r"^\d+-[\d.]+$")


def _usb_port_component(devpath: str):
    """The DEEPEST USB port-chain component of a sysfs device path (e.g.
    ``1-4.3.4.4`` from ``.../usb1/1-4/1-4.3/1-4.3.4/1-4.3.4.4/1-4.3.4.4:1.0``),
    or None. Matching hints against THIS (not the whole path) is load-bearing:
    a parent-hub dir like ``1-4.4`` sits in every descendant's full path, so a
    substring match there once resolved BOTH sides to one interface."""
    comp = None
    for part in devpath.split("/"):
        base = part.split(":", 1)[0]
        if _USB_PORT_RE.match(base):
            comp = base
    return comp


def _resolve_iface_by_path(path_hint: str, net_root="/sys/class/net"):
    """Return the canN interface whose deepest USB port component ENDS WITH
    path_hint, or None.

    Kernel canN numbering follows plug order, so it changes across replugs; the
    A single-channel adapter's USB *path* is the stable identity. Resolving by path keeps left/right
    correct no matter how the interfaces were renumbered (a left/right swap on
    energized arms is dangerous). Only interfaces driven by ``_ARM_CAN_DRIVER``
    qualify (the lift XCAN's port ``...4.4.3`` also ends with ``.4.3``)."""
    net = Path(net_root)
    if not net.exists():
        return None
    for entry in sorted(net.iterdir()):
        if not entry.name.startswith("can"):
            continue
        try:
            devpath = (entry / "device").resolve().as_posix()
        except OSError:
            continue
        try:
            driver = (entry / "device" / "driver").resolve().name
        except OSError:
            driver = None
        if driver is not None and driver not in _ARM_CAN_DRIVERS:
            continue
        port = _usb_port_component(devpath)
        if port is not None and port.endswith(path_hint):
            return entry.name
    return None


def resolve_sides():
    """Map side -> live canN from the persistent-label wrapper environment.

    ``dread.sh`` passes this machine's commissioned channels as
    ``M1_CAN_LEFT``/``M1_CAN_RIGHT``. This is independent of USB topology. The
    old USB-path resolver remains only as a compatibility fallback for offline
    tests and direct legacy invocation.

    If resolution is AMBIGUOUS (both sides land on one interface -- e.g. an
    adapter re-plugged so only one hint matches and the other side's default
    aliases it), fall back to the plug-order defaults for BOTH sides and say
    so loudly: a distinct-but-possibly-renumbered guess is recoverable, two
    sides driving one bus is not."""
    env = {side: os.environ.get(f"M1_CAN_{side.upper()}", "").strip()
           for side in ("left", "right")}
    if any(env.values()):
        if not all(env.values()):
            raise RuntimeError(
                "M1_CAN_LEFT and M1_CAN_RIGHT must be provided together")
        if any(not re.fullmatch(r"can[0-9]+", iface) for iface in env.values()):
            raise RuntimeError(f"invalid M1_CAN arm interface mapping: {env}")
        if env["left"] == env["right"]:
            raise RuntimeError(f"ambiguous M1_CAN arm interface mapping: {env}")
        return env

    sides = {side: (_resolve_iface_by_path(hint) or _SIDES_DEFAULT[side])
             for side, hint in ADAPTER_PATH_HINTS.items()}
    if len(set(sides.values())) < len(sides):
        print(f"[calibrate] WARNING: side->bus resolution ambiguous ({sides}); "
              f"falling back to plug-order defaults {_SIDES_DEFAULT} -- verify "
              f"the wiring before energizing", file=sys.stderr)
        return dict(_SIDES_DEFAULT)
    return sides


SIDES = resolve_sides()
EXPECTED_IDS = set(range(1, 9))
EXPECTED_MODELS = (
    ("DM8009",), ("DM8009",),
    ("DM4340", "DM4340_V20"), ("DM4340", "DM4340_V20"),
    ("DM4310", "DM4310P"), ("DM4310", "DM4310P"),
    ("DM4310", "DM4310P"), ("DM4310", "DM4310P"),
)
MAX_ABS_VEL = 0.08
MAX_TEMP_C = 65
GRIPPER_TRAVEL = {"left": 0.044, "right": 0.044}
LEGACY_GRIPPER_MOTOR_SPAN = 1.0472

# This robot's URDF limits.  They are command limits, not inferred hardstops.
URDF_LIMITS = {
    "left": [
        (-3.4907, 1.3963), (-3.3161, 0.17453), (-1.5708, 1.5708),
        (0.0, 2.4435), (-1.5708, 1.5708), (-0.7854, 0.7854),
        (-1.5708, 1.5708), (0.0, 0.044),
    ],
    "right": [
        (-1.3963, 3.4907), (-0.17453, 3.3161), (-1.5708, 1.5708),
        (0.0, 2.4435), (-1.5708, 1.5708), (-0.7854, 0.7854),
        (-1.5708, 1.5708), (0.0, 0.044),
    ],
}


def joint_name(side: str, motor_id: int) -> str:
    suffix = f"joint{motor_id}" if motor_id <= 7 else "finger_joint1"
    return f"openarm_{side}_{suffix}"


def side_map(full_map: dict, side: str) -> dict:
    prefix = f"openarm_{side}_"
    return {k: copy.deepcopy(v) for k, v in full_map.items()
            if k.startswith(prefix) and not k.endswith("finger_joint2")}


def current_gripper_mapping(info: dict, side: str) -> bool:
    """Whether a preserved J8 map uses this URDF's 0..44 mm joint domain."""
    try:
        limits = info["soft_limits"]["pos"]
        expected = URDF_LIMITS[side][7]
        return (len(limits) == 2 and
                all(math.isfinite(float(value)) for value in limits) and
                abs(float(limits[0]) - expected[0]) <= 1.0e-12 and
                abs(float(limits[1]) - expected[1]) <= 1.0e-12)
    except (KeyError, TypeError, ValueError):
        return False


def validate_map_shape(m: dict) -> None:
    for side in SIDES:
        sm = side_map(m, side)
        ids = {int(v["id"]) for v in sm.values()}
        if len(sm) != 8 or ids != EXPECTED_IDS:
            raise ValueError(
                f"{side}: expected exactly motor ids 1..8, got {sorted(ids)}")
        for mid, models in enumerate(EXPECTED_MODELS, 1):
            actual = sm[joint_name(side, mid)].get("model")
            if actual not in models:
                raise ValueError(
                    f"{side} J{mid}: expected OpenArm v1.0 model in {models}, "
                    f"got {actual!r}")
        masters = {int(v.get("master_id", int(v["id"]) + 0x10)) for v in sm.values()}
        if len(masters) != 8:
            raise ValueError(f"{side}: duplicate master_id in motor map")
        for name, info in sm.items():
            d = float(info.get("dir", 1))
            s = float(info.get("scale", 1))
            off = float(info.get("offset", 0))
            if d not in (-1, 1):
                raise ValueError(f"{name}: dir must be +1 or -1")
            if not math.isfinite(s) or s <= 0:
                raise ValueError(f"{name}: scale must be finite and > 0")
            if not math.isfinite(off):
                raise ValueError(f"{name}: offset must be finite")


def _validate_adapter_properties(props: str, iface: str, path_hint: str) -> None:
    if (not any(f"ID_MODEL={model}" in props for model in _ARM_CAN_MODELS)
            or path_hint not in props):
        raise RuntimeError(
            f"{iface}: adapter identity mismatch (expected OpenArm CAN adapter "
            f"at USB path containing {path_hint})")


def _iface_preflight(iface: str, path_hint: str) -> None:
    state = Path(f"/sys/class/net/{iface}/operstate")
    if not state.exists():
        raise RuntimeError(f"{iface}: interface does not exist")
    if state.read_text().strip() not in ("up", "unknown"):
        raise RuntimeError(f"{iface}: interface is not UP")
    try:
        p = subprocess.run(
            ["ip", "-details", "link", "show", iface],
            capture_output=True, text=True, timeout=3, check=True)
        text = p.stdout.replace("\n", " ")
        if "bitrate 1000000" not in text:
            raise RuntimeError(
                f"{iface}: expected 1000000 bit/s arbitration bitrate")
    except FileNotFoundError:
        pass
    try:
        p = subprocess.run(
            ["udevadm", "info", "-q", "property", "-p",
             f"/sys/class/net/{iface}"],
            capture_output=True, text=True, timeout=3, check=True)
        # A wrapper-provided channel assignment supersedes the legacy path
        # hint. Still require a supported OpenArm USB-CAN adapter model.
        if iface in (os.environ.get("M1_CAN_LEFT"), os.environ.get("M1_CAN_RIGHT")):
            if not any(f"ID_MODEL={model}" in p.stdout for model in _ARM_CAN_MODELS):
                raise RuntimeError(f"{iface}: expected an OpenArm USB-CAN adapter")
        else:
            _validate_adapter_properties(p.stdout, iface, path_hint)
    except (FileNotFoundError, subprocess.CalledProcessError):
        # The minimal Jazzy deployment image has sysfs mounted from the host but
        # intentionally omits udevadm. Verify the same two facts directly.
        device = Path(f"/sys/class/net/{iface}/device").resolve()
        driver = Path(f"/sys/class/net/{iface}/device/driver").resolve().name
        props = (
            f"ID_MODEL={'PCAN-USB_FD' if driver == 'peak_usb' else ('DM-USB2FDCAN' if driver == 'gs_usb' else 'unknown')}\n"
            f"ID_PATH={device}\n")
        if iface in (os.environ.get("M1_CAN_LEFT"), os.environ.get("M1_CAN_RIGHT")):
            if driver not in _ARM_CAN_DRIVERS:
                raise RuntimeError(f"{iface}: unsupported CAN adapter driver {driver}")
        else:
            _validate_adapter_properties(props, iface, path_hint)


def _conflicting_processes(ifaces: Iterable[str]) -> list:
    needles = ("m1_hwconfig", "ros2_control_node", "controller_manager",
               "OpenArm/app.py", "backend.py")
    found = []
    me = os.getpid()
    for p in Path("/proc").iterdir():
        if not p.name.isdigit() or int(p.name) == me:
            continue
        try:
            cmd = (p / "cmdline").read_bytes().replace(b"\0", b" ").decode()
        except (OSError, UnicodeDecodeError):
            continue
        if any(n in cmd for n in needles) and (
                any(i in cmd for i in ifaces) or "ros2_control_node" in cmd):
            found.append(f"pid {p.name}: {cmd[:180]}")
    return found


def _can_socket_preflight(ifaces: Iterable[str]) -> None:
    """Refuse if the host network namespace already has CAN receive sockets."""
    text = ""
    for name in ("rcvlist_all", "rcvlist_fil", "rcvlist_inv", "rcvlist_sff"):
        path = Path("/proc/net/can") / name
        if path.exists():
            text += path.read_text()
    busy = []
    for iface in ifaces:
        lines = [line.strip() for line in text.splitlines() if f"({iface}:" in line]
        if any("no entry" not in line for line in lines):
            busy.append(iface)
    if busy:
        raise RuntimeError(
            "exclusive bus ownership failed: existing SocketCAN receiver on "
            + ", ".join(busy) + " (stop hwconfig/ros2_control first)")


class BusLocks:
    def __init__(self, ifaces: Iterable[str]):
        self.ifaces = tuple(ifaces)
        self.stack = ExitStack()

    def __enter__(self):
        for iface in self.ifaces:
            lock_dir = Path.home() / ".config/m1"
            lock_dir.mkdir(parents=True, exist_ok=True)
            path = lock_dir / f".calibration-{iface}.lock"
            fh = self.stack.enter_context(open(path, "w"))
            try:
                fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(f"{iface}: calibration lock already held") from exc
            fh.write(f"{os.getpid()}\n")
            fh.flush()
        return self

    def __exit__(self, *exc):
        self.stack.close()


def sample_bus(bus: MotorBus, expected: set, samples: int = 4) -> dict:
    seen: Dict[str, list] = {j: [] for j in expected}
    for _ in range(samples):
        got = bus.telemetry_all_raw(timeout=0.03)
        for name, fb in got.items():
            if name in seen:
                seen[name].append(fb)
        time.sleep(0.04)
    missing = sorted(j for j, values in seen.items() if not values)
    if missing:
        raise RuntimeError(f"missing telemetry: {', '.join(missing)}")
    out = {}
    for name, values in seen.items():
        if any(int(v["err"]) != 0 for v in values):
            raise RuntimeError(f"{name}: motor error {[v['err'] for v in values]}")
        if max(abs(float(v["vel"])) for v in values) > MAX_ABS_VEL:
            raise RuntimeError(f"{name}: motor is moving; wait until fully still")
        hot = max(max(float(v["t_mos"]), float(v["t_rotor"])) for v in values)
        if hot > MAX_TEMP_C:
            raise RuntimeError(f"{name}: temperature {hot:.0f} C exceeds {MAX_TEMP_C} C")
        out[name] = {
            key: sum(float(v[key]) for v in values) / len(values)
            for key in ("pos", "vel", "torque", "t_mos", "t_rotor")
        }
        out[name]["err"] = 0
    return out


def import_hardstops(path: Path) -> dict:
    if not path.exists():
        return {}
    data = json.loads(path.read_text())
    out = {}
    # Decode for LEGACY hardstop records (2026-07-09 captures, deprecated path):
    # the channel<->side truth was fixed at THAT session's wiring, so this map is
    # deliberately NOT updated for the 2026-07-11 side swap. It only feeds a span
    # self-check, and mirrored templates have identical spans, so the side label
    # cannot change the check's outcome.
    channel_side = {"0": "right", "1": "left"}
    for rec in data.values():
        try:
            side = channel_side[str(rec["channel"])]
            mid = int(rec["motor_id"])
            result = rec.get("result") or {}
            if "span" in result:
                span = float(result["span"])
            else:
                span = float(rec["pos_max"]) - float(rec["pos_min"])
            if mid in EXPECTED_IDS and span > 0:
                out[(side, mid)] = {
                    "span": span, "method": rec.get("method"),
                    "saved_at": rec.get("saved_at"),
                }
        except (KeyError, TypeError, ValueError):
            continue
    return out


def build_calibrated_map(base: dict, closed: dict, opened: dict,
                         hardstops: dict) -> Tuple[dict, list]:
    result = copy.deepcopy(base)
    notes = []
    for side in SIDES:
        for mid in range(1, 9):
            name = joint_name(side, mid)
            info = result[name]
            raw0 = float(closed[name]["pos"])
            lo, hi = URDF_LIMITS[side][mid - 1]
            info["soft_limits"]["pos"] = [lo, hi]
            if mid <= 7:
                # The exact hardware-proven OpenArm-C app maps J1..J7 raw motor
                # angles directly on both arm buses (no side-specific sign).
                info["dir"] = 1
                info["scale"] = 1.0
                info["offset"] = -raw0
                source = "proven_openarm_raw_identity+dropped_reference"
            else:
                raw1 = float(opened[name]["pos"])
                delta = raw1 - raw0
                desired = GRIPPER_TRAVEL[side]
                if abs(delta) < 0.25 or abs(delta) > 3.0:
                    raise ValueError(
                        f"{side} gripper travel {delta:.4f} motor rad is implausible")
                info["dir"] = 1 if desired / delta > 0 else -1
                info["scale"] = abs(desired / delta)
                info["offset"] = -info["dir"] * info["scale"] * raw0
                source = "passive_closed_open_measurement"
                mismatch = abs(abs(delta) - LEGACY_GRIPPER_MOTOR_SPAN)
                if mismatch > 0.20:
                    notes.append(
                        f"{side} gripper motor span {abs(delta):.3f} differs from "
                        f"legacy approximate {LEGACY_GRIPPER_MOTOR_SPAN:.3f}; "
                        "measured scale was used")
            hs = hardstops.get((side, mid))
            info["calibration"] = {
                "source": source,
                "captured_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                "raw_reference": round(raw0, 7),
                "hardstop": hs,
                "firmware_zero_changed": False,
            }
            if hs is None:
                notes.append(
                    f"{side} J{mid}: no historical hardstop record; URDF command "
                    "limits retained (no sweep performed)")
    validate_map_shape(result)
    # Mathematical zero check for every captured reference.
    for side in SIDES:
        for mid in range(1, 9):
            name = joint_name(side, mid)
            i = result[name]
            q = i["dir"] * i["scale"] * closed[name]["pos"] + i["offset"]
            if abs(q) > 1e-8:
                raise AssertionError(f"{name}: reference transform did not map to zero")
    return result, notes


def _backup(path: Path, directory: Path) -> None:
    if path.exists():
        shutil.copy2(path, directory / (path.name + ".bak"))


def commit_maps(m: dict, deployed: Path, repo: Path,
                backup_root: Path, dry_run: bool = False) -> Path:
    validate_map_shape(m)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = backup_root / stamp
    if dry_run:
        return backup
    backup.mkdir(parents=True, exist_ok=False)
    _backup(deployed, backup)
    _backup(repo, backup)
    manifest = {
        "deployed": str(deployed), "repo": str(repo),
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    (backup / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    old_deployed = deployed.read_bytes() if deployed.exists() else None
    try:
        save_map(str(deployed), m)
        save_map(str(repo), m)
    except BaseException:
        if old_deployed is None:
            deployed.unlink(missing_ok=True)
        else:
            deployed.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=deployed.parent)
            with os.fdopen(fd, "wb") as fh:
                fh.write(old_deployed)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, deployed)
        raise
    return backup


def rollback(directory: Path) -> None:
    manifest = json.loads((directory / "manifest.json").read_text())
    for key in ("deployed", "repo"):
        dst = Path(manifest[key])
        src = directory / (dst.name + ".bak")
        if not src.exists():
            raise RuntimeError(f"backup lacks {src.name}; refusing partial rollback")
    for key in ("deployed", "repo"):
        dst = Path(manifest[key])
        save_map(str(dst), load_map(str(directory / (dst.name + ".bak"))))


def _transport(kind: str, iface: str, smap: dict):
    if kind == "sim":
        motors = {
            int(v["id"]): {
                "master_id": int(v.get("master_id", int(v["id"]) + 0x10)),
                "model": v["model"], "pos": 0.03 * (int(v["id"]) - 4),
            } for v in smap.values()
        }
        return SimTransport(motors)
    return SocketCanTransport(iface, fd=False)


def _capture(buses: dict) -> dict:
    out = {}
    for side, bus in buses.items():
        out.update(sample_bus(bus, set(bus.joints())))
        print(f"  {side}: ids 1..8 healthy, still, <= {MAX_TEMP_C} C")
    return out


def run(args) -> int:
    if args.rollback:
        rollback(Path(args.rollback))
        print("Rollback restored both motor maps atomically.")
        return 0
    base = load_map(args.source_map)
    validate_map_shape(base)
    if args.transport == "socketcan":
        for side, iface in SIDES.items():
            _iface_preflight(iface, ADAPTER_PATH_HINTS[side])
        _can_socket_preflight(SIDES.values())
        conflicts = _conflicting_processes(SIDES.values())
        if conflicts:
            raise RuntimeError("exclusive bus ownership failed:\n  " + "\n  ".join(conflicts))
    hardstops = import_hardstops(Path(args.hardstops))
    print(f"Imported {len(hardstops)}/16 historical hardstop records "
          "(J2 remains intentionally unswept).")
    with BusLocks(SIDES.values()):
        buses = {}
        try:
            for side, iface in SIDES.items():
                sm = side_map(base, side)
                buses[side] = MotorBus(_transport(args.transport, iface, sm), sm)
            print("Passive preflight (refresh telemetry only; motors remain disabled):")
            closed = _capture(buses)
            if not args.capture_dropped:
                print("PASS. No motor was enabled or commanded; no file was changed.")
                return 0
            if not sys.stdin.isatty() and not args.confirm:
                raise RuntimeError("capture requires an interactive terminal or --confirm")
            phrase = args.confirm or input(
                "\nPhysically align BOTH arms in the URDF dropped zero reference:\n"
                "  links/wrists straight and matched, both grippers fully CLOSED.\n"
                "Keep motor power disabled. Type DROPPED-AND-ALIGNED: ").strip()
            if phrase != "DROPPED-AND-ALIGNED":
                raise RuntimeError("confirmation did not match; nothing saved")
            closed = _capture(buses)
            if args.transport == "sim":
                for bus in buses.values():
                    for motor in bus.transport._m.values():
                        if motor["master_id"] == 0x18:
                            motor["pos"] -= 1.05
            else:
                input(
                    "\nManually open BOTH disabled grippers fully, without moving "
                    "the wrists. Press Enter to capture: ")
            opened = _capture(buses)
            calibrated, notes = build_calibrated_map(
                base, closed, opened, hardstops)
            backup = commit_maps(
                calibrated, Path(args.deployed_map), Path(args.repo_map),
                Path(args.backup_root), dry_run=args.dry_run)
            for note in notes:
                print("NOTE:", note)
            if args.dry_run:
                print("DRY RUN PASS. Validated generated map; no file changed.")
            else:
                print(f"SAVED both maps atomically. Backup/rollback: {backup}")
                print(f"Rollback command: m1_calibrate --rollback {backup}")
            return 0
        finally:
            # Close sockets only.  Never call MotorBus.close(): it sends disable
            # frames and would violate this workflow's refresh-only guarantee.
            for bus in buses.values():
                bus.transport.close()


def parser() -> argparse.ArgumentParser:
    repo = Path.cwd() / "dread/config/motor_map.openarm_v10.yaml"
    hardstops = (
        Path.cwd()
        / "dread/config/openarm_hardstops.m1robot.json"
    )
    deployed = Path.home() / ".config/m1/motor_map.yaml"
    p = argparse.ArgumentParser(
        description="Passive-first OpenArm v1.0 calibration support (never enables motors)")
    p.add_argument("--capture-dropped", action="store_true",
                   help="confirm dropped zero, measure grippers, then save")
    p.add_argument("--transport", choices=("socketcan", "sim"), default="socketcan")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--confirm", help=argparse.SUPPRESS)
    p.add_argument("--source-map", default=str(repo))
    p.add_argument("--repo-map", default=str(repo))
    p.add_argument("--deployed-map", default=str(deployed))
    p.add_argument("--backup-root",
                   default=str(Path.home() / ".config/m1/calibration-backups"))
    p.add_argument("--hardstops", default=str(hardstops))
    p.add_argument("--rollback")
    return p


def main(argv=None):
    try:
        return run(parser().parse_args(argv))
    except KeyboardInterrupt:
        print("\nABORTED. Motors were never enabled or commanded; no map saved.",
              file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAILED: {exc}\nNo map was applied.", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
