import copy
import json
from pathlib import Path

import pytest

from m1_can_tools import dm_protocol as dm
from m1_can_tools.calibrate import (
    _validate_adapter_properties,
    build_calibrated_map, commit_maps, joint_name, run, parser, rollback,
    current_gripper_mapping, validate_map_shape,
)
from m1_can_tools.motor_bus import MotorBus, load_map
from m1_can_tools.transport import FakeTransport


def base_map():
    out = {}
    models = ["DM8009", "DM8009", "DM4340_V20", "DM4340_V20",
              "DM4310P", "DM4310P", "DM4310P", "DM4310P"]
    for side in ("right", "left"):
        for mid, model in enumerate(models, 1):
            out[joint_name(side, mid)] = {
                "id": mid, "master_id": mid + 0x10, "model": model,
                "kp": 10.0, "kd": 0.5, "dir": 1, "scale": 1.0,
                "offset": 0.0,
                "soft_limits": {"pos": [-1.0, 1.0], "vel": 1.0, "effort": 1.0},
            }
    return out


def captures():
    closed, opened = {}, {}
    for side in ("right", "left"):
        for mid in range(1, 9):
            name = joint_name(side, mid)
            closed[name] = {"pos": 0.1 * mid}
            opened[name] = {"pos": 0.1 * mid}
        opened[joint_name(side, 8)]["pos"] -= 1.05
    return closed, opened


def test_transform_scale_math_matches_inverse():
    info = {
        "id": 1, "master_id": 0x11, "model": "DM4310",
        "dir": -1, "scale": 0.5, "offset": 0.2,
    }
    fb = {"pos": 1.2, "vel": 0.4, "torque": 2.0}
    got = MotorBus._to_joint_frame(fb, "j", info)
    assert got["pos"] == pytest.approx(-0.4)
    assert got["vel"] == pytest.approx(-0.2)
    assert got["torque"] == pytest.approx(-4.0)
    motor = (got["pos"] - info["offset"]) / (info["dir"] * info["scale"])
    assert motor == pytest.approx(1.2)


def test_passive_raw_telemetry_sends_refresh_only():
    m = {"j": {"id": 1, "master_id": 0x11, "model": "DM4310"}}
    t = FakeTransport()
    t.inject(0x11, dm.encode_feedback(1, 0.5, 0, 0, 30, 31, "DM4310"))
    got = MotorBus(t, m).telemetry_all_raw()
    assert got["j"]["pos"] == pytest.approx(0.5, abs=0.001)
    assert t.sent == [(dm.PARAM_ARB_ID, dm.refresh_frame(1))]


def test_build_map_uses_identity_arms_and_measured_grippers():
    closed, opened = captures()
    result, notes = build_calibrated_map(base_map(), closed, opened, {})
    assert result[joint_name("left", 1)]["offset"] == pytest.approx(-0.1)
    left = result[joint_name("left", 8)]
    right = result[joint_name("right", 8)]
    assert left["dir"] == -1 and right["dir"] == -1
    assert left["scale"] == pytest.approx(0.044 / 1.05)
    assert right["scale"] == pytest.approx(0.044 / 1.05)
    assert any("J2" in n and "no historical" in n for n in notes)
    validate_map_shape(result)


def test_map_rejects_wrong_variant_decode():
    wrong = base_map()
    wrong[joint_name("right", 8)]["model"] = "DM3507"
    with pytest.raises(ValueError, match="OpenArm v1.0 model"):
        validate_map_shape(wrong)


def test_only_current_prismatic_gripper_map_can_be_preserved():
    info = base_map()[joint_name("left", 8)]
    assert not current_gripper_mapping(info, "left")
    info["soft_limits"]["pos"] = [0.0, 0.044]
    assert current_gripper_mapping(info, "left")
    info["soft_limits"]["pos"] = [-0.7854, 0.0]
    assert not current_gripper_mapping(info, "left")


def test_adapter_identity_is_role_specific():
    props = "ID_MODEL=PCAN-USB_FD\nID_PATH=platform-usb-0:4.4.4.3:1.0\n"
    _validate_adapter_properties(props, "can2", "4.4.4.3")
    with pytest.raises(RuntimeError, match="identity mismatch"):
        _validate_adapter_properties(props, "can3", "4.4.4.4")


def test_atomic_dual_save_backup_and_rollback(tmp_path):
    old = base_map()
    new, _ = build_calibrated_map(old, *captures(), {})
    deployed = tmp_path / "cfg" / "motor_map.yaml"
    repo = tmp_path / "repo.yaml"
    from m1_can_tools.motor_bus import save_map
    save_map(str(deployed), old)
    save_map(str(repo), old)
    backup = commit_maps(new, deployed, repo, tmp_path / "backups")
    assert load_map(str(deployed)) == new == load_map(str(repo))
    assert (backup / "manifest.json").exists()
    rollback(backup)
    assert load_map(str(deployed)) == old == load_map(str(repo))


def test_dry_run_changes_nothing(tmp_path):
    old = base_map()
    new, _ = build_calibrated_map(old, *captures(), {})
    deployed = tmp_path / "deployed.yaml"
    repo = tmp_path / "repo.yaml"
    from m1_can_tools.motor_bus import save_map
    save_map(str(deployed), old)
    save_map(str(repo), old)
    commit_maps(new, deployed, repo, tmp_path / "backups", dry_run=True)
    assert load_map(str(deployed)) == old == load_map(str(repo))
    assert not (tmp_path / "backups").exists()


def test_second_save_failure_rolls_back_deployed(tmp_path, monkeypatch):
    import m1_can_tools.calibrate as cal
    from m1_can_tools.motor_bus import save_map
    old = base_map()
    new, _ = build_calibrated_map(old, *captures(), {})
    deployed, repo = tmp_path / "deployed.yaml", tmp_path / "repo.yaml"
    save_map(str(deployed), old)
    save_map(str(repo), old)
    calls = [0]

    def fail_second(path, value):
        calls[0] += 1
        if calls[0] == 2:
            raise OSError("injected repo failure")
        save_map(path, value)

    monkeypatch.setattr(cal, "save_map", fail_second)
    with pytest.raises(OSError, match="injected"):
        commit_maps(new, deployed, repo, tmp_path / "backups")
    assert load_map(str(deployed)) == old
    assert load_map(str(repo)) == old


def test_aborted_confirmation_writes_nothing(tmp_path):
    source = tmp_path / "source.yaml"
    deployed = tmp_path / "deployed.yaml"
    repo = tmp_path / "repo.yaml"
    from m1_can_tools.motor_bus import save_map
    save_map(str(source), base_map())
    args = parser().parse_args([
        "--transport", "sim", "--capture-dropped", "--confirm", "NO",
        "--source-map", str(source), "--repo-map", str(repo),
        "--deployed-map", str(deployed), "--backup-root", str(tmp_path / "backups"),
        "--hardstops", str(tmp_path / "missing.json"),
    ])
    with pytest.raises(RuntimeError, match="confirmation"):
        run(args)
    assert not deployed.exists() and not repo.exists()


def test_keyboard_abort_returns_130(monkeypatch):
    import m1_can_tools.calibrate as cal
    monkeypatch.setattr(cal, "run", lambda _args: (_ for _ in ()).throw(KeyboardInterrupt()))
    assert cal.main(["--transport", "sim"]) == 130
