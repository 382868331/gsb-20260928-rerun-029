"""机器人交接:归属唯一、双方到位占用、预约不绕行、承诺可调整性。"""

import json

import pytest

from mrc.coordinator import CoordinationError, Coordinator
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.replay import replay
from mrc.robot import Robot, RobotStatus


def open_lane(width=10, height=2):
    obstacles = {(x, y) for x in range(width) for y in range(height) if y != 1}
    return GridMap(width, height, obstacles)


def build(tmp_path, width=12, positions=((0, 1), (11, 1))):
    gmap = open_lane(width)
    robots = [Robot("R1", positions[0]), Robot("R2", positions[1])]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "ho.log")
    ex.register_cargo("C1", "R1")
    return gmap, robots, ex


def logs(ex):
    return [json.loads(line) for line in ex.log_path.read_text().splitlines()]


def test_handover_only_completes_when_both_arrived_and_cargo_moves_once(tmp_path):
    _, robots, ex = build(tmp_path)
    ex.register_handover_zone("Z", (5, 1), (6, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    ex.run(60)
    assert robots[0].status == RobotStatus.IDLE
    assert robots[1].status == RobotStatus.DONE
    assert robots[1].pos == (11, 1)
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R2"
    records = logs(ex)
    arrivals = [r for r in records if r["event"] == "handover_party_arrived"]
    complete = [r for r in records if r["event"] == "handover_complete"]
    assert len(complete) == 1
    assert {r["robot"] for r in arrivals} == {"R1", "R2"}
    first = arrivals[0]
    assert first["owner"] == "R1"  # 一方先到,物体不记两边
    complete_tick = complete[0]["tick"]
    before = [
        r["owners"]["C1"]
        for r in records
        if r["event"] == "cargo_state" and r["tick"] < complete_tick
    ]
    assert set(before) == {"R1"}
    after = [
        r["owners"]["C1"]
        for r in records
        if r["event"] == "cargo_state" and r["tick"] >= complete_tick
    ]
    assert set(after) == {"R2"}
    assert before and after and before[-1] != after[0]
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_handover_uses_shared_reservation_table(tmp_path):
    """交接双方的槽位预约与普通预约一样进表,不能凭交接闯入。"""
    gmap = GridMap(12, 3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (11, 1)), Robot("R3", (6, 2))]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "ho.log")
    ex.register_cargo("C1", "R1")
    ex.assign({"R3": __import__("mrc.robot", fromlist=["Task"]).Task("P", (6, 1))})
    for _ in range(1):
        ex.step()
    ex.pause("R3")
    ex.register_handover_zone("Z", (6, 1), (7, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    ex.run(20)
    commitment = ex.coordinator.get_commitment("H")
    assert commitment.state.value == "in_progress"
    assert robots[2].pos == (6, 1)
    report = replay(ex.log_path)
    assert not report.collisions and not report.edge_swaps
    ex.resume("R3")
    ex.assign({"R3": __import__("mrc.robot", fromlist=["Task"]).Task("P2", (0, 1))})
    ex.run(80)
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R2"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_delay_keeps_ownership_and_waiting_party_options(tmp_path):
    _, robots, ex = build(tmp_path)
    ex.register_handover_zone("Z", (3, 1), (4, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    ex.step()
    ex.pause("R2")  # 接收方延迟
    for _ in range(4):
        ex.step()
    assert robots[0].pos == (3, 1)
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R1"
    commitment = ex.coordinator.get_commitment("H")
    assert commitment.state.value == "in_progress"
    options = {o.kind: o for o in ex.handover_options("H", "R1", evac_cell=(2, 1))}
    assert options["wait"].feasible
    assert options["evacuate"].feasible
    assert not options["repoint"].feasible  # 已开始,不能改换点
    policy = ex.handover_policy("H")
    assert policy["revocable"] is False
    assert policy["actions"] == ["wait", "evacuate"]
    with pytest.raises(CoordinationError):
        ex.cancel_handover("H")
    ex.resume("R2")
    ex.run(40)
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R2"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_repoint_before_start_preserves_executed_paths_and_reservations(tmp_path):
    gmap = open_lane(16)
    robots = [Robot("R1", (0, 1)), Robot("R2", (15, 1))]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "ho.log")
    ex.register_cargo("C1", "R1")
    ex.register_handover_zone("OLD", (4, 1), (5, 1))
    ex.register_handover_zone("NEW", (10, 1), (11, 1))
    comparison = None
    ex.request_handover("H", "C1", "R1", "R2", "OLD", "L1", "L2", "D1", (15, 1))
    comparison = ex.compare_handovers("H")
    zones = {entry["zone"]: entry for entry in comparison}
    assert "OLD" in zones and "NEW" in zones
    assert all(entry["feasible"] for entry in comparison)
    old_plan = list(robots[0].path)
    ex.repoint_handover("H", "NEW")
    assert robots[0].task.goal == (10, 1)
    assert robots[1].task.goal == (11, 1)
    records = logs(ex)
    assert any(
        r["event"] == "plan" and old_plan[0] in [tuple(p) for p in r["plans"].get("R1", [])]
        for r in records
    )
    repoint_records = [r for r in records if r["event"] == "handover_repoint"]
    assert repoint_records and repoint_records[0]["zone"] == "NEW"
    ex.run(60)
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R2"
    assert robots[1].pos == (15, 1)
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_compare_options_is_non_destructive(tmp_path):
    _, robots, ex = build(tmp_path, width=16)
    ex.register_handover_zone("OLD", (4, 1), (5, 1))
    ex.register_handover_zone("NEW", (10, 1), (11, 1))
    ex.request_handover("H", "C1", "R1", "R2", "OLD", "L1", "L2", "D1", (15, 1))
    before = dict(ex.coordinator.reservations.vertex)
    entries = ex.compare_handovers("H")
    after = ex.coordinator.reservations.vertex
    assert before == after
    assert robots[0].task.goal == (4, 1)
    assert {e["zone"] for e in entries} == {"OLD", "NEW"}


def test_cancel_before_start_keeps_owner_and_no_revival(tmp_path):
    _, robots, ex = build(tmp_path)
    ex.register_handover_zone("Z", (5, 1), (6, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    ex.cancel_handover("H")
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R1"
    assert robots[0].task is None and robots[1].task is None
    for _ in range(5):
        ex.step()
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R1"
    assert ex.coordinator.get_commitment("H").state.value == "cancelled"
    report = replay(ex.log_path)
    assert not report.ownership_violations


def test_failure_after_start_terminates_legs_without_resurrecting(tmp_path):
    _, robots, ex = build(tmp_path)
    ex.register_handover_zone("Z", (3, 1), (4, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    for _ in range(2):
        ex.step()
    started = ex.coordinator.get_commitment("H")
    assert started.state.value in ("started", "in_progress")
    from mrc.planner import INFEASIBLE, PlanResult
    ex.coordinator._terminate(
        robots[1], PlanResult(INFEASIBLE, blockers={"battery"}), ex.tick, "故障"
    )
    ex.step()
    commitment = ex.coordinator.get_commitment("H")
    assert commitment.state.value == "failed"
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R1"
    assert robots[0].status == RobotStatus.FAILED
    assert robots[1].status == RobotStatus.FAILED
    for _ in range(5):
        ex.step()
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R1"
    report = replay(ex.log_path)
    assert not report.ownership_violations
    assert any("handover_failed" in item for item in report.terminated)


def test_emergency_insertion_policy_marks_started_commitments_irrevocable(tmp_path):
    _, _, ex = build(tmp_path)
    ex.register_handover_zone("Z", (3, 1), (4, 1))
    ex.register_handover_zone("ALT", (8, 1), (9, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    policy_before = ex.handover_policy("H")
    assert policy_before["adjustable"] is True
    from mrc.robot import Task
    ex.step()
    policy_after = ex.handover_policy("H")
    assert policy_after["adjustable"] is False
    assert policy_after["revocable"] is False
    with pytest.raises(CoordinationError):
        ex.repoint_handover("H", "ALT")


def test_replay_flags_transfer_before_both_parties(tmp_path):
    _, _, ex = build(tmp_path)
    ex.register_handover_zone("Z", (5, 1), (6, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    ex.step()
    ex._log("handover_party_arrived", handover="H", robot="R1", at=[5, 1], cargo="C1", owner="R2", state="in_progress")
    report = replay(ex.log_path)
    assert any("双方未到齐" in item for item in report.ownership_violations)


def test_save_before_arrival_or_after_completion_does_not_recarry_cargo(tmp_path):
    _, robots, ex = build(tmp_path)
    ex.register_handover_zone("Z", (3, 1), (4, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    ex.cancel_handover("H")
    from mrc.robot import Task
    with pytest.raises(CoordinationError):
        ex.request_handover("H2", "C1", "R2", "R1", "Z", "X1", "X2", "X3", (0, 1))
    ex.run(10)
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R1"


def test_waiting_party_cannot_be_overridden_by_plain_task(tmp_path):
    _, robots, ex = build(tmp_path)
    ex.register_handover_zone("Z", (3, 1), (4, 1))
    ex.request_handover("H", "C1", "R1", "R2", "Z", "L1", "L2", "D1", (11, 1))
    ex.pause("R2")
    ex.run(8)  # R1 先到交接槽等待
    assert "R1" in ex.coordinator.handover_waiting
    from mrc.robot import Task
    with pytest.raises(CoordinationError):
        ex.assign({"R1": Task("WILD", (11, 1))})
    assert robots[0].task.task_id == "L1"
    assert ex.coordinator.cargo_registry.owner_of("C1") == "R1"
