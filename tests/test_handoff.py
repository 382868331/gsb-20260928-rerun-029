"""交接测试:双方到位转交、失败保留归属、方案比较、承诺调整、快照."""

import json

from mrc.coordinator import Coordinator
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.handoff import BLOCKED, COMPLETED, PLANNED
from mrc.replay import replay
from mrc.robot import Robot, RobotStatus, Task


def make(tmp_path, n_robots=2):
    gmap = GridMap.corridor_map(length=6, width=3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (9, 1)), Robot("R3", (9, 0))][:n_robots]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    return gmap, robots, ex


def test_handoff_transfers_ownership_only_when_both_present(tmp_path):
    """双方到齐前货物只记在一方,到齐并完成占用后一次性转交."""
    _, robots, ex = make(tmp_path)
    ex.assign_handoff("H1", "C1", "R1", "R2", ((8, 1), (9, 1)))
    for _ in range(3):
        ex.step()
        assert ex.cargo["C1"] == "R1"  # 交出方未到,归属不变
    assert robots[1].pos == (9, 1)     # 接收方已在交接区等待
    assert ex.handoffs["H1"].state == PLANNED
    ex.run()
    ho = ex.handoffs["H1"]
    assert ho.state == COMPLETED
    assert ex.cargo["C1"] == "R2"
    assert ho.started_tick is not None and ho.completed_tick > ho.started_tick
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_handoff_failure_keeps_ownership_and_shows_options(tmp_path):
    """接收方故障终止:归属保留在交出方,给出等待/撤离/换点条件."""
    _, robots, ex = make(tmp_path)
    ex.assign_handoff(
        "H1", "C1", "R1", "R2", ((8, 1), (9, 1)),
        alternatives=[((8, 0), (9, 0))],
        deadline=ex.tick + 2,  # R1 无法按时到达 -> 终止
    )
    assert robots[0].status == RobotStatus.FAILED
    ex.step()
    ho = ex.handoffs["H1"]
    assert ho.state == BLOCKED
    assert ex.cargo["C1"] == "R1"  # 归属保留
    options = ex.handoff_options("H1")
    assert options["owner"] == "R1"
    assert options["wait"]["viable"] is False       # 对方已故障,等待无意义
    assert options["withdraw"]["viable"] is True    # 交接未开始可撤离
    assert options["relocate"][0]["viable"] is True
    text = ex.log_path.read_text()
    assert '"handoff_blocked"' in text and '"options"' in text
    assert ex.cancel_handoff("H1") is True
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_compare_and_switch_handoff_does_not_rewrite_past(tmp_path):
    """比较原交接与替代方案只读评估;换点后已执行路径不被重写."""
    _, robots, ex = make(tmp_path)
    ex.assign_handoff(
        "H1", "C1", "R1", "R2", ((8, 1), (9, 1)),
        alternatives=[((8, 0), (9, 0))],
    )
    for _ in range(2):
        ex.step()
    paths_before = {r.robot_id: list(r.path) for r in robots}
    plans = ex.compare_handoff("H1")
    assert [p["plan"] for p in plans] == ["original", "alternative_0"]
    assert all(p["feasible"] for p in plans)
    assert plans[0]["eta"]["R2"] == ex.tick  # R2 已在交接区等待
    assert plans[0]["eta"]["R1"] > ex.tick
    assert {r.robot_id: list(r.path) for r in robots} == paths_before  # 比较不改写
    assert ex.switch_handoff("H1", ((8, 0), (9, 0))) is True
    assert ex.handoffs["H1"].zone == ((8, 0), (9, 0))
    ex.run()
    assert ex.handoffs["H1"].state == COMPLETED
    assert ex.cargo["C1"] == "R2"
    assert robots[0].pos == (8, 0) and robots[1].pos == (9, 0)
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_emergency_insertion_distinguishes_adjustable_commitments(tmp_path):
    """紧急任务插入:未开始的交接可调整,已开始/已完成的不能撤回."""
    _, robots, ex = make(tmp_path, n_robots=3)
    ex.assign_handoff("H1", "C1", "R1", "R2", ((8, 1), (9, 1)))
    assert ex.commitments() == {"adjustable": ["H1"], "locked": []}
    ex.run()
    assert ex.handoffs["H1"].state == COMPLETED
    ex.assign_handoff("H2", "C2", "R2", "R1", ((9, 2), (8, 2)))
    groups = ex.commitments()
    assert groups["adjustable"] == ["H2"] and groups["locked"] == ["H1"]
    assert ex.switch_handoff("H1", ((8, 0), (9, 0))) is False  # 已完成不可撤回
    ex.assign({"R3": Task("EMG", (0, 1), priority=100, deadline=ex.tick + 30)})
    text = ex.log_path.read_text()
    assert '"handoff_commitments"' in text
    assert '"adjustable": ["H2"]' in text and '"locked": ["H1"]' in text
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_snapshot_before_and_after_handoff_never_duplicates(tmp_path):
    """双方到齐前或交接完成后保存:恢复不重复搬运、不复活已终止任务."""
    _, robots, ex = make(tmp_path)
    ex.assign_handoff("H1", "C1", "R1", "R2", ((8, 1), (9, 1)))
    ex.step()
    early = tmp_path / "early.json"
    ex.save_snapshot(early)  # 双方未到齐时保存
    ex.run()
    assert ex.handoffs["H1"].state == COMPLETED
    late = tmp_path / "late.json"
    ex.save_snapshot(late)   # 交接完成后保存

    # 新执行器先载入完成后的快照,再载入到齐前的旧快照:已完成交接不复活
    ex2 = Executor(Coordinator(GridMap.corridor_map(length=6, width=3)),
                   [Robot("R1", (8, 1)), Robot("R2", (9, 1))], tmp_path / "run2.log")
    ex2.load_snapshot(late)
    ex2.load_snapshot(early)
    assert ex2.cargo["C1"] == "R2"                       # 归属不被旧快照回滚
    assert ex2.handoffs["H1"].state == COMPLETED         # 不重复搬运
    ex2.run()
    report = replay(ex2.log_path)
    assert not report.handoff_violations and not report.ownership_violations


def test_snapshot_restores_terminal_decisions_without_resurrect(tmp_path):
    """快照恢复已终止任务判定,后续重算不复活."""
    gmap = GridMap(6, 1)
    robots = [Robot("R1", (5, 0)), Robot("R2", (0, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("DONE", (5, 0))})
    ex.step()
    ex.assign({"R2": Task("BLOCKED", (5, 0))})
    assert robots[1].status == RobotStatus.FAILED
    snap = tmp_path / "snap.json"
    ex.save_snapshot(snap)

    robots2 = [Robot("R1", (5, 0)), Robot("R2", (0, 0))]
    robots2[0].task = robots[0].task
    robots2[1].task = robots[1].task
    ex2 = Executor(Coordinator(gmap), robots2, tmp_path / "run2.log")
    ex2.load_snapshot(snap)
    ex2.coordinator.plan_all(robots2, ex2.tick, strict=False)
    assert robots2[1].status == RobotStatus.FAILED
    assert robots2[1].task.terminal_reason == "infeasible"
