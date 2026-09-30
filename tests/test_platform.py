"""平台基础测试."""

import json

from mrc.coordinator import Coordinator, CoordinationError
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.planner import plan_path
from mrc.replay import replay
from mrc.reservation import ReservationTable
from mrc.robot import Robot, RobotStatus, Task, WaitReason


def make(tmp_path, n_robots=2):
    gmap = GridMap.corridor_map(length=6, width=3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (9, 1))][:n_robots]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    return gmap, robots, ex


def test_head_on_no_collision(tmp_path):
    """迎面冲突:双方向穿越单宽通道不得相撞."""
    _, _, ex = make(tmp_path)
    ex.assign({"R1": Task("T1", (9, 1)), "R2": Task("T2", (0, 1))})
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_pause_keeps_occupancy_and_resumes(tmp_path):
    """暂停不释放占用,恢复不插队."""
    gmap, robots, ex = make(tmp_path)
    ex.assign({"R1": Task("T1", (9, 1)), "R2": Task("T2", (0, 1))})
    ex.step()
    ex.pause("R1")
    paused_pos = robots[0].pos
    for _ in range(5):
        ex.step()
    assert robots[0].pos == paused_pos  # 暂停期间不动
    ex.resume("R1")
    assert robots[0].pos == paused_pos  # 恢复时刻仍在原地
    assert robots[0].plan_start == ex.tick  # 从当前时刻重排
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_second_round_reuses_log(tmp_path):
    """后续任务分配沿用同一执行记录."""
    _, _, ex = make(tmp_path)
    ex.assign({"R1": Task("T1", (9, 1)), "R2": Task("T2", (0, 1))})
    ex.run()
    ex.assign({"R1": Task("T3", (0, 0))})
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()
    text = ex.log_path.read_text()
    assert '"T3"' in text and '"T1"' in text  # 同一文件含两轮记录


def test_planner_waits_for_reserved_goal():
    """终点被永久预约时,规划器应失败而非硬闯."""
    gmap = GridMap.corridor_map(length=4, width=3)
    res = ReservationTable()
    res.reserve_path("RX", [(7, 1)], 0)
    assert plan_path(gmap, res, (0, 1), (7, 1), 0, horizon=50) is None


def test_robot_planned_cell_clamps():
    r = Robot("R", (0, 0))
    r.path = [(0, 0), (1, 0)]
    r.plan_start = 5
    assert r.planned_cell(3) == (0, 0)
    assert r.planned_cell(6) == (1, 0)
    assert r.planned_cell(99) == (1, 0)


def test_pause_blocks_lane_and_replays_safely(tmp_path):
    gmap = GridMap(10, 3, {(x, y) for x in range(10) for y in (0, 2)})
    robots = [Robot("R1", (3, 1)), Robot("R2", (0, 1))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (9, 1)), "R2": Task("T2", (8, 1))})
    for _ in range(1):
        ex.step()
    ex.pause("R1")
    ex.step()
    assert any("R1" in reason for reason in robots[1].last_blockers)
    ex.resume("R1")
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()
    assert "R1" in report.blocked_reasons.get("R2", [])


def test_later_emergency_task_cannot_steal_in_flight_reservation(tmp_path):
    gmap = GridMap(6, 2)
    robots = [Robot("R1", (0, 1)), Robot("R2", (5, 1)), Robot("R3", (0, 0))]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (4, 1), priority=1)})
    original_path = list(robots[0].path)
    for _ in range(2):
        ex.step()
    ex.assign({"R2": Task("EMG", (0, 1), priority=100, deadline=ex.tick + 2)})
    assert robots[0].path == original_path
    assert robots[0].pos == original_path[2]
    assert "R1" in robots[1].last_blockers
    ex.run()
    assert robots[0].status == RobotStatus.DONE
    assert robots[1].task.on_time is False
    report = replay(ex.log_path)
    assert not report.collisions and not report.edge_swaps
    assert any("EMG" in item for item in report.deadline_misses)


def test_corridor_closure_exit_and_reopen_preserves_valid_reservations(tmp_path):
    gmap = GridMap.corridor_map(length=8, width=3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (11, 1)), Robot("R3", (0, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({
        "R1": Task("T1", (11, 1)),
        "R2": Task("T2", (0, 1)),
        "R3": Task("T3", (1, 0)),
    })
    for _ in range(5):
        ex.step()
    third_path_before = list(robots[2].path)
    ex.close_corridor([(x, 1) for x in range(3, 8)])
    assert robots[0].path[0] == (5, 1)
    assert robots[0].path[-1] == (2, 1)
    assert robots[1].path[0] == (6, 1)
    assert robots[1].path[-1] == (8, 1)
    assert robots[2].path == third_path_before
    for _ in range(3):
        ex.step()
    assert robots[0].pos == (2, 1)
    assert robots[1].pos == (8, 1)
    assert robots[2].path == third_path_before
    ex.open_corridor([(x, 1) for x in range(3, 8)])
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_consecutive_emergency_insertions_keep_deadline_records(tmp_path):
    gmap = GridMap.corridor_map(length=6, width=3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (9, 1)), Robot("R3", (9, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (9, 1))})
    ex.step()
    ex.assign({"R2": Task("E1", (0, 1), priority=10, deadline=ex.tick + 25)})
    ex.run(20)
    ex.assign({"R3": Task("E2", (0, 0), priority=20, deadline=ex.tick + 25)})
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()
    import json
    done_tasks = [
        json.loads(line)["task"]
        for line in ex.log_path.read_text().splitlines()
        if json.loads(line).get("event") == "done"
    ]
    assert "E1" in done_tasks and "E2" in done_tasks


def test_infeasible_deadline_is_distinguished_from_search_pending(tmp_path):
    gmap = GridMap(6, 3, {(x, y) for x in range(6) for y in (0, 2)})
    robots = [Robot("R1", (0, 1)), Robot("R2", (5, 1))]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (4, 1))})
    for _ in range(2):
        ex.step()
    ex.assign({"R2": Task("T2", (0, 1), deadline=ex.tick + 2)})
    assert robots[1].status == RobotStatus.WAITING
    assert robots[1].last_blockers and robots[1].last_blockers != ["search_limit"]
    try:
        ex.coordinator.plan_all(robots, ex.tick, strict=True)
    except CoordinationError as exc:
        assert "尚未找到" in str(exc)
    else:
        raise AssertionError("deadline 内应报告尚未找到方案")


def test_permanently_occupied_goal_terminates_with_clear_reason(tmp_path):
    """目标被长期停放机器人占据:暂时受阻/无法按时/搜索预算三分类,过期即终止."""
    gmap = GridMap(6, 3, {(x, y) for x in range(6) for y in (0, 2)})
    robots = [Robot("R1", (0, 1)), Robot("R2", (5, 1))]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (4, 1))})  # R1 完成后永久停在 (4,1)
    ex.run()
    assert robots[0].status == RobotStatus.DONE
    ex.assign({"R2": Task("T2", (4, 1), deadline=ex.tick + 3)})
    assert robots[1].status in (RobotStatus.WAITING, RobotStatus.TERMINATED)
    ex.run()
    assert robots[1].status == RobotStatus.TERMINATED
    assert robots[1].end_reason in ("hard_blocked", "deadline_infeasible")
    assert robots[1].task.on_time is False
    report = replay(ex.log_path)
    assert report.terminated
    assert any("T2" in item for item in report.terminated)
    assert not report.collisions


def test_temporary_dock_does_not_preempt_active_reservation(tmp_path):
    """封闭通道撤离:允许临时停靠位,但不抢占正在执行的预约."""
    gmap = GridMap.corridor_map(length=8, width=3)
    robots = [Robot("R1", (4, 1)), Robot("R2", (8, 1))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({"R2": Task("T2", (0, 1))})
    ex.step()  # R2 向通道移动,预约占住前方格子
    r2_path = list(robots[1].path)
    ex.add_staging_cells([(3, 1), (5, 1)])
    ex.close_corridor([(x, 1) for x in range(3, 8)])
    # R1 撤离安排不得占用 R2 已预约的格子
    for cell in robots[0].path:
        owner = ex.coordinator.reservations.vertex.get((cell, ex.tick))
        assert owner in (None, "R1")
    decision = ex.coordinator.last_dispositions.get("R1")
    assert decision is not None and decision.decision in ("exit", "stage", "wait")
    ex.run()
    report = replay(ex.log_path)
    assert not report.collisions and not report.edge_swaps and not report.closed_violations


def test_emergency_closure_permanent_occupy_dispositions(tmp_path):
    """紧急任务、永久占位与新封路同时出现:每个任务都有继续/退出/终止原因."""
    gmap = GridMap.corridor_map(length=8, width=3)
    robots = [Robot("R1", (3, 1)), Robot("R2", (11, 1)), Robot("R3", (9, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (11, 1))})
    for _ in range(3):
        ex.step()
    # R3 永久停在 (9,0);R2 目标正好是永久占位;同时封闭通道并插入紧急任务
    ex.close_corridor([(x, 1) for x in range(4, 9)])
    ex.assign({
        "R2": Task("EMG", (9, 0), priority=100, deadline=ex.tick + 4),
    })
    ex.run()
    decisions = {}
    for line in ex.log_path.read_text().splitlines():
        rec = json.loads(line)
        if rec.get("event") == "disposition":
            decisions.setdefault(rec["robot"], []).append(
                (rec["decision"], rec["reason"]),
            )
    assert any(d[0] == "exit" or d[0] == "stage" or ("wait", d[1]) for d in decisions.get("R1", []))
    assert any(d[0] in ("terminate", "wait") for d in decisions.get("R2", []))
    report = replay(ex.log_path)
    assert not report.collisions and not report.closed_violations


def test_resume_does_not_reactivate_terminated_task(tmp_path):
    """重开调度记录后,已判不可行的任务不能被重新激活."""
    gmap = GridMap(6, 3, {(x, y) for x in range(6) for y in (0, 2)})
    robots = [Robot("R1", (0, 1)), Robot("R2", (5, 1))]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (4, 1))})
    ex.run()
    ex.assign({"R2": Task("T2", (4, 1), deadline=ex.tick + 2)})
    ex.run()
    assert robots[1].status == RobotStatus.TERMINATED
    log_path = ex.log_path

    gmap2 = GridMap(6, 3, {(x, y) for x in range(6) for y in (0, 2)})
    robots2 = [Robot("R1", (0, 1)), Robot("R2", (5, 1))]
    resumed = Executor.resume_from_log(gmap2, robots2, log_path)
    assert robots2[1].status == RobotStatus.TERMINATED
    assert "R2" in resumed.coordinator.terminated
    resumed.assign({"R2": Task("T2-again", (3, 1), deadline=resumed.tick + 5)})
    assert robots2[1].status == RobotStatus.TERMINATED
    report = replay(log_path)
    assert any("T2-again" in item for item in report.reactivations)
    assert report.ok, report.summary()
