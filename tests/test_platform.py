"""平台基础测试."""

from mrc.coordinator import Coordinator, CoordinationError
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.planner import plan_path
from mrc.replay import replay
from mrc.reservation import ReservationTable
from mrc.robot import Robot, RobotStatus, Task


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
    assert robots[1].status == RobotStatus.FAILED
    assert robots[1].last_reason == "deadline_infeasible"
    assert "R1" in robots[1].last_blockers
    ex.coordinator.plan_all(robots, ex.tick, strict=True)
    assert robots[1].status == RobotStatus.FAILED
    assert robots[1].last_reason == "deadline_infeasible"


def test_permanent_goal_terminates_and_remains_infeasible(tmp_path):
    gmap = GridMap(6, 1)
    robots = [Robot("R1", (5, 0)), Robot("R2", (0, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("DONE", (5, 0))})
    ex.step()
    assert robots[0].status == RobotStatus.DONE
    ex.assign({"R2": Task("BLOCKED", (5, 0))})
    assert robots[1].status == RobotStatus.FAILED
    assert robots[1].task.terminal_reason == "infeasible"
    assert "R1" in robots[1].last_blockers
    ex.coordinator.plan_all(robots, ex.tick + 3, strict=False)
    assert robots[1].status == RobotStatus.FAILED
    assert robots[1].task.terminal_reason == "infeasible"
    report = replay(ex.log_path)
    assert not report.unfinished
    assert any("BLOCKED=infeasible" in item for item in report.terminated)

    reopened = Coordinator(gmap)
    reopened.load_terminal_decisions(ex.log_path)
    reopened.plan_all(robots, ex.tick + 5, strict=False)
    assert robots[1].status == RobotStatus.FAILED
    assert robots[1].task.terminal_reason == "infeasible"


def test_search_budget_status_is_distinct_from_temporary_block(tmp_path):
    gmap = GridMap(6, 3, {(x, y) for x in range(6) for y in (0, 2)})
    robots = [Robot("R1", (0, 1)), Robot("R2", (5, 1))]
    ex = Executor(Coordinator(gmap, horizon=80), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (4, 1))})
    for _ in range(2):
        ex.step()
    ex.assign({"R2": Task("T2", (0, 1))})
    assert robots[1].status == RobotStatus.WAITING
    assert robots[1].last_reason == "search_limit"
    assert "R1" in robots[1].last_blockers


def test_emergency_permanent_goal_and_closure_get_distinct_decisions(tmp_path):
    gmap = GridMap.corridor_map(length=8, width=3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (11, 1)), Robot("R3", (1, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.assign({"R1": Task("T1", (11, 1)), "R2": Task("PARK", (11, 1))})
    for _ in range(5):
        ex.step()
    ex.close_corridor([(x, 1) for x in range(3, 8)])
    ex.assign({"R3": Task("EMG", (11, 0), priority=100, deadline=20)})
    ex.run(20)
    text = ex.log_path.read_text()
    assert '"action": "exit"' in text
    assert '"action": "continue"' in text
    assert '"action": "terminate"' in text
    assert robots[1].status == RobotStatus.DONE
    report = replay(ex.log_path)
    assert not report.collisions and not report.edge_swaps and not report.closed_violations
