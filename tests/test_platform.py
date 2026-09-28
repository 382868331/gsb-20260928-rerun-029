"""平台基础测试."""

from mrc.coordinator import Coordinator
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
    for _ in range(3):
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
