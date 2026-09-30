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
    result = plan_path(gmap, res, (0, 1), (7, 1), 0, horizon=50)
    assert result.status == "infeasible"
    assert "永久预约" in result.blockers[0]


def test_robot_planned_cell_clamps():
    r = Robot("R", (0, 0))
    r.path = [(0, 0), (1, 0)]
    r.plan_start = 5
    assert r.planned_cell(3) == (0, 0)
    assert r.planned_cell(6) == (1, 0)
    assert r.planned_cell(99) == (1, 0)


# ---------- 第二轮:封路退出 / 紧急不抢占 / 截止与阻塞原因 / 重开保留预约 ----------

def test_closure_evacuation_and_reopen(tmp_path):
    gmap = GridMap.corridor_map(length=8, width=3)
    robots = [Robot('R1', (0, 1)), Robot('R2', (11, 1))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / 'close.log')
    ex.assign({'R1': Task('T1', (11, 1)), 'R2': Task('T2', (0, 1))})
    for _ in range(4):
        ex.step()
    r1_in = robots[0].pos
    assert r1_in in {(x, 1) for x in range(2, 10)}
    # 封闭 R1 所在区段:R1 必须立即获得退出安排,且不与 R2 碰撞
    cells = {tuple(r1_in)} | {tuple(robots[0].planned_cell(ex.tick + i)) for i in range(1, 3)}
    summary = ex.close_segment(cells)
    assert summary.get('R1') == 'evacuating'
    assert robots[0].status == RobotStatus.EVACUATING
    for _ in range(8):
        ex.step()
    assert robots[0].pos not in cells  # 已撤出封闭区段
    report = replay(ex.log_path)
    assert not report.collisions, report.summary()
    assert not report.violations, report.summary()
    # 重新开放后:R2 立即获得完整安排(预约连续未丢失),任务最终完成
    before = {rid: (r.pos, r.committed, r.status) for rid, r in ex.robots.items()}
    ex.reopen_segment(cells)
    assert ex.robots['R2'].committed is True
    assert ex.robots['R2'].pos == before['R2'][0]
    assert ex.robots['R2'].status == RobotStatus.MOVING
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_emergency_cannot_steal_committed_reservation(tmp_path):
    gmap = GridMap.corridor_map(length=6, width=3)
    robots = [Robot('R1', (0, 1)), Robot('R2', (9, 1)), Robot('R3', (0, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / 'em.log')
    ex.assign({'R1': Task('T1', (9, 1)), 'R2': Task('T2', (0, 1))})
    for _ in range(3):
        ex.step()
    committed_path_r1 = list(robots[0].path)
    committed_start = robots[0].plan_start
    # 高优先紧急任务晚到,截止很紧:不能抢走 R1 的在途预约
    urgent = Task('T9', (9, 1), deadline=ex.tick + 4, priority=10)
    summary = ex.assign({'R3': urgent})
    assert robots[0].path == committed_path_r1 and robots[0].plan_start == committed_start
    assert summary.get('R3') in ('infeasible', 'not_found')
    assert robots[2].status == RobotStatus.BLOCKED
    assert robots[2].block_reason  # 能说明被谁挡住
    for _ in range(6):
        ex.step()
    report = replay(ex.log_path)
    assert not report.collisions, report.summary()
    assert not report.edge_swaps, report.summary()
    assert not report.violations, report.summary()


def test_blocked_explains_occupants(tmp_path):
    gmap = GridMap.corridor_map(length=6, width=3)
    robots = [Robot('R1', (0, 1)), Robot('R2', (0, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / 'blk.log')
    ex.assign({'R1': Task('T1', (9, 1))})
    for _ in range(3):
        ex.step()
    ex.pause('R1')  # R1 占住通道格
    ex.assign({'R2': Task('T2', (9, 1))})
    r2 = robots[1]
    assert r2.status == RobotStatus.BLOCKED
    assert 'R1' in r2.block_reason  # 指出阻挡来源


def test_infeasible_vs_not_found_distinguished():
    from mrc.robot import RobotStatus as RS
    gmap = GridMap.corridor_map(length=6, width=3)
    res = ReservationTable()
    # 终点永久占用:静态无可行安排
    res.reserve_path('RX', [(9, 1)], 0)
    r1 = plan_path(gmap, res, (0, 1), (9, 1), 0, horizon=60)
    assert r1.status == 'infeasible'
    # 截止早于最早到达:不可行
    r2 = plan_path(gmap, ReservationTable(), (0, 1), (9, 1), 0, deadline=3)
    assert r2.status == 'infeasible'
    # 通道中段被短时预约:时间窗内找不到,但不是静态不可行
    res3 = ReservationTable()
    for t in range(0, 30):
        res3.vertex[((5, 1), t)] = 'RB'
    r3 = plan_path(gmap, res3, (0, 1), (9, 1), 0, horizon=20)
    assert r3.status == 'not_found'


def test_repeated_emergency_insertions_no_collision(tmp_path):
    gmap = GridMap.corridor_map(length=8, width=3)
    robots = [Robot('R1', (0, 1)), Robot('R2', (11, 1)),
             Robot('R3', (0, 0)), Robot('R4', (11, 2))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / 'ins.log')
    ex.assign({'R1': Task('T1', (11, 1)), 'R2': Task('T2', (0, 1))})
    for i in range(1, 6):
        ex.step()
        if i % 2 == 1:
            rid = 'R3' if i == 1 else 'R4'
            goal = (11, 1) if rid == 'R3' else (0, 1)
            ex.assign({rid: Task('TE' + str(i), goal, deadline=ex.tick + 6,
                                 priority=20)})
    ex.run()
    report = replay(ex.log_path)
    assert not report.collisions, report.summary()
    assert not report.edge_swaps, report.summary()
    assert not report.violations, report.summary()


def test_reopen_keeps_unaffected_committed_reservation_v2(tmp_path):
    # R3 在右侧开阔区内的在途预约不触及封闭格,重开后必须原样保留。
    gmap = GridMap.corridor_map(length=10, width=3)
    robots = [Robot('R1', (0, 1)), Robot('R2', (13, 1)), Robot('R3', (12, 0))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / 'keep2.log')
    ex.assign({'R1': Task('T1', (13, 1)), 'R2': Task('T2', (0, 1)),
               'R3': Task('T3', (12, 2))})
    ex.step()
    r3_path, r3_start = list(robots[2].path), robots[2].plan_start
    ex.close_segment({(7, 1), (8, 1)})
    ex.reopen_segment({(7, 1), (8, 1)})
    assert robots[2].path == r3_path and robots[2].plan_start == r3_start
    assert robots[2].committed is True
    ex.run()
    assert replay(ex.log_path).ok
