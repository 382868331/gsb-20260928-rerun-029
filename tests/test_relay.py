"""接力运输与托管区测试:归属唯一、容量与取放占用、方案比较、故障重排、快照续跑."""

import json

from mrc.coordinator import Coordinator
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.relay import BLOCKED as RELAY_BLOCKED, COMPLETED as RELAY_COMPLETED, slot_owner
from mrc.replay import replay
from mrc.robot import Robot, RobotStatus, Task


def make(tmp_path, n_robots=3, length=6):
    gmap = GridMap.corridor_map(length=length, width=3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (9, 1)), Robot("R3", (9, 0))][:n_robots]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    return gmap, robots, ex


def add_area(ex, capacity=1):
    ex.add_staging_area("S1", [((0, 0), (0, 1)), ((1, 0), (1, 1))], capacity=capacity)


def test_relay_via_staging_unique_ownership(tmp_path):
    """接力每一刻归属唯一:机器人 -> 托管位 -> 机器人,不会被拆成多份."""
    _, _, ex = make(tmp_path)
    add_area(ex)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("stage", "S1")], final_goal=(9, 1))
    ex.step()
    ex.step()
    owner = ex.cargo["C1"]
    assert owner == slot_owner("S1#0")          # 已放下,归属托管位
    assert ex.staging["S1"].slots[0].occupied
    slot_cell = ex.staging["S1"].slots[0].cell
    assert ex.coordinator.external_holds.get(slot_cell) == slot_owner("S1#0")
    ex.run()
    relay = ex.relays["RL1"]
    assert relay.state == RELAY_COMPLETED
    assert ex.cargo["C1"] == "R2"               # 最终归属唯一
    assert not ex.staging["S1"].slots[0].occupied
    assert slot_cell not in ex.coordinator.external_holds
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_relay_rejects_splitting_cargo(tmp_path):
    """同一件物体不能同时进入两个接力计划."""
    _, _, ex = make(tmp_path)
    add_area(ex)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("stage", "S1")], final_goal=(9, 1))
    try:
        ex.assign_relay("RL2", "C1", ["R3"], [], final_goal=(9, 2))
    except ValueError:
        pass
    else:
        raise AssertionError("应拒绝拆分货物")


def test_staging_capacity_contention_blocks_and_recovers(tmp_path):
    """多件物体争用一个托管位:容量满时后到的等待,空位释放后自动继续."""
    _, _, ex = make(tmp_path)
    add_area(ex, capacity=1)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("stage", "S1")], final_goal=(9, 1))
    ex.step()
    ex.step()
    assert ex.cargo["C1"] == slot_owner("S1#0")
    ex.assign_relay("RL2", "C2", ["R3", "R2"], [("stage", "S1")], final_goal=(9, 2))
    leg = ex.relays["RL2"].legs[0]
    assert leg.state == RELAY_BLOCKED and leg.note == "blocked:staging_full"
    assert ex.cargo["C2"] == "R3"               # 预约不等于转移,归属仍在承运方
    text = ex.log_path.read_text()
    assert '"staging_full"' in text
    ex.run()
    assert ex.relays["RL1"].state == RELAY_COMPLETED
    assert ex.relays["RL2"].state == RELAY_COMPLETED
    assert ex.cargo["C1"] == "R2" and ex.cargo["C2"] == "R2"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_pickup_and_place_same_tick(tmp_path):
    """同一托管区同一 tick:先取走再放入,容量不被同刻两件撑爆."""
    gmap = GridMap(7, 3)
    robots = [Robot("R1", (3, 1)), Robot("R2", (3, 3 - 1)), Robot("R3", (4, 3 - 1))]
    robots[1].pos = (3, 2) if False else robots[1].pos
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.add_staging_area("S1", [((3, 0), (3, 1)), ((4, 0), (4, 1))], capacity=2)
    robots[1].pos = (3, 2)
    robots[2].pos = (4, 2)
    # R1 已在取放站位,第一拍即放入;R2/R3 距各自站位 1 格,同刻取/放
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("stage", "S1")], final_goal=(6, 2))
    ex.step()  # R1 放入 S1#0 并撤退,R2 前往取货
    assert ex.cargo["C1"] == slot_owner("S1#0")
    ex.assign_relay("RL2", "C2", ["R3", "R1"], [("stage", "S1")], final_goal=(0, 2))
    ex.run()
    ticks = {}
    for line in ex.log_path.read_text().splitlines():
        rec = json.loads(line)
        if rec["event"] in ("stage_picked", "stage_placed"):
            ticks.setdefault(rec["event"], []).append((rec["tick"], rec["cargo"]))
    picked = [t for t, c in ticks["stage_picked"] if c == "C1"]
    placed = [t for t, c in ticks["stage_placed"] if c == "C2"]
    assert picked and placed and picked[0] == placed[0]  # 取物与新放入同刻发生
    assert ex.cargo["C1"] == "R2" and ex.cargo["C2"] == "R1"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_receiver_leaves_then_replan_keeps_completed_parts(tmp_path):
    """接收者离开:交接阻塞,只重排未执行部分,已完成段与归属保留."""
    _, robots, ex = make(tmp_path)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("handoff", ((8, 1), (9, 1)))], final_goal=(9, 1))
    while ex.handoffs["RL1:T0"].state != "both_present":
        ex.step()
    ex.assign({"R2": Task("AWAY", (8, 2))})     # 接收者离开交接区
    ex.step()
    relay = ex.relays["RL1"]
    assert relay.state == RELAY_BLOCKED
    assert ex.cargo["C1"] == "R1"               # 归属保留在交出方
    ok = ex.replan_relay("RL1", ["R1", "R3"], [("handoff", ((8, 1), (9, 0)))], final_goal=(9, 2))
    assert ok
    ex.run()
    assert relay.state == RELAY_COMPLETED
    assert ex.cargo["C1"] == "R3"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_urgent_task_insertion_blocks_carrier_then_recovers(tmp_path):
    """紧急任务插入:承运方被占用时段阻塞,紧急任务完成后自动继续."""
    _, _, ex = make(tmp_path)
    add_area(ex)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("stage", "S1")], final_goal=(9, 1))
    ex.assign({"R2": Task("EMG", (2, 1), priority=100)})
    ex.step()
    ex.step()
    leg = ex.relays["RL1"].legs[1]
    assert leg.state == RELAY_BLOCKED and leg.note == "blocked:carrier_busy"
    assert ex.cargo["C1"] == slot_owner("S1#0")  # 货物安全待在托管位
    ex.run()
    assert ex.relays["RL1"].state == RELAY_COMPLETED
    assert ex.cargo["C1"] == "R2"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_compare_relay_offers_wait_relay_and_staging(tmp_path):
    """比较直接等待、换机器人接力、临时托管三种安排,含冲突与后续交接核对."""
    _, _, ex = make(tmp_path)
    add_area(ex)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("handoff", ((8, 1), (9, 1)))], final_goal=(9, 1))
    for _ in range(2):
        ex.step()
    options = ex.compare_relay("RL1")
    kinds = [opt["option"] for opt in options]
    assert "direct_wait" in kinds and "robot_relay" in kinds and "staging" in kinds
    wait = options[kinds.index("direct_wait")]
    assert "added_wait" in wait and "corridor_conflicts" in wait
    relay_opts = [opt for opt in options if opt["option"] == "robot_relay"]
    assert {opt["candidate"] for opt in relay_opts} == {"R2", "R3"}
    # 单宽通道被在途目标锁占住,当面换机器人接力不可行(通道冲突)
    assert all(not opt["feasible"] for opt in relay_opts)
    staging = options[kinds.index("staging")]
    assert staging["area"] == "S1" and staging["feasible"]
    assert all("downstream_unfulfilled" in opt for opt in options)
    # 比较是只读的:路径与归属不变
    assert ex.cargo["C1"] == "R1"
    ex.run()
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_snapshot_between_place_and_pickup_resume(tmp_path):
    """放下后、领取前保存:续跑不重新放置、不提前释放位置、不交给失去资格的旧接收者."""
    _, robots, ex = make(tmp_path)
    add_area(ex)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("stage", "S1")], final_goal=(9, 1))
    while ex.cargo.get("C1") != slot_owner("S1#0"):
        ex.step()
    snap = tmp_path / "snap.json"
    ex.save_snapshot(snap)
    slot_cell = ex.staging["S1"].slots[0].cell

    # 新执行器:旧接收者 R2 已故障失去资格
    robots2 = [Robot("R1", robots[0].pos), Robot("R2", robots[1].pos), Robot("R3", robots[2].pos)]
    robots2[1].status = RobotStatus.FAILED
    ex2 = Executor(Coordinator(GridMap.corridor_map(length=6, width=3)), robots2, tmp_path / "run2.log")
    ex2.load_snapshot(snap)
    assert ex2.cargo["C1"] == slot_owner("S1#0")          # 归属仍是托管位
    assert ex2.coordinator.external_holds.get(slot_cell) == slot_owner("S1#0")  # 位置未提前释放
    assert robots2[1].task is None                        # 旧接收者没有领到任务
    leg = ex2.relays["RL1"].legs[1]
    assert leg.state == RELAY_BLOCKED                     # 不交给失去资格的旧接收者
    text = ex2.log_path.read_text()
    assert '"stage_placed"' not in text                  # 不重新放置
    assert ex2.replan_relay("RL1", ["R3"], [], final_goal=(9, 2))
    ex2.run()
    assert ex2.relays["RL1"].state == RELAY_COMPLETED
    assert ex2.cargo["C1"] == "R3"
    assert slot_cell not in ex2.coordinator.external_holds
    report = replay(ex2.log_path)
    assert report.ok, report.summary()


def test_cancel_leg_keeps_completed_parts_and_occupancy(tmp_path):
    """取消只影响未执行部分:已放入托管的货物与实际占用保留."""
    _, _, ex = make(tmp_path)
    add_area(ex)
    ex.assign_relay("RL1", "C1", ["R1", "R2"], [("stage", "S1")], final_goal=(9, 1))
    while ex.cargo.get("C1") != slot_owner("S1#0"):
        ex.step()
    assert ex.cancel_relay("RL1")
    relay = ex.relays["RL1"]
    assert relay.legs[0].state == RELAY_COMPLETED         # 已完成段保留
    assert ex.cargo["C1"] == slot_owner("S1#0")          # 托管中的货物不动
    assert ex.staging["S1"].slots[0].occupied             # 实际占用保留
    slot_cell = ex.staging["S1"].slots[0].cell
    assert ex.coordinator.external_holds.get(slot_cell) == slot_owner("S1#0")
    report = replay(ex.log_path)
    assert report.ok, report.summary()
