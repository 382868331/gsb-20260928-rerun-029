"""接力运输与临时托管测试:归属唯一、容量与取放限制、故障重排、快照恢复."""

import pytest

from mrc.coordinator import Coordinator
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.relay import RELAY_BLOCKED, RELAY_DONE, slot_owner_id
from mrc.replay import replay
from mrc.robot import Robot, RobotStatus, Task

SLOT_A = (8, 0)
SLOT_B = (9, 0)
ACCESS_A = (8, 1)
ACCESS_B = (9, 1)


def make(tmp_path, robots=None):
    gmap = GridMap.corridor_map(length=6, width=3)
    robots = robots or [Robot("R1", (0, 1)), Robot("R2", (9, 1)), Robot("R3", (0, 2))]
    ex = Executor(Coordinator(gmap), robots, tmp_path / "run.log")
    ex.add_staging_area("S1", {SLOT_A: ACCESS_A, SLOT_B: ACCESS_B})
    return gmap, robots, ex


def step_until(cond, ex, limit=300):
    for _ in range(limit):
        if cond():
            return True
        ex.step()
    return cond()


def test_relay_with_staging_keeps_unique_ownership(tmp_path):
    """机器人接力+临时托管:每一刻货物只属于一台机器人或一个托管位."""
    _, robots, ex = make(tmp_path)
    ex.assign_relay("RL1", "C1", [
        {"kind": "carry", "carrier": "R1", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
        {"kind": "carry", "carrier": "R2", "to": (9, 2)},
    ])
    assert ex.cargo["C1"] == "R1"
    owners = []
    parked = False
    for _ in range(300):
        ex.step()
        owners.append(ex.cargo["C1"])
        if ex.relays["RL1"].state == RELAY_DONE:
            break
        if not parked and owners[-1] == slot_owner_id("S1", SLOT_A):
            ex.assign({"R1": Task("PARK", (0, 1))})  # 放下后让出取放站位
            parked = True
    assert slot_owner_id("S1", SLOT_A) in owners         # 曾在托管位
    assert owners[-1] == "R2"                             # 最终归属接手机器人
    assert all(isinstance(o, str) and o for o in owners)  # 每刻归属唯一且明确
    assert ex.relays["RL1"].state == RELAY_DONE
    assert robots[1].pos == (9, 2)
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_cargo_cannot_be_split_by_two_relays(tmp_path):
    """同一件物体不能同时有两个进行中的接力计划."""
    _, _, ex = make(tmp_path)
    ex.assign_relay("RL1", "C1", [
        {"kind": "carry", "carrier": "R1", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
        {"kind": "carry", "carrier": "R2", "to": (9, 2)},
    ])
    with pytest.raises(ValueError):
        ex.assign_relay("RL2", "C1", [
            {"kind": "carry", "carrier": "R3", "to": (0, 0)},
        ])


def test_two_cargos_contend_for_one_slot(tmp_path):
    """两件货物争用一个托管位:先到先放,未被接走前第二件只能等待."""
    _, robots, ex = make(tmp_path)
    ex.assign_relay("RL1", "C1", [
        {"kind": "carry", "carrier": "R2", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
        {"kind": "carry", "carrier": "R3", "to": (0, 0)},
    ])
    ex.assign_relay("RL2", "C2", [
        {"kind": "carry", "carrier": "R1", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
        {"kind": "carry", "carrier": "R2", "to": (9, 2)},
    ])
    assert step_until(lambda: ex.cargo["C1"] == slot_owner_id("S1", SLOT_A), ex)
    ex.assign({"R2": Task("PARK", (9, 1))})  # 放置者让出站位
    # R1 到达时托管位仍被 C1 占用,只能在一旁等待的占位由调度协调
    assert step_until(lambda: robots[0].pos == ACCESS_A and ex.cargo["C2"] == "R1", ex)
    for _ in range(3):
        ex.step()
        assert ex.cargo["C2"] == "R1"  # 未接走前第二件不能放入
        assert ex.cargo["C1"] == slot_owner_id("S1", SLOT_A)
    ex.assign({"R1": Task("AWAY", (9, 2))})  # 让出站位,R3 来取 C1
    assert step_until(lambda: ex.cargo["C1"] == "R3", ex)
    ex.assign({"R1": Task("BACK", ACCESS_A)})  # 托管位空出,R1 回来放 C2
    assert step_until(lambda: ex.cargo["C2"] == slot_owner_id("S1", SLOT_A), ex)
    ex.assign({"R1": Task("PARK2", (8, 2))})
    ex.run()
    assert ex.cargo["C1"] == "R3" and ex.cargo["C2"] == "R2"
    text = ex.log_path.read_text()
    assert '"stage_wait_slot"' in text  # 记录了争用等待
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_pick_and_place_same_tick_are_serialized(tmp_path):
    """取物与新放入同刻发生:受取放占用限制,同刻只执行一个,另一个顺延."""
    robots = [Robot("R1", ACCESS_B), Robot("R2", (9, 2)), Robot("R3", ACCESS_A)]
    _, _, ex = make(tmp_path, robots=robots)
    ex.assign_relay("RL0", "C1", [
        {"kind": "carry", "carrier": "R3", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
    ])
    assert ex.cargo["C1"] == slot_owner_id("S1", SLOT_A)  # 立即放置,用掉本刻取放名额
    ex.assign_relay("RL1", "C1", [
        {"kind": "carry", "carrier": "R3", "to": (0, 0)},
    ])
    ex.assign_relay("RL2", "C2", [
        {"kind": "carry", "carrier": "R1", "to": ACCESS_B},
        {"kind": "stage", "area": "S1", "slot": SLOT_B},
        {"kind": "carry", "carrier": "R2", "to": (9, 2)},
    ])
    assert ex.cargo["C1"] == slot_owner_id("S1", SLOT_A)  # 取出顺延到下一刻
    assert ex.cargo["C2"] == "R1"
    ex.step()
    assert ex.cargo["C1"] == "R3"                          # 第二刻取出
    assert ex.cargo["C2"] == "R1"                          # 放入仍顺延
    ex.step()
    assert ex.cargo["C2"] == slot_owner_id("S1", SLOT_B)  # 第三刻放入
    text = ex.log_path.read_text()
    assert '"stage_op_deferred"' in text
    ex.assign({"R1": Task("PARK", (8, 2))})
    ex.run()
    assert ex.cargo["C2"] == "R2"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_receiver_failure_keeps_cargo_staged_and_replan_rest(tmp_path):
    """接收者失去资格:货物留在托管位,只重排未执行段,旧接收者不得取货."""
    _, robots, ex = make(tmp_path)
    ex.assign_relay("RL1", "C1", [
        {"kind": "carry", "carrier": "R1", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
        {"kind": "carry", "carrier": "R2", "to": (9, 2)},
    ])
    assert step_until(lambda: ex.cargo["C1"] == slot_owner_id("S1", SLOT_A), ex)
    ex.assign({"R1": Task("PARK", (8, 2))})
    # 接收者 R2 被紧急任务占去并判定不可行 -> 失去接收资格
    ex.assign({"R2": Task("BLOCK", (0, 2), deadline=ex.tick + 1)})
    assert robots[1].status == RobotStatus.FAILED
    ex.step()
    relay = ex.relays["RL1"]
    assert relay.state == RELAY_BLOCKED
    assert ex.cargo["C1"] == slot_owner_id("S1", SLOT_A)  # 归属不动
    done_before = [leg.leg_id for leg in relay.legs if leg.state == "done"]
    ex.replan_relay("RL1", [{"kind": "carry", "carrier": "R3", "to": (0, 0)}])
    assert [leg.leg_id for leg in relay.legs if leg.state == "done"] == done_before
    ex.run()
    assert ex.cargo["C1"] == "R3"          # 交给新接收者
    assert robots[1].status == RobotStatus.FAILED
    report = replay(ex.log_path)
    assert report.ok, report.summary()
    assert not report.staging_violations


def test_emergency_insertion_and_relay_commitments(tmp_path):
    """紧急任务插入:已执行的段与托管占用锁定,未执行的段可调整."""
    _, _, ex = make(tmp_path)
    ex.assign_relay("RL1", "C1", [
        {"kind": "carry", "carrier": "R1", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
        {"kind": "carry", "carrier": "R2", "to": (9, 2)},
    ])
    assert step_until(lambda: ex.cargo["C1"] == slot_owner_id("S1", SLOT_A), ex)
    groups = ex.relay_commitments()
    assert groups["locked"] == ["RL1:L0", "RL1:L1"]   # 已完成的承运与放置
    assert groups["adjustable"] == ["RL1:L2"]          # 未执行的取出段
    ex.assign({"R1": Task("EMG", (0, 0), priority=100, deadline=ex.tick + 30)})
    ex.run()
    assert ex.cargo["C1"] == "R2"
    report = replay(ex.log_path)
    assert report.ok, report.summary()


def test_snapshot_while_staged_resume_without_replace_or_early_release(tmp_path):
    """放下未领取时保存:恢复后不重复放置、不提前释放、不交给失去资格的旧接收者."""
    _, robots, ex = make(tmp_path)
    ex.assign_relay("RL1", "C1", [
        {"kind": "carry", "carrier": "R1", "to": ACCESS_A},
        {"kind": "stage", "area": "S1", "slot": SLOT_A},
        {"kind": "carry", "carrier": "R2", "to": (9, 2)},
    ])
    assert step_until(lambda: ex.cargo["C1"] == slot_owner_id("S1", SLOT_A), ex)
    snap = tmp_path / "staged.json"
    ex.save_snapshot(snap)  # 已放下、下一台尚未领取

    robots2 = [Robot("R1", ACCESS_A), Robot("R2", (9, 1)), Robot("R3", (0, 2))]
    robots2[1].status = RobotStatus.FAILED  # 旧接收者已失去资格
    ex2 = Executor(Coordinator(GridMap.corridor_map(length=6, width=3)),
                   robots2, tmp_path / "run2.log")
    ex2.add_staging_area("S1", {SLOT_A: ACCESS_A, SLOT_B: ACCESS_B})
    ex2.load_snapshot(snap)
    assert ex2.cargo["C1"] == slot_owner_id("S1", SLOT_A)
    holds = ex2.coordinator.reservations.holds
    assert holds.get(SLOT_A) == slot_owner_id("S1", SLOT_A)  # 位置不提前释放
    for _ in range(3):
        ex2.step()
        assert ex2.cargo["C1"] == slot_owner_id("S1", SLOT_A)  # 不交给旧接收者
    text = ex2.log_path.read_text()
    assert '"stage_place"' not in text  # 不重复放置
    assert ex2.relays["RL1"].state == RELAY_BLOCKED
    ex2.assign({"R1": Task("PARK", (8, 2))})
    ex2.replan_relay("RL1", [{"kind": "carry", "carrier": "R3", "to": (0, 0)}])
    ex2.run()
    assert ex2.cargo["C1"] == "R3"
    assert ex2.coordinator.reservations.holds.get(SLOT_A) is None  # 取走后才释放
    report = replay(ex2.log_path)
    assert report.ok, report.summary()


def test_compare_three_arrangements(tmp_path):
    """比较直接等待、换机器人接力、临时托管:新增等待、冲突与未兑现交接."""
    _, _, ex = make(tmp_path)
    ex.cargo["C1"] = "R1"
    ex.assign_handoff("H9", "C9", "R3", "R1", ((0, 2), (0, 1)))
    options = ex.compare_relay(
        "C1", (9, 2), reliever="R3", area_id="S1", slot=SLOT_A,
        handoff_zone=((8, 1), (8, 2)),
    )
    assert [o["plan"] for o in options] == ["direct_wait", "robot_relay", "staging"]
    direct, relay_opt, staging = options
    assert direct["feasible"] and direct["eta"] is not None
    assert staging["feasible"] and staging["added_wait"] > 0  # 托管带来新增等待
    handoffs = [h["handoff"] for h in relay_opt["unfulfilled_handoffs"]]
    assert "H9" in handoffs  # 占用 R3 导致 H9 无法按原计划兑现
    # 托管位被占用后,托管方案不可行并给出占用者
    ex.staging["S1"].occupancy[SLOT_A] = "CX"
    options2 = ex.compare_relay(
        "C1", (9, 2), reliever="R3", area_id="S1", slot=SLOT_A,
        handoff_zone=((8, 1), (8, 2)),
    )
    assert options2[2]["feasible"] is False
    assert slot_owner_id("S1", SLOT_A) in options2[2]["conflicts"]
