"""演示:狭窄通道会车 + 暂停/恢复 + 第二轮任务 + 双机器人交接."""

from mrc.coordinator import Coordinator
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.replay import replay
from mrc.robot import Robot, Task

LOG = "run.log"


def main() -> None:
    gmap = GridMap.corridor_map(length=8, width=3)
    mid = 1
    robots = [
        Robot("R1", (0, mid)),   # 左 -> 右
        Robot("R2", (1, mid)),   # 左 -> 右
        Robot("R3", (11, mid)),  # 右 -> 左,与 R1/R2 迎面
    ]
    print(gmap.render({r.pos: r.robot_id[0] + r.robot_id[1] for r in robots}))
    print()

    executor = Executor(Coordinator(gmap), robots, LOG)

    # 第一轮:穿过狭窄通道
    executor.assign({
        "R1": Task("T1", (11, mid)),
        "R2": Task("T2", (11, 0)),
        "R3": Task("T3", (0, mid)),
    })
    for _ in range(4):
        executor.step()
    executor.pause("R2")          # R2 暂停,占用不释放
    for _ in range(3):
        executor.step()
    executor.resume("R2")         # 从当前位置继续,不插队
    executor.run()

    # 第二轮:新任务沿用同一执行记录
    executor.assign({
        "R1": Task("T4", (0, 0)),
        "R3": Task("T5", (11, 2)),
    })
    executor.run()

    # 第三轮:R1 把货物 C1 在交接区转交给 R3,双方到齐后归属一次性转移
    executor.assign_handoff(
        "H1", "C1", giver="R1", receiver="R3",
        zone=((1, 0), (0, 0)),
        alternatives=[((1, 2), (0, 2))],
    )
    for plan in executor.compare_handoff("H1"):
        print("交接方案:", plan)
    executor.save_snapshot("handoff_snapshot.json")
    executor.run()
    print("货物归属:", executor.cargo, " 交接状态:", executor.handoffs["H1"].state)
    print("交接承诺:", executor.commitments())
    print()

    report = replay(LOG)
    print(report.summary())


if __name__ == "__main__":
    main()


def demo_relay() -> None:
    """接力运输:R1 经托管区 S1 把货物交给 R2,中途比较三种安排."""
    gmap = GridMap.corridor_map(length=6, width=3)
    robots = [Robot("R1", (0, 1)), Robot("R2", (9, 1)), Robot("R3", (9, 0))]
    executor = Executor(Coordinator(gmap), robots, LOG)
    executor.add_staging_area("S1", [((0, 0), (0, 1)), ((1, 0), (1, 1))], capacity=1)
    executor.assign_relay("RL1", "C9", ["R1", "R2"], [("stage", "S1")], final_goal=(9, 1))
    for plan in executor.compare_relay("RL1"):
        print("接力方案:", plan)
    executor.step()
    executor.step()
    executor.save_snapshot("relay_snapshot.json")  # 已放下、未领取时保存
    print("托管中归属:", executor.cargo["C9"])
    executor.run()
    print("接力完成归属:", executor.cargo["C9"], " 状态:", executor.relays["RL1"].state)
    report = replay(LOG)
    print(report.summary())


if __name__ == "__main__":
    demo_relay()
