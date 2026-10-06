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

    # 第四轮:接力运输 + 临时托管,R1 放到托管区,R3 稍后取出继续送
    executor.add_staging_area("S1", {(1, 2): (1, 1)})
    executor.assign_relay("RL1", "C2", [
        {"kind": "carry", "carrier": "R1", "to": (1, 1)},
        {"kind": "stage", "area": "S1", "slot": (1, 2)},
        {"kind": "carry", "carrier": "R3", "to": (11, 1)},
    ])
    for plan in executor.compare_relay(
        "C2", (11, 1), reliever="R3", area_id="S1", slot=(1, 2),
        handoff_zone=((1, 1), (0, 1)),
    ):
        print("接力方案:", plan)
    executor.save_snapshot("relay_snapshot.json")
    while executor.cargo.get("C2") != "staging:S1@1,2":
        executor.step()
    executor.assign({"R1": Task("T6", (0, 2))})  # 放下后让出取放站位
    executor.run()
    print("货物归属:", executor.cargo, " 接力承诺:", executor.relay_commitments())
    print()

    report = replay(LOG)
    print(report.summary())


if __name__ == "__main__":
    main()
