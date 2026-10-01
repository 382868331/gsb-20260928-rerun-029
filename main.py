"""演示:狭窄通道会车 + 暂停/恢复 + 第二轮任务沿用同一执行记录."""

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

    report = replay(LOG)
    print(report.summary())

    # 第三轮:R1 与 R3 在开阔区两侧槽位交接同一件运输物,随后 R3 送货
    executor.register_cargo("C1", owner="R1")
    executor.register_handover_zone("HO", carrier_slot=(10, 1), receiver_slot=(10, 2))
    executor.request_handover(
        "H1", cargo_id="C1", carrier="R1", receiver="R3", zone_id="HO",
        carrier_task_id="HO-CARRY", receiver_task_id="HO-MEET",
        delivery_task_id="HO-DELIVER", delivery_goal=(0, 1),
    )
    executor.run()
    print(replay(LOG).summary())


if __name__ == "__main__":
    main()
