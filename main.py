# -*- coding: utf-8 -*-
"""演示:会车 + 暂停/恢复 + 封路退出/重开 + 紧急任务连续插入。"""

from mrc.coordinator import Coordinator
from mrc.executor import Executor
from mrc.gridmap import GridMap
from mrc.replay import replay
from mrc.robot import Robot, Task

LOG = 'run.log'


def main() -> None:
    gmap = GridMap.corridor_map(length=8, width=3)
    mid = 1
    robots = [
        Robot('R1', (0, mid)),
        Robot('R2', (11, mid)),
        Robot('R3', (0, 0)),
        Robot('R4', (11, 2)),
    ]
    print(gmap.render({r.pos: r.robot_id for r in robots}))
    print()

    executor = Executor(Coordinator(gmap), robots, LOG)

    executor.assign({
        'R1': Task('T1', (11, mid)),
        'R2': Task('T2', (0, mid)),
    })
    for _ in range(4):
        executor.step()
    executor.pause('R2')
    for _ in range(3):
        executor.step()
    executor.resume('R2')

    # 封路:封闭通道中段;R1 已在途,若在区内则立即获得退出安排
    for _ in range(2):
        executor.step()
    segment = {(6, mid), (7, mid)}
    print('封闭:', sorted(segment))
    print('封路处理:', executor.close_segment(segment))
    for _ in range(6):
        executor.step()
    print('重开,重排摘要:', executor.reopen_segment(segment))

    # 连续插入两台带截止时间的紧急任务(不得抢占在途预约)
    executor.assign({'R3': Task('E1', (0, 2), deadline=executor.tick + 8, priority=10)})
    for _ in range(3):
        executor.step()
    executor.assign({'R4': Task('E2', (11, 0), deadline=executor.tick + 10, priority=10)})
    executor.run()

    report = replay(LOG)
    print(report.summary())


if __name__ == '__main__':
    main()
