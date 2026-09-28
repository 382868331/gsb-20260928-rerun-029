"""协调器:优先级规划 + 失败重排,处理迎面冲突与等待死结."""

from __future__ import annotations

import itertools

from .gridmap import GridMap
from .planner import plan_path
from .reservation import ReservationTable
from .robot import Robot, RobotStatus


class CoordinationError(Exception):
    pass


class Coordinator:
    """所有机器人共享一张预约表,不能各算各的最短路."""

    def __init__(self, gmap: GridMap, horizon: int = 200):
        self.gmap = gmap
        self.reservations = ReservationTable()
        self.horizon = horizon

    def plan_all(self, robots: list[Robot], start_tick: int, strict: bool = True) -> None:
        """按优先级依次规划;失败则换优先级顺序整体重排,解开死结.

        strict=False(暂停场景)时,被暂停者堵死的机器人原地等待,
        等恢复事件触发重排,而不是直接判定失败.
        """
        active = [r for r in robots
                  if r.status not in (RobotStatus.PAUSED, RobotStatus.DONE) and r.goal]
        # 暂停的机器人不重新规划,其预约原样保留(占用不凭空释放)
        for order in self._priority_orders(active):
            self.reservations = ReservationTable()
            for r in robots:
                if r.status == RobotStatus.PAUSED and r.path:
                    # 已占用轨迹截断到当前时刻,当前格保持占用
                    kept = r.path[: max(0, start_tick - r.plan_start)] + [r.pos]
                    self.reservations.reserve_path(r.robot_id, kept, r.plan_start)
                elif r not in active:
                    # 已完成/空闲机器人停在原地,其占用格永久保留
                    self.reservations.reserve_path(r.robot_id, [r.pos], start_tick)
            if self._try_order(order, start_tick):
                return
        if not strict:
            self._best_effort(active, start_tick)
            return
        raise CoordinationError("所有优先级顺序均无法规划,地图可能不连通")

    def _best_effort(self, robots: list[Robot], start_tick: int) -> None:
        for robot in robots:
            path = plan_path(
                self.gmap, self.reservations, robot.pos, robot.goal,
                start_tick, self.horizon,
            )
            if path is None:
                path = [robot.pos]  # 原地等待,占用当前格
            robot.path = path
            robot.plan_start = start_tick
            robot.status = RobotStatus.MOVING
            self.reservations.reserve_path(robot.robot_id, path, start_tick)

    def _priority_orders(self, robots: list[Robot]):
        robots = sorted(robots, key=lambda r: r.robot_id)
        if len(robots) <= 1:
            yield robots
            return
        # 先试按累计等待降序(饿久的先行),再尝试全排列
        yield sorted(robots, key=lambda r: -r.total_wait)
        yield from itertools.permutations(robots)

    def _try_order(self, order, start_tick: int) -> bool:
        planned: list[Robot] = []
        for robot in order:
            path = plan_path(
                self.gmap, self.reservations, robot.pos, robot.goal,
                start_tick, self.horizon,
            )
            if path is None:
                for r in planned:
                    self.reservations.release(r.robot_id)
                return False
            robot.path = path
            robot.plan_start = start_tick
            robot.status = RobotStatus.MOVING
            self.reservations.reserve_path(robot.robot_id, path, start_tick)
            planned.append(robot)
        return True
