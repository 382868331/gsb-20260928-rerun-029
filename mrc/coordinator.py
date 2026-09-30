# -*- coding: utf-8 -*-
"""协调器:共享预约表上的优先级规划.

关键约定:
- 所有机器人共用一张预约表,不允许各算各的最短路。
- 已经在途(committed)的预约、暂停占位、封路退出安排均被锁定,
  后来提交的高优先(紧急)任务只能在剩余时空中寻找安排,不能抢占。
- 规划失败区分 infeasible(无可行安排)与 not_found(时间窗内尚未找到),
  并记录阻挡来源(被谁的预约挡住)。
- 封路:区段内机器人有明确退出安排;区段外预约路径穿入封闭区的部分作废。
- 重开:仍有效的在途预约原样保留,只重排受影响者。
"""

from __future__ import annotations

import itertools

from .gridmap import GridMap
from .planner import PlanResult, plan_path
from .reservation import ReservationTable
from .robot import Robot, RobotStatus


class CoordinationError(Exception):
    pass


def _reserve_locked(table: ReservationTable, robot: Robot, start_tick: int) -> None:
    if robot.path and robot.plan_start <= start_tick:
        kept = robot.path[max(0, start_tick - robot.plan_start):]
        if not kept or kept[0] != robot.pos:
            kept = [robot.pos] + kept
    else:
        kept = [robot.pos]
    table.reserve_path(robot.robot_id, kept, start_tick)


class Coordinator:
    def __init__(self, gmap: GridMap, horizon: int = 200):
        self.gmap = gmap
        self.reservations = ReservationTable()
        self.horizon = horizon

    def _seed_table(self, robots: list[Robot], start_tick: int) -> ReservationTable:
        table = ReservationTable()
        horizon_tick = start_tick + self.horizon
        for r in robots:
            if r.status == RobotStatus.PAUSED:
                table.reserve_holding(r.robot_id, r.pos, start_tick, horizon_tick)
            elif r.status == RobotStatus.EVACUATING and r.committed:
                # 退出安排:撤出路径有效,但撤出点是临时停留,不作永久终点预约
                if r.path and r.plan_start <= start_tick:
                    kept = r.path[max(0, start_tick - r.plan_start):]
                    if not kept or kept[0] != r.pos:
                        kept = [r.pos] + kept
                else:
                    kept = [r.pos]
                for i, cell in enumerate(kept):
                    table.vertex[(cell, start_tick + i)] = r.robot_id
                if kept:
                    table.reserve_holding(r.robot_id, kept[-1],
                                          start_tick + len(kept) - 1, horizon_tick)
            elif r.status == RobotStatus.MOVING and r.committed:
                if r.path and r.plan_start <= start_tick:
                    kept = r.path[max(0, start_tick - r.plan_start):]
                    if not kept or kept[0] != r.pos:
                        kept = [r.pos] + kept
                else:
                    kept = [r.pos]
                # 逐拍占用顶点与边;终点格仅在其确为任务终点时永久保留
                for i, cell in enumerate(kept):
                    table.vertex[(cell, start_tick + i)] = r.robot_id
                    if i > 0:
                        table.edge[table._edge_key(
                            kept[i - 1], cell, start_tick + i)] = r.robot_id
                if kept and r.path and r.goal == r.path[-1]:
                    table.goal[kept[-1]] = r.robot_id
            elif r.status == RobotStatus.DONE or not r.goal:
                table.reserve_path(r.robot_id, [r.pos], start_tick)
            elif r.status == RobotStatus.BLOCKED:
                table.reserve_holding(r.robot_id, r.pos, start_tick, horizon_tick)
        return table

    def _active(self, robots: list[Robot]) -> list[Robot]:
        return [r for r in robots
                if r.goal and r.status in (RobotStatus.IDLE, RobotStatus.BLOCKED)
                and not r.committed]

    @staticmethod
    def _priority_key(robot: Robot):
        deadline = robot.task.deadline if robot.task and robot.task.deadline is not None else 1 << 30
        return (-robot.priority, deadline, -robot.total_wait, robot.robot_id)

    def _orders(self, robots: list[Robot]):
        by_priority = sorted(robots, key=self._priority_key)
        yield by_priority
        yield sorted(robots, key=lambda r: (-r.total_wait, r.robot_id))
        if len(robots) <= 6:
            for order in itertools.permutations(by_priority):
                yield list(order)

    def plan_all(self, robots: list[Robot], start_tick: int, strict: bool = True) -> dict:
        """为所有未锁定的机器人协调安排。返回 robot_id -> 状态。"""
        active = self._active(robots)
        for robot in active:
            robot.path = []
            robot.plan_start = start_tick

        for order in self._orders(active):
            table = self._seed_table(robots, start_tick)
            ok = True
            for robot in order:
                deadline = robot.task.deadline if robot.task else None
                result = plan_path(
                    self.gmap, table, robot.pos, robot.goal,
                    start_tick, self.horizon, deadline=deadline)
                if result.status != 'feasible':
                    ok = False
                    break
                robot.path = result.path
                robot.plan_start = start_tick
                table.reserve_path(robot.robot_id, result.path, start_tick)
            if ok:
                self.reservations = table
                for robot in order:
                    robot.status = RobotStatus.MOVING
                    robot.infeasible = False
                    robot.block_reason = ''
                return {robot.robot_id: 'feasible' for robot in order}

        best: tuple[int, dict[str, PlanResult], list[Robot]] | None = None
        for order in self._orders(active):
            table = self._seed_table(robots, start_tick)
            trial: dict[str, PlanResult] = {}
            planned: list[Robot] = []
            winners: set[str] = set()
            for robot in order:
                deadline = robot.task.deadline if robot.task else None
                result = plan_path(
                    self.gmap, table, robot.pos, robot.goal,
                    start_tick, self.horizon, deadline=deadline)
                trial[robot.robot_id] = result
                if result.status == 'feasible':
                    table.reserve_path(robot.robot_id, result.path, start_tick)
                    planned.append(robot)
                    winners.add(robot.robot_id)
                else:
                    # 该顺序下此机暂无安排:立即占位,后续规划者不得穿其当前格
                    table.reserve_holding(
                        robot.robot_id, robot.pos, start_tick, start_tick + self.horizon)
            if best is None or len(planned) > len(best[2]):
                best = (len(planned), trial, planned)
            if len(planned) == len(order):
                break
        assert best is not None
        _, trial, planned = best
        table = self._seed_table(robots, start_tick)
        failures: list[str] = []
        summary: dict[str, str] = {}
        for robot in active:
            result = trial[robot.robot_id]
            if result.status == 'feasible':
                robot.path = result.path
                robot.plan_start = start_tick
                robot.status = RobotStatus.MOVING
                robot.infeasible = False
                robot.block_reason = ''
                table.reserve_path(robot.robot_id, result.path, start_tick)
            else:
                robot.path = [robot.pos]
                robot.plan_start = start_tick
                robot.status = RobotStatus.BLOCKED
                robot.infeasible = result.status == 'infeasible'
                robot.block_reason = '; '.join(result.blockers)
                table.reserve_holding(
                    robot.robot_id, robot.pos, start_tick, start_tick + self.horizon)
                failures.append(
                    robot.robot_id + ':' + result.status + '(' + robot.block_reason + ')')
            summary[robot.robot_id] = result.status
        self.reservations = table
        if strict and failures:
            raise CoordinationError('无全局可行安排: ' + ' | '.join(failures))
        return summary

    def close_segment(self, robots: list[Robot], tick: int, cells: set[Cell]) -> dict:
        """封闭区段:区内机器人退出;区外在途预约穿入封闭区的部分作废。"""
        cells = set(cells)
        self.gmap.close_cells(cells)
        summary: dict[str, str] = {}

        for robot in robots:
            if not robot.path and robot.pos not in cells:
                continue
            future = robot.path[max(0, tick - robot.plan_start):] if robot.path else []
            if robot.pos in cells:
                others = [r for r in robots if r is not robot]
                table = self._seed_table(others, tick)
                result = plan_path(
                    self.gmap, table, robot.pos, robot.pos, tick,
                    self.horizon, exit_zone=True)
                robot.evacuating = True
                robot.committed = True
                if result.status == 'feasible':
                    robot.path = result.path
                    robot.plan_start = tick
                    robot.status = RobotStatus.EVACUATING
                    summary[robot.robot_id] = 'evacuating'
                else:
                    robot.status = RobotStatus.BLOCKED
                    robot.block_reason = '无法撤出封闭区段: ' + '; '.join(result.blockers)
                    summary[robot.robot_id] = 'trapped'
            elif any(c in cells for c in future) and robot.committed:
                cut = max(0, tick - robot.plan_start)
                for cell in future:
                    if cell in cells:
                        break
                    cut += 1
                robot.path = robot.path[:cut] or [robot.pos]
                robot.committed = False
                robot.status = RobotStatus.IDLE
                summary[robot.robot_id] = 'rerouted_pending'

        self.reservations = self._seed_table(robots, tick)
        summary.update(self.plan_all(robots, tick, strict=False))
        return summary

    def reopen_segment(self, robots: list[Robot], tick: int, cells: set[Cell]) -> dict:
        """重新开放:仍有效的预约保留,只重排受影响者。"""
        self.gmap.open_cells(set(cells))
        for robot in robots:
            if robot.committed and robot.path:
                for cell in robot.path[max(0, tick - robot.plan_start):]:
                    if not self.gmap.passable(cell):
                        robot.committed = False
                        robot.path = [robot.pos]
                        robot.status = RobotStatus.IDLE
                        break
        for robot in robots:
            if robot.status == RobotStatus.EVACUATING and robot.pos not in self.gmap.closures:
                robot.evacuating = False
                if robot.goal and robot.pos != robot.goal:
                    robot.committed = False
                    robot.status = RobotStatus.IDLE
            elif robot.status == RobotStatus.BLOCKED:
                # 封路期间的临时占位不再有效,重开后以未锁定身份重新规划
                robot.status = RobotStatus.IDLE
                robot.infeasible = False
                robot.block_reason = ''
        return self.plan_all(robots, tick, strict=False)

    def advance(self, robots: list[Robot], tick: int) -> None:
        """每个执行拍后调用:按当前时钟重建锁定预约,供后续插入任务使用。"""
        self.reservations = self._seed_table(robots, tick)
