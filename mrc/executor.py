# -*- coding: utf-8 -*-
"""执行器:按 tick 推进,记录调度与实际移动日志。"""

from __future__ import annotations

import json
from pathlib import Path

from .coordinator import Coordinator
from .robot import Robot, RobotStatus, Task


class Executor:
    def __init__(self, coordinator: Coordinator, robots: list[Robot], log_path: str | Path):
        self.coordinator = coordinator
        self.robots = {r.robot_id: r for r in robots}
        self.tick = 0
        self.log_path = Path(log_path)
        self.log_path.write_text('')
        self.starvation_limit = 30
        self._deadline_warned: set[str] = set()

    def _log(self, event: str, **data) -> None:
        record = {'tick': self.tick, 'event': event, **data}
        with self.log_path.open('a') as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + '\n')

    def _log_plan(self, tick: int | None = None) -> None:
        tick = self.tick if tick is None else tick
        plans = {
            r.robot_id: r.path
            for r in self.robots.values()
            if r.path and r.status in (
                RobotStatus.MOVING, RobotStatus.EVACUATING, RobotStatus.BLOCKED)
            and r.plan_start == tick
        }
        self._log('plan', plans=plans)
        for r in self.robots.values():
            if r.status == RobotStatus.BLOCKED:
                self._log('blocked', robot=r.robot_id, task=r.task.task_id if r.task else None,
                          kind='infeasible' if r.infeasible else 'not_found',
                          reason=r.block_reason)

    def assign(self, assignments: dict[str, Task], strict: bool = False) -> dict:
        """分配任务(可带 priority/deadline),沿用同一执行记录文件。"""
        for rid, task in assignments.items():
            robot = self.robots[rid]
            task.assigned_tick = self.tick
            robot.task = task
            robot.priority = getattr(task, "priority", 0)
            robot.status = RobotStatus.IDLE
            robot.committed = False
            robot.evacuating = False
            robot.infeasible = False
            robot.block_reason = ''
            robot.path = []
            self._log('assign', robot=rid, goal=list(task.goal), task=task.task_id,
                      priority=robot.priority, deadline=task.deadline)
        summary = self.coordinator.plan_all(list(self.robots.values()), self.tick, strict=strict)
        for robot in self.robots.values():
            if robot.status in (RobotStatus.MOVING, RobotStatus.EVACUATING):
                robot.committed = True  # 预约即刻锁定,后来任务不可抢占
        self._log_plan()
        return summary

    def pause(self, robot_id: str) -> None:
        robot = self.robots[robot_id]
        robot.status = RobotStatus.PAUSED
        self._log('pause', robot=robot_id, at=list(robot.pos))
        # 暂停者占住当前格:其余在途机器人若未来预约与该格冲突,必须绕行/等待
        for other in self.robots.values():
            if other is robot or not other.path:
                continue
            future = other.path[max(0, self.tick - other.plan_start):]
            if robot.pos in future:
                other.committed = False
                other.status = RobotStatus.IDLE
                other.path = []
        self.coordinator.plan_all(list(self.robots.values()), self.tick, strict=False)
        for other in self.robots.values():
            if other.status == RobotStatus.MOVING and not other.committed:
                other.committed = True
        self.coordinator.advance(list(self.robots.values()), self.tick)
        self._log_plan()

    def resume(self, robot_id: str) -> None:
        robot = self.robots[robot_id]
        robot.status = RobotStatus.IDLE
        # 暂停期间其未来预约已被他人重排覆盖,恢复时以新参与者身份从当前位置协调
        robot.committed = False
        robot.path = []
        self._log('resume', robot=robot_id, at=list(robot.pos))
        self.coordinator.plan_all(list(self.robots.values()), self.tick, strict=False)
        for r in self.robots.values():
            if r.status == RobotStatus.MOVING and not r.committed:
                r.committed = True
        self._log_plan()

    def close_segment(self, cells: set[Cell]) -> dict:
        cells = set(cells)
        self._log('close', cells=[list(c) for c in sorted(cells)])
        summary = self.coordinator.close_segment(list(self.robots.values()), self.tick, cells)
        for r in self.robots.values():
            if r.status == RobotStatus.EVACUATING:
                self._log('evacuate', robot=r.robot_id, path=r.path, plan_start=r.plan_start)
        self._log_plan()
        return summary

    def reopen_segment(self, cells: set[Cell]) -> dict:
        cells = set(cells)
        self._log('open', cells=[list(c) for c in sorted(cells)])
        summary = self.coordinator.reopen_segment(list(self.robots.values()), self.tick, cells)
        for r in self.robots.values():
            if r.status == RobotStatus.MOVING and not r.committed:
                r.committed = True
        self._log_plan()
        return summary

    def step(self) -> bool:
        self.tick += 1
        positions: dict[str, list[int]] = {}
        moved_any = False
        for robot in self.robots.values():
            if robot.status in (RobotStatus.MOVING, RobotStatus.EVACUATING):
                nxt = robot.planned_cell(self.tick)
                moved = nxt != robot.pos
                robot.pos = nxt
                moved_any = moved_any or moved
                if moved:
                    robot.wait_ticks = 0
                else:
                    robot.wait_ticks += 1
                    robot.total_wait += 1
                    if robot.wait_ticks == self.starvation_limit:
                        self._log('starve_risk', robot=robot.robot_id, waited=robot.wait_ticks)
                if (robot.status == RobotStatus.MOVING
                        and robot.task and robot.pos == robot.goal):
                    robot.status = RobotStatus.DONE
                    self._log('done', robot=robot.robot_id, task=robot.task.task_id,
                              tick_done=self.tick, deadline=robot.task.deadline)
                task = robot.task
                if (task and task.deadline is not None and robot.status != RobotStatus.DONE
                        and self.tick > task.deadline and task.task_id not in self._deadline_warned):
                    self._deadline_warned.add(task.task_id)
                    self._log('deadline_missed', robot=robot.robot_id, task=task.task_id,
                              deadline=task.deadline)
            positions[robot.robot_id] = list(robot.pos)
        self._log('positions', pos=positions)

        blocked = [r for r in self.robots.values() if r.status == RobotStatus.BLOCKED]
        if blocked and moved_any:
            # 阻挡预约随时间推进而变化,被堵者在他人移动后重新尝试
            for r in blocked:
                r.status = RobotStatus.IDLE
            self.coordinator.plan_all(list(self.robots.values()), self.tick, strict=False)
            for r in self.robots.values():
                if r.status in (RobotStatus.MOVING, RobotStatus.EVACUATING) and not r.committed:
                    r.committed = True
            self._log_plan()
        else:
            self.coordinator.advance(list(self.robots.values()), self.tick)
        return any(r.status != RobotStatus.DONE for r in self.robots.values())

    def run(self, max_ticks: int = 500) -> None:
        while self.step() and self.tick < max_ticks:
            pass
