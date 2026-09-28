"""执行器:按 tick 推进,记录调度与实际移动日志."""

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
        self.log_path.write_text("")
        self.starvation_limit = 30  # 连续等待超过该值记为饥饿风险

    def _log(self, event: str, **data) -> None:
        record = {"tick": self.tick, "event": event, **data}
        with self.log_path.open("a") as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    def assign(self, assignments: dict[str, Task]) -> None:
        """任务分配,沿用同一执行记录文件."""
        for rid, task in assignments.items():
            robot = self.robots[rid]
            task.assigned_tick = self.tick
            robot.task = task
            robot.status = RobotStatus.IDLE
            self._log("assign", robot=rid, goal=list(task.goal), task=task.task_id)
        self.coordinator.plan_all(list(self.robots.values()), self.tick)
        self._log("plan", plans={r.robot_id: r.path for r in self.robots.values() if r.path})

    def pause(self, robot_id: str) -> None:
        robot = self.robots[robot_id]
        robot.status = RobotStatus.PAUSED
        # 不释放预约:占用格在暂停期间仍然有效
        self._log("pause", robot=robot_id, at=list(robot.pos))
        # 其余机器人绕开暂停者重新协调
        self.coordinator.plan_all(list(self.robots.values()), self.tick, strict=False)
        self._log("plan", plans={r.robot_id: r.path for r in self.robots.values() if r.path})

    def resume(self, robot_id: str) -> None:
        robot = self.robots[robot_id]
        robot.status = RobotStatus.IDLE
        self._log("resume", robot=robot_id, at=list(robot.pos))
        # 从当前时刻、当前位置重新协调,不允许跳到前面
        self.coordinator.plan_all(list(self.robots.values()), self.tick)
        self._log("plan", plans={r.robot_id: r.path for r in self.robots.values() if r.path})

    def step(self) -> bool:
        """推进一个 tick,返回是否仍有未完成任务."""
        self.tick += 1
        positions: dict[str, list[int]] = {}
        for robot in self.robots.values():
            if robot.status == RobotStatus.PAUSED:
                pass  # 原地不动,预约保留
            elif robot.status == RobotStatus.MOVING:
                nxt = robot.planned_cell(self.tick)
                moved = nxt != robot.pos
                robot.pos = nxt
                if moved:
                    robot.wait_ticks = 0
                else:
                    robot.wait_ticks += 1
                    robot.total_wait += 1
                    if robot.wait_ticks == self.starvation_limit:
                        self._log("starve_risk", robot=robot.robot_id,
                                  waited=robot.wait_ticks)
                if robot.pos == robot.goal:
                    robot.status = RobotStatus.DONE
                    self._log("done", robot=robot.robot_id, task=robot.task.task_id)
            positions[robot.robot_id] = list(robot.pos)
        self._log("positions", pos=positions)
        return any(r.status not in (RobotStatus.DONE,) for r in self.robots.values())

    def run(self, max_ticks: int = 500) -> None:
        while self.step() and self.tick < max_ticks:
            pass
