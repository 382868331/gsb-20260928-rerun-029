"""执行器:按 tick 推进,记录调度、封路与实际移动日志."""

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
        self.starvation_limit = 30

    def _log(self, event: str, **data) -> None:
        record = {"tick": self.tick, "event": event, **data}
        with self.log_path.open("a") as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    def assign(self, assignments: dict[str, Task]) -> None:
        for rid, task in assignments.items():
            robot = self.robots[rid]
            task.assigned_tick = self.tick
            robot.task = task
            robot.status = RobotStatus.IDLE
            robot.wait_ticks = 0
            robot.last_blockers = []
            self._log(
                "assign",
                robot=rid,
                goal=list(task.goal),
                task=task.task_id,
                priority=task.priority,
                deadline=task.deadline,
            )
        self._replan(strict=False)

    def pause(self, robot_id: str) -> None:
        robot = self.robots[robot_id]
        robot.status = RobotStatus.PAUSED
        self._log("pause", robot=robot_id, at=list(robot.pos))
        self._replan(strict=False)

    def resume(self, robot_id: str) -> None:
        robot = self.robots[robot_id]
        robot.status = RobotStatus.IDLE
        self._log("resume", robot=robot_id, at=list(robot.pos))
        self._replan(strict=False)

    def close_corridor(self, cells: set[tuple[int, int]] | list[tuple[int, int]]) -> None:
        self.coordinator.close_corridor(cells)
        self._log("close", cells=[list(cell) for cell in cells])
        self._replan(strict=False)

    def open_corridor(self, cells: set[tuple[int, int]] | list[tuple[int, int]]) -> None:
        self.coordinator.open_corridor(cells)
        self._log("open", cells=[list(cell) for cell in cells])
        self._replan(strict=False)

    def _replan(self, strict: bool = True) -> None:
        self._suppress_wait_replan = True
        self.coordinator.plan_all(list(self.robots.values()), self.tick, strict=strict)
        plans = {r.robot_id: r.path for r in self.robots.values() if r.path}
        waiting = {
            r.robot_id: r.last_blockers
            for r in self.robots.values()
            if r.status == RobotStatus.WAITING and r.last_blockers
        }
        self._log("plan", plans=plans, waiting=waiting)
        for robot in self.robots.values():
            if robot.status == RobotStatus.WAITING and robot.last_blockers:
                self._log(
                    "blocked",
                    robot=robot.robot_id,
                    at=list(robot.pos),
                    blockers=robot.last_blockers,
                )

    def step(self) -> bool:
        if getattr(self, "_suppress_wait_replan", False):
            self._suppress_wait_replan = False
        elif any(
            robot.status == RobotStatus.WAITING
            and robot.task is not None
            and robot.robot_id not in self.coordinator.exiting
            for robot in self.robots.values()
        ):
            self._replan(strict=False)
        self.tick += 1
        positions: dict[str, list[int]] = {}
        for robot in self.robots.values():
            if robot.status == RobotStatus.PAUSED:
                pass
            elif robot.status in (RobotStatus.MOVING, RobotStatus.WAITING):
                nxt_reserved = self.coordinator.reservations.is_free(
                    robot.planned_cell(self.tick),
                    self.tick,
                    robot.pos,
                    owner=robot.robot_id,
                )
                if not nxt_reserved:
                    robot.status = RobotStatus.WAITING
                    robot.path = [robot.pos]
                    robot.plan_start = self.tick
                    self._replan(strict=False)
                if robot.status == RobotStatus.WAITING and robot.task.deadline is not None and self.tick > robot.task.deadline:
                    robot.task.on_time = False
                    self._log(
                        "deadline_missed",
                        robot=robot.robot_id,
                        task=robot.task.task_id,
                        deadline=robot.task.deadline,
                        blockers=robot.last_blockers,
                    )
                nxt = robot.planned_cell(self.tick)
                moved = nxt != robot.pos
                robot.pos = nxt
                if moved:
                    robot.wait_ticks = 0
                else:
                    robot.wait_ticks += 1
                    robot.total_wait += 1
                    if robot.wait_ticks == self.starvation_limit:
                        self._log(
                            "starve_risk",
                            robot=robot.robot_id,
                            waited=robot.wait_ticks,
                            blockers=robot.last_blockers,
                        )
                if robot.task and robot.pos == robot.task.goal and robot.robot_id not in self.coordinator.exiting:
                    robot.status = RobotStatus.DONE
                    on_time = robot.task.deadline is None or self.tick <= robot.task.deadline
                    robot.task.on_time = on_time
                    self._log(
                        "done",
                        robot=robot.robot_id,
                        task=robot.task.task_id,
                        tick=self.tick,
                        deadline=robot.task.deadline,
                        on_time=on_time,
                    )
                elif robot.status == RobotStatus.WAITING and not moved:
                    self._log(
                        "blocked",
                        robot=robot.robot_id,
                        at=list(robot.pos),
                        blockers=robot.last_blockers,
                    )
            positions[robot.robot_id] = list(robot.pos)
        reached = self.coordinator.reconcile(list(self.robots.values()), self.tick)
        if reached:
            self._log(
                "plan",
                plans={r.robot_id: r.path for r in self.robots.values() if r.path},
                waiting={
                    r.robot_id: r.last_blockers
                    for r in self.robots.values()
                    if r.status == RobotStatus.WAITING and r.last_blockers
                },
            )
        self._log("positions", pos=positions)
        return any(r.status not in (RobotStatus.DONE, RobotStatus.FAILED) for r in self.robots.values())

    def run(self, max_ticks: int = 500) -> None:
        while self.step() and self.tick < max_ticks:
            pass
