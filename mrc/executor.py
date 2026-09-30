"""执行器:按 tick 推进,记录调度、封路与实际移动日志."""

from __future__ import annotations

import json
from pathlib import Path

from .coordinator import Coordinator
from .gridmap import Cell, GridMap
from .robot import Robot, RobotStatus, Task, WaitReason


class Executor:
    def __init__(self, coordinator: Coordinator, robots: list[Robot], log_path: str | Path):
        self.coordinator = coordinator
        self.robots = {r.robot_id: r for r in robots}
        self.tick = 0
        self.log_path = Path(log_path)
        self.log_path.write_text("")
        self.starvation_limit = 30

    @classmethod
    def resume_from_log(
        cls,
        gmap: GridMap,
        robots: list[Robot],
        log_path: str | Path,
    ) -> "Executor":
        """从既有调度记录恢复:已终止任务保持终止,不重新激活."""
        coordinator = Coordinator(gmap)
        executor = cls.__new__(cls)
        executor.coordinator = coordinator
        executor.robots = {r.robot_id: r for r in robots}
        executor.log_path = Path(log_path)
        executor.starvation_limit = 30
        executor.tick = _warm_state_from_log(executor, gmap, coordinator)
        return executor

    def add_staging_cells(self, cells: set[Cell] | list[Cell]) -> None:
        self.coordinator.add_staging_cells(cells)
        self._log("staging", cells=[list(cell) for cell in cells])

    def _log(self, event: str, **data) -> None:
        record = {"tick": self.tick, "event": event, **data}
        with self.log_path.open("a") as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    def assign(self, assignments: dict[str, Task]) -> None:
        for rid, task in assignments.items():
            robot = self.robots[rid]
            if robot.status == RobotStatus.TERMINATED or rid in self.coordinator.terminated:
                self._log(
                    "reject_reactivation",
                    robot=rid,
                    task=task.task_id,
                    prior_reason=robot.end_reason or "terminated",
                )
                continue
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
        self.coordinator.open_corridor(cells, list(self.robots.values()))
        self._log("open", cells=[list(cell) for cell in cells])
        self._replan(strict=False)

    def _replan(self, strict: bool = True) -> None:
        self._suppress_wait_replan = True
        self.coordinator.plan_all(list(self.robots.values()), self.tick, strict=strict)
        for disposition in self.coordinator.last_dispositions.values():
            self._log(
                "disposition",
                robot=disposition.robot_id,
                task=disposition.task_id,
                decision=disposition.decision,
                reason=disposition.reason,
                blockers=disposition.blockers,
                **{f"detail_{key}": _jsonable(value) for key, value in disposition.detail.items()},
            )
            if disposition.decision == "terminate":
                robot = self.robots[disposition.robot_id]
                self._log(
                    "terminated",
                    robot=disposition.robot_id,
                    task=disposition.task_id,
                    at=list(robot.pos),
                    end_reason=disposition.reason,
                    blockers=disposition.blockers,
                )
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
                if robot.status == RobotStatus.TERMINATED:
                    positions[robot.robot_id] = list(robot.pos)
                    continue
                if (
                    robot.status == RobotStatus.WAITING
                    and robot.task is not None
                    and robot.task.deadline is not None
                    and self.tick > robot.task.deadline
                    and robot.robot_id not in self.coordinator.exiting
                ):
                    reason = (
                        "deadline_infeasible"
                        if robot.wait_reason == WaitReason.DEADLINE_INFEASIBLE
                        else "expired"
                    )
                    blockers = list(robot.last_blockers)
                    self.coordinator._terminate(  # noqa: SLF001
                        robot, self.tick, reason=reason, blockers=blockers,
                    )
                    self._log(
                        "disposition",
                        robot=robot.robot_id,
                        task=robot.task.task_id,
                        decision="terminate",
                        reason=reason,
                        blockers=blockers,
                    )
                    self._log(
                        "deadline_missed",
                        robot=robot.robot_id,
                        task=robot.task.task_id,
                        deadline=robot.task.deadline,
                        blockers=blockers,
                    )
                    self._log(
                        "terminated",
                        robot=robot.robot_id,
                        task=robot.task.task_id,
                        at=list(robot.pos),
                        end_reason=reason,
                        blockers=blockers,
                    )
                    positions[robot.robot_id] = list(robot.pos)
                    continue
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
                    robot.task.end_tick = self.tick
                    robot.task.end_reason = "done"
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
        return any(
            r.status not in (RobotStatus.DONE, RobotStatus.FAILED, RobotStatus.TERMINATED)
            for r in self.robots.values()
        )

    def run(self, max_ticks: int = 500) -> None:
        while self.step() and self.tick < max_ticks:
            pass


def _jsonable(value):
    if isinstance(value, tuple):
        return list(value)
    return value


def _warm_state_from_log(executor: "Executor", gmap: GridMap, coordinator: Coordinator) -> int:
    """回放日志恢复:位置、任务终态、封路、临时停靠与已终止任务."""
    last_tick = 0
    if not executor.log_path.exists():
        return 0
    robots = executor.robots
    terminated_ids: set[str] = set()
    for line in executor.log_path.read_text().splitlines():
        rec = json.loads(line)
        last_tick = max(last_tick, rec["tick"])
        ev = rec["event"]
        rid = rec.get("robot")
        if ev == "assign" and rid in robots and rid not in terminated_ids:
            robot = robots[rid]
            robot.task = Task(
                rec["task"], tuple(rec["goal"]),
                assigned_tick=rec["tick"],
                priority=rec.get("priority", 0),
                deadline=rec.get("deadline"),
            )
            robot.status = RobotStatus.WAITING
            robot.path = [robot.pos]
            robot.end_reason = None
        elif ev == "terminated" and rid in robots:
            terminated_ids.add(rid)
            coordinator.terminated.add(rid)
            robot = robots[rid]
            robot.status = RobotStatus.TERMINATED
            robot.end_reason = rec.get("end_reason", "terminated")
            if robot.task:
                robot.task.end_reason = robot.end_reason
                robot.task.on_time = False
        elif ev == "done" and rid in robots and rid not in terminated_ids:
            robot = robots[rid]
            robot.status = RobotStatus.DONE
            if robot.task:
                robot.task.on_time = rec.get("on_time")
                robot.task.end_reason = "done"
        elif ev == "close":
            coordinator.close_corridor({tuple(cell) for cell in rec["cells"]})
        elif ev == "open":
            coordinator.open_corridor({tuple(cell) for cell in rec["cells"]})
        elif ev == "staging":
            coordinator.add_staging_cells({tuple(cell) for cell in rec["cells"]})
        elif ev == "positions":
            for robot_id, xy in rec["pos"].items():
                if robot_id in robots:
                    robots[robot_id].pos = tuple(xy)
    for rid, robot in robots.items():
        if rid in terminated_ids:
            robot.status = RobotStatus.TERMINATED
        if robot.status in (RobotStatus.DONE, RobotStatus.TERMINATED):
            coordinator.reservations.reserve_hold(rid, robot.pos, permanent=True)
    return last_tick
