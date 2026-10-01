"""执行器:按 tick 推进,记录调度、封路与实际移动日志."""

from __future__ import annotations

import json
from pathlib import Path

from .coordinator import Coordinator, CoordinationError
from .cargo import HandoverState
from .planner import DEADLINE_INFEASIBLE, PlanResult
from .robot import Robot, RobotStatus, Task


class Executor:
    def __init__(self, coordinator: Coordinator, robots: list[Robot], log_path: str | Path):
        self.coordinator = coordinator
        self.robots = {r.robot_id: r for r in robots}
        self.tick = 0
        self.log_path = Path(log_path)
        self.log_path.write_text("")
        self.starvation_limit = 30
        self.terminated_logged: set[tuple[str, str]] = set()
        self._handover_failures_logged: set[str] = set()

    def register_cargo(self, cargo_id: str, owner: str) -> None:
        self.coordinator.register_cargo(cargo_id, owner)
        self._log("cargo_register", cargo=cargo_id, owner=owner)

    def register_handover_zone(self, zone_id: str, carrier_slot: tuple[int, int], receiver_slot: tuple[int, int]) -> None:
        self.coordinator.register_handover_zone(zone_id, carrier_slot, receiver_slot)
        self._log(
            "handover_zone",
            zone=zone_id,
            carrier_slot=list(carrier_slot),
            receiver_slot=list(receiver_slot),
        )

    def request_handover(
        self,
        handover_id: str,
        cargo_id: str,
        carrier: str,
        receiver: str,
        zone_id: str,
        carrier_task_id: str,
        receiver_task_id: str,
        delivery_task_id: str,
        delivery_goal: tuple[int, int],
        priority: int = 0,
        deadline: int | None = None,
    ) -> None:
        """建立交接承诺并分别派发前往交接槽的两段任务,全程走同一预约表。"""
        commitment = self.coordinator.create_handover(
            handover_id, cargo_id, carrier, receiver, zone_id,
            carrier_task_id, receiver_task_id, delivery_goal, delivery_task_id,
            self.tick, priority=priority, deadline=deadline,
        )
        self.assign({
            carrier: Task(
                carrier_task_id, commitment.carrier_slot,
                priority=priority, cargo_id=cargo_id,
                handover_id=handover_id, handover_role="carrier",
            ),
            receiver: Task(
                receiver_task_id, commitment.receiver_slot,
                priority=priority,
                handover_id=handover_id, handover_role="receiver",
            ),
        })
        for rid in (carrier, receiver):
            robot = self.robots[rid]
            robot.handover_id = handover_id
            robot.handover_role = "carrier" if rid == carrier else "receiver"
            robot.leg_start_cell = robot.pos
        self._log(
            "handover_request",
            handover=handover_id,
            cargo=cargo_id,
            carrier=carrier,
            receiver=receiver,
            zone=zone_id,
            carrier_task=carrier_task_id,
            receiver_task=receiver_task_id,
            delivery_task=delivery_task_id,
            delivery_goal=list(delivery_goal),
            owner=self.coordinator.cargo_registry.owner_of(cargo_id),
        )

    def cancel_handover(self, handover_id: str, reason: str = "cancelled") -> None:
        commitment = self.coordinator.cancel_handover(
            list(self.robots.values()), handover_id, self.tick, reason,
        )
        self._log(
            "handover_cancelled",
            handover=handover_id,
            cargo=commitment.cargo_id,
            owner=self.coordinator.cargo_registry.owner_of(commitment.cargo_id),
            reason=reason,
        )
        for rid in commitment.participants():
            self.terminated_logged.add((rid, commitment.carrier_task if rid == commitment.carrier else commitment.receiver_task))

    def repoint_handover(self, handover_id: str, new_zone_id: str) -> None:
        commitment = self.coordinator.repoint_handover(
            list(self.robots.values()), handover_id, new_zone_id, self.tick,
        )
        self._log(
            "handover_repoint",
            handover=handover_id,
            zone=new_zone_id,
            carrier_slot=list(commitment.carrier_slot),
            receiver_slot=list(commitment.receiver_slot),
        )
        self._replan(strict=False)

    def handover_options(self, handover_id: str, robot_id: str, evac_cell=None):
        return self.coordinator.party_options(
            list(self.robots.values()), handover_id, robot_id, self.tick, evac_cell,
        )

    def handover_policy(self, handover_id: str):
        return self.coordinator.commitment_policy(handover_id)

    def compare_handovers(self, handover_id: str, candidate_zones=None):
        return self.coordinator.compare_options(
            list(self.robots.values()), handover_id, candidate_zones,
        )

    def _log(self, event: str, **data) -> None:
        record = {"tick": self.tick, "event": event, **data}
        with self.log_path.open("a") as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    def assign(self, assignments: dict[str, Task]) -> None:
        for rid, task in assignments.items():
            robot = self.robots[rid]
            commitment = self.coordinator.cargo_registry.active_commitment_of(rid)
            if commitment is not None and rid not in self.coordinator.handover_waiting:
                commitment = None
            is_handover_delivery = (
                commitment is not None
                and task.handover_id == commitment.handover_id
                and task.handover_role == "delivery"
                and rid == commitment.receiver
            )
            if commitment is not None and not is_handover_delivery:
                raise CoordinationError(
                    f"{rid} 正在执行交接 {commitment.handover_id},不能用普通任务覆盖;"
                    "请先取消(仅未开始)或等待交接失败/完成"
                )
            if robot.status == RobotStatus.DONE:
                self.coordinator.reservations.permanent = {
                    cell: owner
                    for cell, owner in self.coordinator.reservations.permanent.items()
                    if owner != rid
                }
            task.assigned_tick = self.tick
            robot.task = task
            robot.status = RobotStatus.IDLE
            robot.wait_ticks = 0
            robot.last_blockers = []
            robot.last_reason = None
            self.terminated_logged.discard((rid, task.task_id))
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
        for decision in self.coordinator.decisions:
            self._log("task_decision", **decision)
            if decision.get("action") == "terminate":
                robot = self.robots[str(decision["robot"])]
                key = (robot.robot_id, str(decision["task"]))
                if robot.task and key not in self.terminated_logged:
                    self.terminated_logged.add(key)
                    self._log(
                        "task_terminated",
                        robot=robot.robot_id,
                        task=robot.task.task_id,
                        reason=robot.last_reason,
                        blockers=robot.last_blockers,
                    )
        for robot in self.robots.values():
            if robot.status in (RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT) and robot.last_blockers:
                self._log(
                    "search_budget" if robot.status == RobotStatus.SEARCH_LIMIT else "blocked",
                    robot=robot.robot_id,
                    at=list(robot.pos),
                    reason=robot.last_reason,
                    blockers=robot.last_blockers,
                )

    def step(self) -> bool:
        self.coordinator.fail_broken_handovers(list(self.robots.values()), self.tick)
        for commitment in self.coordinator.failed_commitments():
            self._emit_handover_failure(commitment)
        if getattr(self, "_suppress_wait_replan", False):
            self._suppress_wait_replan = False
        elif any(
            robot.status == RobotStatus.WAITING and robot.robot_id in self.coordinator.exiting
            for robot in self.robots.values()
        ):
            for robot in self.robots.values():
                if robot.status == RobotStatus.WAITING and robot.robot_id in self.coordinator.exiting:
                    self.coordinator._replan_exiting(robot, self.tick)
        elif any(
            robot.status in (RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT)
            and robot.task is not None
            and robot.robot_id not in self.coordinator.exiting
            and robot.robot_id not in self.coordinator.handover_waiting
            for robot in self.robots.values()
        ):
            self._replan(strict=False)
        self.tick += 1
        positions: dict[str, list[int]] = {}
        for robot in self.robots.values():
            if robot.status == RobotStatus.PAUSED:
                pass
            elif robot.status in (RobotStatus.MOVING, RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT):
                nxt_reserved = self.coordinator.reservations.is_free(
                    robot.planned_cell(self.tick),
                    self.tick,
                    robot.pos,
                    owner=robot.robot_id,
                )
                if not nxt_reserved:
                    was_exiting = robot.robot_id in self.coordinator.exiting
                    robot.status = RobotStatus.WAITING
                    robot.path = [robot.pos]
                    robot.plan_start = self.tick
                    if was_exiting:
                        robot.last_reason = "temporary_block"
                        robot.last_blockers = ["temporary_block"]
                    else:
                        self._replan(strict=False)
                if robot.status in (RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT) and robot.task.deadline is not None and self.tick > robot.task.deadline:
                    result = PlanResult(
                        DEADLINE_INFEASIBLE,
                        blockers=set(robot.last_blockers or [DEADLINE_INFEASIBLE]),
                    )
                    self.coordinator._terminate(robot, result, self.tick, "等待后已经无法按时完成")
                    if self.coordinator.decisions:
                        self._log("task_decision", **self.coordinator.decisions[-1])
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
                    if robot.wait_ticks == self.starvation_limit and robot.robot_id not in self.coordinator.handover_waiting:
                        self._log(
                            "starve_risk",
                            robot=robot.robot_id,
                            waited=robot.wait_ticks,
                            blockers=robot.last_blockers,
                        )
                if (
                    robot.task
                    and robot.pos == robot.task.goal
                    and robot.robot_id not in self.coordinator.exiting
                    and robot.robot_id not in self.coordinator.handover_waiting
                    and not self.coordinator.is_handover_arrival(robot)
                ):
                    robot.task.on_time = robot.task.deadline is None or self.tick <= robot.task.deadline
                    robot.status = RobotStatus.DONE
                    on_time = robot.task.on_time
                    self._log(
                        "done",
                        robot=robot.robot_id,
                        task=robot.task.task_id,
                        tick=self.tick,
                        deadline=robot.task.deadline,
                        on_time=on_time,
                    )
                elif self.coordinator.is_handover_arrival(robot):
                    commitment = self.coordinator.handover_arrival(robot, self.tick)
                    if commitment is not None:
                        self._log(
                            "handover_party_arrived",
                            handover=commitment.handover_id,
                            robot=robot.robot_id,
                            at=list(robot.pos),
                            cargo=commitment.cargo_id,
                            owner=self.coordinator.cargo_registry.owner_of(commitment.cargo_id),
                            state=commitment.state.value,
                        )
                elif robot.status == RobotStatus.FAILED:
                    key = (robot.robot_id, robot.task.task_id)
                    if key not in self.terminated_logged:
                        self.terminated_logged.add(key)
                        self._log(
                            "task_terminated",
                            robot=robot.robot_id,
                            task=robot.task.task_id,
                            reason=robot.last_reason,
                            blockers=robot.last_blockers,
                        )
                elif robot.status in (RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT) and not moved:
                    self._log(
                        "search_budget" if robot.status == RobotStatus.SEARCH_LIMIT else "blocked",
                        robot=robot.robot_id,
                        at=list(robot.pos),
                        reason=robot.last_reason,
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
                    if r.status in (RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT) and r.last_blockers
                },
            )
        self.coordinator.mark_leg_started(list(self.robots.values()))
        self.coordinator.fail_broken_handovers(list(self.robots.values()), self.tick)
        for commitment in self.coordinator.failed_commitments():
            self._emit_handover_failure(commitment)
            for rid in commitment.participants():
                robot = self.robots[rid]
                if robot.status == RobotStatus.FAILED and robot.task:
                    key = (rid, robot.task.task_id)
                    if key not in self.terminated_logged:
                        self.terminated_logged.add(key)
                        self._log(
                            "task_terminated",
                            robot=rid,
                            task=robot.task.task_id,
                            reason=robot.last_reason,
                            blockers=robot.last_blockers,
                            handover=commitment.handover_id,
                        )
        for commitment in self.coordinator.complete_handovers(
            list(self.robots.values()), self.tick
        ):
            self._log(
                "handover_complete",
                handover=commitment.handover_id,
                cargo=commitment.cargo_id,
                carrier=commitment.carrier,
                receiver=commitment.receiver,
                at=[list(commitment.carrier_slot), list(commitment.receiver_slot)],
                owner=self.coordinator.cargo_registry.owner_of(commitment.cargo_id),
            )
            for rid, task_id in (
                (commitment.carrier, commitment.carrier_task),
                (commitment.receiver, commitment.receiver_task),
            ):
                self._log(
                    "done",
                    robot=rid,
                    task=task_id,
                    tick=self.tick,
                    deadline=None,
                    on_time=True,
                    handover=commitment.handover_id,
                )
            if commitment.delivery_task is not None and commitment.delivery_goal is not None:
                self.assign({
                    commitment.receiver: Task(
                        commitment.delivery_task,
                        commitment.delivery_goal,
                        priority=commitment.delivery_priority,
                        deadline=commitment.delivery_deadline,
                        cargo_id=commitment.cargo_id,
                        handover_id=commitment.handover_id,
                        handover_role="delivery",
                    ),
                })
        cargo_owners = {
            cargo_id: cargo.owner
            for cargo_id, cargo in self.coordinator.cargo_registry.cargo.items()
        }
        if cargo_owners:
            self._log("cargo_state", owners=cargo_owners)
        self._log("positions", pos=positions)
        return any(r.status not in (RobotStatus.DONE, RobotStatus.FAILED) for r in self.robots.values())

    def _emit_handover_failure(self, commitment) -> None:
        if commitment.handover_id not in self._handover_failures_logged:
            self._handover_failures_logged.add(commitment.handover_id)
            self._log(
                "handover_failed",
                handover=commitment.handover_id,
                cargo=commitment.cargo_id,
                owner=self.coordinator.cargo_registry.owner_of(commitment.cargo_id),
                reason=commitment.reason,
            )
        for rid in commitment.participants():
            robot = self.robots[rid]
            task_id = robot.task.task_id if robot.task else (
                commitment.carrier_task if rid == commitment.carrier else commitment.receiver_task
            )
            key = (rid, task_id)
            if key not in self.terminated_logged:
                self.terminated_logged.add(key)
                reason = (
                    robot.last_reason
                    if robot.status == RobotStatus.FAILED and robot.last_reason
                    else "handover_failed"
                )
                blockers = (
                    robot.last_blockers
                    if robot.status == RobotStatus.FAILED and robot.last_blockers
                    else [commitment.reason or "handover_failed"]
                )
                self._log(
                    "task_terminated",
                    robot=rid,
                    task=task_id,
                    reason=reason,
                    blockers=blockers,
                    handover=commitment.handover_id,
                )

    def run(self, max_ticks: int = 500) -> None:
        while self.step() and self.tick < max_ticks:
            pass
