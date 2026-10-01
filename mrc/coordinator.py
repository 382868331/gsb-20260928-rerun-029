"""协调器:锁定在途预约,新任务在剩余时空资源中规划."""

from __future__ import annotations

import itertools
import json
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

from .cargo import (
    Cargo,
    CargoRegistry,
    HandoverCommitment,
    HandoverOption,
    HandoverState,
)
from .gridmap import Cell, GridMap
from .planner import DEADLINE_INFEASIBLE, INFEASIBLE, SEARCH_LIMIT, PlanResult, plan_path_result
from .reservation import ReservationTable
from .robot import Robot, RobotStatus


class CoordinationError(Exception):
    def __init__(self, message: str, results: dict[str, PlanResult] | None = None):
        super().__init__(message)
        self.results = results or {}


@dataclass
class Closure:
    cells: set[Cell]


class Coordinator:
    """所有机器人共享一张预约表,重规划不能抢掉仍有效的在途预约."""

    def __init__(self, gmap: GridMap, horizon: int = 200):
        self.gmap = gmap
        self.reservations = ReservationTable()
        self.horizon = horizon
        self.closures: list[Closure] = []
        self.closed_cells: set[Cell] = set()
        self.pending: set[str] = set()
        self.exiting: dict[str, Cell] = {}
        self.evacuating_tasks: set[str] = set()
        self.terminal_tasks: dict[str, dict[str, object]] = {}
        self.decisions: list[dict[str, object]] = []
        self.cargo_registry = CargoRegistry()
        self.handover_zones: dict[str, tuple[Cell, Cell]] = {}
        self.handover_waiting: set[str] = set()
        self._robots_cache: list[Robot] = []
        self._tick_cache = 0
        self._force_replan: set[str] = set()

    def register_handover_zone(self, zone_id: str, slot_a: Cell, slot_b: Cell) -> None:
        """登记交接区:两个相邻、可通行的占用槽,沿用既有预约规则。"""
        if not (self.gmap.passable(slot_a) and self.gmap.passable(slot_b)):
            raise CoordinationError("交接槽不可通行")
        if abs(slot_a[0] - slot_b[0]) + abs(slot_a[1] - slot_b[1]) != 1:
            raise CoordinationError("交接槽必须相邻")
        self.handover_zones[zone_id] = (slot_a, slot_b)

    def create_handover(
        self,
        handover_id: str,
        cargo_id: str,
        carrier_id: str,
        receiver_id: str,
        zone_id: str,
        carrier_task_id: str,
        receiver_task_id: str,
        delivery_goal: Cell,
        delivery_task_id: str,
        tick: int,
        priority: int = 0,
        deadline: int | None = None,
    ) -> HandoverCommitment:
        cargo = self.cargo_registry.cargo.get(cargo_id)
        if cargo is None:
            raise CoordinationError(f"运输物 {cargo_id} 未登记")
        if cargo.owner != carrier_id:
            raise CoordinationError("只有当前归属机器人能作为交出方")
        if handover_id in self.cargo_registry.commitments:
            raise CoordinationError(f"交接 {handover_id} 已存在")
        if zone_id not in self.handover_zones:
            raise CoordinationError(f"交接区 {zone_id} 未登记")
        slot_a, slot_b = self.handover_zones[zone_id]
        commitment = HandoverCommitment(
            handover_id=handover_id,
            cargo_id=cargo_id,
            carrier=carrier_id,
            receiver=receiver_id,
            rendezvous=slot_a,
            carrier_slot=slot_a,
            receiver_slot=slot_b,
            carrier_task=carrier_task_id,
            receiver_task=receiver_task_id,
            delivery_task=delivery_task_id,
            created_tick=tick,
            alternatives=[],
        )
        commitment.delivery_goal = delivery_goal  # type: ignore[attr-defined]
        commitment.delivery_priority = priority
        commitment.delivery_deadline = deadline
        commitment.zone_id = zone_id
        self.cargo_registry.commitments[handover_id] = commitment
        return commitment

    def get_commitment(self, handover_id: str) -> HandoverCommitment:
        return self.cargo_registry.commitments[handover_id]

    def register_cargo(self, cargo_id: str, owner: str) -> Cargo:
        cargo = Cargo(cargo_id, owner)
        self.cargo_registry.add_cargo(cargo)
        return cargo

    def _handover_of_task(self, task_id: str) -> HandoverCommitment | None:
        for commitment in self.cargo_registry.commitments.values():
            if task_id in (commitment.carrier_task, commitment.receiver_task):
                return commitment
        return None

    def _handover_robot(self, robots: list[Robot], robot_id: str) -> Robot | None:
        return next((r for r in robots if r.robot_id == robot_id), None)

    def mark_leg_started(self, robots: list[Robot]) -> None:
        """机器人沿交接段离开起点格即视为承诺开始,此后不可撤回。"""
        for robot in robots:
            if not robot.handover_id or robot.leg_start_cell is None:
                continue
            commitment = self.cargo_registry.commitments.get(robot.handover_id)
            if commitment is None or commitment.state != HandoverState.PROMISED:
                continue
            if robot.pos != robot.leg_start_cell:
                commitment.state = HandoverState.STARTED
                commitment.started_tick = self._tick_cache

    def is_handover_arrival(self, robot: Robot) -> bool:
        """机器人到达的是自己的交接槽而非普通任务终点:交接前不能记 done。"""
        if not robot.handover_id or robot.task is None:
            return False
        if robot.robot_id in self.handover_waiting:
            return False
        commitment = self.cargo_registry.commitments.get(robot.handover_id)
        if commitment is None or commitment.state == HandoverState.COMPLETED:
            return False
        slot = commitment.slot_of(robot.robot_id)
        return slot is not None and robot.pos == slot and robot.task.goal == slot

    def handover_arrival(self, robot: Robot, tick: int) -> HandoverCommitment | None:
        """机器人完成交接段到达自己的交接槽:停下占用,等待另一方。"""
        if not robot.handover_id or not robot.task:
            return None
        commitment = self.cargo_registry.commitments.get(robot.handover_id)
        if commitment is None:
            return None
        slot = commitment.slot_of(robot.robot_id)
        if slot is None or robot.pos != slot or robot.task.goal != slot:
            return None
        if robot.robot_id == commitment.carrier:
            commitment.carrier_arrived = True
        else:
            commitment.receiver_arrived = True
        if commitment.state == HandoverState.PROMISED:
            commitment.state = HandoverState.STARTED
            commitment.started_tick = tick
        commitment.state = HandoverState.IN_PROGRESS
        self.handover_waiting.add(robot.robot_id)
        robot.status = RobotStatus.WAITING
        robot.last_reason = "await_handover"
        robot.last_blockers = [commitment.other(robot.robot_id)]
        self._release_future(robot, tick)
        robot.path = [robot.pos]
        robot.plan_start = tick
        self.reservations.reserve_hold(robot.robot_id, robot.pos)
        self.reservations.reserve_path(robot.robot_id, [robot.pos], tick, reserve_goal=True)
        return commitment

    def complete_handovers(self, robots: list[Robot], tick: int) -> list[HandoverCommitment]:
        """双方都实际占据交接槽才转交归属;此前物体只属于交出方。"""
        completed: list[HandoverCommitment] = []
        for commitment in list(self.cargo_registry.commitments.values()):
            if commitment.state not in (HandoverState.STARTED, HandoverState.IN_PROGRESS):
                continue
            carrier = self._handover_robot(robots, commitment.carrier)
            receiver = self._handover_robot(robots, commitment.receiver)
            if carrier is None or receiver is None:
                continue
            if carrier.pos != commitment.carrier_slot or receiver.pos != commitment.receiver_slot:
                continue
            if carrier.status != RobotStatus.WAITING or receiver.status != RobotStatus.WAITING:
                continue
            if carrier.task is None or carrier.task.task_id != commitment.carrier_task:
                continue
            if receiver.task is None or receiver.task.task_id != commitment.receiver_task:
                continue
            commitment.state = HandoverState.COMPLETED
            commitment.completed_tick = tick
            commitment.end_tick = tick
            self.cargo_registry.transfer(commitment.cargo_id, commitment.receiver, tick)
            self.handover_waiting -= commitment.participants()
            for robot, task_id in (
                (carrier, commitment.carrier_task),
                (receiver, commitment.receiver_task),
            ):
                if robot.task and robot.task.task_id == task_id:
                    robot.task.terminal_reason = "handover_complete"
            self._finish_handover_legs(carrier, receiver, commitment, tick)
            completed.append(commitment)
        return completed

    def fail_broken_handovers(self, robots: list[Robot], tick: int) -> list[HandoverCommitment]:
        """延迟之外的硬故障:参与方 FAILED 立即终止交接,归属保持不变。"""
        broken: list[HandoverCommitment] = []
        for commitment in list(self.cargo_registry.commitments.values()):
            if commitment.state in (HandoverState.COMPLETED, HandoverState.FAILED, HandoverState.CANCELLED):
                continue
            for robot_id in commitment.participants():
                robot = self._handover_robot(robots, robot_id)
                if robot is not None and robot.status == RobotStatus.FAILED:
                    reason = robot.last_reason or "robot_failed"
                    self._fail_handover_leg(robot, tick, reason)
                    broken.append(commitment)
                    break
        return broken

    def failed_commitments(self) -> list[HandoverCommitment]:
        return [
            commitment
            for commitment in self.cargo_registry.commitments.values()
            if commitment.state == HandoverState.FAILED
        ]

    def _finish_handover_legs(
        self,
        carrier: Robot,
        receiver: Robot,
        commitment: HandoverCommitment,
        tick: int,
    ) -> None:
        carrier.task = None
        carrier.handover_id = None
        carrier.handover_role = None
        carrier.leg_start_cell = None
        carrier.path = [carrier.pos]
        carrier.plan_start = tick
        carrier.status = RobotStatus.IDLE
        carrier.last_blockers = []
        carrier.last_reason = None
        self.reservations.release_hold(carrier.robot_id)
        self.reservations.goal = {
            cell: owner for cell, owner in self.reservations.goal.items()
            if owner != carrier.robot_id
        }
        self.reservations.reserve_hold(carrier.robot_id, carrier.pos)

    def cancel_handover(
        self, robots: list[Robot], handover_id: str, tick: int, reason: str = "cancelled"
    ) -> HandoverCommitment:
        """只允许取消尚未开始的承诺;物体归属保持不变。"""
        commitment = self.get_commitment(handover_id)
        if commitment.state != HandoverState.PROMISED:
            raise CoordinationError("交接已开始,不能撤回,只能等待/撤离/改换方案绕行")
        commitment.state = HandoverState.CANCELLED
        commitment.end_tick = tick
        commitment.reason = reason
        for robot_id, task_id in (
            (commitment.carrier, commitment.carrier_task),
            (commitment.receiver, commitment.receiver_task),
        ):
            self.terminal_tasks[task_id] = {
                "robot": robot_id,
                "reason": "handover_cancelled",
                "blockers": ["handover_cancelled"],
                "message": f"交接 {handover_id} 在开始前取消,{task_id} 不再执行",
                "tick": tick,
                "handover": handover_id,
            }
        for robot_id in commitment.participants():
            self._force_replan.add(robot_id)
            robot = self._handover_robot(robots, robot_id)
            if robot is None:
                continue
            self._release_future(robot, tick)
            self.reservations.release_hold(robot.robot_id)
            robot.task = None
            robot.handover_id = None
            robot.handover_role = None
            robot.leg_start_cell = None
            robot.path = []
            robot.status = RobotStatus.IDLE
            robot.last_reason = "handover_cancelled"
        return commitment

    def repoint_handover(
        self,
        robots: list[Robot],
        handover_id: str,
        new_zone_id: str,
        tick: int,
    ) -> HandoverCommitment:
        """未开始的交接可改换交接区;已执行路径与预约保留在日志中,不被重写。"""
        commitment = self.get_commitment(handover_id)
        if commitment.state != HandoverState.PROMISED:
            raise CoordinationError("交接已开始,不能改换交接点")
        if new_zone_id not in self.handover_zones:
            raise CoordinationError(f"交接区 {new_zone_id} 未登记")
        commitment.alternatives.append(commitment.carrier_slot)
        slot_a, slot_b = self.handover_zones[new_zone_id]
        self.reservations.goal = {
            cell: owner
            for cell, owner in self.reservations.goal.items()
            if owner not in commitment.participants()
        }
        self.reservations.goal_ready = {
            cell: ready for cell, ready in self.reservations.goal_ready.items()
            if cell in self.reservations.goal
        }
        for robot_id in commitment.participants():
            self._force_replan.add(robot_id)
            robot = self._handover_robot(robots, robot_id)
            if robot is not None:
                self.reservations.vertex = {
                    key: owner
                    for key, owner in self.reservations.vertex.items()
                    if not (owner == robot_id and key[1] > tick)
                }
                self.reservations.edge = {
                    key: owner
                    for key, owner in self.reservations.edge.items()
                    if not (owner == robot_id and key[2] > tick)
                }
        commitment.zone_id = new_zone_id
        commitment.carrier_slot = slot_a
        commitment.receiver_slot = slot_b
        commitment.rendezvous = slot_a
        for robot_id in commitment.participants():
            robot = self._handover_robot(robots, robot_id)
            if robot is None or robot.task is None:
                continue
            robot.task.goal = commitment.slot_of(robot_id)  # type: ignore[assignment]
            robot.leg_start_cell = robot.pos
        return commitment

    def commitment_policy(self, handover_id: str) -> dict[str, object]:
        """紧急插入时给出承诺可调整性。"""
        commitment = self.get_commitment(handover_id)
        adjustable = commitment.state == HandoverState.PROMISED
        return {
            "handover": handover_id,
            "state": commitment.state.value,
            "adjustable": adjustable,
            "revocable": adjustable,
            "repointable": adjustable,
            "actions": (
                ["wait", "evacuate", "repoint", "cancel"]
                if adjustable
                else ["wait", "evacuate"]
            ),
        }

    def party_options(
        self,
        robots: list[Robot],
        handover_id: str,
        robot_id: str,
        tick: int,
        evac_cell: Cell | None = None,
    ) -> list[HandoverOption]:
        commitment = self.get_commitment(handover_id)
        if robot_id not in commitment.participants():
            raise CoordinationError("机器人不是该交接的参与方")
        options: list[HandoverOption] = []
        started = commitment.state != HandoverState.PROMISED
        peer = commitment.other(robot_id)
        options.append(HandoverOption(
            kind="wait",
            feasible=True,
            detail=f"占用自己的交接槽等待 {peer};归属保持不变",
        ))
        evac_ok = False
        evac_detail = "未提供撤离格"
        if evac_cell is not None:
            robot = self._handover_robot(robots, robot_id)
            if robot is not None:
                result = plan_path_result(
                    self.gmap, self.reservations, robot.pos, evac_cell, tick,
                    self.horizon, robot_id=robot_id,
                )
                evac_ok = result.found
                evac_detail = "可撤离,撤离前预约保留" if result.found else f"撤离受阻:{sorted(result.blockers)}"
        options.append(HandoverOption(
            kind="evacuate",
            feasible=evac_ok,
            detail=evac_detail,
            rendezvous=evac_cell,
        ))
        for zone_id, slots in self.handover_zones.items():
            if zone_id == commitment.zone_id:
                continue
            feasible = not started
            options.append(HandoverOption(
                kind="repoint",
                feasible=feasible,
                rendezvous=slots[0],
                detail=(
                    f"改换到交接区 {zone_id},原路径/预约保留不重写"
                    if feasible
                    else f"交接已开始,不能改换交接区 {zone_id}"
                ),
            ))
        if not any(option.kind == "repoint" for option in options):
            options.append(HandoverOption(
                kind="repoint",
                feasible=False,
                detail="交接已开始且没有可用替代交接区",
            ))
        return options

    def compare_options(
        self,
        robots: list[Robot],
        handover_id: str,
        candidate_zones: list[str] | None = None,
    ) -> list[dict[str, object]]:
        """比较原交接与替代交接:在预约表副本上 dry-run,绝不改动在途预约。"""
        commitment = self.get_commitment(handover_id)
        carrier = self._handover_robot(robots, commitment.carrier)
        receiver = self._handover_robot(robots, commitment.receiver)
        if carrier is None or receiver is None:
            return []
        zones = candidate_zones or [
            zone_id for zone_id in self.handover_zones if zone_id != commitment.zone_id
        ]
        entries: list[dict[str, object]] = []
        entries.append(self._dry_run_zone(robots, commitment, commitment.zone_id, original=True))
        for zone_id in zones:
            entries.append(self._dry_run_zone(robots, commitment, zone_id, original=False))
        return entries

    def _dry_run_zone(
        self,
        robots: list[Robot],
        commitment: HandoverCommitment,
        zone_id: str,
        original: bool,
    ) -> dict[str, object]:
        slot_a, slot_b = self.handover_zones[zone_id]
        carrier = self._handover_robot(robots, commitment.carrier)
        receiver = self._handover_robot(robots, commitment.receiver)
        assert carrier is not None and receiver is not None
        scratch = deepcopy(self.reservations)
        tick = self._tick_cache
        participants = {commitment.carrier, commitment.receiver}
        scratch.vertex = {
            key: owner for key, owner in scratch.vertex.items()
            if not (owner in participants and key[1] > tick)
        }
        scratch.edge = {
            key: owner for key, owner in scratch.edge.items()
            if not (owner in participants and key[2] > tick)
        }
        scratch.goal = {
            cell: owner for cell, owner in scratch.goal.items() if owner not in participants
        }
        scratch.goal_ready = {cell: t for cell, t in scratch.goal_ready.items() if cell in scratch.goal}
        carrier_leg = plan_path_result(
            self.gmap, scratch, carrier.pos, slot_a, tick, self.horizon,
            robot_id=carrier.robot_id,
        )
        receiver_leg = None
        delivery = None
        if carrier_leg.found:
            scratch.reserve_path(carrier.robot_id, carrier_leg.path, tick, reserve_goal=True)
            receiver_leg = plan_path_result(
                self.gmap, scratch, receiver.pos, slot_b, tick, self.horizon,
                robot_id=receiver.robot_id,
            )
        if carrier_leg.found and receiver_leg and receiver_leg.found:
            meet_tick = max(tick + len(carrier_leg.path) - 1, tick + len(receiver_leg.path) - 1)
            scratch.reserve_path(receiver.robot_id, receiver_leg.path, tick, reserve_goal=True)
            if commitment.delivery_goal is not None:
                delivery = plan_path_result(
                    self.gmap, scratch, slot_b, commitment.delivery_goal, meet_tick,
                    self.horizon, robot_id=receiver.robot_id,
                )
        feasible = bool(
            carrier_leg.found
            and receiver_leg is not None and receiver_leg.found
            and (delivery is None or delivery.found)
        )
        return {
            "zone": zone_id,
            "original": original,
            "feasible": feasible,
            "carrier_slot": slot_a,
            "receiver_slot": slot_b,
            "carrier_eta": (len(carrier_leg.path) - 1 if carrier_leg.found else None),
            "receiver_eta": (len(receiver_leg.path) - 1 if receiver_leg and receiver_leg.found else None),
            "delivery_found": delivery.found if delivery is not None else None,
            "carrier_blockers": sorted(carrier_leg.blockers),
            "receiver_blockers": sorted(receiver_leg.blockers) if receiver_leg else [],
        }

    def _record_decision(self, robot: Robot, action: str, tick: int, result: PlanResult) -> None:
        self.decisions.append({
            "robot": robot.robot_id,
            "task": robot.task.task_id if robot.task else None,
            "action": action,
            "tick": tick,
            "reason": result.status,
            "blockers": sorted(result.blockers),
        })

    def _terminate(self, robot: Robot, result: PlanResult, tick: int, message: str) -> None:
        robot.status = RobotStatus.FAILED
        robot.last_reason = result.status
        robot.last_blockers = sorted(result.blockers) or self._local_blockers(robot, tick)
        robot.wait_ticks = 0
        if robot.task:
            robot.task.terminal_reason = result.status
            robot.task.on_time = False if robot.task.deadline is not None else None
            self.terminal_tasks[robot.task.task_id] = {
                "robot": robot.robot_id,
                "reason": result.status,
                "blockers": robot.last_blockers,
                "message": message,
                "tick": tick,
            }
            self._record_decision(robot, "terminate", tick, result)
            self.evacuating_tasks.discard(robot.task.task_id)
            self._fail_handover_leg(robot, tick, result.status)

    def _fail_handover_leg(self, robot: Robot, tick: int, reason: str) -> None:
        """交接一方终止:交接失败,运输物归属保持在当前持有者,不复活任何段。"""
        commitment = self.cargo_registry.active_commitment_of(robot.robot_id)
        if commitment is None:
            return
        if commitment.state == HandoverState.COMPLETED:
            return
        commitment.state = HandoverState.FAILED
        commitment.end_tick = tick
        commitment.reason = reason
        self.handover_waiting -= commitment.participants()
        cargo = self.cargo_registry.cargo[commitment.cargo_id]
        cargo.terminal = True
        other_id = commitment.other(robot.robot_id)
        for robot_id, task_id, role in (
            (commitment.carrier, commitment.carrier_task, "carrier"),
            (commitment.receiver, commitment.receiver_task, "receiver"),
        ):
            self.terminal_tasks.setdefault(task_id, {
                "robot": robot_id,
                "reason": "handover_failed",
                "blockers": [reason],
                "message": f"交接 {commitment.handover_id} 因 {reason} 失败,{role} 段终止",
                "tick": tick,
                "handover": commitment.handover_id,
            })
        if other_id is not None:
            other = next((item for item in self._robots_cache if item.robot_id == other_id), None)
            if other is not None and other.task is not None:
                other.status = RobotStatus.FAILED
                other.task.terminal_reason = "handover_failed"
                other.last_reason = "handover_failed"
                other.last_blockers = [reason]
                self.handover_waiting.discard(other_id)

    def _restore_terminals(self, robots: list[Robot], tick: int) -> None:
        for robot in robots:
            if robot.task and robot.task.task_id in self.terminal_tasks:
                record = self.terminal_tasks[robot.task.task_id]
                result = PlanResult(
                    str(record["reason"]),
                    blockers=set(record.get("blockers", [])),
                )
                self._terminate(robot, result, tick, str(record["message"]))

    def load_terminal_decisions(self, log_path: str | Path) -> None:
        path = Path(log_path)
        if not path.exists():
            return
        for line in path.read_text().splitlines():
            rec = json.loads(line)
            if rec.get("event") == "task_terminated":
                task_id = str(rec["task"])
                reason = str(rec.get("reason", INFEASIBLE))
                self.terminal_tasks[task_id] = {
                    "robot": rec.get("robot"),
                    "reason": reason,
                    "blockers": list(rec.get("blockers") or []),
                    "message": "任务已在历史调度记录中判定不可行",
                    "tick": int(rec.get("tick", 0)),
                }

    def close_corridor(self, cells: set[Cell] | list[Cell]) -> None:
        cells = {cell for cell in cells if self.gmap.passable(cell)}
        if not cells:
            return
        self.closures.append(Closure(set(cells)))
        self.closed_cells.update(cells)
        self.gmap.close_cells(cells)

    def open_corridor(self, cells: set[Cell] | list[Cell]) -> None:
        wanted = set(cells)
        remaining: list[Closure] = []
        for closure in self.closures:
            left = closure.cells - wanted
            if left:
                closure.cells = left
                remaining.append(closure)
        self.closures = remaining
        self.closed_cells = set().union(*(c.cells for c in self.closures)) if self.closures else set()
        self.gmap.closed_cells = set(self.closed_cells)
        self.exiting = {}
        self.evacuating_tasks.clear()

    def plan_all(self, robots: list[Robot], start_tick: int, strict: bool = True) -> None:
        self.decisions = []
        self._robots_cache = list(robots)
        self._tick_cache = start_tick
        self._restore_terminals(robots, start_tick)
        self._refresh_base_holds(robots, start_tick)
        self._clear_replanned_futures(robots, start_tick)
        affected = self._closure_affected(robots, start_tick)

        for robot in affected:
            if robot.task:
                self.evacuating_tasks.add(robot.task.task_id)
            self._release_future(robot, start_tick)
            exit_cell = self._nearest_exit(robot.pos)
            if exit_cell is None:
                result = PlanResult(INFEASIBLE, blockers={"closed"})
                self._terminate(robot, result, start_tick, "封闭通道无出口")
                if strict:
                    raise CoordinationError("机器人被困在封闭通道中", {robot.robot_id: result})
                continue
            if robot.pos != exit_cell:
                self.exiting[robot.robot_id] = exit_cell
            else:
                self.exiting.pop(robot.robot_id, None)
            result = plan_path_result(
                self.gmap,
                self.reservations,
                robot.pos,
                exit_cell,
                start_tick,
                self.horizon,
                robot_id=robot.robot_id,
                allowed_closed=self.closed_cells if robot.pos in self.closed_cells else None,
            )
            if not result.found:
                robot.path = [robot.pos]
                robot.plan_start = start_tick
                robot.last_blockers = sorted(result.blockers) or ["closed"]
                robot.status = RobotStatus.WAITING
                robot.last_reason = result.status
                self.reservations.reserve_path(robot.robot_id, robot.path, start_tick, reserve_goal=False)
                self._record_decision(robot, "wait", start_tick, result)
                if strict:
                    raise CoordinationError("封闭通道内机器人无退出安排", {robot.robot_id: result})
            else:
                robot.path = result.path
                robot.plan_start = start_tick
                robot.status = RobotStatus.WAITING
                robot.last_blockers = []
                robot.last_reason = None
                self.reservations.reserve_path(robot.robot_id, result.path, start_tick, reserve_goal=False)
                self._record_decision(robot, "exit", start_tick, result)

        self._plan_handover_legs(robots, start_tick)
        candidates = self._candidates(robots, affected, strict, start_tick)
        ordered = sorted(
            candidates,
            key=lambda r: (
                0 if self.pending.intersection({r.robot_id}) else 1,
                -(r.task.priority if r.task else 0),
                r.task.deadline if r.task and r.task.deadline is not None else 1 << 30,
                -r.total_wait,
                r.robot_id,
            ),
        )
        attempts = self._candidate_orders(ordered)
        last_results: dict[str, PlanResult] = {}
        for attempt in attempts:
            planned: list[Robot] = []
            results: dict[str, PlanResult] = {}
            for robot in attempt:
                result = plan_path_result(
                    self.gmap,
                    self.reservations,
                    robot.pos,
                    robot.goal,
                    start_tick,
                    self.horizon,
                    robot_id=robot.robot_id,
                    deadline_tick=robot.task.deadline if robot.task else None,
                )
                results[robot.robot_id] = result
                if not result.found:
                    for planned_robot in planned:
                        self._release_future(planned_robot, start_tick)
                        self.reservations.reserve_path(
                            planned_robot.robot_id, planned_robot.path, start_tick,
                        )
                    break
                self._release_future(robot, start_tick)
                self.reservations.release_hold(robot.robot_id)
                robot.path = result.path
                robot.plan_start = start_tick
                robot.status = RobotStatus.MOVING
                robot.last_blockers = []
                robot.last_reason = None
                self.pending.discard(robot.robot_id)
                self.reservations.reserve_path(robot.robot_id, result.path, start_tick)
                self._record_decision(robot, "continue", start_tick, result)
                planned.append(robot)
            if len(planned) == len(attempt):
                return
            last_results = results
        for robot in ordered:
            result = last_results.get(robot.robot_id, PlanResult(SEARCH_LIMIT))
            self._release_future(robot, start_tick)
            existing_end = robot.path[-1] if robot.path else robot.pos
            robot.path = [robot.pos]
            robot.plan_start = start_tick
            blockers = sorted(result.blockers) or self._local_blockers(robot, start_tick)
            if result.status in (INFEASIBLE, DEADLINE_INFEASIBLE):
                self._terminate(
                    robot, result, start_tick,
                    "无法按时完成" if result.status == DEADLINE_INFEASIBLE else "任务不可行",
                )
                continue
            concrete_blockers = bool(blockers and blockers != [SEARCH_LIMIT])
            robot.status = RobotStatus.WAITING if concrete_blockers or result.status != SEARCH_LIMIT else RobotStatus.SEARCH_LIMIT
            robot.last_blockers = blockers
            robot.last_reason = result.status
            self.reservations.reserve_path(
                robot.robot_id, robot.path, start_tick,
                reserve_goal=(existing_end == robot.goal),
            )
            self._record_decision(
                robot,
                "search_pending" if robot.status == RobotStatus.SEARCH_LIMIT else "wait",
                start_tick,
                result,
            )
        if strict:
            statuses = {item.status for item in last_results.values()}
            message = "无可行安排" if statuses == {INFEASIBLE} else "尚未找到可行安排"
            raise CoordinationError(message, last_results)

    def _replan_exiting(self, robot: Robot, tick: int) -> None:
        exit_cell = self.exiting.get(robot.robot_id)
        if exit_cell is None:
            return
        result = plan_path_result(
            self.gmap,
            self.reservations,
            robot.pos,
            exit_cell,
            tick,
            self.horizon,
            robot_id=robot.robot_id,
            allowed_closed=self.closed_cells if robot.pos in self.closed_cells else None,
        )
        if result.found:
            self._release_future(robot, tick)
            robot.path = result.path
            robot.plan_start = tick
            robot.status = RobotStatus.WAITING
            robot.last_reason = None
            robot.last_blockers = []
            self.reservations.reserve_path(robot.robot_id, result.path, tick, reserve_goal=False)
        else:
            robot.path = [robot.pos]
            robot.plan_start = tick
            robot.status = RobotStatus.WAITING
            robot.last_reason = result.status
            robot.last_blockers = sorted(result.blockers) or ["closed"]
            self.reservations.reserve_path(robot.robot_id, robot.path, tick, reserve_goal=False)

    def _plan_handover_legs(self, robots: list[Robot], start_tick: int) -> None:
        """交接段独立规划:一方受阻不影响另一方继续前往自己的交接槽。"""
        for robot in robots:
            if not robot.handover_id or robot.task is None:
                continue
            if robot.status in (RobotStatus.DONE, RobotStatus.PAUSED, RobotStatus.FAILED):
                continue
            if robot.robot_id in self.handover_waiting or robot.robot_id in self.exiting:
                continue
            commitment = self.cargo_registry.commitments[robot.handover_id]
            if commitment.state in (HandoverState.COMPLETED, HandoverState.FAILED, HandoverState.CANCELLED):
                continue
            slot = commitment.slot_of(robot.robot_id)
            if slot is None or robot.task.goal != slot:
                continue
            if (
                robot.robot_id not in self._force_replan
                and
                robot.status == RobotStatus.MOVING
                and self._plan_still_valid(robot)
            ):
                continue
            result = plan_path_result(
                self.gmap,
                self.reservations,
                robot.pos,
                slot,
                start_tick,
                self.horizon,
                robot_id=robot.robot_id,
                deadline_tick=robot.task.deadline,
            )
            if result.found:
                self._release_future(robot, start_tick)
                self.reservations.release_hold(robot.robot_id)
                robot.path = result.path
                robot.plan_start = start_tick
                robot.status = RobotStatus.MOVING
                robot.last_blockers = []
                robot.last_reason = None
                self.reservations.reserve_path(robot.robot_id, result.path, start_tick)
                self._record_decision(robot, "continue", start_tick, result)
                self._force_replan.discard(robot.robot_id)
            else:
                self._release_future(robot, start_tick)
                robot.path = [robot.pos]
                robot.plan_start = start_tick
                robot.status = (
                    RobotStatus.WAITING
                    if result.status in (INFEASIBLE, DEADLINE_INFEASIBLE)
                    else RobotStatus.SEARCH_LIMIT
                )
                robot.last_blockers = sorted(result.blockers) or self._local_blockers(robot, start_tick)
                robot.last_reason = result.status
                self.reservations.reserve_path(robot.robot_id, robot.path, start_tick)
                self._record_decision(robot, "wait", start_tick, result)

    def _candidate_orders(self, ordered: list[Robot]):
        yield ordered
        if len(ordered) <= 1:
            return
        seen = {tuple(robot.robot_id for robot in ordered)}
        for permutation in itertools.permutations(ordered):
            key = tuple(robot.robot_id for robot in permutation)
            if key not in seen:
                seen.add(key)
                yield list(permutation)

    def _refresh_base_holds(self, robots: list[Robot], start_tick: int) -> None:
        active_ids = {r.robot_id for r in robots}
        self.reservations.vertex = {
            key: owner for key, owner in self.reservations.vertex.items()
            if key[1] >= start_tick and owner in active_ids
        }
        self.reservations.edge = {
            key: owner for key, owner in self.reservations.edge.items()
            if key[2] >= start_tick and owner in active_ids
        }
        self.reservations.goal = {
            cell: owner for cell, owner in self.reservations.goal.items() if owner in active_ids
        }
        self.reservations.holds = {
            cell: owner for cell, owner in self.reservations.holds.items() if owner in active_ids
        }
        self.reservations.permanent = {
            cell: owner for cell, owner in self.reservations.permanent.items() if owner in active_ids
        }
        for robot in robots:
            if robot.status == RobotStatus.DONE:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
                self.reservations.reserve_permanent(robot.robot_id, robot.pos)
            elif robot.status == RobotStatus.FAILED:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
            elif robot.status == RobotStatus.PAUSED:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
                self.reservations.vertex = {
                    key: owner for key, owner in self.reservations.vertex.items()
                    if not (owner == robot.robot_id and key[1] > start_tick)
                }
                self.reservations.edge = {
                    key: owner for key, owner in self.reservations.edge.items()
                    if not (owner == robot.robot_id and key[2] > start_tick)
                }
                self.reservations.goal = {
                    cell: owner for cell, owner in self.reservations.goal.items()
                    if owner != robot.robot_id
                }
            elif not robot.task:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
            elif robot.robot_id in self.handover_waiting:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
            elif robot.path and robot.plan_start <= start_tick:
                idx = start_tick - robot.plan_start
                if idx < len(robot.path):
                    self.reservations.vertex[(robot.path[idx], start_tick)] = robot.robot_id

    def _clear_replanned_futures(self, robots: list[Robot], start_tick: int) -> None:
        for robot in robots:
            if robot.status == RobotStatus.DONE or robot.status == RobotStatus.PAUSED:
                continue
            if robot.robot_id in self.handover_waiting:
                continue
            should_clear = robot.status in (RobotStatus.IDLE, RobotStatus.WAITING, RobotStatus.FAILED)
            if robot.robot_id in self.exiting:
                should_clear = False
            if should_clear:
                self.reservations.goal = {
                    cell: owner
                    for cell, owner in self.reservations.goal.items()
                    if owner != robot.robot_id
                }
                self._release_future(robot, start_tick)

    def _closure_affected(self, robots: list[Robot], start_tick: int) -> list[Robot]:
        if not self.closed_cells:
            for robot in robots:
                if robot.robot_id in self.exiting:
                    if robot.pos == self.exiting.pop(robot.robot_id):
                        robot.status = RobotStatus.IDLE
            return []
        affected: list[Robot] = []
        for robot in robots:
            if robot.status in (RobotStatus.DONE, RobotStatus.FAILED) or not robot.path:
                continue
            if robot.pos in self.closed_cells and robot.status != RobotStatus.PAUSED:
                affected.append(robot)
        return affected

    def _candidates(self, robots: list[Robot], affected: list[Robot], strict: bool, start_tick: int) -> list[Robot]:
        affected_ids = {r.robot_id for r in affected}
        out: list[Robot] = []
        for robot in robots:
            if not robot.goal or robot.status in (RobotStatus.DONE, RobotStatus.PAUSED, RobotStatus.FAILED):
                continue
            if robot.robot_id in self.handover_waiting:
                continue
            if robot.handover_id and robot.task is not None:
                commitment = self.cargo_registry.commitments.get(robot.handover_id)
                if (
                    commitment is not None
                    and commitment.state in (HandoverState.PROMISED, HandoverState.STARTED, HandoverState.IN_PROGRESS)
                    and robot.task.goal == commitment.slot_of(robot.robot_id)
                ):
                    continue
            if robot.robot_id in self.exiting:
                continue
            if robot.task and robot.task.task_id in self.evacuating_tasks:
                exit_cell = self.exiting.get(robot.robot_id)
                if exit_cell is None or robot.pos != exit_cell:
                    continue
                self.evacuating_tasks.discard(robot.task.task_id)
            if robot.goal in self.closed_cells:
                self._release_future(robot, start_tick)
                robot.path = [robot.pos]
                robot.plan_start = start_tick
                robot.status = RobotStatus.WAITING
                robot.last_reason = "closed_goal"
                robot.last_blockers = ["closed"]
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
                self._record_decision(
                    robot,
                    "wait",
                    start_tick,
                    PlanResult("closed_goal", blockers={"closed"}),
                )
                continue
            if strict and robot.status in (RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT) and robot.path == [robot.pos] and robot.total_wait >= 2:
                continue
            if robot.status == RobotStatus.MOVING and robot.robot_id not in affected_ids and self._plan_still_valid(robot):
                continue
            out.append(robot)
        return out

    def _plan_still_valid(self, robot: Robot) -> bool:
        if not robot.path:
            return False
        for offset, cell in enumerate(robot.path):
            tick = robot.plan_start + offset
            if tick < 0:
                continue
            if not self.gmap.passable(cell):
                return False
            if self.reservations.holds.get(cell) not in (None, robot.robot_id):
                return False
            owner = self.reservations.vertex.get((cell, tick))
            if owner is not None and owner != robot.robot_id:
                return False
        return True

    def reconcile(self, robots: list[Robot], tick: int) -> list[Robot]:
        reached: list[Robot] = []
        for robot in robots:
            exit_cell = self.exiting.get(robot.robot_id)
            if exit_cell is not None and robot.pos == exit_cell:
                self.exiting.pop(robot.robot_id, None)
                reached.append(robot)
        for robot in reached:
            robot.status = RobotStatus.IDLE
            if robot.task:
                self.evacuating_tasks.discard(robot.task.task_id)
        if reached:
            self._refresh_base_holds(robots, tick)
            self._plan_waiting(robots, tick)
        return reached

    def _plan_waiting(self, robots: list[Robot], tick: int) -> None:
        candidates = [
            robot for robot in robots
            if robot.goal
            and robot.status in (RobotStatus.IDLE, RobotStatus.WAITING)
            and robot.robot_id not in self.exiting
            and robot.robot_id not in self.handover_waiting
        ]
        ordered = sorted(
            candidates,
            key=lambda r: (
                -(r.task.priority if r.task else 0),
                r.task.deadline if r.task and r.task.deadline is not None else 1 << 30,
                -r.total_wait,
                r.robot_id,
            ),
        )
        for robot in ordered:
            result = plan_path_result(
                self.gmap,
                self.reservations,
                robot.pos,
                robot.goal,
                tick,
                self.horizon,
                robot_id=robot.robot_id,
                deadline_tick=robot.task.deadline if robot.task else None,
            )
            if result.found:
                self._release_future(robot, tick)
                robot.path = result.path
                robot.plan_start = tick
                robot.status = RobotStatus.MOVING
                robot.last_blockers = []
                self.reservations.reserve_path(robot.robot_id, result.path, tick)
            else:
                self._release_future(robot, tick)
                robot.path = [robot.pos]
                robot.plan_start = tick
                robot.status = RobotStatus.WAITING
                robot.last_blockers = sorted(result.blockers) or self._local_blockers(robot, tick)
                self.reservations.reserve_path(robot.robot_id, robot.path, tick)

    def _release_future(self, robot: Robot, start_tick: int) -> None:
        self.reservations.vertex = {
            key: owner for key, owner in self.reservations.vertex.items()
            if not (owner == robot.robot_id and key[1] >= start_tick)
        }
        self.reservations.edge = {
            key: owner for key, owner in self.reservations.edge.items()
            if not (owner == robot.robot_id and key[2] >= start_tick)
        }
        self.reservations.goal = {
            cell: owner for cell, owner in self.reservations.goal.items() if owner != robot.robot_id
        }

    def _local_blockers(self, robot: Robot, start_tick: int) -> list[str]:
        blockers: set[str] = set()
        for nxt in self.gmap.neighbors(robot.pos) + [robot.pos]:
            blockers.update(self.reservations.blockers(nxt, start_tick + 1, robot.pos, robot.robot_id))
        return sorted(blockers) or ([SEARCH_LIMIT] if not blockers else [])

    def _nearest_exit(self, start: Cell) -> Cell | None:
        if start not in self.closed_cells:
            return start
        pending = [(start, 0)]
        seen = {start}
        best: tuple[int, Cell] | None = None
        while pending:
            cell, dist = pending.pop(0)
            if cell not in self.closed_cells:
                candidate = (dist, cell)
                if best is None or candidate < best:
                    best = candidate
                continue
            for nxt in self._physical_neighbors(cell):
                if nxt not in seen:
                    seen.add(nxt)
                    pending.append((nxt, dist + 1))
        return best[1] if best else None

    def _physical_neighbors(self, cell: Cell) -> list[Cell]:
        x, y = cell
        out = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (x + dx, y + dy)
            if self.gmap.in_bounds(nxt) and nxt not in self.gmap.obstacles:
                out.append(nxt)
        return out
