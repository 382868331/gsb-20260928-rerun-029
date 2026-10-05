"""执行器:按 tick 推进,记录调度、封路与实际移动日志."""

from __future__ import annotations

import json
from pathlib import Path

from .coordinator import Coordinator
from .handoff import BOTH_PRESENT, CANCELLED, COMPLETED, PLANNED, BLOCKED, Handoff
from .planner import DEADLINE_INFEASIBLE, PlanResult, plan_path_result
from .relay import (
    ACTIVE as RELAY_ACTIVE,
    BLOCKED as RELAY_BLOCKED,
    CANCELLED as RELAY_CANCELLED,
    COMPLETED as RELAY_COMPLETED,
    HANDOFF as RELAY_HANDOFF,
    PENDING as RELAY_PENDING,
    STAGE as RELAY_STAGE,
    Relay,
    RelayLeg,
    RelayTransfer,
    StagingArea,
    StagingSlot,
    slot_owner,
)
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
        self.cargo: dict[str, str] = {}
        self.handoffs: dict[str, Handoff] = {}
        self.staging: dict[str, StagingArea] = {}
        self.relays: dict[str, Relay] = {}

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
        if self.handoffs:
            active = [h for h in self.handoffs.values() if h.state not in (COMPLETED, CANCELLED)]
            if active:
                self._log("handoff_commitments", **self.commitments())
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
                    if robot.wait_ticks == self.starvation_limit:
                        self._log(
                            "starve_risk",
                            robot=robot.robot_id,
                            waited=robot.wait_ticks,
                            blockers=robot.last_blockers,
                        )
                if robot.task and robot.pos == robot.task.goal and robot.robot_id not in self.coordinator.exiting:
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
        self._update_handoffs()
        self._update_relays()
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
        self._log("positions", pos=positions)
        if any(
            r.task is not None and r.status not in (RobotStatus.DONE, RobotStatus.FAILED)
            for r in self.robots.values()
        ):
            return True
        # 双方已到位但交接尚未完成的,继续推进直到归属转交
        if any(
            ho.state in (PLANNED, BOTH_PRESENT)
            and all(self.robots[rid].status != RobotStatus.FAILED for rid in (ho.giver, ho.receiver))
            for ho in self.handoffs.values()
        ):
            return True
        # 接力仍有执行中的段时继续推进
        return any(
            leg.state == RELAY_ACTIVE
            for relay in self.relays.values()
            for leg in relay.legs
        )

    def run(self, max_ticks: int = 500) -> None:
        while self.step() and self.tick < max_ticks:
            pass

    # ---------- 交接:双方到位后转交同一件运输物 ----------

    def assign_handoff(
        self,
        handoff_id: str,
        cargo_id: str,
        giver: str,
        receiver: str,
        zone: tuple[tuple[int, int], tuple[int, int]],
        alternatives: list[tuple[tuple[int, int], tuple[int, int]]] | None = None,
        priority: int = 0,
        deadline: int | None = None,
    ) -> None:
        """登记一次交接:双方各自走普通规划进入交接区,不绕过通道预约."""
        if giver == receiver:
            raise ValueError("交接双方不能是同一机器人")
        owner = self.cargo.get(cargo_id)
        if owner is not None and owner != giver:
            raise ValueError(f"货物 {cargo_id} 当前归属 {owner},不能由 {giver} 交出")
        handoff = Handoff(
            handoff_id=handoff_id,
            cargo_id=cargo_id,
            giver=giver,
            receiver=receiver,
            zone=(tuple(zone[0]), tuple(zone[1])),
            alternatives=[(tuple(a), tuple(b)) for a, b in (alternatives or [])],
            planned_tick=self.tick,
        )
        self.handoffs[handoff_id] = handoff
        self.cargo[cargo_id] = giver
        self._log(
            "handoff_planned",
            handoff=handoff_id,
            cargo=cargo_id,
            giver=giver,
            receiver=receiver,
            zone=[list(handoff.zone[0]), list(handoff.zone[1])],
            alternatives=[[list(a), list(b)] for a, b in handoff.alternatives],
            owner=giver,
        )
        self.assign({
            giver: Task(f"{handoff_id}:deliver", handoff.zone[0], priority=priority, deadline=deadline),
            receiver: Task(f"{handoff_id}:collect", handoff.zone[1], priority=priority, deadline=deadline),
        })

    def handoff_options(self, handoff_id: str) -> dict[str, object]:
        """一方延迟/故障/取消时,另一方可等待、撤离或改换交接点的条件."""
        ho = self.handoffs[handoff_id]
        owner = self.cargo.get(ho.cargo_id, ho.giver)
        other_id = ho.receiver if owner == ho.giver else ho.giver
        other = self.robots[other_id]
        temporary = other.status in (
            RobotStatus.MOVING, RobotStatus.WAITING, RobotStatus.SEARCH_LIMIT, RobotStatus.PAUSED,
        )
        relocate = []
        for alt in ho.alternatives:
            viable = all(
                self.coordinator.gmap.passable(cell)
                and self.coordinator.reservations.permanent.get(cell) in (None, ho.giver, ho.receiver)
                for cell in alt
            )
            relocate.append({"zone": [list(alt[0]), list(alt[1])], "viable": viable})
        return {
            "owner": owner,
            "state": ho.state,
            "wait": {
                "viable": temporary,
                "condition": "对方阻塞为临时占用/暂停/在途时可等待其恢复;已故障终止则不可等待",
            },
            "withdraw": {
                "viable": not ho.started,
                "condition": "交接未开始时非持有方可撤离;持有方保留货物,归属不变",
            },
            "relocate": relocate,
        }

    def compare_handoff(self, handoff_id: str) -> list[dict[str, object]]:
        """比较原交接与替代交接方案;只读评估,不改写已执行路径和预约."""
        ho = self.handoffs[handoff_id]
        giver = self.robots[ho.giver]
        receiver = self.robots[ho.receiver]
        out = []
        for label, zone in [("original", ho.zone)] + [
            (f"alternative_{i}", alt) for i, alt in enumerate(ho.alternatives)
        ]:
            etas: dict[str, int | None] = {}
            feasible = True
            blockers: set[str] = set()
            for robot, cell in ((giver, zone[0]), (receiver, zone[1])):
                if robot.pos == cell:
                    etas[robot.robot_id] = self.tick
                    continue
                result = plan_path_result(
                    self.coordinator.gmap,
                    self.coordinator.reservations,
                    robot.pos,
                    cell,
                    self.tick,
                    self.coordinator.horizon,
                    robot_id=robot.robot_id,
                )
                if result.found:
                    etas[robot.robot_id] = self.tick + len(result.path) - 1
                else:
                    etas[robot.robot_id] = None
                    feasible = False
                    blockers.update(result.blockers)
            out.append({
                "plan": label,
                "zone": [list(zone[0]), list(zone[1])],
                "feasible": feasible,
                "eta": {rid: eta for rid, eta in etas.items()},
                "blockers": sorted(blockers),
            })
        return out

    def switch_handoff(self, handoff_id: str, zone: tuple[tuple[int, int], tuple[int, int]]) -> bool:
        """改换交接点:仅限尚未开始的承诺;已执行路径与预约保持原样."""
        ho = self.handoffs[handoff_id]
        if not ho.adjustable:
            self._log("handoff_switch_refused", handoff=handoff_id, state=ho.state)
            return False
        ho.zone = (tuple(zone[0]), tuple(zone[1]))
        if ho.state == BLOCKED:
            ho.state = PLANNED
        self._log(
            "handoff_switched",
            handoff=handoff_id,
            zone=[list(ho.zone[0]), list(ho.zone[1])],
            owner=self.cargo.get(ho.cargo_id),
        )
        self.assign({
            ho.giver: Task(f"{handoff_id}:deliver", ho.zone[0]),
            ho.receiver: Task(f"{handoff_id}:collect", ho.zone[1]),
        })
        return True

    def cancel_handoff(self, handoff_id: str) -> bool:
        """取消交接:仅限未开始;货物归属保持现状,双方任务撤回."""
        ho = self.handoffs[handoff_id]
        if not ho.adjustable:
            self._log("handoff_cancel_refused", handoff=handoff_id, state=ho.state)
            return False
        ho.state = CANCELLED
        for rid in (ho.giver, ho.receiver):
            robot = self.robots[rid]
            if robot.task and robot.task.task_id.startswith(handoff_id + ":"):
                self.coordinator._release_future(robot, self.tick)
                self.coordinator.reservations.release_hold(rid)
                robot.task = None
                robot.path = [robot.pos]
                robot.plan_start = self.tick
                robot.status = RobotStatus.IDLE
                robot.last_blockers = []
                robot.last_reason = None
                self.coordinator.reservations.reserve_hold(rid, robot.pos)
        self._log("handoff_cancelled", handoff=handoff_id, owner=self.cargo.get(ho.cargo_id))
        self._replan(strict=False)
        return True

    def commitments(self) -> dict[str, list[str]]:
        """紧急任务插入时:哪些交接承诺可调整,哪些已开始不能撤回."""
        adjustable, locked = [], []
        for ho in self.handoffs.values():
            if ho.state in (COMPLETED, CANCELLED) or ho.started:
                locked.append(ho.handoff_id)
            else:
                adjustable.append(ho.handoff_id)
        return {"adjustable": sorted(adjustable), "locked": sorted(locked)}

    def save_snapshot(self, path: str | Path) -> None:
        """保存货物归属与交接状态;无论何时保存,归属始终唯一."""
        data = {
            "tick": self.tick,
            "cargo": dict(self.cargo),
            "handoffs": [ho.to_dict() for ho in self.handoffs.values()],
            "staging": [area.to_dict() for area in self.staging.values()],
            "relays": [relay.to_dict() for relay in self.relays.values()],
            "terminal_tasks": self.coordinator.terminal_tasks,
        }
        Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2))

    def load_snapshot(self, path: str | Path) -> None:
        """恢复快照:已完成交接不重复搬运,已终止任务不复活."""
        data = json.loads(Path(path).read_text())
        settled_cargo = {
            ho.cargo_id: ho.receiver
            for ho in self.handoffs.values() if ho.state == COMPLETED
        }
        for rec in data.get("handoffs", []):
            handoff = Handoff.from_dict(rec)
            existing = self.handoffs.get(handoff.handoff_id)
            if existing is not None and existing.state in (COMPLETED, CANCELLED):
                continue  # 已完结的交接不被旧快照复活
            if handoff.state == COMPLETED:
                self.cargo[handoff.cargo_id] = handoff.receiver
                settled_cargo[handoff.cargo_id] = handoff.receiver
            self.handoffs[handoff.handoff_id] = handoff
        for cargo_id, owner in data.get("cargo", {}).items():
            if cargo_id in settled_cargo:
                self.cargo[cargo_id] = settled_cargo[cargo_id]  # 已完成的归属不回滚
            else:
                self.cargo[cargo_id] = owner
        for task_id, record in data.get("terminal_tasks", {}).items():
            self.coordinator.terminal_tasks.setdefault(task_id, record)
        # 恢复托管区占用:静置格重新计入预约,不提前释放位置
        for rec in data.get("staging", []):
            area = self.staging.get(str(rec["area_id"]))
            if area is None:
                slots = [
                    (tuple(item["cell"]), tuple(item["access"])) for item in rec.get("slots", [])
                ]
                self.add_staging_area(str(rec["area_id"]), slots, capacity=int(rec.get("capacity", len(slots))))
                area = self.staging[str(rec["area_id"])]
            for item in rec.get("slots", []):
                slot = area.slot(str(item["slot_id"]))
                if slot is None or slot.occupied:
                    continue  # 已占用的不被旧快照覆盖
                slot.cargo_id = item.get("cargo_id")
                slot.placed_tick = item.get("placed_tick")
                slot.reserved_by = item.get("reserved_by")
                if slot.occupied:
                    self.coordinator.external_holds[slot.cell] = slot_owner(slot.slot_id)
        # 恢复接力:已完成的段不重新执行,已放下的不重新放置
        settled_relays = {
            rid for rid, r in self.relays.items() if r.state in (RELAY_COMPLETED, RELAY_CANCELLED)
        }
        for rec in data.get("relays", []):
            relay = Relay.from_dict(rec)
            if relay.relay_id in settled_relays:
                continue
            self.relays[relay.relay_id] = relay
            for transfer in relay.transfers:
                if transfer.kind == RELAY_STAGE and transfer.placed and not transfer.picked:
                    slot = self._transfer_slot(transfer)
                    if slot and not slot.occupied:
                        slot.cargo_id = relay.cargo_id
                        slot.placed_tick = self.tick
                        self.coordinator.external_holds[slot.cell] = slot_owner(slot.slot_id)
                    self.cargo[relay.cargo_id] = slot_owner(transfer.slot_id)
        for relay in self.relays.values():
            if relay.state in (RELAY_COMPLETED, RELAY_CANCELLED):
                continue
            relay.state = RELAY_ACTIVE
            leg = relay.next_leg()
            if leg is not None:
                leg.state = RELAY_PENDING
                leg.note = None
                self._activate_leg(relay, leg)
        self._log(
            "snapshot_restored",
            cargo=dict(self.cargo),
            slots={
                slot.slot_id: slot.cargo_id
                for area in self.staging.values()
                for slot in area.slots
                if slot.occupied
            },
        )
        # 放下后尚未领取时保存:仍站在取放站位上的放置方让位,下一台才能进入
        for area in self.staging.values():
            for slot in area.slots:
                if not slot.occupied:
                    continue
                for robot in self.robots.values():
                    if robot.pos == slot.access and robot.task is None \
                            and robot.status != RobotStatus.FAILED:
                        self._retreat_placer("snapshot", robot, slot)

    def _update_handoffs(self) -> None:
        for ho in self.handoffs.values():
            if ho.state in (COMPLETED, CANCELLED):
                continue
            giver = self.robots[ho.giver]
            receiver = self.robots[ho.receiver]
            failed = [
                robot.robot_id for robot in (giver, receiver)
                if robot.status == RobotStatus.FAILED
            ]
            if failed:
                if ho.state != BLOCKED:
                    ho.state = BLOCKED
                    self._log(
                        "handoff_blocked",
                        handoff=ho.handoff_id,
                        cargo=ho.cargo_id,
                        failed=failed,
                        owner=self.cargo.get(ho.cargo_id),
                        options=self.handoff_options(ho.handoff_id),
                    )
                continue
            giver_in = giver.pos == ho.zone[0]
            receiver_in = receiver.pos == ho.zone[1]
            if ho.state in (PLANNED, BLOCKED) and giver_in and receiver_in:
                ho.state = BOTH_PRESENT
                ho.started_tick = self.tick
                self._log(
                    "handoff_started",
                    handoff=ho.handoff_id,
                    cargo=ho.cargo_id,
                    zone=[list(ho.zone[0]), list(ho.zone[1])],
                    owner=self.cargo.get(ho.cargo_id),
                )
            elif ho.state == BOTH_PRESENT:
                if giver_in and receiver_in:
                    ho.state = COMPLETED
                    ho.completed_tick = self.tick
                    self.cargo[ho.cargo_id] = ho.receiver
                    self._log(
                        "handoff_complete",
                        handoff=ho.handoff_id,
                        cargo=ho.cargo_id,
                        giver=ho.giver,
                        receiver=ho.receiver,
                        owner=ho.receiver,
                    )
                else:
                    ho.state = BLOCKED
                    self._log(
                        "handoff_blocked",
                        handoff=ho.handoff_id,
                        cargo=ho.cargo_id,
                        failed=[],
                        reason="party_left_zone",
                        owner=self.cargo.get(ho.cargo_id),
                        options=self.handoff_options(ho.handoff_id),
                    )

    # ---------- 接力运输与临时托管区 ----------

    def add_staging_area(
        self,
        area_id: str,
        slots: list[tuple[tuple[int, int], tuple[int, int]]],
        capacity: int | None = None,
    ) -> None:
        """登记托管区:slots 为 (静置格, 取放站位) 对,容量不超过托管位数量."""
        built: list[StagingSlot] = []
        seen: set[tuple[int, int]] = set()
        for i, (cell, access) in enumerate(slots):
            cell, access = tuple(cell), tuple(access)
            if not self.coordinator.gmap.passable(cell) or not self.coordinator.gmap.passable(access):
                raise ValueError(f"托管位 {cell}/{access} 不可通行")
            if cell == access or cell in seen or access in seen:
                raise ValueError("托管位静置格与站位不能互相重叠")
            seen.add(cell)
            seen.add(access)
            built.append(StagingSlot(f"{area_id}#{i}", cell, access))
        cap = len(built) if capacity is None else min(capacity, len(built))
        self.staging[area_id] = StagingArea(area_id, built, cap)
        self._log(
            "staging_area",
            area=area_id,
            capacity=cap,
            slots=[{"slot": s.slot_id, "cell": list(s.cell), "access": list(s.access)} for s in built],
        )

    def assign_relay(
        self,
        relay_id: str,
        cargo_id: str,
        carriers: list[str],
        transfers: list[tuple],
        final_goal: tuple[int, int],
        priority: int = 0,
        deadline: int | None = None,
    ) -> None:
        """登记接力:carriers 依次承运;transfers[i] 为 ('stage', area) 或 ('handoff', (交出格, 接收格)).

        同一件物体同一时刻只能属于一个接力计划,归属不会被拆成多份。
        """
        if len(carriers) < 1 or len(transfers) != max(0, len(carriers) - 1):
            raise ValueError("转移次数必须等于段数减一")
        active = [r for r in self.relays.values() if r.state not in (RELAY_COMPLETED, RELAY_CANCELLED)]
        if any(r.cargo_id == cargo_id for r in active):
            raise ValueError(f"货物 {cargo_id} 已在接力 {active[0].relay_id} 中,不能拆成多份")
        owner = self.cargo.get(cargo_id)
        if owner is not None and owner != carriers[0]:
            raise ValueError(f"货物 {cargo_id} 当前归属 {owner},不能由 {carriers[0]} 承运")
        legs = [
            RelayLeg(
                leg_id=f"{relay_id}:L{i}",
                index=i,
                carrier=rid,
                goal=tuple(final_goal) if i == len(carriers) - 1 else None,
            )
            for i, rid in enumerate(carriers)
        ]
        relay_transfers: list[RelayTransfer] = []
        for i, spec in enumerate(transfers):
            if spec[0] == "stage":
                if spec[1] not in self.staging:
                    raise ValueError(f"未知托管区 {spec[1]}")
                relay_transfers.append(RelayTransfer(index=i, kind=RELAY_STAGE, area=spec[1]))
            elif spec[0] == "handoff":
                zone = (tuple(spec[1][0]), tuple(spec[1][1]))
                relay_transfers.append(RelayTransfer(index=i, kind=RELAY_HANDOFF, zone=zone))
            else:
                raise ValueError(f"未知转移方式 {spec[0]}")
        relay = Relay(
            relay_id=relay_id,
            cargo_id=cargo_id,
            legs=legs,
            transfers=relay_transfers,
            state=RELAY_ACTIVE,
            planned_tick=self.tick,
        )
        self.relays[relay_id] = relay
        self.cargo[cargo_id] = carriers[0]
        self._log(
            "relay_planned",
            relay=relay_id,
            cargo=cargo_id,
            carriers=list(carriers),
            transfers=[t.to_dict() for t in relay_transfers],
            final_goal=list(final_goal),
            owner=carriers[0],
        )
        self._activate_leg(relay, legs[0], priority=priority, deadline=deadline)

    # ---- 内部:段激活与状态推进 ----

    def _carrier_state(self, robot: Robot, relay_id: str) -> str:
        """承运方可用性:failed / busy(被紧急任务等占用) / ok."""
        if robot.status == RobotStatus.FAILED:
            return "failed"
        if robot.task is None or robot.task.task_id.startswith(relay_id + ":"):
            return "ok"
        if robot.status in (RobotStatus.DONE, RobotStatus.IDLE):
            return "ok"
        return "busy"

    def _assign_leg_task(self, relay: Relay, leg: RelayLeg, goal, priority: int = 0, deadline=None) -> None:
        self._log(
            "relay_leg_started",
            relay=relay.relay_id,
            leg=leg.leg_id,
            carrier=leg.carrier,
            goal=list(goal),
            note=leg.note,
            owner=self.cargo.get(relay.cargo_id),
        )
        self.assign({leg.carrier: Task(leg.leg_id, tuple(goal), priority=priority, deadline=deadline)})

    def _activate_leg(self, relay: Relay, leg: RelayLeg, priority: int = 0, deadline=None) -> None:
        carrier = self.robots[leg.carrier]
        state = self._carrier_state(carrier, relay.relay_id)
        if state != "ok":
            self._block_leg(relay, leg, f"carrier_{state}")
            return
        incoming = relay.transfer_after(leg.index - 1)
        if incoming and incoming.kind == RELAY_STAGE and incoming.placed and not incoming.picked:
            slot = self._transfer_slot(incoming)
            leg.note = "pickup"
            leg.state = RELAY_ACTIVE
            self._assign_leg_task(relay, leg, slot.access, priority, deadline)
            return
        outgoing = relay.transfer_after(leg.index)
        if outgoing and outgoing.kind == RELAY_HANDOFF:
            nxt = relay.legs[leg.index + 1]
            receiver_state = self._carrier_state(self.robots[nxt.carrier], relay.relay_id)
            if receiver_state != "ok":
                self._block_leg(relay, leg, f"receiver_{receiver_state}")
                return
            leg.state = RELAY_ACTIVE
            outgoing.state = RELAY_ACTIVE
            self._log(
                "relay_leg_started",
                relay=relay.relay_id,
                leg=leg.leg_id,
                carrier=leg.carrier,
                goal=list(outgoing.zone[0]),
                note="handoff",
                owner=self.cargo.get(relay.cargo_id),
            )
            self.assign_handoff(
                f"{relay.relay_id}:T{leg.index}", relay.cargo_id,
                giver=leg.carrier, receiver=nxt.carrier, zone=outgoing.zone,
                priority=priority, deadline=deadline,
            )
            return
        if outgoing and outgoing.kind == RELAY_STAGE:
            area = self.staging[outgoing.area]
            slot = area.free_slot(relay.relay_id)
            if slot is None:
                self._block_leg(relay, leg, "staging_full")
                return
            slot.reserved_by = relay.relay_id  # 预约不等于占用
            outgoing.slot_id = slot.slot_id
            outgoing.state = RELAY_ACTIVE
            leg.state = RELAY_ACTIVE
            self._assign_leg_task(relay, leg, slot.access, priority, deadline)
            return
        leg.state = RELAY_ACTIVE
        self._assign_leg_task(relay, leg, leg.goal, priority, deadline)

    def _block_leg(self, relay: Relay, leg: RelayLeg, reason: str) -> None:
        leg.state = RELAY_BLOCKED
        leg.note = f"blocked:{reason}"
        relay.state = RELAY_BLOCKED
        self._log(
            "relay_blocked",
            relay=relay.relay_id,
            leg=leg.leg_id,
            cargo=relay.cargo_id,
            reason=reason,
            owner=self.cargo.get(relay.cargo_id),
        )

    def _transfer_slot(self, transfer: RelayTransfer) -> StagingSlot:
        return self.staging[transfer.area].slot(transfer.slot_id)

    def _withdraw_robot_task(self, robot: Robot) -> None:
        self.coordinator._release_future(robot, self.tick)
        self.coordinator.reservations.release_hold(robot.robot_id)
        self._log("task_withdrawn", robot=robot.robot_id,
                  task=robot.task.task_id if robot.task else None)
        robot.task = None
        robot.path = [robot.pos]
        robot.plan_start = self.tick
        robot.status = RobotStatus.IDLE
        robot.last_blockers = []
        robot.last_reason = None
        self.coordinator.reservations.reserve_hold(robot.robot_id, robot.pos)

    def _retreat_placer(self, task_prefix: str, robot: Robot, slot: StagingSlot) -> None:
        """放入后让出取放站位,避免挡住后续取货机器人."""
        staging_cells = {slot.cell}
        staging_cells.update(
            item.access for area in self.staging.values() for item in area.slots
        )
        pending = [robot.pos]
        seen = {robot.pos}
        while pending:
            current = pending.pop(0)
            for cell in self.coordinator.gmap.neighbors(current):
                if cell in seen or cell in staging_cells:
                    continue
                seen.add(cell)
                hold = self.coordinator.reservations.permanent.get(cell)
                if hold is None or hold == robot.robot_id:
                    self.assign({robot.robot_id: Task(f"{task_prefix}:retreat", cell)})
                    return
                pending.append(cell)

    def _update_relays(self) -> None:
        relays = [r for r in self.relays.values() if r.state not in (RELAY_COMPLETED, RELAY_CANCELLED)]
        # 先处理取走:同一 tick 内取物先于新放入,空出容量与托管位
        for relay in relays:
            for leg in relay.legs:
                if leg.state != RELAY_ACTIVE or leg.note != "pickup":
                    continue
                incoming = relay.transfer_after(leg.index - 1)
                slot = self._transfer_slot(incoming)
                robot = self.robots[leg.carrier]
                if robot.pos != slot.access or slot.cargo_id != relay.cargo_id:
                    continue
                slot.cargo_id = None
                slot.placed_tick = None
                slot.reserved_by = None
                incoming.picked = True
                incoming.state = RELAY_COMPLETED
                self.coordinator.external_holds.pop(slot.cell, None)
                self.cargo[relay.cargo_id] = leg.carrier
                leg.note = None
                self._log(
                    "stage_picked",
                    relay=relay.relay_id,
                    cargo=relay.cargo_id,
                    slot=slot.slot_id,
                    area=incoming.area,
                    picker=leg.carrier,
                    owner=leg.carrier,
                )
                self._advance_carry(relay, leg)
        # 再处理放入、交接完成与阻塞
        for relay in relays:
            if relay.state in (RELAY_COMPLETED, RELAY_CANCELLED):
                continue
            for leg in relay.legs:
                if leg.state == RELAY_BLOCKED and leg.note and leg.note.startswith("blocked:"):
                    reason = leg.note.split(":", 1)[1]
                    if reason in ("staging_full", "carrier_busy", "receiver_busy"):
                        leg.state = RELAY_PENDING
                        leg.note = None
                        relay.state = RELAY_ACTIVE
                        self._activate_leg(relay, leg)
                    continue
                if leg.state != RELAY_ACTIVE or leg.note == "pickup":
                    continue
                robot = self.robots[leg.carrier]
                if robot.status == RobotStatus.FAILED:
                    self._block_leg(relay, leg, "carrier_failed")
                    continue
                if robot.task is None or not robot.task.task_id.startswith(relay.relay_id + ":"):
                    self._block_leg(relay, leg, "carrier_reassigned")
                    continue
                outgoing = relay.transfer_after(leg.index)
                if outgoing is None:
                    if robot.status == RobotStatus.DONE and robot.pos == leg.goal:
                        leg.state = RELAY_COMPLETED
                        self._log("relay_leg_completed", relay=relay.relay_id, leg=leg.leg_id,
                                  carrier=leg.carrier, owner=self.cargo.get(relay.cargo_id))
                        if all(item.state == RELAY_COMPLETED for item in relay.legs):
                            relay.state = RELAY_COMPLETED
                            self._log("relay_completed", relay=relay.relay_id,
                                      cargo=relay.cargo_id, owner=self.cargo.get(relay.cargo_id))
                    continue
                if outgoing.kind == RELAY_STAGE and not outgoing.placed:
                    slot = self._transfer_slot(outgoing)
                    if robot.pos == slot.access and not slot.occupied:
                        slot.cargo_id = relay.cargo_id
                        slot.placed_tick = self.tick
                        slot.reserved_by = None
                        outgoing.placed = True
                        self.coordinator.external_holds[slot.cell] = slot_owner(slot.slot_id)
                        self.cargo[relay.cargo_id] = slot_owner(slot.slot_id)
                        leg.state = RELAY_COMPLETED
                        self._log(
                            "stage_placed",
                            relay=relay.relay_id,
                            cargo=relay.cargo_id,
                            slot=slot.slot_id,
                            area=outgoing.area,
                            placer=leg.carrier,
                            owner=slot_owner(slot.slot_id),
                        )
                        self._log("relay_leg_completed", relay=relay.relay_id, leg=leg.leg_id,
                                  carrier=leg.carrier, owner=self.cargo.get(relay.cargo_id))
                        self._retreat_placer(relay.relay_id, robot, slot)
                        self._activate_leg(relay, relay.legs[leg.index + 1])
                elif outgoing.kind == RELAY_HANDOFF:
                    handoff = self.handoffs.get(f"{relay.relay_id}:T{leg.index}")
                    if handoff is None:
                        continue
                    if handoff.state == COMPLETED:
                        outgoing.state = RELAY_COMPLETED
                        leg.state = RELAY_COMPLETED
                        self._log("relay_leg_completed", relay=relay.relay_id, leg=leg.leg_id,
                                  carrier=leg.carrier, owner=self.cargo.get(relay.cargo_id))
                        self._activate_leg(relay, relay.legs[leg.index + 1])
                    elif handoff.state == BLOCKED:
                        outgoing.state = RELAY_BLOCKED
                        self._block_leg(relay, leg, "handoff_blocked")

    def _advance_carry(self, relay: Relay, leg: RelayLeg) -> None:
        """取货之后继续本段运输:走向下一转移点或最终目标."""
        outgoing = relay.transfer_after(leg.index)
        if outgoing is None:
            self._assign_leg_task(relay, leg, leg.goal)
            return
        if outgoing.kind == RELAY_STAGE:
            area = self.staging[outgoing.area]
            slot = area.free_slot(relay.relay_id)
            if slot is None:
                self._block_leg(relay, leg, "staging_full")
                return
            slot.reserved_by = relay.relay_id
            outgoing.slot_id = slot.slot_id
            outgoing.state = RELAY_ACTIVE
            self._assign_leg_task(relay, leg, slot.access)
            return
        nxt = relay.legs[leg.index + 1]
        receiver_state = self._carrier_state(self.robots[nxt.carrier], relay.relay_id)
        if receiver_state != "ok":
            self._block_leg(relay, leg, f"receiver_{receiver_state}")
            return
        outgoing.state = RELAY_ACTIVE
        self.assign_handoff(
            f"{relay.relay_id}:T{leg.index}", relay.cargo_id,
            giver=leg.carrier, receiver=nxt.carrier, zone=outgoing.zone,
        )

    # ---- 方案比较 / 重排 / 取消 ----

    def _eta_to(self, robot: Robot, goal) -> tuple[int | None, set[str]]:
        if robot.pos == tuple(goal):
            return self.tick, set()
        result = plan_path_result(
            self.coordinator.gmap,
            self.coordinator.reservations,
            robot.pos,
            tuple(goal),
            self.tick,
            self.coordinator.horizon,
            robot_id=robot.robot_id,
        )
        if result.found:
            return self.tick + len(result.path) - 1, set()
        return None, set(result.blockers)

    def compare_relay(self, relay_id: str) -> list[dict[str, object]]:
        """比较当前未执行段的三种安排:直接等待、换机器人接力、临时托管.

        只读评估:不改写任何已执行路径、预约与归属。
        """
        relay = self.relays[relay_id]
        leg = relay.next_leg()
        if leg is None:
            return []
        owner = self.cargo.get(relay.cargo_id)
        holder = self.robots.get(owner) if owner in self.robots else None
        outgoing = relay.transfer_after(leg.index)
        options: list[dict[str, object]] = []

        # 1) 直接等待:承运方按原计划前往转移点,等待接收方/空位
        wait_opt: dict[str, object] = {"option": "direct_wait", "feasible": True}
        if holder is not None and outgoing is not None:
            if outgoing.kind == RELAY_STAGE:
                area = self.staging[outgoing.area]
                slot = area.free_slot(relay_id) if area.staged_count() < area.capacity else None
                if slot is None:
                    wait_opt.update(feasible=False, reason="staging_full",
                                    staged=area.staged_count(), capacity=area.capacity)
                else:
                    eta, blockers = self._eta_to(holder, slot.access)
                    wait_opt.update(eta=eta, corridor_conflicts=sorted(blockers),
                                    added_wait=0, target=list(slot.access))
            else:
                receiver = self.robots[relay.legs[leg.index + 1].carrier]
                eta_g, blk_g = self._eta_to(holder, outgoing.zone[0])
                eta_r, blk_r = self._eta_to(receiver, outgoing.zone[1])
                feasible = eta_g is not None and eta_r is not None
                wait_opt.update(
                    feasible=feasible,
                    eta=max(eta_g, eta_r) if feasible else None,
                    added_wait=abs(eta_g - eta_r) if feasible else None,
                    corridor_conflicts=sorted(blk_g | blk_r),
                )
        elif holder is not None:
            eta, blockers = self._eta_to(holder, leg.goal)
            wait_opt.update(eta=eta, corridor_conflicts=sorted(blockers), added_wait=0)
        options.append(wait_opt)

        # 2) 换机器人接力:货物在当前位置当面交给另一台空闲机器人
        if holder is not None:
            for candidate in self.robots.values():
                if candidate.robot_id == holder.robot_id or candidate.status == RobotStatus.FAILED:
                    continue
                if self._carrier_state(candidate, relay_id) != "ok":
                    continue
                best = None
                for cell in self.coordinator.gmap.neighbors(holder.pos):
                    eta, blockers = self._eta_to(candidate, cell)
                    if eta is None:
                        continue
                    if best is None or eta < best[0]:
                        best = (eta, cell, blockers)
                options.append({
                    "option": "robot_relay",
                    "candidate": candidate.robot_id,
                    "feasible": best is not None,
                    "eta": best[0] if best else None,
                    "zone": [list(holder.pos), list(best[1])] if best else None,
                    "corridor_conflicts": sorted(best[2]) if best else [],
                    "added_wait": max(0, best[0] - self.tick) if best else None,
                })

        # 3) 临时托管:放入托管区等待后续承运
        if holder is not None:
            for area in self.staging.values():
                slot = area.free_slot(relay_id)
                entry: dict[str, object] = {
                    "option": "staging",
                    "area": area.area_id,
                    "staged": area.staged_count(),
                    "capacity": area.capacity,
                }
                if slot is None:
                    entry.update(feasible=False, reason="staging_full")
                else:
                    eta, blockers = self._eta_to(holder, slot.access)
                    entry.update(feasible=eta is not None, eta=eta,
                                 slot=slot.slot_id, corridor_conflicts=sorted(blockers))
                options.append(entry)

        # 后续交接兑现检查:接收方故障/被占用、托管容量不足
        unfulfilled: list[str] = []
        for transfer in relay.transfers:
            if transfer.executed or transfer.index < leg.index:
                continue
            if transfer.kind == RELAY_HANDOFF:
                receiver = self.robots[relay.legs[transfer.index + 1].carrier]
                state = self._carrier_state(receiver, relay_id)
                if state != "ok":
                    unfulfilled.append(f"{relay_id}:T{transfer.index}:receiver_{state}")
            else:
                area = self.staging[transfer.area]
                pending = sum(
                    1 for t in relay.transfers
                    if t.kind == RELAY_STAGE and t.area == area.area_id and not t.executed
                )
                if area.staged_count() + pending > area.capacity:
                    unfulfilled.append(f"{relay_id}:T{transfer.index}:staging_capacity")
        for opt in options:
            opt["downstream_unfulfilled"] = unfulfilled
        return options

    def replan_relay(
        self,
        relay_id: str,
        carriers: list[str],
        transfers: list[tuple],
        final_goal: tuple[int, int],
    ) -> bool:
        """故障/取消后只重排尚未执行的部分;已完成的段与实际占用保留."""
        relay = self.relays[relay_id]
        if relay.state in (RELAY_COMPLETED, RELAY_CANCELLED):
            self._log("relay_replan_refused", relay=relay_id, state=relay.state)
            return False
        first_open = next(
            (i for i, leg in enumerate(relay.legs) if leg.state != RELAY_COMPLETED),
            len(relay.legs),
        )
        for transfer in relay.transfers:
            if transfer.index < first_open or transfer.executed:
                continue
            if transfer.kind == RELAY_HANDOFF:
                handoff = self.handoffs.get(f"{relay_id}:T{transfer.index}")
                if handoff is not None and handoff.state == BOTH_PRESENT:
                    # 已开始的交接承诺不可撤回
                    self._log("relay_replan_refused", relay=relay_id,
                              reason="handoff_started", handoff=handoff.handoff_id)
                    return False
        owner = self.cargo.get(relay.cargo_id)
        if owner in self.robots and owner != carriers[0]:
            raise ValueError(f"货物 {relay.cargo_id} 当前归属 {owner},重排首段必须由 {owner} 承运")
        kept_legs = relay.legs[:first_open]
        kept_transfers = [t for t in relay.transfers if t.executed]
        for leg in relay.legs[first_open:]:
            robot = self.robots[leg.carrier]
            if robot.task and robot.task.task_id.startswith(relay_id + ":"):
                self._withdraw_robot_task(robot)
            leg.state = RELAY_CANCELLED
        for transfer in relay.transfers:
            if transfer.executed or transfer.index < first_open:
                continue
            if transfer.kind == RELAY_STAGE and transfer.slot_id:
                slot = self.staging[transfer.area].slot(transfer.slot_id)
                if slot and slot.reserved_by == relay_id and not slot.occupied:
                    slot.reserved_by = None
            if transfer.kind == RELAY_HANDOFF:
                handoff = self.handoffs.get(f"{relay_id}:T{transfer.index}")
                if handoff is not None and handoff.adjustable:
                    self.cancel_handoff(handoff.handoff_id)
                elif handoff is not None and handoff.state == BLOCKED:
                    # 接收者离开/故障导致的阻塞承诺可替换,归属不变
                    for rid in (handoff.giver, handoff.receiver):
                        robot = self.robots[rid]
                        if robot.task and robot.task.task_id.startswith(handoff.handoff_id + ":"):
                            self._withdraw_robot_task(robot)
                    handoff.state = CANCELLED
                    self._log("handoff_cancelled", handoff=handoff.handoff_id,
                              owner=self.cargo.get(handoff.cargo_id))
            transfer.state = RELAY_CANCELLED
        new_legs = [
            RelayLeg(
                leg_id=f"{relay_id}:L{first_open + i}",
                index=first_open + i,
                carrier=rid,
                goal=tuple(final_goal) if i == len(carriers) - 1 else None,
            )
            for i, rid in enumerate(carriers)
        ]
        new_transfers: list[RelayTransfer] = []
        for i, spec in enumerate(transfers):
            if spec[0] == "stage":
                new_transfers.append(RelayTransfer(index=first_open + i, kind=RELAY_STAGE, area=spec[1]))
            else:
                zone = (tuple(spec[1][0]), tuple(spec[1][1]))
                new_transfers.append(RelayTransfer(index=first_open + i, kind=RELAY_HANDOFF, zone=zone))
        relay.legs = kept_legs + new_legs
        relay.transfers = kept_transfers + new_transfers
        relay.state = RELAY_ACTIVE
        self._log(
            "relay_replanned",
            relay=relay_id,
            cargo=relay.cargo_id,
            kept_completed=[leg.leg_id for leg in kept_legs],
            new_carriers=list(carriers),
            owner=owner,
        )
        self._activate_leg(relay, relay.legs[first_open])
        return True

    def cancel_relay(self, relay_id: str) -> bool:
        """取消未执行部分;已完成的段与托管位实际占用保留,归属保持现状."""
        relay = self.relays[relay_id]
        if relay.state in (RELAY_COMPLETED, RELAY_CANCELLED):
            return False
        for leg in relay.legs:
            if leg.state == RELAY_COMPLETED:
                continue
            robot = self.robots[leg.carrier]
            if robot.task and robot.task.task_id.startswith(relay_id + ":"):
                self._withdraw_robot_task(robot)
            leg.state = RELAY_CANCELLED
        for transfer in relay.transfers:
            if transfer.executed:
                continue
            if transfer.kind == RELAY_STAGE and transfer.slot_id:
                slot = self.staging[transfer.area].slot(transfer.slot_id)
                if slot and slot.reserved_by == relay_id and not slot.occupied:
                    slot.reserved_by = None
            if transfer.kind == RELAY_HANDOFF:
                handoff = self.handoffs.get(f"{relay_id}:T{transfer.index}")
                if handoff is not None and handoff.adjustable:
                    self.cancel_handoff(handoff.handoff_id)
            transfer.state = RELAY_CANCELLED
        relay.state = RELAY_CANCELLED
        self._log("relay_cancelled", relay=relay_id, cargo=relay.cargo_id,
                  owner=self.cargo.get(relay.cargo_id))
        self._replan(strict=False)
        return True
