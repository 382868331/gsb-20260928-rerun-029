"""执行器:按 tick 推进,记录调度、封路与实际移动日志."""

from __future__ import annotations

import json
from pathlib import Path

from .coordinator import Coordinator
from .handoff import BOTH_PRESENT, CANCELLED, COMPLETED, PLANNED, BLOCKED, Handoff
from .planner import DEADLINE_INFEASIBLE, PlanResult, plan_path_result
from .relay import (
    CARRY,
    HANDOFF,
    RELAY_ACTIVE,
    RELAY_BLOCKED,
    RELAY_CANCELLED,
    RELAY_DONE,
    STAGE,
    LEG_ACTIVE,
    LEG_CANCELLED,
    LEG_DONE,
    LEG_FAILED,
    LEG_PLANNED,
    RelayLeg,
    RelayPlan,
    StagingArea,
    slot_owner_id,
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
        self.relays: dict[str, RelayPlan] = {}

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
        return any(
            ho.state in (PLANNED, BOTH_PRESENT)
            and all(self.robots[rid].status != RobotStatus.FAILED for rid in (ho.giver, ho.receiver))
            for ho in self.handoffs.values()
        ) or any(relay.pending for relay in self.relays.values())

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
        for rec in data.get("staging", []):
            area = StagingArea.from_dict(rec)
            existing = self.staging.get(area.area_id)
            if existing is not None:
                existing.occupancy.update(area.occupancy)
                area = existing
            else:
                self.staging[area.area_id] = area
            for slot in area.occupancy:
                # 托管中的货物继续占住托管位,不提前释放
                self.coordinator.reservations.reserve_hold(slot_owner_id(area.area_id, slot), slot)
        settled_relays = {
            rid for rid, relay in self.relays.items() if relay.state in (RELAY_DONE, RELAY_CANCELLED)
        }
        for rec in data.get("relays", []):
            relay = RelayPlan.from_dict(rec)
            if relay.relay_id in settled_relays:
                continue  # 已完结的接力不被旧快照复活
            self.relays[relay.relay_id] = relay
        for relay in self.relays.values():
            if relay.state != RELAY_ACTIVE:
                continue
            for leg in relay.legs:
                if leg.state == LEG_ACTIVE:
                    # 恢复后继续:未执行完的段重新安排,已执行的保持 DONE 不重复放置
                    leg.state = LEG_PLANNED
        self._log(
            "snapshot_loaded",
            cargo=dict(self.cargo),
            staging=[area.to_dict() for area in self.staging.values()],
        )

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

    # ---------- 接力运输与临时托管 ----------

    def add_staging_area(
        self,
        area_id: str,
        slots: dict[tuple[int, int], tuple[int, int]],
        ops_per_tick: int = 1,
    ) -> None:
        """登记托管区:托管位 -> 取放站位;每位容量一件,每刻取放次数有限."""
        area = StagingArea(
            area_id=area_id,
            slots={tuple(slot): tuple(access) for slot, access in slots.items()},
            ops_per_tick=ops_per_tick,
        )
        self.staging[area_id] = area
        self._log(
            "staging_area",
            area=area_id,
            slots=[list(slot) for slot in area.slots],
            ops_per_tick=ops_per_tick,
        )

    def assign_relay(self, relay_id: str, cargo_id: str, legs: list[dict[str, object]]) -> None:
        """登记接力运输方案:段按序执行,货物任意时刻只属于一台机器人或一个托管位."""
        if any(
            relay.cargo_id == cargo_id and relay.state == RELAY_ACTIVE
            for relay in self.relays.values()
        ):
            raise ValueError(f"货物 {cargo_id} 已有进行中的接力计划,不能拆成多份")
        built: list[RelayLeg] = []
        for index, spec in enumerate(legs):
            leg = RelayLeg(
                leg_id=f"{relay_id}:L{index}",
                index=index,
                kind=str(spec["kind"]),
                carrier=spec.get("carrier"),
                to=tuple(spec["to"]) if spec.get("to") else None,
                zone=(tuple(spec["zone"][0]), tuple(spec["zone"][1])) if spec.get("zone") else None,
                area_id=spec.get("area"),
                slot=tuple(spec["slot"]) if spec.get("slot") else None,
            )
            built.append(leg)
        self._validate_relay(cargo_id, built)
        relay = RelayPlan(relay_id=relay_id, cargo_id=cargo_id, legs=built)
        self.relays[relay_id] = relay
        first = built[0]
        if first.kind == CARRY and self.cargo.get(cargo_id) is None:
            self.cargo[cargo_id] = first.carrier  # 初始归属:第一个承运人
        self._log(
            "relay_planned",
            relay=relay_id,
            cargo=cargo_id,
            legs=[leg.to_dict() for leg in built],
            owner=self.cargo.get(cargo_id),
        )
        self._update_relays()

    def _validate_relay(self, cargo_id: str, legs: list[RelayLeg]) -> None:
        owner = self.cargo.get(cargo_id)
        for index, leg in enumerate(legs):
            if leg.kind == CARRY:
                if not leg.carrier or leg.to is None:
                    raise ValueError("承运段需要 carrier 与 to")
            elif leg.kind == STAGE:
                if leg.area_id not in self.staging:
                    raise ValueError(f"未知托管区 {leg.area_id}")
                area = self.staging[leg.area_id]
                if leg.slot not in area.slots:
                    raise ValueError(f"托管区 {leg.area_id} 没有托管位 {leg.slot}")
                if index == 0 or legs[index - 1].kind != CARRY:
                    raise ValueError("托管段前必须有一段承运")
            elif leg.kind == HANDOFF:
                if leg.zone is None:
                    raise ValueError("交接段需要 zone")
                if index + 1 >= len(legs) or legs[index + 1].kind != CARRY:
                    raise ValueError("交接段后必须有一段承运")
            else:
                raise ValueError(f"未知段类型 {leg.kind}")
        first = legs[0]
        if (
            owner is not None
            and first.kind == CARRY
            and first.carrier != owner
            and not str(owner).startswith("staging:")
        ):
            raise ValueError(f"货物 {cargo_id} 当前归属 {owner},首段承运人必须一致")

    def _carry_goal(self, relay: RelayPlan, leg: RelayLeg) -> tuple[int, int]:
        """承运段的实际行驶目标:下一段是托管则去取放站位,是交接则去交接区."""
        nxt = relay.legs[leg.index + 1] if leg.index + 1 < len(relay.legs) else None
        if nxt is not None and nxt.kind == STAGE:
            return self.staging[nxt.area_id].slots[nxt.slot]
        if nxt is not None and nxt.kind == HANDOFF:
            return nxt.zone[0]
        return leg.to

    def _activate_leg(self, relay: RelayPlan, leg: RelayLeg) -> None:
        owner = self.cargo.get(relay.cargo_id)
        if leg.kind == CARRY:
            robot = self.robots[leg.carrier]
            if robot.status == RobotStatus.FAILED:
                self._fail_leg(relay, leg, f"承运人 {leg.carrier} 已故障,失去承运资格")
                return
            if str(owner).startswith("staging:"):
                area_id, slot = self._owner_slot(owner)
                goal = self.staging[area_id].slots[slot]  # 先去取放站位取货
                reservations = self.coordinator.reservations
                stand_blocker = reservations.permanent.get(goal) or reservations.holds.get(goal)
                if stand_blocker is None:
                    for other in self.robots.values():
                        parked = other.task is None or other.status in (
                            RobotStatus.DONE, RobotStatus.PAUSED, RobotStatus.FAILED,
                        )
                        if other.robot_id != leg.carrier and other.pos == goal and parked:
                            stand_blocker = other.robot_id
                            break
                if stand_blocker is None:
                    for other in self.robots.values():
                        if other.robot_id != leg.carrier and other.goal == goal:
                            stand_blocker = other.robot_id  # 有人正前往同一站位
                            break
                for other in self.robots.values():
                    if other.robot_id != leg.carrier and other.pos == goal:
                        stand_blocker = other.robot_id
                        break
                if stand_blocker not in (None, leg.carrier):
                    # 取放站位仍被占用(如放置方尚未离开):等待,不把取件任务判死
                    self._log(
                        "stage_wait_access",
                        relay=relay.relay_id,
                        cargo=relay.cargo_id,
                        leg=leg.leg_id,
                        goal=list(goal),
                        blocker=stand_blocker,
                    )
                    return
            else:
                goal = self._carry_goal(relay, leg)
            leg.state = LEG_ACTIVE
            self.assign({leg.carrier: Task(leg.leg_id, goal)})
        elif leg.kind == STAGE:
            leg.state = LEG_ACTIVE  # 放置动作在 _update_relays 中按取放限制执行
        elif leg.kind == HANDOFF:
            receiver = relay.legs[leg.index + 1].carrier
            if self.robots[receiver].status == RobotStatus.FAILED:
                self._fail_leg(relay, leg, f"接收者 {receiver} 已故障,失去接收资格")
                return
            handoff = Handoff(
                handoff_id=leg.leg_id,
                cargo_id=relay.cargo_id,
                giver=str(owner),
                receiver=receiver,
                zone=leg.zone,
                planned_tick=self.tick,
            )
            self.handoffs[leg.leg_id] = handoff
            leg.state = LEG_ACTIVE
            self._log(
                "handoff_planned",
                handoff=leg.leg_id,
                cargo=relay.cargo_id,
                giver=owner,
                receiver=receiver,
                zone=[list(leg.zone[0]), list(leg.zone[1])],
                alternatives=[],
                owner=owner,
            )
            self.assign({
                str(owner): Task(f"{leg.leg_id}:deliver", leg.zone[0]),
                receiver: Task(f"{leg.leg_id}:collect", leg.zone[1]),
            })

    def _fail_leg(self, relay: RelayPlan, leg: RelayLeg, reason: str) -> None:
        leg.state = LEG_FAILED
        relay.state = RELAY_BLOCKED
        self._log(
            "relay_blocked",
            relay=relay.relay_id,
            cargo=relay.cargo_id,
            leg=leg.leg_id,
            reason=reason,
            owner=self.cargo.get(relay.cargo_id),
        )

    def _owner_slot(self, owner: str) -> tuple[str, tuple[int, int]]:
        _, rest = owner.split(":", 1)
        area_id, coord = rest.split("@")
        x, y = coord.split(",")
        return area_id, (int(x), int(y))

    def _finish_leg(self, relay: RelayPlan, leg: RelayLeg) -> None:
        leg.state = LEG_DONE
        leg.executed_tick = self.tick
        self._log(
            "relay_leg_done",
            relay=relay.relay_id,
            cargo=relay.cargo_id,
            leg=leg.leg_id,
            kind=leg.kind,
            owner=self.cargo.get(relay.cargo_id),
        )
        if relay.current_leg is None:
            relay.state = RELAY_DONE
            self._log(
                "relay_done",
                relay=relay.relay_id,
                cargo=relay.cargo_id,
                owner=self.cargo.get(relay.cargo_id),
            )

    def _update_relays(self) -> None:
        for relay in self.relays.values():
            if relay.state != RELAY_ACTIVE:
                continue
            guard = 0
            while relay.state == RELAY_ACTIVE and guard < 8:
                guard += 1
                leg = relay.current_leg
                if leg is None:
                    relay.state = RELAY_DONE
                    self._log(
                        "relay_done",
                        relay=relay.relay_id,
                        cargo=relay.cargo_id,
                        owner=self.cargo.get(relay.cargo_id),
                    )
                    break
                if leg.state == LEG_PLANNED:
                    self._activate_leg(relay, leg)
                    if leg.state == LEG_PLANNED:
                        break  # 激活条件未满足(如站位被占),下一刻重试
                    continue
                if not self._progress_leg(relay, leg):
                    break

    def _progress_leg(self, relay: RelayPlan, leg: RelayLeg) -> bool:
        """推进当前段;返回 True 表示本段已完成可继续下一段."""
        owner = self.cargo.get(relay.cargo_id)
        if leg.kind == CARRY:
            return self._progress_carry(relay, leg, owner)
        if leg.kind == STAGE:
            return self._progress_stage(relay, leg, owner)
        return self._progress_handoff(relay, leg)

    def _progress_carry(self, relay: RelayPlan, leg: RelayLeg, owner: str) -> bool:
        robot = self.robots[leg.carrier]
        if robot.status == RobotStatus.FAILED:
            self._fail_leg(relay, leg, f"承运人 {leg.carrier} 已故障,失去承运资格")
            return False
        if str(owner).startswith("staging:"):
            area_id, slot = self._owner_slot(owner)
            area = self.staging[area_id]
            access = area.slots[slot]
            if robot.pos != access:
                return False  # 还在前往取放站位
            if not area.can_operate(self.tick):
                self._log(
                    "stage_op_deferred",
                    relay=relay.relay_id,
                    cargo=relay.cargo_id,
                    area=area_id,
                    slot=list(slot),
                    op="pick",
                    reason="取放占用限制",
                )
                return False
            del area.occupancy[slot]
            area.record_op(self.tick)
            self.coordinator.reservations.holds.pop(slot, None)
            self.cargo[relay.cargo_id] = leg.carrier
            self._log(
                "stage_pick",
                relay=relay.relay_id,
                cargo=relay.cargo_id,
                area=area_id,
                slot=list(slot),
                carrier=leg.carrier,
                owner=leg.carrier,
            )
            self.assign({leg.carrier: Task(leg.leg_id, self._carry_goal(relay, leg))})
            return False
        if owner != leg.carrier:
            self._fail_leg(relay, leg, f"货物归属 {owner} 与承运人 {leg.carrier} 不一致")
            return False
        if robot.pos == self._carry_goal(relay, leg):
            self._finish_leg(relay, leg)
            return True
        return False

    def _progress_stage(self, relay: RelayPlan, leg: RelayLeg, owner: str) -> bool:
        area = self.staging[leg.area_id]
        slot = leg.slot
        access = area.slots[slot]
        if str(owner).startswith("staging:"):
            # 货物已在托管位(例如快照恢复):不重复放置
            self._finish_leg(relay, leg)
            return True
        robot = self.robots.get(str(owner))
        if robot is None:
            self._fail_leg(relay, leg, f"归属 {owner} 无法执行放置")
            return False
        if robot.status == RobotStatus.FAILED:
            self._fail_leg(relay, leg, f"承运人 {owner} 已故障,货物留在原地")
            return False
        if robot.pos != access:
            return False  # 承运人尚未到取放站位
        if slot in area.occupancy:
            self._log(
                "stage_wait_slot",
                relay=relay.relay_id,
                cargo=relay.cargo_id,
                area=leg.area_id,
                slot=list(slot),
                occupant=area.occupancy[slot],
            )
            return False
        if not area.can_operate(self.tick):
            self._log(
                "stage_op_deferred",
                relay=relay.relay_id,
                cargo=relay.cargo_id,
                area=leg.area_id,
                slot=list(slot),
                op="place",
                reason="取放占用限制",
            )
            return False
        area.occupancy[slot] = relay.cargo_id
        area.record_op(self.tick)
        self.coordinator.reservations.reserve_hold(slot_owner_id(leg.area_id, slot), slot)
        self.cargo[relay.cargo_id] = slot_owner_id(leg.area_id, slot)
        self._log(
            "stage_place",
            relay=relay.relay_id,
            cargo=relay.cargo_id,
            area=leg.area_id,
            slot=list(slot),
            carrier=str(owner),
            owner=slot_owner_id(leg.area_id, slot),
        )
        self._finish_leg(relay, leg)
        return True

    def _progress_handoff(self, relay: RelayPlan, leg: RelayLeg) -> bool:
        ho = self.handoffs[leg.leg_id]
        if ho.state == COMPLETED:
            self._finish_leg(relay, leg)
            return True
        failed = [
            rid for rid in (ho.giver, ho.receiver)
            if self.robots[rid].status == RobotStatus.FAILED
        ]
        if failed:
            self._fail_leg(relay, leg, f"交接方 {','.join(failed)} 已故障")
        return False

    def _release_leg_tasks(self, relay: RelayPlan, leg: RelayLeg) -> list[str]:
        """撤回未执行段占用的机器人任务,只释放未来预约,不动已执行占用."""
        released: list[str] = []
        for robot in self.robots.values():
            task = robot.task
            if task is None:
                continue
            if task.task_id == leg.leg_id or task.task_id.startswith(leg.leg_id + ":"):
                self.coordinator._release_future(robot, self.tick)
                self.coordinator.reservations.release_hold(robot.robot_id)
                robot.task = None
                robot.path = [robot.pos]
                robot.plan_start = self.tick
                robot.status = RobotStatus.IDLE
                robot.last_blockers = []
                robot.last_reason = None
                self.coordinator.reservations.reserve_hold(robot.robot_id, robot.pos)
                released.append(robot.robot_id)
        ho = self.handoffs.get(leg.leg_id)
        if ho is not None and ho.state not in (COMPLETED, CANCELLED):
            ho.state = CANCELLED
        return released

    def replan_relay(self, relay_id: str, new_legs: list[dict[str, object]]) -> None:
        """故障/取消后重排:只替换尚未执行的段,已完成的接力与实际占用保留."""
        relay = self.relays[relay_id]
        if relay.state in (RELAY_DONE, RELAY_CANCELLED):
            raise ValueError(f"接力 {relay_id} 已完结,不能重排")
        kept = [leg for leg in relay.legs if leg.state == LEG_DONE]
        released: list[str] = []
        for leg in relay.legs:
            if leg.state == LEG_DONE:
                continue
            released.extend(self._release_leg_tasks(relay, leg))
            leg.state = LEG_CANCELLED
        built: list[RelayLeg] = []
        offset = len(kept)
        for index, spec in enumerate(new_legs):
            built.append(RelayLeg(
                leg_id=f"{relay_id}:R{offset + index}",
                index=offset + index,
                kind=str(spec["kind"]),
                carrier=spec.get("carrier"),
                to=tuple(spec["to"]) if spec.get("to") else None,
                zone=(tuple(spec["zone"][0]), tuple(spec["zone"][1])) if spec.get("zone") else None,
                area_id=spec.get("area"),
                slot=tuple(spec["slot"]) if spec.get("slot") else None,
            ))
        self._validate_relay(relay.cargo_id, built)
        relay.legs = kept + built
        relay.state = RELAY_ACTIVE
        self._log(
            "relay_replanned",
            relay=relay_id,
            cargo=relay.cargo_id,
            kept=[leg.leg_id for leg in kept],
            new_legs=[leg.to_dict() for leg in built],
            released=sorted(set(released)),
            owner=self.cargo.get(relay.cargo_id),
        )
        self._update_relays()

    def cancel_relay(self, relay_id: str) -> bool:
        """取消接力:只撤回未执行的段;已完成的放置与实际托管占用保留."""
        relay = self.relays[relay_id]
        if relay.state in (RELAY_DONE, RELAY_CANCELLED):
            return False
        released: list[str] = []
        for leg in relay.legs:
            if leg.state == LEG_DONE:
                continue
            released.extend(self._release_leg_tasks(relay, leg))
            leg.state = LEG_CANCELLED
        relay.state = RELAY_CANCELLED
        self._log(
            "relay_cancelled",
            relay=relay_id,
            cargo=relay.cargo_id,
            released=sorted(set(released)),
            owner=self.cargo.get(relay.cargo_id),
        )
        self._replan(strict=False)
        return True

    def relay_commitments(self) -> dict[str, list[str]]:
        """紧急任务插入时:哪些接力段已执行/占用不能撤回,哪些还可调整."""
        adjustable: list[str] = []
        locked: list[str] = []
        for relay in self.relays.values():
            if relay.state in (RELAY_DONE, RELAY_CANCELLED):
                locked.extend(leg.leg_id for leg in relay.legs)
                continue
            for leg in relay.legs:
                if leg.state == LEG_DONE:
                    locked.append(leg.leg_id)
                else:
                    adjustable.append(leg.leg_id)
        return {"adjustable": sorted(adjustable), "locked": sorted(locked)}

    def compare_relay(
        self,
        cargo_id: str,
        goal: tuple[int, int],
        reliever: str,
        area_id: str,
        slot: tuple[int, int],
        handoff_zone: tuple[tuple[int, int], tuple[int, int]],
    ) -> list[dict[str, object]]:
        """比较直接等待、换机器人接力、临时托管三种安排;只读评估,不改写预约.

        每个方案给出预计完成时刻、相对静态最短路的新增等待、通道冲突(占用者)
        以及会被拖累而无法兑现的后续交接。
        """
        goal = tuple(goal)
        slot = tuple(slot)
        zone = (tuple(handoff_zone[0]), tuple(handoff_zone[1]))
        owner = str(self.cargo.get(cargo_id))
        carrier = self.robots[owner]
        second = self.robots[reliever]
        area = self.staging[area_id]
        access = area.slots[slot]

        def eta_of(robot: Robot, target: tuple[int, int]) -> tuple[int | None, set[str]]:
            if robot.pos == target:
                return self.tick, set()
            result = plan_path_result(
                self.coordinator.gmap,
                self.coordinator.reservations,
                robot.pos,
                target,
                self.tick,
                self.coordinator.horizon,
                robot_id=robot.robot_id,
            )
            if result.found:
                return self.tick + len(result.path) - 1, set(result.blockers)
            return None, set(result.blockers)

        def static_dist(a: tuple[int, int], b: tuple[int, int]) -> int:
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def side_effects(robots_used: list[str], eta: int | None) -> list[dict[str, object]]:
            affected: list[dict[str, object]] = []
            for ho in self.handoffs.values():
                if ho.state in (COMPLETED, CANCELLED) or ho.started:
                    continue
                if ho.giver in robots_used or ho.receiver in robots_used:
                    affected.append({
                        "handoff": ho.handoff_id,
                        "cargo": ho.cargo_id,
                        "delayed_until": eta,
                        "consequence": "承运机器人被占用,该后续交接无法按原计划兑现",
                    })
            return affected

        options: list[dict[str, object]] = []

        # 方案一:直接等待,当前持有者自己运到终点
        eta, blockers = eta_of(carrier, goal)
        options.append({
            "plan": "direct_wait",
            "feasible": eta is not None,
            "eta": eta,
            "added_wait": None if eta is None else max(0, eta - self.tick - static_dist(carrier.pos, goal)),
            "conflicts": sorted(blockers),
            "unfulfilled_handoffs": side_effects([carrier.robot_id], eta),
        })

        # 方案二:换机器人接力,在交接区转交
        eta_give, block_give = eta_of(carrier, zone[0])
        eta_take, block_take = eta_of(second, zone[1])
        conflicts = block_give | block_take
        if eta_give is None or eta_take is None:
            options.append({
                "plan": "robot_relay",
                "feasible": False,
                "eta": None,
                "added_wait": None,
                "conflicts": sorted(conflicts),
                "unfulfilled_handoffs": side_effects([carrier.robot_id, reliever], None),
            })
        else:
            handoff_tick = max(eta_give, eta_take) + 1  # 双方到齐后下一刻完成转移
            eta2 = handoff_tick + static_dist(zone[1], goal)
            options.append({
                "plan": "robot_relay",
                "feasible": True,
                "eta": eta2,
                "added_wait": max(0, eta2 - self.tick - static_dist(carrier.pos, goal)),
                "conflicts": sorted(conflicts),
                "unfulfilled_handoffs": side_effects([carrier.robot_id, reliever], eta2),
            })

        # 方案三:临时托管,先放入托管位,接手机器人稍后取出
        eta_place, block_place = eta_of(carrier, access)
        eta_collect, block_collect = eta_of(second, access)
        conflicts = block_place | block_collect
        slot_occupant = area.occupancy.get(slot)
        if slot_occupant is not None:
            conflicts = set(conflicts) | {slot_owner_id(area_id, slot)}
        if eta_place is None or eta_collect is None or slot_occupant is not None:
            options.append({
                "plan": "staging",
                "feasible": False,
                "eta": None,
                "added_wait": None,
                "conflicts": sorted(conflicts),
                "unfulfilled_handoffs": side_effects([carrier.robot_id, reliever], None),
            })
        else:
            place_tick = eta_place + 1
            pick_tick = max(place_tick, eta_collect)
            eta3 = pick_tick + static_dist(access, goal)
            options.append({
                "plan": "staging",
                "feasible": True,
                "eta": eta3,
                "added_wait": max(0, eta3 - self.tick - static_dist(carrier.pos, goal)),
                "conflicts": sorted(conflicts),
                "unfulfilled_handoffs": side_effects([carrier.robot_id, reliever], eta3),
            })
        self._log("relay_compared", cargo=cargo_id, goal=list(goal), options=options)
        return options
