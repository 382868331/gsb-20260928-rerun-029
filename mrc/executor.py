"""执行器:按 tick 推进,记录调度、封路与实际移动日志."""

from __future__ import annotations

import json
from pathlib import Path

from .coordinator import Coordinator
from .handoff import BOTH_PRESENT, CANCELLED, COMPLETED, PLANNED, BLOCKED, Handoff
from .planner import DEADLINE_INFEASIBLE, PlanResult, plan_path_result
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
