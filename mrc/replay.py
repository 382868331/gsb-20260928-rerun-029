"""重放检查:碰撞、对向换边、封闭区闯入、截止时间和完成情况."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class ReplayReport:
    collisions: list[str] = field(default_factory=list)
    edge_swaps: list[str] = field(default_factory=list)
    closed_violations: list[str] = field(default_factory=list)
    deadline_misses: list[str] = field(default_factory=list)
    unfinished: list[str] = field(default_factory=list)
    starved: list[str] = field(default_factory=list)
    blocked_reasons: dict[str, list[str]] = field(default_factory=dict)
    terminated: list[str] = field(default_factory=list)
    ownership_violations: list[str] = field(default_factory=list)
    handoff_violations: list[str] = field(default_factory=list)
    staging_violations: list[str] = field(default_factory=list)
    resurrected: list[str] = field(default_factory=list)
    decision_reasons: dict[str, list[str]] = field(default_factory=dict)
    ticks: int = 0

    @property
    def ok(self) -> bool:
        return not (
            self.collisions
            or self.edge_swaps
            or self.closed_violations
            or self.unfinished
            or self.ownership_violations
            or self.handoff_violations
            or self.staging_violations
            or self.resurrected
        )

    def summary(self) -> str:
        lines = [f"重放 {self.ticks} ticks"]
        lines.append(
            f"碰撞: {len(self.collisions)}  对向换边: {len(self.edge_swaps)}  "
            f"闯封闭区: {len(self.closed_violations)}  超时: {len(self.deadline_misses)}"
        )
        lines.append(f"未完成: {self.unfinished or '无'}  饥饿风险: {self.starved or '无'}")
        lines.append(
            f"归属异常: {self.ownership_violations or '无'}  "
            f"交接异常: {self.handoff_violations or '无'}  复活任务: {self.resurrected or '无'}"
        )
        lines.append(f"托管异常: {self.staging_violations or '无'}")
        if self.blocked_reasons:
            reasons = "; ".join(f"{rid}: {','.join(reasons)}" for rid, reasons in self.blocked_reasons.items())
            lines.append(f"阻塞原因: {reasons}")
        lines.append("结论: " + ("通过" if self.ok else "未通过"))
        return "\n".join(lines)


def replay(log_path: str | Path, starvation_limit: int = 30) -> ReplayReport:
    report = ReplayReport()
    prev: dict[str, tuple[int, int]] = {}
    wait_run: dict[str, int] = {}
    done: set[str] = set()
    assigned: set[str] = set()
    terminated: set[str] = set()
    paused: set[str] = set()
    closed: set[tuple[int, int]] = set()
    closures: list[set[tuple[int, int]]] = []
    exiting_allowed: set[str] = set()
    cargo_owner: dict[str, str] = {}
    handoffs: dict[str, dict] = {}
    completed_handoffs: set[str] = set()
    cancelled_handoffs: set[str] = set()
    released: set[str] = set()
    terminated_tasks: set[str] = set()
    pending_zone_checks: list[tuple[int, str]] = []
    staging_areas: dict[str, dict] = {}
    slot_occupancy: dict[tuple[int, int], str] = {}
    slot_ops: dict[tuple[tuple[int, int], int], int] = {}
    staged_cargo: dict[str, tuple[int, int]] = {}
    relay_carriers: dict[str, set[str]] = {}

    for line in Path(log_path).read_text().splitlines():
        rec = json.loads(line)
        ev = rec["event"]
        if ev == "assign":
            assigned.add(rec["robot"])
            task_id = str(rec.get("task"))
            if task_id in terminated_tasks:
                report.resurrected.append(
                    f"tick {rec['tick']}: 已终止任务 {task_id} 被重新分配给 {rec['robot']}"
                )
        elif ev == "handoff_planned":
            hid = rec["handoff"]
            if hid in completed_handoffs or hid in cancelled_handoffs:
                report.handoff_violations.append(
                    f"tick {rec['tick']}: 已完结交接 {hid} 被重复规划"
                )
            handoffs[hid] = {
                "cargo": rec["cargo"],
                "giver": rec["giver"],
                "receiver": rec["receiver"],
                "zone": (tuple(rec["zone"][0]), tuple(rec["zone"][1])),
                "started": False,
            }
            owner = cargo_owner.get(rec["cargo"])
            if owner is not None and owner != rec["giver"]:
                report.ownership_violations.append(
                    f"tick {rec['tick']}: 货物 {rec['cargo']} 归属 {owner},却由 {rec['giver']} 交出"
                )
            cargo_owner[rec["cargo"]] = rec["giver"]
        elif ev == "handoff_switched":
            info = handoffs.get(rec["handoff"])
            if info is not None:
                info["zone"] = (tuple(rec["zone"][0]), tuple(rec["zone"][1]))
        elif ev == "handoff_started":
            hid = rec["handoff"]
            if hid in handoffs:
                handoffs[hid]["started"] = True
        elif ev == "handoff_complete":
            hid = rec["handoff"]
            info = handoffs.get(hid)
            if hid in completed_handoffs:
                report.handoff_violations.append(
                    f"tick {rec['tick']}: 交接 {hid} 重复完成(重复搬运)"
                )
            elif info is None or not info["started"]:
                report.handoff_violations.append(
                    f"tick {rec['tick']}: 交接 {hid} 未等双方到齐就完成"
                )
            else:
                cargo = info["cargo"]
                if cargo_owner.get(cargo) != info["giver"]:
                    report.ownership_violations.append(
                        f"tick {rec['tick']}: 交接 {hid} 完成前货物 {cargo} 归属异常"
                    )
                cargo_owner[cargo] = info["receiver"]
                pending_zone_checks.append((rec["tick"], hid))
            completed_handoffs.add(hid)
        elif ev == "handoff_cancelled":
            hid = rec["handoff"]
            cancelled_handoffs.add(hid)
            info = handoffs.get(hid)
            if info:
                released.add(info["giver"])
                released.add(info["receiver"])
        elif ev == "staging_area":
            staging_areas[rec["area"]] = {
                "slots": {tuple(s) for s in rec["slots"]},
                "ops": int(rec.get("ops_per_tick", 1)),
            }
        elif ev in ("relay_planned", "relay_replanned"):
            legs = rec.get("legs") or rec.get("new_legs") or []
            relay_carriers[rec["relay"]] = {
                str(leg["carrier"]) for leg in legs if leg.get("carrier")
            }
            owner = rec.get("owner")
            if owner is not None:
                prev_owner = cargo_owner.get(rec["cargo"])
                if prev_owner is not None and prev_owner != owner:
                    report.ownership_violations.append(
                        f"tick {rec['tick']}: 货物 {rec['cargo']} 归属 {prev_owner},接力计划却记为 {owner}"
                    )
                cargo_owner[rec["cargo"]] = owner
        elif ev in ("stage_place", "stage_pick"):
            slot = tuple(rec["slot"])
            cargo = rec["cargo"]
            tick = rec["tick"]
            area_id = rec["area"]
            owner_id = f"staging:{area_id}@{slot[0]},{slot[1]}"
            key = (area_id, tick)
            slot_ops[key] = slot_ops.get(key, 0) + 1
            limit = staging_areas.get(area_id, {}).get("ops", 1)
            if slot_ops[key] > limit:
                report.staging_violations.append(
                    f"tick {tick}: 托管区 {area_id} 一刻内取放次数超过限制 {limit}"
                )
            if ev == "stage_place":
                if slot in slot_occupancy:
                    report.staging_violations.append(
                        f"tick {tick}: 托管位 {slot} 已有 {slot_occupancy[slot]},又放入 {cargo}"
                    )
                if cargo_owner.get(cargo) != rec["carrier"]:
                    report.ownership_violations.append(
                        f"tick {tick}: 货物 {cargo} 归属 {cargo_owner.get(cargo)},却由 {rec['carrier']} 放置"
                    )
                if cargo in staged_cargo:
                    report.staging_violations.append(
                        f"tick {tick}: 货物 {cargo} 未被取出就重复放置"
                    )
                cargo_owner[cargo] = owner_id
                slot_occupancy[slot] = cargo
                staged_cargo[cargo] = slot
            else:
                if cargo_owner.get(cargo) != owner_id:
                    report.ownership_violations.append(
                        f"tick {tick}: 货物 {cargo} 不在托管位 {slot},却被取出"
                    )
                if slot_occupancy.get(slot) != cargo:
                    report.staging_violations.append(
                        f"tick {tick}: 托管位 {slot} 中没有货物 {cargo},取出无效"
                    )
                carrier = rec["carrier"]
                if carrier in terminated:
                    report.staging_violations.append(
                        f"tick {tick}: 货物 {cargo} 被交给已失去资格的接收者 {carrier}"
                    )
                allowed = relay_carriers.get(rec.get("relay"), set())
                if allowed and carrier not in allowed:
                    report.staging_violations.append(
                        f"tick {tick}: {carrier} 不是接力 {rec.get('relay')} 的承运人,不能取走 {cargo}"
                    )
                cargo_owner[cargo] = carrier
                slot_occupancy.pop(slot, None)
                staged_cargo.pop(cargo, None)
        elif ev == "relay_cancelled":
            released.update(rec.get("released") or [])
        elif ev == "snapshot_loaded":
            for area in rec.get("staging", []):
                for key, cargo in dict(area.get("occupancy", {})).items():
                    x, y = key.split(",")
                    slot = (int(x), int(y))
                    slot_occupancy[slot] = cargo
                    staged_cargo[cargo] = slot
            for cargo, owner in dict(rec.get("cargo", {})).items():
                cargo_owner[cargo] = owner
        elif ev == "done":
            done.add(rec["robot"])
            if rec.get("on_time") is False:
                report.deadline_misses.append(
                    f"{rec['robot']} 任务 {rec['task']} 在 tick {rec['tick']} 超过截止 {rec['deadline']}"
                )
        elif ev == "deadline_missed":
            report.deadline_misses.append(
                f"{rec['robot']} 任务 {rec['task']} 在 tick {rec['tick']} 超过截止 {rec['deadline']}"
            )
            if rec.get("blockers"):
                report.blocked_reasons.setdefault(rec["robot"], []).extend(rec["blockers"])
        elif ev == "task_terminated":
            terminated.add(rec["robot"])
            terminated_tasks.add(str(rec["task"]))
            reason = f"{rec['task']}={rec.get('reason', 'infeasible')}"
            report.terminated.append(f"{rec['robot']} {reason}: {','.join(rec.get('blockers') or [])}")
            report.blocked_reasons.setdefault(rec["robot"], []).append(reason)
            if rec.get("reason") == "deadline_infeasible":
                report.deadline_misses.append(
                    f"{rec['robot']} 任务 {rec['task']} 在 tick {rec['tick']} 已无法满足截止时间"
                )
        elif ev == "task_decision" and rec.get("reason"):
            report.decision_reasons.setdefault(rec["robot"], []).append(
                f"{rec.get('action')}:{rec['reason']}"
            )
        elif ev == "pause":
            paused.add(rec["robot"])
        elif ev == "resume":
            paused.discard(rec["robot"])
        elif ev == "starve_risk":
            report.starved.append(f"{rec['robot']} 等待 {rec['waited']} ticks")
            if rec.get("blockers"):
                report.blocked_reasons.setdefault(rec["robot"], []).extend(rec["blockers"])
        elif ev == "blocked":
            blockers = rec.get("blockers") or []
            if blockers:
                report.blocked_reasons.setdefault(rec["robot"], []).extend(blockers)
        elif ev == "close":
            cells = {tuple(cell) for cell in rec["cells"]}
            closures.append(cells)
            closed.update(cells)
            for rid, cell in prev.items():
                if cell in closed:
                    exiting_allowed.add(rid)
        elif ev == "open":
            wanted = {tuple(cell) for cell in rec["cells"]}
            remaining: list[set[tuple[int, int]]] = []
            for closure in closures:
                left = closure - wanted
                if left:
                    remaining.append(left)
            closures = remaining
            closed = set().union(*closures) if closures else set()
            exiting_allowed.clear()
        elif ev == "positions":
            tick = rec["tick"]
            report.ticks = max(report.ticks, tick)
            pos = {rid: tuple(p) for rid, p in rec["pos"].items()}
            seen: dict[tuple[int, int], str] = {}
            for rid, cell in pos.items():
                if cell in seen:
                    report.collisions.append(
                        f"tick {tick}: {seen[cell]} 与 {rid} 同在 {cell}")
                seen[cell] = rid
                if cell in closed and rid not in exiting_allowed:
                    report.closed_violations.append(f"tick {tick}: {rid} 位于封闭格 {cell}")
                if cell not in closed:
                    exiting_allowed.discard(rid)
            for rid, cell in pos.items():
                if rid in prev:
                    for other, ocell in pos.items():
                        if other != rid and prev.get(other) == cell and ocell == prev[rid]:
                            report.edge_swaps.append(
                                f"tick {tick}: {rid} 与 {other} 对向换边")
            for rid, cell in pos.items():
                if rid in paused or rid in done or rid not in assigned:
                    wait_run[rid] = 0
                elif prev.get(rid) == cell:
                    wait_run[rid] = wait_run.get(rid, 0) + 1
                    if wait_run[rid] == starvation_limit:
                        report.starved.append(f"{rid} 连续 {starvation_limit} ticks 未移动")
                else:
                    wait_run[rid] = 0
            for check_tick, hid in pending_zone_checks:
                if check_tick == tick and hid in handoffs:
                    info = handoffs[hid]
                    zone = info["zone"]
                    if pos.get(info["giver"]) != zone[0] or pos.get(info["receiver"]) != zone[1]:
                        report.handoff_violations.append(
                            f"tick {tick}: 交接 {hid} 完成时双方未都在交接区"
                        )
            pending_zone_checks = [c for c in pending_zone_checks if c[0] > tick]
            prev = pos

    report.unfinished = sorted(assigned - done - terminated - released)
    for rid, reasons in list(report.blocked_reasons.items()):
        unique: list[str] = []
        for reason in reasons:
            if reason not in unique:
                unique.append(reason)
        report.blocked_reasons[rid] = unique
    return report
