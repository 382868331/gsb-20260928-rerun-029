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
    wait_classifications: dict[str, list[str]] = field(default_factory=dict)
    terminated: list[str] = field(default_factory=list)
    reactivations: list[str] = field(default_factory=list)
    dispositions: list[str] = field(default_factory=list)
    ticks: int = 0

    @property
    def ok(self) -> bool:
        return not (
            self.collisions
            or self.edge_swaps
            or self.closed_violations
            or [u for u in self.unfinished if not any(u in item for item in self.terminated)]
            or [
                item for item in self.reactivations
                if "试图重新激活" in item
            ]
        )

    def summary(self) -> str:
        lines = [f"重放 {self.ticks} ticks"]
        lines.append(
            f"碰撞: {len(self.collisions)}  对向换边: {len(self.edge_swaps)}  "
            f"闯封闭区: {len(self.closed_violations)}  超时: {len(self.deadline_misses)}"
        )
        lines.append(f"未完成: {self.unfinished or '无'}  饥饿风险: {self.starved or '无'}")
        if self.terminated:
            lines.append(f"已终止: {self.terminated}")
        if self.reactivations:
            lines.append(f"违规再激活: {self.reactivations}")
        if self.blocked_reasons:
            reasons = "; ".join(f"{rid}: {','.join(reasons)}" for rid, reasons in self.blocked_reasons.items())
            lines.append(f"阻塞原因: {reasons}")
        if self.wait_classifications:
            classes = "; ".join(
                f"{rid}: {','.join(reasons)}"
                for rid, reasons in self.wait_classifications.items()
            )
            lines.append(f"等待分类: {classes}")
        lines.append("结论: " + ("通过" if self.ok else "未通过"))
        return "\n".join(lines)


def replay(log_path: str | Path, starvation_limit: int = 30) -> ReplayReport:
    report = ReplayReport()
    prev: dict[str, tuple[int, int]] = {}
    wait_run: dict[str, int] = {}
    done: set[str] = set()
    assigned: set[str] = set()
    paused: set[str] = set()
    closed: set[tuple[int, int]] = set()
    closures: list[set[tuple[int, int]]] = []
    exiting_allowed: set[str] = set()
    terminated_ids: dict[str, str] = {}

    for line in Path(log_path).read_text().splitlines():
        rec = json.loads(line)
        ev = rec["event"]
        if ev == "assign":
            assigned.add(rec["robot"])
            if rec["robot"] in terminated_ids:
                report.reactivations.append(
                    f"tick {rec['tick']}: {rec['robot']} 任务 {rec['task']} "
                    f"试图重新激活已终止任务({terminated_ids[rec['robot']]})"
                )
        elif ev == "reject_reactivation":
            report.reactivations.append(
                f"tick {rec['tick']}: {rec['robot']} 任务 {rec['task']} "
                f"再分配被拒绝({rec.get('prior_reason')})"
            )
        elif ev == "terminated":
            terminated_ids[rec["robot"]] = rec.get("end_reason", "terminated")
            report.terminated.append(
                f"{rec['robot']} 任务 {rec.get('task')} 终止于 tick {rec['tick']}: "
                f"{rec.get('end_reason')}"
            )
        elif ev == "disposition":
            report.dispositions.append(
                f"tick {rec['tick']} {rec['robot']}({rec.get('task')}): "
                f"{rec['decision']}/{rec['reason']}"
            )
            reason = rec.get("reason")
            if reason in ("blocked", "deadline_infeasible", "search_limit",
                          "exit_blocked", "await_reopen"):
                report.wait_classifications.setdefault(rec["robot"], []).append(reason)
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
            prev = pos

    report.unfinished = sorted(assigned - done - set(terminated_ids))
    for rid, reasons in list(report.blocked_reasons.items()):
        unique: list[str] = []
        for reason in reasons:
            if reason not in unique:
                unique.append(reason)
        report.blocked_reasons[rid] = unique
    return report
