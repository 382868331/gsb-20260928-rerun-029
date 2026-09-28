"""重放检查:安全距离、完成情况、饥饿."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class ReplayReport:
    collisions: list[str] = field(default_factory=list)
    edge_swaps: list[str] = field(default_factory=list)
    unfinished: list[str] = field(default_factory=list)
    starved: list[str] = field(default_factory=list)
    ticks: int = 0

    @property
    def ok(self) -> bool:
        return not (self.collisions or self.edge_swaps or self.unfinished or self.starved)

    def summary(self) -> str:
        lines = [f"重放 {self.ticks} ticks"]
        lines.append(f"碰撞: {len(self.collisions)}  对向换边: {len(self.edge_swaps)}")
        lines.append(f"未完成: {self.unfinished or '无'}  饥饿: {self.starved or '无'}")
        lines.append("结论: " + ("通过" if self.ok else "未通过"))
        return "\n".join(lines)


def replay(log_path: str | Path, starvation_limit: int = 30) -> ReplayReport:
    report = ReplayReport()
    prev: dict[str, tuple[int, int]] = {}
    wait_run: dict[str, int] = {}
    done: set[str] = set()
    assigned: set[str] = set()
    paused: set[str] = set()

    for line in Path(log_path).read_text().splitlines():
        rec = json.loads(line)
        ev = rec["event"]
        if ev == "assign":
            assigned.add(rec["robot"])
        elif ev == "done":
            done.add(rec["robot"])
        elif ev == "pause":
            paused.add(rec["robot"])
        elif ev == "resume":
            paused.discard(rec["robot"])
        elif ev == "starve_risk":
            report.starved.append(f"{rec['robot']} 等待 {rec['waited']} ticks")
        elif ev == "positions":
            tick = rec["tick"]
            report.ticks = max(report.ticks, tick)
            pos = {rid: tuple(p) for rid, p in rec["pos"].items()}
            # 顶点碰撞 / 安全距离:任意两机不得同格
            seen: dict[tuple[int, int], str] = {}
            for rid, cell in pos.items():
                if cell in seen:
                    report.collisions.append(
                        f"tick {tick}: {seen[cell]} 与 {rid} 同在 {cell}")
                seen[cell] = rid
            # 对向换边
            for rid, cell in pos.items():
                if rid in prev:
                    for other, ocell in pos.items():
                        if other != rid and prev.get(other) == cell and ocell == prev[rid]:
                            report.edge_swaps.append(
                                f"tick {tick}: {rid} 与 {other} 对向换边")
            # 饥饿:连续未移动
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

    report.unfinished = sorted(assigned - done)
    return report
