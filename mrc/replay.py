# -*- coding: utf-8 -*-
"""逐时刻重放检查:碰撞、安全距离、对向换边、封路合规、计划一致、完成与饥饿。"""

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
    violations: list[str] = field(default_factory=list)
    blocked_notes: list[str] = field(default_factory=list)
    infeasible: list[str] = field(default_factory=list)
    deadline_missed: list[str] = field(default_factory=list)
    ticks: int = 0

    @property
    def ok(self) -> bool:
        return not (self.collisions or self.edge_swaps or self.unfinished
                    or self.starved or self.violations or self.deadline_missed)

    def summary(self) -> str:
        lines = [f'重放 {self.ticks} ticks']
        lines.append(f'碰撞: {len(self.collisions)}  对向换边: {len(self.edge_swaps)}')
        lines.append(f'违规: {len(self.violations)}  截止超时: {len(self.deadline_missed)}')
        lines.append(f'未完成: {self.unfinished or "无"}  饥饿: {self.starved or "无"}')
        lines.append(f'无可行安排: {self.infeasible or "无"}')
        lines.append(f'尚未找到(等待中): {self.blocked_notes or "无"}')
        lines.append('结论: ' + ('通过' if self.ok else '未通过'))
        return '\n'.join(lines)


def replay(log_path: str | Path, starvation_limit: int = 30) -> ReplayReport:
    report = ReplayReport()
    prev: dict[str, tuple[int, int]] = {}
    wait_run: dict[str, int] = {}
    paused: set[str] = set()
    closed: set[tuple[int, int]] = set()
    evacuating: set[str] = set()
    plans: dict[str, list[dict]] = {}
    tasks: dict[str, str] = {}
    done_tasks: set[str] = set()
    deadlines: dict[str, int] = {}
    done_robots: set[str] = set()

    for line in Path(log_path).read_text().splitlines():
        rec = json.loads(line)
        ev = rec['event']
        if ev == 'assign':
            rid = rec['robot']
            tasks[rid] = rec['task']
            done_robots.discard(rid)
            if rec.get('deadline') is not None:
                deadlines[rid] = rec['deadline']
        elif ev == 'done':
            done_tasks.add(rec['task'])
            done_robots.add(rec['robot'])
            if (rec.get('deadline') is not None
                    and rec['tick_done'] > rec['deadline']):
                report.deadline_missed.append(
                    f"{rec['task']} 完成于 tick {rec['tick_done']} > 截止 {rec['deadline']}")
        elif ev == 'pause':
            paused.add(rec['robot'])
        elif ev == 'resume':
            paused.discard(rec['robot'])
        elif ev == 'evacuate':
            evacuating.add(rec['robot'])
        elif ev == 'close':
            closed |= {tuple(c) for c in rec['cells']}
        elif ev == 'open':
            closed -= {tuple(c) for c in rec['cells']}
        elif ev == 'blocked':
            note = f"{rec['robot']}:{rec['reason']}"
            if rec['kind'] == 'infeasible':
                report.infeasible.append(note)
            else:
                report.blocked_notes.append(note)
        elif ev == 'deadline_missed':
            report.deadline_missed.append(
                f"{rec['task']} tick {rec['tick']} 已超过截止 {rec['deadline']}")
        elif ev == 'starve_risk':
            report.starved.append(f"{rec['robot']} 等待 {rec['waited']} ticks")
        elif ev == 'plan':
            for rid, path in rec['plans'].items():
                plans.setdefault(rid, []).append(
                    {'start': rec['tick'], 'path': [tuple(c) for c in path]})
        elif ev == 'positions':
            tick = rec['tick']
            report.ticks = max(report.ticks, tick)
            pos = {rid: tuple(p) for rid, p in rec['pos'].items()}
            seen: dict[tuple[int, int], str] = {}
            for rid, cell in pos.items():
                if cell in seen:
                    report.collisions.append(
                        f'tick {tick}: {seen[cell]} 与 {rid} 同在 {cell}')
                seen[cell] = rid
            for rid, cell in pos.items():
                if rid in prev:
                    for other, ocell in pos.items():
                        if other != rid and prev.get(other) == cell and ocell == prev[rid]:
                            report.edge_swaps.append(
                                f'tick {tick}: {rid} 与 {other} 对向换边')
            for rid, cell in pos.items():
                if cell in closed and rid not in evacuating:
                    report.violations.append(
                        f'tick {tick}: 非退出机器人 {rid} 进入封闭格 {cell}')
                plan = None
                history = plans.get(rid, [])
                for i, candidate in enumerate(history):
                    end = candidate['start'] + len(candidate['path'])
                    if i + 1 < len(history):
                        end = min(end, history[i + 1]['start'])
                    if candidate['start'] <= tick < end:
                        plan = candidate
                if plan and rid not in paused and rid not in done_robots:
                    idx = tick - plan['start']
                    path = plan['path']
                    if (0 <= idx < len(path) and path[idx] != cell
                            and any(p != path[0] for p in path)):
                        report.violations.append(
                            f'tick {tick}: {rid} 在 {cell},预约要求 {path[idx]}')
                if rid in paused or rid in done_robots or rid not in tasks:
                    wait_run[rid] = 0
                elif prev.get(rid) == cell:
                    wait_run[rid] = wait_run.get(rid, 0) + 1
                    if wait_run[rid] == starvation_limit:
                        report.starved.append(
                            f'{rid} 连续 {starvation_limit} ticks 未移动')
                else:
                    wait_run[rid] = 0
            for rid in list(evacuating):
                if rid in pos and pos[rid] not in closed:
                    evacuating.discard(rid)
            prev = pos

    unfinished_tasks = sorted(set(tasks.values()) - done_tasks)
    report.unfinished = unfinished_tasks
    return report
