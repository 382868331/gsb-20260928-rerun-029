# -*- coding: utf-8 -*-
"""时间窗 A*:在 (格, 时刻) 空间内搜索,绕开他人预约.

结果三态:
- feasible:找到可行安排;
- infeasible:静态判定无可行安排(不连通 / 终点封闭 / 截止时间不可达);
- not_found:在给定时间窗(horizon)内尚未找到方案,但不排除未来可达。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from .gridmap import Cell, GridMap
from .reservation import ReservationTable


def _heuristic(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


@dataclass
class PlanResult:
    status: str  # feasible | infeasible | not_found
    path: list[Cell] | None = None
    blockers: list[str] = field(default_factory=list)


def _static_distance(gmap: GridMap, start: Cell, goal: Cell,
                     allow_closed: bool = False) -> int | None:
    """忽略时空预约的 BFS,判定静态可达性与最短步数."""
    if start == goal:
        return 0
    dist = {start: 0}
    queue = [start]
    while queue:
        cell = queue.pop(0)
        for nxt in gmap.neighbors(cell, allow_closed=allow_closed):
            if nxt in dist:
                continue
            dist[nxt] = dist[cell] + 1
            if nxt == goal:
                return dist[nxt]
            queue.append(nxt)
    return None


def plan_path(
    gmap: GridMap,
    reservations: ReservationTable,
    start: Cell,
    goal: Cell,
    start_tick: int,
    horizon: int = 200,
    deadline: int | None = None,
    allow_closed: bool = False,
    exit_zone: bool = False,
) -> PlanResult:
    """时间窗 A*。成功时 path[i] 为 start_tick+i 时刻所在格(含起点)。

    deadline: 最晚到达时刻;最早可达晚于截止则 infeasible。
    allow_closed + exit_zone:封路退出模式,允许穿越封闭格,
        目标改为离开封闭区段(到达封闭区之外的相邻开放格)。
    """
    if not exit_zone and not gmap.passable(goal, allow_closed=allow_closed):
        return PlanResult("infeasible", blockers=[f"终点 {goal} 不可通行(障碍或封闭)"])
    if not exit_zone and goal in reservations.goal:
        owner = reservations.goal[goal]
        return PlanResult("infeasible", blockers=[f"终点 {goal} 被 {owner} 永久预约"])

    if exit_zone:
        if start not in gmap.closures:
            return PlanResult("feasible", path=[start])
        lower = None
        for target in gmap.closures:
            for nxt in gmap.neighbors(target, allow_closed=True):
                if nxt not in gmap.closures:
                    d = _static_distance(gmap, start, nxt, allow_closed=True)
                    if d is not None and (lower is None or d < lower):
                        lower = d
        if lower is None:
            return PlanResult("infeasible", blockers=["封闭区段无通往外界的出口"])
    else:
        static_dist = _static_distance(
            gmap, start, goal, allow_closed=allow_closed)
        if static_dist is None:
            return PlanResult("infeasible", blockers=["起点与终点在当前路网下不连通"])
        lower = static_dist
        if deadline is not None and start_tick + static_dist > deadline:
            return PlanResult(
                "infeasible",
                blockers=[f"最早到达 tick {start_tick + static_dist} 晚于截止 {deadline}"])

    def is_goal(cell: Cell) -> bool:
        if not exit_zone:
            return cell == goal
        return cell not in gmap.closures and cell != start

    time_budget = horizon
    if deadline is not None:
        time_budget = min(horizon, max(0, deadline - start_tick))

    open_heap: list[tuple[int, int, Cell]] = [(_heuristic(start, goal), start_tick, start)]
    came_from: dict[tuple[Cell, int], tuple[Cell, int] | None] = {(start, start_tick): None}
    g_score: dict[tuple[Cell, int], int] = {(start, start_tick): 0}
    blockers: set[str] = set()

    while open_heap:
        _, t, cell = heapq.heappop(open_heap)
        if is_goal(cell):
            path = [cell]
            key = (cell, t)
            while came_from[key] is not None:
                key = came_from[key]  # type: ignore[assignment]
                path.append(key[0])
            path.reverse()
            return PlanResult("feasible", path=path)
        if t - start_tick >= time_budget:
            continue
        for nxt in gmap.neighbors(cell, allow_closed=allow_closed or exit_zone) + [cell]:
            nt = t + 1
            if not reservations.is_free(nxt, nt, from_cell=cell):
                owner = reservations.reserved_by(nxt, nt, from_cell=cell)
                if owner and t - start_tick < 6:
                    blockers.add(f"({nxt},t{nt}) 被 {owner} 占用")
                continue
            if not exit_zone and nt + _heuristic(nxt, goal) > (deadline or 1 << 30):
                blockers.add(f"经 {nxt} 的路径无法在截止 {deadline} 前到达")
                continue
            key = (nxt, nt)
            if g_score.get(key, 1 << 30) > t + 1 - start_tick:
                g_score[key] = t + 1 - start_tick
                came_from[key] = (cell, t)
                heapq.heappush(
                    open_heap,
                    (t + 1 - start_tick + _heuristic(nxt, goal), nt, nxt))

    if deadline is not None:
        return PlanResult("infeasible",
                          blockers=sorted(blockers) or [f"截止 {deadline} 前无时空安排"])
    return PlanResult("not_found",
                      blockers=sorted(blockers) or ["时间窗内尚未找到可插入的时空安排"])
