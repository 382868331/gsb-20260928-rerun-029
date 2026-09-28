"""时间窗 A*:在 (格, 时刻) 空间内搜索,绕开他人预约."""

from __future__ import annotations

import heapq

from .gridmap import Cell, GridMap
from .reservation import ReservationTable


def _heuristic(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def plan_path(
    gmap: GridMap,
    reservations: ReservationTable,
    start: Cell,
    goal: Cell,
    start_tick: int,
    horizon: int = 200,
) -> list[Cell] | None:
    """返回含起点的占用序列 path,path[i] 为 start_tick+i 时刻所在格."""
    open_heap: list[tuple[int, int, Cell]] = [( _heuristic(start, goal), start_tick, start)]
    came_from: dict[tuple[Cell, int], tuple[Cell, int] | None] = {(start, start_tick): None}
    g_score: dict[tuple[Cell, int], int] = {(start, start_tick): 0}

    while open_heap:
        _, t, cell = heapq.heappop(open_heap)
        if cell == goal:
            path = [cell]
            key = (cell, t)
            while came_from[key] is not None:
                key = came_from[key]  # type: ignore[assignment]
                path.append(key[0])
            path.reverse()
            return path
        if t - start_tick >= horizon:
            continue
        for nxt in gmap.neighbors(cell) + [cell]:  # 允许原地等待
            nt = t + 1
            if not reservations.is_free(nxt, nt, from_cell=cell):
                continue
            key = (nxt, nt)
            if g_score.get(key, 1 << 30) > t + 1 - start_tick:
                g_score[key] = t + 1 - start_tick
                came_from[key] = (cell, t)
                heapq.heappush(open_heap, (t + 1 - start_tick + _heuristic(nxt, goal), nt, nxt))
    return None
