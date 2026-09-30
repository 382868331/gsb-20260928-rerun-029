"""时间窗 A*:在 (格, 时刻) 空间内搜索,绕开他人预约."""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from .gridmap import Cell, GridMap
from .reservation import ReservationTable


INFEASIBLE = "infeasible"
SEARCH_LIMIT = "search_limit"


def _heuristic(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


@dataclass
class PlanResult:
    status: str
    path: list[Cell] | None = None
    blockers: set[str] = field(default_factory=set)

    @property
    def found(self) -> bool:
        return self.path is not None


def plan_path_result(
    gmap: GridMap,
    reservations: ReservationTable,
    start: Cell,
    goal: Cell,
    start_tick: int,
    horizon: int = 200,
    robot_id: str | None = None,
    deadline_tick: int | None = None,
    allowed_closed: set[Cell] | None = None,
) -> PlanResult:
    """返回含起点的占用序列 path,path[i] 为 start_tick+i 时刻所在格."""
    allowed_closed = allowed_closed or set()
    start_ok = gmap.passable(start) or start in allowed_closed
    goal_ok = gmap.passable(goal) or goal in allowed_closed
    if not start_ok or not goal_ok:
        return PlanResult(INFEASIBLE, blockers={"closed"} if not gmap.passable(goal) else set())

    deadline_remaining = None if deadline_tick is None else max(0, deadline_tick - start_tick)
    limit = min(horizon, deadline_remaining if deadline_remaining is not None else horizon)
    blockers: set[str] = set()
    open_heap: list[tuple[int, int, Cell]] = [
        (_heuristic(start, goal), start_tick, start)
    ]
    came_from: dict[tuple[Cell, int], tuple[Cell, int] | None] = {(start, start_tick): None}
    g_score: dict[tuple[Cell, int], int] = {(start, start_tick): 0}
    tried = 0
    max_states = max(1000, (gmap.width * gmap.height) * max(4, min(limit + 1, 40)))

    while open_heap:
        _, t, cell = heapq.heappop(open_heap)
        tried += 1
        if cell == goal:
            path = [cell]
            key = (cell, t)
            while came_from[key] is not None:
                key = came_from[key]  # type: ignore[assignment]
                path.append(key[0])
            path.reverse()
            return PlanResult("found", path)
        if t - start_tick >= limit or tried >= max_states:
            continue
        neighbors = list(gmap.neighbors(cell))
        if cell in allowed_closed:
            x, y = cell
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt_closed = (x + dx, y + dy)
                if (
                    gmap.in_bounds(nxt_closed)
                    and nxt_closed not in gmap.obstacles
                    and nxt_closed in allowed_closed
                    and nxt_closed not in neighbors
                ):
                    neighbors.append(nxt_closed)
        for nxt in neighbors + [cell]:  # 允许原地等待
            nt = t + 1
            if not reservations.is_free(nxt, nt, from_cell=cell, owner=robot_id):
                blockers.update(reservations.blockers(nxt, nt, cell, robot_id))
                continue
            key = (nxt, nt)
            next_g = t + 1 - start_tick
            if g_score.get(key, 1 << 30) > next_g:
                g_score[key] = next_g
                came_from[key] = (cell, t)
                heapq.heappush(
                    open_heap,
                    (next_g + _heuristic(nxt, goal), nt, nxt),
                )

    static_start = set(reservations.blockers(start, start_tick, owner=robot_id))
    static_goal = set(reservations.blockers(goal, start_tick, owner=robot_id))
    timed_owners = _timed_reservation_owners(
        gmap, reservations, start, start_tick, limit, robot_id,
    )
    goal_owner = reservations.goal.get(goal)
    if goal_owner is not None and goal_owner != robot_id:
        timed_owners.add(goal_owner)
    if static_start or static_goal or _static_unreachable(gmap, start, goal, allowed_closed):
        blockers.update(static_start)
        blockers.update(static_goal)
        blockers.update(timed_owners)
        return PlanResult(INFEASIBLE, blockers=blockers)
    blockers.update(timed_owners)
    return PlanResult(SEARCH_LIMIT, blockers=blockers)


def _timed_reservation_owners(
    gmap: GridMap,
    reservations: ReservationTable,
    start: Cell,
    start_tick: int,
    limit: int,
    robot_id: str | None,
) -> set[str]:
    pending = [start]
    seen = {start}
    owners: set[str] = set()
    while pending:
        cell = pending.pop()
        for tick in range(start_tick, start_tick + limit + 1):
            owners.update(reservations.blockers(cell, tick, owner=robot_id))
        for nxt in gmap.neighbors(cell):
            if nxt not in seen:
                seen.add(nxt)
                pending.append(nxt)
    return owners


def _static_unreachable(
    gmap: GridMap,
    start: Cell,
    goal: Cell,
    allowed_closed: set[Cell] | None = None,
) -> bool:
    pending = [start]
    seen = {start}
    while pending:
        cell = pending.pop()
        if cell == goal:
            return False
        neighbors = list(gmap.neighbors(cell))
        allowed_closed = allowed_closed or set()
        if cell in allowed_closed:
            x, y = cell
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt_closed = (x + dx, y + dy)
                if (
                    gmap.in_bounds(nxt_closed)
                    and nxt_closed not in gmap.obstacles
                    and nxt_closed in allowed_closed
                ):
                    neighbors.append(nxt_closed)
        for nxt in neighbors:
            if nxt not in seen:
                seen.add(nxt)
                pending.append(nxt)
    return True


def plan_path(
    gmap: GridMap,
    reservations: ReservationTable,
    start: Cell,
    goal: Cell,
    start_tick: int,
    horizon: int = 200,
    robot_id: str | None = None,
    deadline_tick: int | None = None,
    allowed_closed: set[Cell] | None = None,
) -> list[Cell] | None:
    result = plan_path_result(
        gmap, reservations, start, goal, start_tick, horizon,
        robot_id=robot_id, deadline_tick=deadline_tick, allowed_closed=allowed_closed,
    )
    return result.path
