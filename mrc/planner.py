"""时间窗 A*:在 (格, 时刻) 空间内搜索,绕开他人预约."""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from .gridmap import Cell, GridMap
from .reservation import ReservationTable


INFEASIBLE = "infeasible"
SEARCH_LIMIT = "search_limit"
DEADLINE_INFEASIBLE = "deadline_infeasible"
FOUND = "found"


def _heuristic(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _permanent_goal_blockers(
    reservations: ReservationTable,
    goal: Cell,
    start_tick: int,
    robot_id: str | None,
) -> set[str]:
    """goal 锁在整个搜索窗口内都有效时才算永久占位(holds 也算)."""
    owner = reservations.goal.get(goal)
    out: set[str] = set()
    if owner not in (None, robot_id) and reservations.permanent_holds.get(goal) == owner:
        out.add(owner)
    hold_owner = reservations.permanent_holds.get(goal)
    if hold_owner not in (None, robot_id):
        out.add(hold_owner)
    return out


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
    first = _run_search(
        gmap, reservations, start, goal, start_tick, horizon,
        robot_id, deadline_tick, allowed_closed,
    )
    if first.found:
        return first
    blockers: set[str] = set(first.blockers)
    deadline_remaining = None if deadline_tick is None else max(0, deadline_tick - start_tick)
    limit = min(horizon, deadline_remaining if deadline_remaining is not None else horizon)

    static_goal = _permanent_goal_blockers(reservations, goal, start_tick, robot_id)
    if static_goal or _static_unreachable(
        gmap, start, goal, allowed_closed, reservations, robot_id,
        traverse_closed=not allowed_closed,
    ):
        timed_owners = _timed_reservation_owners(
            gmap, reservations, start, start_tick, limit, robot_id,
        )
        blockers.update(static_goal)
        blockers.update(timed_owners)
        return PlanResult(INFEASIBLE, blockers=blockers)

    # 起点/终点当前未被永久锁死:区分"暂时受阻(迟早能到但赶不上)"与
    # "搜索预算不足"。给一次不受截止约束的完整预算搜索。
    relaxed = _run_search(
        gmap, reservations, start, goal, start_tick, horizon,
        robot_id, deadline_tick=None, allowed_closed=allowed_closed,
    )
    if relaxed.found:
        timed_owners = _timed_reservation_owners(
            gmap, reservations, start, start_tick, limit, robot_id,
        )
        blockers.update(t for t in timed_owners if t)
        status = DEADLINE_INFEASIBLE if deadline_tick is not None else SEARCH_LIMIT
        return PlanResult(status, blockers=blockers)
    if relaxed.status == INFEASIBLE:
        blockers.update(relaxed.blockers)
        return PlanResult(INFEASIBLE, blockers=blockers)
    timed_owners = _timed_reservation_owners(
        gmap, reservations, start, start_tick, limit, robot_id,
    )
    blockers.update(t for t in timed_owners if t)
    blockers.update(relaxed.blockers)
    return PlanResult(SEARCH_LIMIT, blockers=blockers)


def _run_search(
    gmap: GridMap,
    reservations: ReservationTable,
    start: Cell,
    goal: Cell,
    start_tick: int,
    horizon: int,
    robot_id: str | None,
    deadline_tick: int | None,
    allowed_closed: set[Cell],
) -> PlanResult:
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
            return PlanResult(FOUND, path)
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
            sticky = reservations.sticky_holds.get(nxt)
            if sticky not in (None, robot_id):
                blockers.add(sticky)
                continue
            if not reservations.is_free(
                nxt, nt, from_cell=cell, owner=robot_id,
                ignore_holds=(nt > start_tick + 1),
            ):
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

    static_goal = _permanent_goal_blockers(reservations, goal, start_tick, robot_id)
    if static_goal or _static_unreachable(
        gmap, start, goal, allowed_closed, reservations, robot_id,
    ):
        timed_owners = _timed_reservation_owners(
            gmap, reservations, start, start_tick, limit, robot_id,
        )
        blockers.update(static_goal)
        blockers.update(timed_owners)
        return PlanResult(INFEASIBLE, blockers=blockers)
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
    reservations: ReservationTable | None = None,
    robot_id: str | None = None,
    traverse_closed: bool = False,
) -> bool:
    pending = [start]
    seen = {start}

    def permanently_blocked(cell: Cell) -> bool:
        if reservations is None:
            return False
        hold_owner = reservations.permanent_holds.get(cell)
        if hold_owner not in (None, robot_id):
            return True
        goal_owner = reservations.goal.get(cell)
        if goal_owner not in (None, robot_id) and reservations.goal_ready.get(cell, 0) == 0:
            return True
        return False

    while pending:
        cell = pending.pop()
        if cell == goal:
            return False
        neighbors = list(gmap.neighbors(cell))
        if traverse_closed:
            x, y = cell
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt0 = (x + dx, y + dy)
                if (
                    gmap.in_bounds(nxt0)
                    and nxt0 not in gmap.obstacles
                    and nxt0 in gmap.closed_cells
                    and nxt0 not in neighbors
                ):
                    neighbors.append(nxt0)
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
            if nxt != goal and permanently_blocked(nxt):
                continue
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
