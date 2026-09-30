"""时空预约表:协调共享通道的核心数据结构."""

from __future__ import annotations

from .gridmap import Cell


class ReservationTable:
    """顶点预约 + 边预约(防止对向换边).终点格永久预约."""

    def __init__(self) -> None:
        self.vertex: dict[tuple[Cell, int], str] = {}
        self.edge: dict[tuple[Cell, Cell, int], str] = {}
        self.goal: dict[Cell, str] = {}
        self.goal_ready: dict[Cell, int] = {}
        self.holds: dict[Cell, str] = {}
        self.permanent: dict[Cell, str] = {}

    def is_free(
        self,
        cell: Cell,
        t: int,
        from_cell: Cell | None = None,
        owner: str | None = None,
    ) -> bool:
        if self.vertex.get((cell, t)) not in (None, owner):
            return False
        goal_owner = self.goal.get(cell)
        if goal_owner not in (None, owner) and self.goal_ready.get(cell, 0) <= t:
            return False
        if self.holds.get(cell) not in (None, owner):
            return False
        if self.permanent.get(cell) not in (None, owner):
            return False
        if from_cell is not None and self.edge.get((cell, from_cell, t)) not in (None, owner):
            return False
        return True

    def blockers(
        self,
        cell: Cell,
        t: int,
        from_cell: Cell | None = None,
        owner: str | None = None,
    ) -> list[str]:
        out: set[str] = set()
        vertex_owner = self.vertex.get((cell, t))
        goal_owner = self.goal.get(cell)
        if goal_owner is not None and goal_owner != owner and self.goal_ready.get(cell, 0) <= t:
            out.add(goal_owner)
        hold_owner = self.holds.get(cell)
        permanent_owner = self.permanent.get(cell)
        edge_owner = None if from_cell is None else self.edge.get((cell, from_cell, t))
        for blocker in (vertex_owner, hold_owner, permanent_owner, edge_owner):
            if blocker is not None and blocker != owner:
                out.add(blocker)
        return sorted(out)

    def reserve_path(
        self,
        robot_id: str,
        path: list[Cell],
        start: int,
        reserve_goal: bool = True,
        goal_ready: int | None = None,
    ) -> None:
        for i, cell in enumerate(path):
            self.vertex[(cell, start + i)] = robot_id
            if i > 0:
                self.edge[(path[i - 1], cell, start + i)] = robot_id
        if path and reserve_goal:
            self.goal[path[-1]] = robot_id
            ready = start + len(path) - 1 if goal_ready is None else goal_ready
            self.goal_ready[path[-1]] = max(ready, start + 1)

    def reserve_hold(self, robot_id: str, cell: Cell) -> None:
        self.holds[cell] = robot_id

    def reserve_permanent(self, robot_id: str, cell: Cell) -> None:
        self.permanent[cell] = robot_id

    def release_hold(self, robot_id: str) -> None:
        self.holds = {cell: owner for cell, owner in self.holds.items() if owner != robot_id}
        self.permanent = {cell: owner for cell, owner in self.permanent.items() if owner != robot_id}

    def release(self, robot_id: str) -> None:
        self.vertex = {k: v for k, v in self.vertex.items() if v != robot_id}
        self.edge = {k: v for k, v in self.edge.items() if v != robot_id}
        self.goal = {k: v for k, v in self.goal.items() if v != robot_id}
        self.goal_ready = {k: v for k, v in self.goal_ready.items() if k in self.goal}
        self.release_hold(robot_id)
        self.permanent = {cell: owner for cell, owner in self.permanent.items() if owner != robot_id}
