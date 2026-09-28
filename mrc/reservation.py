"""时空预约表:协调共享通道的核心数据结构."""

from __future__ import annotations

from .gridmap import Cell


class ReservationTable:
    """顶点预约 + 边预约(防止对向换边).终点格永久预约."""

    def __init__(self) -> None:
        self.vertex: dict[tuple[Cell, int], str] = {}
        self.edge: dict[tuple[Cell, Cell, int], str] = {}
        self.goal: dict[Cell, str] = {}

    def is_free(self, cell: Cell, t: int, from_cell: Cell | None = None) -> bool:
        if (cell, t) in self.vertex:
            return False
        if cell in self.goal:
            return False
        if from_cell is not None and (cell, from_cell, t) in self.edge:
            return False
        return True

    def reserve_path(self, robot_id: str, path: list[Cell], start: int) -> None:
        for i, cell in enumerate(path):
            self.vertex[(cell, start + i)] = robot_id
            if i > 0:
                self.edge[(path[i - 1], cell, start + i)] = robot_id
        if path:
            self.goal[path[-1]] = robot_id

    def release(self, robot_id: str) -> None:
        self.vertex = {k: v for k, v in self.vertex.items() if v != robot_id}
        self.edge = {k: v for k, v in self.edge.items() if v != robot_id}
        self.goal = {k: v for k, v in self.goal.items() if v != robot_id}
