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
        if from_cell is not None and self._edge_key(from_cell, cell, t) in self.edge:
            return False
        return True

    @staticmethod
    def _edge_key(a: Cell, b: Cell, t: int) -> tuple[Cell, Cell, int]:
        return (min(a, b), max(a, b), t)

    def reserve_path(self, robot_id: str, path: list[Cell], start: int) -> None:
        for i, cell in enumerate(path):
            self.vertex[(cell, start + i)] = robot_id
            if i > 0:
                self.edge[self._edge_key(path[i - 1], cell, start + i)] = robot_id
        if path:
            self.goal[path[-1]] = robot_id

    def reserve_holding(self, robot_id: str, cell: Cell, start: int, until: int) -> None:
        """持续占位(等待),不作为终点永久预约;到时后该格可重新被规划。"""
        for t in range(start, until + 1):
            self.vertex[(cell, t)] = robot_id

    def reserved_by(self, cell: Cell, t: int, from_cell: Cell | None = None) -> str | None:
        """返回该时空格的占用者,用于说明阻塞来源."""
        if (cell, t) in self.vertex:
            return self.vertex[(cell, t)]
        if cell in self.goal:
            return f"{self.goal[cell]}:goal"
        if from_cell is not None:
            key = self._edge_key(from_cell, cell, t)
            if key in self.edge:
                return f"{self.edge[key]}:edge"
        return None

    def release(self, robot_id: str) -> None:
        self.vertex = {k: v for k, v in self.vertex.items() if v != robot_id}
        self.edge = {k: v for k, v in self.edge.items() if v != robot_id}
        self.goal = {k: v for k, v in self.goal.items() if v != robot_id}
