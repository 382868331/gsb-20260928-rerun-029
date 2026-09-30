"""栅格地图:障碍物、邻接、狭窄通道."""

from __future__ import annotations

Cell = tuple[int, int]


class GridMap:
    def __init__(self, width: int, height: int, obstacles: set[Cell] | None = None):
        self.width = width
        self.height = height
        self.obstacles: set[Cell] = set(obstacles or ())
        # 运行时封闭(封路)的格:除正在退出的机器人外不得进入
        self.closures: set[Cell] = set()

    def close_cells(self, cells: set[Cell]) -> None:
        """封闭通道区段."""
        for cell in cells:
            if not self.in_bounds(cell) or cell in self.obstacles:
                raise ValueError(f"无法封闭 {cell}: 越界或障碍物")
        self.closures |= set(cells)

    def open_cells(self, cells: set[Cell]) -> None:
        """重新开放通道区段."""
        self.closures -= set(cells)

    @classmethod
    def corridor_map(cls, length: int = 8, width: int = 3) -> "GridMap":
        """中间一行是宽度为 1 的狭窄通道,两端是开阔区."""
        obstacles: set[Cell] = set()
        mid = width // 2
        for x in range(2, length + 2):
            for y in range(width):
                if y != mid:
                    obstacles.add((x, y))
        return cls(length + 4, width, obstacles)

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, cell: Cell, allow_closed: bool = False) -> bool:
        if not self.in_bounds(cell) or cell in self.obstacles:
            return False
        if not allow_closed and cell in self.closures:
            return False
        return True

    def neighbors(self, cell: Cell, allow_closed: bool = False) -> list[Cell]:
        x, y = cell
        out = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (x + dx, y + dy)
            if self.passable(nxt, allow_closed=allow_closed):
                out.append(nxt)
        return out

    def render(self, marks: dict[Cell, str] | None = None) -> str:
        marks = marks or {}
        rows = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                cell = (x, y)
                if cell in marks:
                    row.append(marks[cell])
                elif cell in self.obstacles:
                    row.append("#")
                else:
                    row.append(".")
            rows.append("".join(row))
        return "\n".join(rows)
