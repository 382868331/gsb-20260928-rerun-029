"""机器人状态与任务."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .gridmap import Cell


class RobotStatus(Enum):
    IDLE = "idle"
    MOVING = "moving"
    PAUSED = "paused"
    BLOCKED = "blocked"   # 无可行安排或在时间窗内尚未找到方案,原地等待
    EVACUATING = "evacuating"  # 通道封闭,正按退出安排离开封闭区段
    DONE = "done"


@dataclass
class Task:
    task_id: str
    goal: Cell
    assigned_tick: int = 0
    deadline: int | None = None  # 截止时刻(须在该 tick 之前/之时到达);None 表示无截止
    priority: int = 0            # 越大越优先(紧急任务);不影响在途预约的锁定


@dataclass
class Robot:
    robot_id: str
    pos: Cell
    status: RobotStatus = RobotStatus.IDLE
    task: Task | None = None
    path: list[Cell] = field(default_factory=list)  # path[t] 为相对计划起点的占用格
    plan_start: int = 0
    wait_ticks: int = 0
    total_wait: int = 0
    priority: int = 0           # 越大越优先(紧急任务)
    committed: bool = False     # 是否已经在途(预约不可被后来任务抢占)
    evacuating: bool = False    # 是否正执行封路退出安排
    infeasible: bool = False    # 静态判定无可行安排(区别于暂时未找到)
    block_reason: str = ""      # 最近一次阻塞原因(含阻挡预约来源)

    @property
    def goal(self) -> Cell | None:
        return self.task.goal if self.task else None

    def planned_cell(self, tick: int) -> Cell:
        """tick 时刻计划所在格;路径用尽后停在终点."""
        idx = tick - self.plan_start
        if idx < 0:
            return self.pos
        if idx >= len(self.path):
            return self.path[-1] if self.path else self.pos
        return self.path[idx]
