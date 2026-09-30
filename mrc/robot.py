"""机器人状态与任务."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .gridmap import Cell


class RobotStatus(Enum):
    IDLE = "idle"
    MOVING = "moving"
    WAITING = "waiting"
    PAUSED = "paused"
    FAILED = "failed"
    TERMINATED = "terminated"
    DONE = "done"


class WaitReason:
    BLOCKED = "blocked"                    # 暂时受阻:他人在途预约/占用,后续可恢复
    DEADLINE_INFEASIBLE = "deadline_infeasible"  # 已无法在截止前完成
    SEARCH_LIMIT = "search_limit"          # 搜索预算不足,不能据此判定可行与否


@dataclass
class Task:
    task_id: str
    goal: Cell
    assigned_tick: int = 0
    priority: int = 0
    deadline: int | None = None
    on_time: bool | None = None
    end_tick: int | None = None
    end_reason: str | None = None  # done / deadline_infeasible / hard_blocked / trapped / expired


@dataclass
class Robot:
    robot_id: str
    pos: Cell
    status: RobotStatus = RobotStatus.IDLE
    task: Task | None = None
    path: list[Cell] = field(default_factory=list)  # path[t] 为相对计划起点的占用格
    plan_start: int = 0
    last_blockers: list[str] = field(default_factory=list)
    wait_reason: str | None = None
    end_reason: str | None = None
    wait_ticks: int = 0
    total_wait: int = 0

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
