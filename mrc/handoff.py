"""交接:两个机器人在指定交接区转交同一件运输物.

语义:
- 货物任意时刻只有唯一归属方;只有双方都进入交接区并完成占用后,
  归属才在交接完成时刻一次性从交出方切到接收方。
- 交接走普通任务规划,复用同一预约表,不能绕过通道预约。
- 交接开始(双方到齐)后承诺不可撤回;未开始的承诺可调整或取消。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .gridmap import Cell


PLANNED = "planned"            # 已规划,双方尚未同时到位
BOTH_PRESENT = "both_present"  # 双方已到齐,交接开始(不可撤回)
COMPLETED = "completed"        # 归属已转移
BLOCKED = "blocked"            # 一方延迟/故障/取消,归属保持不变
CANCELLED = "cancelled"        # 未开始前被取消


@dataclass
class Handoff:
    handoff_id: str
    cargo_id: str
    giver: str
    receiver: str
    zone: tuple[Cell, Cell]  # (交出方占位格, 接收方占位格)
    alternatives: list[tuple[Cell, Cell]] = field(default_factory=list)
    state: str = PLANNED
    planned_tick: int = 0
    started_tick: int | None = None
    completed_tick: int | None = None

    @property
    def started(self) -> bool:
        """交接是否已开始:已开始或已完成的承诺不可撤回."""
        return self.started_tick is not None

    @property
    def adjustable(self) -> bool:
        """尚未开始的交接承诺可调整(换点/取消)."""
        return self.state in (PLANNED, BLOCKED) and not self.started

    def to_dict(self) -> dict[str, object]:
        return {
            "handoff_id": self.handoff_id,
            "cargo_id": self.cargo_id,
            "giver": self.giver,
            "receiver": self.receiver,
            "zone": [list(self.zone[0]), list(self.zone[1])],
            "alternatives": [[list(a), list(b)] for a, b in self.alternatives],
            "state": self.state,
            "planned_tick": self.planned_tick,
            "started_tick": self.started_tick,
            "completed_tick": self.completed_tick,
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "Handoff":
        zone = data["zone"]
        return cls(
            handoff_id=str(data["handoff_id"]),
            cargo_id=str(data["cargo_id"]),
            giver=str(data["giver"]),
            receiver=str(data["receiver"]),
            zone=(tuple(zone[0]), tuple(zone[1])),
            alternatives=[(tuple(pair[0]), tuple(pair[1])) for pair in data.get("alternatives", [])],
            state=str(data.get("state", PLANNED)),
            planned_tick=int(data.get("planned_tick", 0)),
            started_tick=data.get("started_tick"),
            completed_tick=data.get("completed_tick"),
        )
