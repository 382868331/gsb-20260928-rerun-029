"""接力运输与临时托管.

语义:
- 每一刻货物归属唯一:某台机器人或某个托管位,接力计划不会把货物拆成多份。
- 托管区有容量(托管位数量)与取放占用限制(每位每刻最多 ops_per_tick 次取/放)。
- 放入/取出是执行动作,预约交接不等于完成转移;只有动作完成的时刻归属才切换。
- 托管中的货物占用其格子(计入共享预约表),未被接走会影响后续运输可行性。
- 某一段故障或取消时,只重排尚未执行的部分;已完成的接力与实际占用保留。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .gridmap import Cell


# 段状态
LEG_PLANNED = "planned"      # 已规划,尚未开始执行
LEG_ACTIVE = "active"        # 正在执行(机器人在途/等待取放)
LEG_DONE = "done"            # 已执行完成(放置/取出/交接已发生)
LEG_FAILED = "failed"        # 承运机器人故障/终止,归属保持实际位置
LEG_CANCELLED = "cancelled"  # 尚未执行被取消

# 接力计划状态
RELAY_ACTIVE = "active"
RELAY_DONE = "done"
RELAY_BLOCKED = "blocked"    # 当前段受阻,等待重排
RELAY_CANCELLED = "cancelled"

# 段类型
CARRY = "carry"      # 机器人承运一段
HANDOFF = "handoff"  # 机器人之间在交接区转交
STAGE = "stage"      # 放入托管位,等待后续承运人取出


def slot_owner_id(area_id: str, slot: Cell) -> str:
    """托管位作为归属方/占用方的标识."""
    return f"staging:{area_id}@{slot[0]},{slot[1]}"


@dataclass
class StagingArea:
    """临时托管区:若干托管位,每位容量一件,每刻取放次数有限."""

    area_id: str
    slots: dict[Cell, Cell]          # 托管位 -> 机器人取放站位
    ops_per_tick: int = 1            # 托管区每刻最多取/放次数(取放占用限制)
    occupancy: dict[Cell, str] = field(default_factory=dict)  # 托管位 -> 货物
    op_history: dict[int, int] = field(default_factory=dict)  # 刻 -> 已用取放次数

    def free_slot(self) -> Cell | None:
        for slot in self.slots:
            if slot not in self.occupancy:
                return slot
        return None

    def ops_used(self, tick: int) -> int:
        return self.op_history.get(tick, 0)

    def can_operate(self, tick: int) -> bool:
        return self.ops_used(tick) < self.ops_per_tick

    def record_op(self, tick: int) -> None:
        self.op_history[tick] = self.op_history.get(tick, 0) + 1

    def to_dict(self) -> dict[str, object]:
        return {
            "area_id": self.area_id,
            "slots": {f"{s[0]},{s[1]}": list(a) for s, a in self.slots.items()},
            "ops_per_tick": self.ops_per_tick,
            "occupancy": {f"{s[0]},{s[1]}": c for s, c in self.occupancy.items()},
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "StagingArea":
        slots = {}
        for key, access in dict(data.get("slots", {})).items():
            x, y = key.split(",")
            slots[(int(x), int(y))] = tuple(access)
        area = cls(
            area_id=str(data["area_id"]),
            slots=slots,
            ops_per_tick=int(data.get("ops_per_tick", 1)),
        )
        for key, cargo in dict(data.get("occupancy", {})).items():
            x, y = key.split(",")
            area.occupancy[(int(x), int(y))] = str(cargo)
        return area


@dataclass
class RelayLeg:
    """接力计划中的一段."""

    leg_id: str
    index: int
    kind: str                       # CARRY / HANDOFF / STAGE
    carrier: str | None = None      # CARRY 段承运机器人
    to: Cell | None = None          # CARRY 段目的地
    zone: tuple[Cell, Cell] | None = None  # HANDOFF 段交接区
    area_id: str | None = None      # STAGE 段托管区
    slot: Cell | None = None        # STAGE 段托管位
    state: str = LEG_PLANNED
    executed_tick: int | None = None

    @property
    def executed(self) -> bool:
        """已执行(或已取消)的段不再重排."""
        return self.state in (LEG_DONE, LEG_CANCELLED)

    def to_dict(self) -> dict[str, object]:
        return {
            "leg_id": self.leg_id,
            "index": self.index,
            "kind": self.kind,
            "carrier": self.carrier,
            "to": list(self.to) if self.to else None,
            "zone": [list(self.zone[0]), list(self.zone[1])] if self.zone else None,
            "area_id": self.area_id,
            "slot": list(self.slot) if self.slot else None,
            "state": self.state,
            "executed_tick": self.executed_tick,
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "RelayLeg":
        zone = data.get("zone")
        to = data.get("to")
        slot = data.get("slot")
        return cls(
            leg_id=str(data["leg_id"]),
            index=int(data["index"]),
            kind=str(data["kind"]),
            carrier=data.get("carrier"),
            to=tuple(to) if to else None,
            zone=(tuple(zone[0]), tuple(zone[1])) if zone else None,
            area_id=data.get("area_id"),
            slot=tuple(slot) if slot else None,
            state=str(data.get("state", LEG_PLANNED)),
            executed_tick=data.get("executed_tick"),
        )


@dataclass
class RelayPlan:
    """一件货物的接力运输方案:段按序执行,任意时刻归属唯一."""

    relay_id: str
    cargo_id: str
    legs: list[RelayLeg]
    state: str = RELAY_ACTIVE

    @property
    def current_leg(self) -> RelayLeg | None:
        for leg in self.legs:
            if not leg.executed:
                return leg
        return None

    @property
    def pending(self) -> bool:
        """是否还有未执行的段需要推进."""
        return self.state == RELAY_ACTIVE and self.current_leg is not None

    def to_dict(self) -> dict[str, object]:
        return {
            "relay_id": self.relay_id,
            "cargo_id": self.cargo_id,
            "state": self.state,
            "legs": [leg.to_dict() for leg in self.legs],
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "RelayPlan":
        return cls(
            relay_id=str(data["relay_id"]),
            cargo_id=str(data["cargo_id"]),
            legs=[RelayLeg.from_dict(item) for item in data.get("legs", [])],
            state=str(data.get("state", RELAY_ACTIVE)),
        )
