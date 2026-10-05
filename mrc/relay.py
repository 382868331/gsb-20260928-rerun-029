"""接力运输与临时托管区.

语义:
- 运输物每一刻的归属唯一:某台机器人或某个托管位(slot:<id>),不会被接力计划拆成多份。
- 预约交接/预约托管位不等于完成转移:只有实际放下/取走/双方到齐的时刻归属才切换。
- 托管区有容量上限;托管位被占用时其静置格计入预约表,影响后续运输可行性。
- 同一 tick 内先处理取走再处理放入,取物与新放入可以同刻发生在同一托管区。
- 某段故障或取消时只重排尚未执行的部分;已完成的接力与实际占用保留。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .gridmap import Cell


PENDING = "pending"        # 尚未激活
ACTIVE = "active"          # 承运机器人执行中
COMPLETED = "completed"    # 该段已完成(不可重排)
BLOCKED = "blocked"        # 承运方故障/被改派/托管区满,归属保持不变
CANCELLED = "cancelled"    # 未执行即被取消

STAGE = "stage"
HANDOFF = "handoff"


def slot_owner(slot_id: str) -> str:
    """托管中货物的归属方标识."""
    return f"slot:{slot_id}"


@dataclass
class StagingSlot:
    slot_id: str
    cell: Cell             # 货物静置格,占用期间计入预约表
    access: Cell           # 机器人取放站位(与静置格不同)
    cargo_id: str | None = None
    placed_tick: int | None = None
    reserved_by: str | None = None  # 接力预约(预约不等于占用)

    @property
    def occupied(self) -> bool:
        return self.cargo_id is not None

    def to_dict(self) -> dict[str, object]:
        return {
            "slot_id": self.slot_id,
            "cell": list(self.cell),
            "access": list(self.access),
            "cargo_id": self.cargo_id,
            "placed_tick": self.placed_tick,
            "reserved_by": self.reserved_by,
        }


@dataclass
class StagingArea:
    area_id: str
    slots: list[StagingSlot]
    capacity: int

    def staged_count(self) -> int:
        return sum(1 for slot in self.slots if slot.occupied)

    def free_slot(self, relay_id: str) -> StagingSlot | None:
        """找一个可预约的托管位:未占用、未被其他接力预约、容量未满."""
        for slot in self.slots:
            if slot.reserved_by == relay_id and not slot.occupied:
                return slot
        if self.staged_count() >= self.capacity:
            return None
        for slot in self.slots:
            if not slot.occupied and slot.reserved_by is None:
                return slot
        return None

    def slot(self, slot_id: str) -> StagingSlot | None:
        for slot in self.slots:
            if slot.slot_id == slot_id:
                return slot
        return None

    def to_dict(self) -> dict[str, object]:
        return {
            "area_id": self.area_id,
            "capacity": self.capacity,
            "slots": [slot.to_dict() for slot in self.slots],
        }


@dataclass
class RelayLeg:
    leg_id: str
    index: int
    carrier: str
    goal: Cell | None = None   # 最后一段的送达点;其余段目标由转移方式决定
    state: str = PENDING
    note: str | None = None    # pickup: 需先到托管位取货

    def to_dict(self) -> dict[str, object]:
        return {
            "leg_id": self.leg_id,
            "index": self.index,
            "carrier": self.carrier,
            "goal": list(self.goal) if self.goal else None,
            "state": self.state,
            "note": self.note,
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "RelayLeg":
        goal = data.get("goal")
        return cls(
            leg_id=str(data["leg_id"]),
            index=int(data["index"]),
            carrier=str(data["carrier"]),
            goal=tuple(goal) if goal else None,
            state=str(data.get("state", PENDING)),
            note=data.get("note"),
        )


@dataclass
class RelayTransfer:
    """相邻两段之间的转移:经托管区或双机器人当面交接."""

    index: int                          # 位于 leg[index] 与 leg[index+1] 之间
    kind: str                           # STAGE | HANDOFF
    area: str | None = None
    zone: tuple[Cell, Cell] | None = None
    slot_id: str | None = None          # 预约/实际使用的托管位
    placed: bool = False                # 已实际放入(实际占用,重排保留)
    picked: bool = False                # 已被下一段取走
    state: str = PENDING

    @property
    def executed(self) -> bool:
        """已产生实际占用或已完成的转移不可重排."""
        return self.placed or self.state == COMPLETED

    def to_dict(self) -> dict[str, object]:
        return {
            "index": self.index,
            "kind": self.kind,
            "area": self.area,
            "zone": [list(self.zone[0]), list(self.zone[1])] if self.zone else None,
            "slot_id": self.slot_id,
            "placed": self.placed,
            "picked": self.picked,
            "state": self.state,
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "RelayTransfer":
        zone = data.get("zone")
        return cls(
            index=int(data["index"]),
            kind=str(data["kind"]),
            area=data.get("area"),
            zone=(tuple(zone[0]), tuple(zone[1])) if zone else None,
            slot_id=data.get("slot_id"),
            placed=bool(data.get("placed", False)),
            picked=bool(data.get("picked", False)),
            state=str(data.get("state", PENDING)),
        )


@dataclass
class Relay:
    relay_id: str
    cargo_id: str
    legs: list[RelayLeg]
    transfers: list[RelayTransfer] = field(default_factory=list)
    state: str = ACTIVE
    planned_tick: int = 0

    def next_leg(self) -> RelayLeg | None:
        for leg in self.legs:
            if leg.state in (PENDING, ACTIVE, BLOCKED):
                return leg
        return None

    def transfer_after(self, leg_index: int) -> RelayTransfer | None:
        for transfer in self.transfers:
            if transfer.index == leg_index:
                return transfer
        return None

    def to_dict(self) -> dict[str, object]:
        return {
            "relay_id": self.relay_id,
            "cargo_id": self.cargo_id,
            "legs": [leg.to_dict() for leg in self.legs],
            "transfers": [transfer.to_dict() for transfer in self.transfers],
            "state": self.state,
            "planned_tick": self.planned_tick,
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "Relay":
        return cls(
            relay_id=str(data["relay_id"]),
            cargo_id=str(data["cargo_id"]),
            legs=[RelayLeg.from_dict(item) for item in data.get("legs", [])],
            transfers=[RelayTransfer.from_dict(item) for item in data.get("transfers", [])],
            state=str(data.get("state", ACTIVE)),
            planned_tick=int(data.get("planned_tick", 0)),
        )
