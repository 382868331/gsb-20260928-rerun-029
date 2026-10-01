"""运输物与机器人之间交接的领域模型.

交接语义:
- 同一件运输物在任一时刻只有一个归属机器人;双方未到齐前不转移归属。
- 交接只能发生在已登记的交接格,且双方都实际占据交接格、都完成各自的交接段后。
- 未开始(promised)的交接承诺可以取消或改换交接点;已开始(started)后不可撤回,
  只允许等待、撤离或用不重写既有预约的替代方案绕行。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .gridmap import Cell


class HandoverState(Enum):
    PROMISED = "promised"          # 承诺已建立,双方都尚未真正开始交接段
    STARTED = "started"            # 至少一方已沿交接段迈出一步,承诺不可撤回
    IN_PROGRESS = "in_progress"    # 至少一方已到交接格并占用,等待另一方
    COMPLETED = "completed"        # 双方到齐,运输物归属已转交
    FAILED = "failed"              # 一方故障/取消,归属保留在当前持有者
    CANCELLED = "cancelled"        # 开始之前被取消


@dataclass
class Cargo:
    cargo_id: str
    owner: str                     # 当前归属机器人 ID
    terminal: bool = False         # 随终止任务而终止后,不再被任何交接复活
    history: list[tuple[int, str]] = field(default_factory=list)  # (tick, owner)


@dataclass
class HandoverCommitment:
    handover_id: str
    cargo_id: str
    carrier: str                   # 交出方(当前持有运输物)
    receiver: str                  # 接收方
    rendezvous: Cell               # 交接格:双方必须在此同格完成占用
    carrier_slot: Cell             # 交接区内交出方占用槽
    receiver_slot: Cell            # 交接区内接收方占用槽(与交出方槽相邻)
    carrier_task: str              # 交出方前往交接格的任务 ID
    receiver_task: str             # 接收方前往交接格的会合任务 ID
    delivery_task: str | None      # 交接完成后接收方继续运输的任务 ID
    delivery_goal: Cell | None = None
    delivery_priority: int = 0
    delivery_deadline: int | None = None
    zone_id: str | None = None
    state: HandoverState = HandoverState.PROMISED
    created_tick: int = 0
    started_tick: int | None = None
    completed_tick: int | None = None
    end_tick: int | None = None
    reason: str | None = None
    carrier_arrived: bool = False
    receiver_arrived: bool = False
    alternatives: list[Cell] = field(default_factory=list)

    def participants(self) -> set[str]:
        return {self.carrier, self.receiver}

    def other(self, robot_id: str) -> str | None:
        if robot_id == self.carrier:
            return self.receiver
        if robot_id == self.receiver:
            return self.carrier
        return None

    def slot_of(self, robot_id: str) -> Cell | None:
        if robot_id == self.carrier:
            return self.carrier_slot
        if robot_id == self.receiver:
            return self.receiver_slot
        return None


@dataclass
class HandoverOption:
    kind: str                      # wait / evacuate / repoint
    feasible: bool
    detail: str
    rendezvous: Cell | None = None
    carrier_eta: int | None = None
    receiver_eta: int | None = None


class CargoRegistry:
    """登记运输物归属与交接承诺,是判定唯一归属的权威来源。"""

    def __init__(self) -> None:
        self.cargo: dict[str, Cargo] = {}
        self.commitments: dict[str, HandoverCommitment] = {}

    def add_cargo(self, cargo: Cargo) -> None:
        self.cargo[cargo.cargo_id] = cargo

    def owner_of(self, cargo_id: str) -> str | None:
        cargo = self.cargo.get(cargo_id)
        return cargo.owner if cargo else None

    def transfer(self, cargo_id: str, new_owner: str, tick: int) -> None:
        cargo = self.cargo[cargo_id]
        cargo.owner = new_owner
        cargo.history.append((tick, new_owner))

    def active_commitment_of(self, robot_id: str) -> HandoverCommitment | None:
        for commitment in self.commitments.values():
            if robot_id in commitment.participants() and commitment.state in (
                HandoverState.PROMISED,
                HandoverState.STARTED,
                HandoverState.IN_PROGRESS,
            ):
                return commitment
        return None

    def leg_task(self, robot_id: str) -> str | None:
        commitment = self.active_commitment_of(robot_id)
        if commitment is None:
            return None
        if robot_id == commitment.carrier:
            return commitment.carrier_task
        return commitment.receiver_task

    def is_leg_task(self, task_id: str) -> bool:
        for commitment in self.commitments.values():
            if task_id in (commitment.carrier_task, commitment.receiver_task):
                return True
        return False

    def delivery_of_receiver_task(self, task_id: str) -> HandoverCommitment | None:
        for commitment in self.commitments.values():
            if commitment.receiver_task == task_id:
                return commitment
        return None
