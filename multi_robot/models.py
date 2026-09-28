from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Task:
    task_id: str
    destination: str


@dataclass
class RobotState:
    robot_id: str
    position: str
    paused: bool = False
    task_queue: list[Task] = field(default_factory=list)
    active_task: Task | None = None
    tasks_started: int = 0
    tasks_completed: int = 0
    distance_traveled: int = 0

    def enqueue(self, task):
        self.task_queue.append(task)

    def ensure_active_task(self):
        if self.active_task is None and not self.paused and self.task_queue:
            self.active_task = self.task_queue.pop(0)
            self.tasks_started += 1

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False


@dataclass(frozen=True)
class TickRecord:
    tick: int
    robot_id: str
    position: str
    previous_position: str
    action: str
    task_id: str | None
    destination: str | None
    paused: bool

    def to_dict(self):
        return {
            "tick": self.tick,
            "robot_id": self.robot_id,
            "position": self.position,
            "previous_position": self.previous_position,
            "action": self.action,
            "task_id": self.task_id,
            "destination": self.destination,
            "paused": self.paused,
        }


class ExecutionLog:
    def __init__(self):
        self.records = []

    def add(self, record):
        self.records.append(record)

    def for_tick(self, tick):
        return [record for record in self.records if record.tick == tick]

    def robot_records(self, robot_id):
        return [record for record in self.records if record.robot_id == robot_id]

    def to_list(self):
        return [record.to_dict() for record in self.records]
