from __future__ import annotations

import json

from .models import ExecutionLog, RobotState, Task, TickRecord
from .scheduler import Coordinator


class Simulation:
    def __init__(self, graph, robots, starvation_limit=20):
        self.graph = graph
        self.robots = list(robots)
        self.robot_by_id = {robot.robot_id: robot for robot in self.robots}
        self.coordinator = Coordinator(graph)
        self.log = ExecutionLog()
        self.tick = 0
        self.starvation_limit = starvation_limit
        self.initial_positions = {robot.robot_id: robot.position for robot in self.robots}

    def assign_task(self, robot_id, task):
        robot = self.robot_by_id[robot_id]
        robot.enqueue(task)
        self.log.add(
            TickRecord(
                tick=self.tick,
                robot_id=robot_id,
                position=robot.position,
                previous_position=robot.position,
                action="assigned",
                task_id=task.task_id,
                destination=task.destination,
                paused=robot.paused,
            )
        )

    def assign_tasks_fairly(self, tasks):
        for task in tasks:
            robot = min(
                self.robots,
                key=lambda item: (
                    item.tasks_completed,
                    item.tasks_started,
                    len(item.task_queue),
                    item.robot_id,
                ),
            )
            self.assign_task(robot.robot_id, task)

    def pause(self, robot_id):
        self.robot_by_id[robot_id].pause()

    def resume(self, robot_id):
        self.robot_by_id[robot_id].resume()

    def step(self):
        for robot in self.robots:
            robot.ensure_active_task()

        previous = {robot.robot_id: robot.position for robot in self.robots}
        decisions = self.coordinator.plan_tick(self.robots)
        self.tick += 1

        for robot in self.robots:
            move = decisions[robot.robot_id]
            if robot.paused:
                action = "paused"
            elif move is None:
                if robot.active_task and robot.position != robot.active_task.destination:
                    action = "wait"
                else:
                    action = "idle"
            else:
                robot.position = move[1]
                robot.distance_traveled += 1
                action = "move"

            task = robot.active_task
            if task is not None and robot.position == task.destination and not robot.paused:
                robot.active_task = None
                robot.tasks_completed += 1
                action = "complete"

            self.log.add(
                TickRecord(
                    tick=self.tick,
                    robot_id=robot.robot_id,
                    position=robot.position,
                    previous_position=previous[robot.robot_id],
                    action=action,
                    task_id=task.task_id if task is not None else None,
                    destination=task.destination if task is not None else None,
                    paused=robot.paused,
                )
            )
        return decisions

    def run(self, max_ticks=200):
        for _ in range(max_ticks):
            self.step()
            if all(robot.active_task is None and not robot.task_queue for robot in self.robots):
                return True
        return False

    def export_log(self, path):
        payload = {
            "map": self.graph.to_dict(),
            "starvation_limit": self.starvation_limit,
            "initial_positions": dict(self.initial_positions),
            "records": self.log.to_list(),
        }
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)


def build_robots(items):
    return [RobotState(robot_id, position) for robot_id, position in items]
