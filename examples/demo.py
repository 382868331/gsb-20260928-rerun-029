from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from multi_robot import GraphMap, RobotState, Simulation, Task, replay_log
from multi_robot.map import Passage


def build_demo():
    nodes = [
        "WA",
        "WB",
        "WC",
        "WD",
        "W1",
        "P0",
        "P1",
        "P2",
        "P3",
        "E1",
        "EA",
        "EB",
        "EC",
        "ED",
    ]
    line = ["W1", "P0", "P1", "P2", "P3", "E1"]
    west_edges = [("WA", "W1"), ("WB", "W1"), ("WC", "W1"), ("WD", "W1")]
    east_edges = [("E1", "EA"), ("E1", "EB"), ("E1", "EC"), ("E1", "ED")]
    edges = list(zip(line, line[1:])) + west_edges + east_edges
    graph = GraphMap(
        nodes,
        edges,
        [Passage("narrow-corridor", tuple(line))],
    )
    robots = [
        RobotState("r1", "WA"),
        RobotState("r2", "WB"),
        RobotState("r3", "EA"),
        RobotState("r4", "EB"),
    ]
    simulation = Simulation(graph, robots, starvation_limit=30)
    first_round = [
        Task("east-1", "EC"),
        Task("east-2", "ED"),
        Task("west-1", "WC"),
        Task("west-2", "WD"),
    ]
    for robot, task in zip(robots, first_round):
        simulation.assign_task(robot.robot_id, task)

    second_round = [
        Task("return-r1", "WA"),
        Task("return-r2", "WB"),
        Task("return-r3", "EA"),
        Task("return-r4", "EB"),
    ]
    simulation.assign_tasks_fairly(second_round)
    return simulation


def main():
    simulation = build_demo()
    simulation.step()
    simulation.step()
    simulation.pause("r1")
    for _ in range(3):
        simulation.step()
    simulation.resume("r1")
    if not simulation.run(max_ticks=80):
        raise RuntimeError("demo did not finish before max_ticks")

    output = Path(__file__).with_name("execution_log.json")
    simulation.export_log(output)
    result = replay_log(output)
    print(f"completed {len(result['completed_tasks'])} tasks in {result['ticks']} ticks")
    print(f"maximum fair-wait streaks: {result['max_wait']}")
    print(f"log written to {output}")


if __name__ == "__main__":
    main()
