from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

from multi_robot import GraphMap, RobotState, Simulation, Task, replay_log
from multi_robot.map import Passage

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))

from demo import build_demo


class CoordinationTest(unittest.TestCase):
    def test_paused_robot_keeps_occupancy_and_log_replays(self):
        simulation = build_demo()
        simulation.step()
        simulation.step()
        self.assertEqual(simulation.robot_by_id["r1"].position, "P0")
        simulation.pause("r1")
        for _ in range(3):
            simulation.step()

        paused_record = next(
            record
            for record in simulation.log.for_tick(simulation.tick)
            if record.robot_id == "r1"
        )
        self.assertEqual(paused_record.position, "P0")
        self.assertEqual(paused_record.action, "paused")

        simulation.resume("r1")
        self.assertTrue(simulation.run(max_ticks=80))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "log.json"
            simulation.export_log(path)
            result = replay_log(path)

        self.assertEqual(len(result["completed_tasks"]), 8)
        self.assertLessEqual(max(result["max_wait"].values()), 30)

    def test_head_on_robots_do_not_enter_corridor_together(self):
        graph = GraphMap(
            ["aa", "a", "c0", "c1", "b", "bb"],
            [("aa", "a"), ("a", "c0"), ("c0", "c1"), ("c1", "b"), ("b", "bb")],
            [Passage("p", ("a", "c0", "c1", "b"))],
        )
        left = RobotState("left", "aa")
        right = RobotState("right", "bb")
        left.active_task = Task("l", "b")
        right.active_task = Task("r", "a")
        simulation = Simulation(graph, [left, right])
        decisions = simulation.step()
        self.assertEqual(decisions["left"], ("aa", "a"))
        self.assertIsNone(decisions["right"])


if __name__ == "__main__":
    unittest.main()
