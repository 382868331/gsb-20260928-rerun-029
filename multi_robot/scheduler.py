from __future__ import annotations


class Coordinator:
    def __init__(self, graph):
        self.graph = graph
        self.passage_owner = {}
        self.wait_streak = {}

    def plan_tick(self, robots):
        positions = {robot.robot_id: robot.position for robot in robots}
        if len(set(positions.values())) != len(positions):
            raise ValueError("initial positions contain a collision")

        for robot in robots:
            self.wait_streak.setdefault(robot.robot_id, 0)

        desired = {}
        wanted_direction = {}
        for robot in robots:
            desired[robot.robot_id] = self._desired_move(robot)
            move = desired[robot.robot_id]
            if move is None:
                continue
            passage_info = self.graph.passage_for_move(*move)
            if passage_info is not None:
                passage, direction = passage_info
                wanted_direction.setdefault(passage.passage_id, {}).setdefault(direction, []).append(
                    robot.robot_id
                )

        selected_direction = {}
        for passage_id, requests in wanted_direction.items():
            existing_owner = self.passage_owner.get(passage_id)
            candidates = [
                (
                    (
                        existing_owner is not None and direction != existing_owner,
                        -self.wait_streak.get(robot_id, 0),
                        robot_id,
                    ),
                    direction,
                )
                for direction, robot_ids in requests.items()
                for robot_id in robot_ids
            ]
            selected_direction[passage_id] = min(candidates, key=lambda item: item[0])[1]

        for passage in self.graph.passages:
            occupied = any(passage.contains(robot.position) for robot in robots)
            if not occupied and passage.passage_id in self.passage_owner:
                del self.passage_owner[passage.passage_id]

        decisions = {robot.robot_id: None for robot in robots}
        reserved_nodes = set(positions.values())
        reserved_edges = set()
        order = sorted(robots, key=lambda robot: (-self.wait_streak[robot.robot_id], robot.robot_id))

        for robot in order:
            robot_id = robot.robot_id
            move = desired[robot_id]
            if robot.paused or move is None:
                self.wait_streak[robot_id] = 0
                continue

            source, target = move
            passage_info = self.graph.passage_for_move(source, target)
            if not self._node_available(source, target, reserved_nodes, reserved_edges):
                self.wait_streak[robot_id] += 1
                continue
            selected = selected_direction.get(passage_info[0].passage_id) if passage_info is not None else None
            if not self._passage_available(passage_info, robot, selected):
                self.wait_streak[robot_id] += 1
                continue

            if passage_info is not None:
                passage, direction = passage_info
                self.passage_owner[passage.passage_id] = direction
            reserved_nodes.discard(source)
            reserved_nodes.add(target)
            reserved_edges.add((source, target))
            decisions[robot_id] = move
            self.wait_streak[robot_id] = 0

        return decisions

    def _desired_move(self, robot):
        if robot.paused or robot.active_task is None:
            return None
        if robot.position == robot.active_task.destination:
            return None
        path = self.graph.shortest_path(robot.position, robot.active_task.destination)
        return path[0], path[1]

    @staticmethod
    def _node_available(source, target, reserved_nodes, reserved_edges):
        if target in reserved_nodes:
            return False
        return (target, source) not in reserved_edges

    def _passage_available(self, passage_info, robot, selected_direction):
        if passage_info is None:
            return True
        passage, direction = passage_info
        owner = self.passage_owner.get(passage.passage_id)
        committed = passage.contains(robot.position)
        if owner is not None and owner != direction:
            return False

        if owner is None and selected_direction is not None and direction != selected_direction:
            return False
        return True
