from __future__ import annotations

import json
from collections import defaultdict

from .map import GraphMap


class ReplayError(AssertionError):
    pass


def replay_log(path, safe_distance=1, starvation_limit=None):
    with open(path, "r", encoding="utf-8") as stream:
        payload = json.load(stream)
    graph = GraphMap.from_dict(payload["map"])
    limit = starvation_limit if starvation_limit is not None else payload.get("starvation_limit", 20)
    records = payload["records"]
    if not records:
        raise ReplayError("execution log is empty")

    adjacency = graph.neighbors
    positions = dict(payload["initial_positions"])
    seen_tasks = set()
    completed_tasks = set()
    wait_streak = defaultdict(int)
    max_wait = defaultdict(int)
    ticks = defaultdict(list)
    for record in records:
        ticks[record["tick"]].append(record)

    for tick in sorted(ticks):
        before = dict(positions)
        for record in ticks[tick]:
            robot_id = record["robot_id"]
            source = record["previous_position"]
            target = record["position"]
            positions.setdefault(robot_id, source)
            if positions[robot_id] != source:
                raise ReplayError(f"tick {tick}, {robot_id}: replay position does not match log")
            if source not in adjacency or target not in adjacency:
                raise ReplayError(f"tick {tick}, {robot_id}: unknown node")

            action = record["action"]
            if action == "move":
                if target not in adjacency[source] or target == source:
                    raise ReplayError(f"tick {tick}, {robot_id}: teleport or invalid edge")
            elif action in {"assigned", "wait", "paused", "idle"}:
                if target != source:
                    raise ReplayError(f"tick {tick}, {robot_id}: non-move action changed position")
            elif action == "complete":
                if target != source and target not in adjacency[source]:
                    raise ReplayError(f"tick {tick}, {robot_id}: completion used an invalid edge")
            else:
                raise ReplayError(f"tick {tick}, {robot_id}: unknown action {action}")

            if action == "paused" and not record["paused"]:
                raise ReplayError(f"tick {tick}, {robot_id}: pause state inconsistent")
            if record["paused"] and target != source:
                raise ReplayError(f"tick {tick}, {robot_id}: paused robot moved")

            task_id = record["task_id"]
            if task_id is not None:
                seen_tasks.add(task_id)
                if action == "complete" and target != record["destination"]:
                    raise ReplayError(f"tick {tick}, {robot_id}: completed task away from destination")
            if action == "complete":
                if task_id is None:
                    raise ReplayError(f"tick {tick}, {robot_id}: completion without task id")
                completed_tasks.add(task_id)

            if action == "wait":
                wait_streak[robot_id] += 1
                max_wait[robot_id] = max(max_wait[robot_id], wait_streak[robot_id])
            else:
                wait_streak[robot_id] = 0
            positions[robot_id] = target

        occupied = defaultdict(list)
        for robot_id, node in positions.items():
            occupied[node].append(robot_id)
        for node, robots in occupied.items():
            if len(robots) > 1:
                raise ReplayError(f"tick {tick}: robots {robots} collide at {node}")

        current_positions = list(positions.items())
        for index, (robot_a, node_a) in enumerate(current_positions):
            for robot_b, node_b in current_positions[index + 1 :]:
                if before[robot_a] == node_b and before[robot_b] == node_a:
                    raise ReplayError(f"tick {tick}: {robot_a} and {robot_b} swap along one edge")
                if safe_distance > 1:
                    distance = _edge_distance(graph, node_a, node_b, safe_distance)
                    if distance is not None and distance < safe_distance:
                        raise ReplayError(f"tick {tick}: unsafe distance between {robot_a} and {robot_b}")

        for robot_id in positions:
            if max_wait[robot_id] > limit:
                raise ReplayError(f"tick {tick}, {robot_id}: starved after {max_wait[robot_id]} ticks")

    unfinished = seen_tasks - completed_tasks
    if unfinished:
        raise ReplayError(f"unfinished tasks: {sorted(unfinished)}")
    return {
        "ticks": len(ticks),
        "robots": sorted(positions),
        "completed_tasks": sorted(completed_tasks),
        "max_wait": dict(max_wait),
    }


def _edge_distance(graph, source, target, cap):
    if source == target:
        return 0
    frontier = {source}
    visited = {source}
    for distance in range(1, cap + 1):
        next_frontier = set()
        for node in frontier:
            for neighbor in graph.neighbors[node]:
                if neighbor == target:
                    return distance
                if neighbor not in visited:
                    visited.add(neighbor)
                    next_frontier.add(neighbor)
        frontier = next_frontier
    return None
