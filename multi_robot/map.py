from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Passage:
    passage_id: str
    nodes: tuple[str, ...]

    def edge_direction(self, source: str, target: str) -> int | None:
        if source not in self.nodes or target not in self.nodes:
            return None
        source_index = self.nodes.index(source)
        target_index = self.nodes.index(target)
        if abs(source_index - target_index) != 1:
            return None
        return 1 if target_index > source_index else -1

    def contains(self, node: str) -> bool:
        return node in self.nodes


class GraphMap:
    def __init__(self, nodes, edges, passages=()):
        self.nodes = tuple(nodes)
        self.edges = tuple((a, b) for a, b in edges)
        self.passages = tuple(passages)
        node_set = set(self.nodes)
        if len(node_set) != len(self.nodes):
            raise ValueError("duplicate nodes are not allowed")
        for a, b in self.edges:
            if a not in node_set or b not in node_set or a == b:
                raise ValueError(f"invalid edge: {(a, b)}")
        self.neighbors = {node: set() for node in self.nodes}
        for a, b in self.edges:
            self.neighbors[a].add(b)
            self.neighbors[b].add(a)

    def passage_for_edge(self, source, target):
        for passage in self.passages:
            direction = passage.edge_direction(source, target)
            if direction is not None:
                return passage, direction
        return None

    def passage_for_move(self, source, target):
        internal = self.passage_for_edge(source, target)
        if internal is not None:
            return internal
        for passage in self.passages:
            if source not in passage.nodes and target in passage.nodes:
                target_index = passage.nodes.index(target)
                if target_index == 0:
                    return passage, 1
                if target_index == len(passage.nodes) - 1:
                    return passage, -1
        return None

    def shortest_path(self, source, target):
        if source == target:
            return [source]
        previous = {source: None}
        queue = deque([source])
        while queue:
            current = queue.popleft()
            if current == target:
                break
            for neighbor in sorted(self.neighbors[current]):
                if neighbor not in previous:
                    previous[neighbor] = current
                    queue.append(neighbor)
        if target not in previous:
            raise ValueError(f"no path from {source} to {target}")
        path = []
        current = target
        while current is not None:
            path.append(current)
            current = previous[current]
        return path[::-1]

    def to_dict(self):
        return {
            "nodes": list(self.nodes),
            "edges": [list(edge) for edge in self.edges],
            "passages": [
                {"passage_id": passage.passage_id, "nodes": list(passage.nodes)}
                for passage in self.passages
            ],
        }

    @classmethod
    def from_dict(cls, data):
        return cls(
            data["nodes"],
            [tuple(edge) for edge in data["edges"]],
            [Passage(item["passage_id"], tuple(item["nodes"])) for item in data.get("passages", [])],
        )
