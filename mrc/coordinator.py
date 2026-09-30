"""协调器:锁定在途预约,新任务在剩余时空资源中规划."""

from __future__ import annotations

import itertools
from dataclasses import dataclass

from .gridmap import Cell, GridMap
from .planner import INFEASIBLE, SEARCH_LIMIT, PlanResult, plan_path_result
from .reservation import ReservationTable
from .robot import Robot, RobotStatus


class CoordinationError(Exception):
    def __init__(self, message: str, results: dict[str, PlanResult] | None = None):
        super().__init__(message)
        self.results = results or {}


@dataclass
class Closure:
    cells: set[Cell]


class Coordinator:
    """所有机器人共享一张预约表,重规划不能抢掉仍有效的在途预约."""

    def __init__(self, gmap: GridMap, horizon: int = 200):
        self.gmap = gmap
        self.reservations = ReservationTable()
        self.horizon = horizon
        self.closures: list[Closure] = []
        self.closed_cells: set[Cell] = set()
        self.pending: set[str] = set()
        self.exiting: dict[str, Cell] = {}

    def close_corridor(self, cells: set[Cell] | list[Cell]) -> None:
        cells = {cell for cell in cells if self.gmap.passable(cell)}
        if not cells:
            return
        self.closures.append(Closure(set(cells)))
        self.closed_cells.update(cells)
        self.gmap.close_cells(cells)

    def open_corridor(self, cells: set[Cell] | list[Cell]) -> None:
        wanted = set(cells)
        remaining: list[Closure] = []
        for closure in self.closures:
            left = closure.cells - wanted
            if left:
                closure.cells = left
                remaining.append(closure)
        self.closures = remaining
        self.closed_cells = set().union(*(c.cells for c in self.closures)) if self.closures else set()
        self.gmap.closed_cells = set(self.closed_cells)
        self.exiting = {}

    def plan_all(self, robots: list[Robot], start_tick: int, strict: bool = True) -> None:
        self._refresh_base_holds(robots, start_tick)
        self._clear_replanned_futures(robots, start_tick)
        affected = self._closure_affected(robots, start_tick)

        for robot in affected:
            self._release_future(robot, start_tick)
            exit_cell = self._nearest_exit(robot.pos)
            if exit_cell is None:
                robot.last_blockers = ["closed"]
                robot.status = RobotStatus.FAILED if strict else RobotStatus.WAITING
                raise CoordinationError("机器人被困在封闭通道中", {robot.robot_id: PlanResult(INFEASIBLE)})
            if robot.pos != exit_cell:
                self.exiting[robot.robot_id] = exit_cell
            else:
                self.exiting.pop(robot.robot_id, None)
            result = plan_path_result(
                self.gmap,
                self.reservations,
                robot.pos,
                exit_cell,
                start_tick,
                self.horizon,
                robot_id=robot.robot_id,
                allowed_closed=self.closed_cells if robot.pos in self.closed_cells else None,
            )
            if not result.found:
                robot.path = [robot.pos]
                robot.plan_start = start_tick
                robot.last_blockers = sorted(result.blockers) or ["closed"]
                robot.status = RobotStatus.WAITING
                self.reservations.reserve_path(robot.robot_id, robot.path, start_tick, reserve_goal=False)
                if strict:
                    raise CoordinationError("封闭通道内机器人无退出安排", {robot.robot_id: result})
            else:
                robot.path = result.path
                robot.plan_start = start_tick
                robot.status = RobotStatus.WAITING
                robot.last_blockers = []
                self.reservations.reserve_path(robot.robot_id, result.path, start_tick, reserve_goal=False)

        candidates = self._candidates(robots, affected, strict)
        ordered = sorted(
            candidates,
            key=lambda r: (
                0 if self.pending.intersection({r.robot_id}) else 1,
                -(r.task.priority if r.task else 0),
                r.task.deadline if r.task and r.task.deadline is not None else 1 << 30,
                -r.total_wait,
                r.robot_id,
            ),
        )
        attempts = self._candidate_orders(ordered)
        last_results: dict[str, PlanResult] = {}
        for attempt in attempts:
            planned: list[Robot] = []
            results: dict[str, PlanResult] = {}
            for robot in attempt:
                result = plan_path_result(
                    self.gmap,
                    self.reservations,
                    robot.pos,
                    robot.goal,
                    start_tick,
                    self.horizon,
                    robot_id=robot.robot_id,
                    deadline_tick=robot.task.deadline if robot.task else None,
                )
                results[robot.robot_id] = result
                if not result.found:
                    for planned_robot in planned:
                        self._release_future(planned_robot, start_tick)
                        self.reservations.reserve_path(
                            planned_robot.robot_id, planned_robot.path, start_tick,
                        )
                    break
                self._release_future(robot, start_tick)
                self.reservations.release_hold(robot.robot_id)
                robot.path = result.path
                robot.plan_start = start_tick
                robot.status = RobotStatus.MOVING
                robot.last_blockers = []
                self.pending.discard(robot.robot_id)
                self.reservations.reserve_path(robot.robot_id, result.path, start_tick)
                planned.append(robot)
            if len(planned) == len(attempt):
                return
            last_results = results
        for robot in ordered:
            result = last_results.get(robot.robot_id, PlanResult(SEARCH_LIMIT))
            self._release_future(robot, start_tick)
            existing_end = robot.path[-1] if robot.path else robot.pos
            robot.path = [robot.pos]
            robot.plan_start = start_tick
            robot.status = RobotStatus.WAITING
            robot.last_blockers = sorted(result.blockers) or self._local_blockers(robot, start_tick)
            self.reservations.reserve_path(
                robot.robot_id, robot.path, start_tick,
                reserve_goal=(existing_end == robot.goal),
            )
        if strict:
            statuses = {item.status for item in last_results.values()}
            message = "无可行安排" if statuses == {INFEASIBLE} else "尚未找到可行安排"
            raise CoordinationError(message, last_results)

    def _candidate_orders(self, ordered: list[Robot]):
        yield ordered
        if len(ordered) <= 1:
            return
        seen = {tuple(robot.robot_id for robot in ordered)}
        for permutation in itertools.permutations(ordered):
            key = tuple(robot.robot_id for robot in permutation)
            if key not in seen:
                seen.add(key)
                yield list(permutation)

    def _refresh_base_holds(self, robots: list[Robot], start_tick: int) -> None:
        active_ids = {r.robot_id for r in robots}
        self.reservations.vertex = {
            key: owner for key, owner in self.reservations.vertex.items()
            if key[1] >= start_tick and owner in active_ids
        }
        self.reservations.edge = {
            key: owner for key, owner in self.reservations.edge.items()
            if key[2] >= start_tick and owner in active_ids
        }
        self.reservations.goal = {
            cell: owner for cell, owner in self.reservations.goal.items() if owner in active_ids
        }
        self.reservations.holds = {
            cell: owner for cell, owner in self.reservations.holds.items() if owner in active_ids
        }
        for robot in robots:
            if robot.status == RobotStatus.DONE:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
            elif robot.status == RobotStatus.PAUSED:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
                self.reservations.vertex = {
                    key: owner for key, owner in self.reservations.vertex.items()
                    if not (owner == robot.robot_id and key[1] > start_tick)
                }
                self.reservations.edge = {
                    key: owner for key, owner in self.reservations.edge.items()
                    if not (owner == robot.robot_id and key[2] > start_tick)
                }
                self.reservations.goal = {
                    cell: owner for cell, owner in self.reservations.goal.items()
                    if owner != robot.robot_id
                }
            elif not robot.task:
                self.reservations.reserve_hold(robot.robot_id, robot.pos)
            elif robot.path and robot.plan_start <= start_tick:
                idx = start_tick - robot.plan_start
                if idx < len(robot.path):
                    self.reservations.vertex[(robot.path[idx], start_tick)] = robot.robot_id

    def _clear_replanned_futures(self, robots: list[Robot], start_tick: int) -> None:
        for robot in robots:
            if robot.status == RobotStatus.DONE or robot.status == RobotStatus.PAUSED:
                continue
            should_clear = robot.status in (RobotStatus.IDLE, RobotStatus.WAITING, RobotStatus.FAILED)
            if robot.robot_id in self.exiting:
                should_clear = False
            if should_clear:
                self.reservations.goal = {
                    cell: owner
                    for cell, owner in self.reservations.goal.items()
                    if owner != robot.robot_id
                }
                self._release_future(robot, start_tick)

    def _closure_affected(self, robots: list[Robot], start_tick: int) -> list[Robot]:
        if not self.closed_cells:
            for robot in robots:
                if robot.robot_id in self.exiting:
                    if robot.pos == self.exiting.pop(robot.robot_id):
                        robot.status = RobotStatus.IDLE
            return []
        affected: list[Robot] = []
        for robot in robots:
            if robot.status in (RobotStatus.DONE, RobotStatus.FAILED) or not robot.path:
                continue
            if robot.pos in self.closed_cells and robot.status != RobotStatus.PAUSED:
                affected.append(robot)
        return affected

    def _candidates(self, robots: list[Robot], affected: list[Robot], strict: bool) -> list[Robot]:
        affected_ids = {r.robot_id for r in affected}
        out: list[Robot] = []
        for robot in robots:
            if not robot.goal or robot.status in (RobotStatus.DONE, RobotStatus.PAUSED, RobotStatus.FAILED):
                continue
            if robot.robot_id in self.exiting:
                continue
            if strict and robot.status == RobotStatus.WAITING and robot.path == [robot.pos] and robot.total_wait >= 2:
                continue
            if robot.status == RobotStatus.MOVING and robot.robot_id not in affected_ids and self._plan_still_valid(robot):
                continue
            out.append(robot)
        return out

    def _plan_still_valid(self, robot: Robot) -> bool:
        if not robot.path:
            return False
        for offset, cell in enumerate(robot.path):
            tick = robot.plan_start + offset
            if tick < 0:
                continue
            if not self.gmap.passable(cell):
                return False
            if self.reservations.holds.get(cell) not in (None, robot.robot_id):
                return False
            owner = self.reservations.vertex.get((cell, tick))
            if owner is not None and owner != robot.robot_id:
                return False
        return True

    def reconcile(self, robots: list[Robot], tick: int) -> list[Robot]:
        reached: list[Robot] = []
        for robot in robots:
            exit_cell = self.exiting.get(robot.robot_id)
            if exit_cell is not None and robot.pos == exit_cell:
                self.exiting.pop(robot.robot_id, None)
                reached.append(robot)
        for robot in reached:
            robot.status = RobotStatus.IDLE
        if reached:
            self._refresh_base_holds(robots, tick)
            self._plan_waiting(robots, tick)
        return reached

    def _plan_waiting(self, robots: list[Robot], tick: int) -> None:
        candidates = [
            robot for robot in robots
            if robot.goal
            and robot.status in (RobotStatus.IDLE, RobotStatus.WAITING)
            and robot.robot_id not in self.exiting
        ]
        ordered = sorted(
            candidates,
            key=lambda r: (
                -(r.task.priority if r.task else 0),
                r.task.deadline if r.task and r.task.deadline is not None else 1 << 30,
                -r.total_wait,
                r.robot_id,
            ),
        )
        for robot in ordered:
            result = plan_path_result(
                self.gmap,
                self.reservations,
                robot.pos,
                robot.goal,
                tick,
                self.horizon,
                robot_id=robot.robot_id,
                deadline_tick=robot.task.deadline if robot.task else None,
            )
            if result.found:
                self._release_future(robot, tick)
                robot.path = result.path
                robot.plan_start = tick
                robot.status = RobotStatus.MOVING
                robot.last_blockers = []
                self.reservations.reserve_path(robot.robot_id, result.path, tick)
            else:
                self._release_future(robot, tick)
                robot.path = [robot.pos]
                robot.plan_start = tick
                robot.status = RobotStatus.WAITING
                robot.last_blockers = sorted(result.blockers) or self._local_blockers(robot, tick)
                self.reservations.reserve_path(robot.robot_id, robot.path, tick)

    def _release_future(self, robot: Robot, start_tick: int) -> None:
        self.reservations.vertex = {
            key: owner for key, owner in self.reservations.vertex.items()
            if not (owner == robot.robot_id and key[1] >= start_tick)
        }
        self.reservations.edge = {
            key: owner for key, owner in self.reservations.edge.items()
            if not (owner == robot.robot_id and key[2] >= start_tick)
        }
        self.reservations.goal = {
            cell: owner for cell, owner in self.reservations.goal.items() if owner != robot.robot_id
        }

    def _local_blockers(self, robot: Robot, start_tick: int) -> list[str]:
        blockers: set[str] = set()
        for nxt in self.gmap.neighbors(robot.pos) + [robot.pos]:
            blockers.update(self.reservations.blockers(nxt, start_tick + 1, robot.pos, robot.robot_id))
        return sorted(blockers) or ([SEARCH_LIMIT] if not blockers else [])

    def _nearest_exit(self, start: Cell) -> Cell | None:
        if start not in self.closed_cells:
            return start
        pending = [(start, 0)]
        seen = {start}
        best: tuple[int, Cell] | None = None
        while pending:
            cell, dist = pending.pop(0)
            if cell not in self.closed_cells:
                candidate = (dist, cell)
                if best is None or candidate < best:
                    best = candidate
                continue
            for nxt in self._physical_neighbors(cell):
                if nxt not in seen:
                    seen.add(nxt)
                    pending.append((nxt, dist + 1))
        return best[1] if best else None

    def _physical_neighbors(self, cell: Cell) -> list[Cell]:
        x, y = cell
        out = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (x + dx, y + dy)
            if self.gmap.in_bounds(nxt) and nxt not in self.gmap.obstacles:
                out.append(nxt)
        return out
