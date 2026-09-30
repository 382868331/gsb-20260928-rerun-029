"""协调器:锁定在途预约,新任务在剩余时空资源中规划."""

from __future__ import annotations

import itertools
from dataclasses import dataclass

from .gridmap import Cell, GridMap
from .planner import (
    DEADLINE_INFEASIBLE,
    INFEASIBLE,
    SEARCH_LIMIT,
    PlanResult,
    _static_unreachable,
    plan_path_result,
)
from .reservation import ReservationTable
from .robot import Robot, RobotStatus, WaitReason


class CoordinationError(Exception):
    def __init__(self, message: str, results: dict[str, PlanResult] | None = None):
        super().__init__(message)
        self.results = results or {}


@dataclass
class Closure:
    cells: set[Cell]


@dataclass
class Disposition:
    robot_id: str
    task_id: str | None
    decision: str  # continue / wait / exit / stage / terminate
    reason: str
    blockers: list[str]
    detail: dict


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
        self.staging_cells: set[Cell] = set()
        self.staged: set[str] = set()
        self.terminated: set[str] = set()
        self.last_dispositions: dict[str, Disposition] = {}

    def add_staging_cells(self, cells: set[Cell] | list[Cell]) -> None:
        """登记临时停靠位置;仅在原格非封闭且可通行时可用."""
        for cell in cells:
            if self.gmap.in_bounds(cell) and cell not in self.gmap.obstacles:
                self.staging_cells.add(cell)

    def close_corridor(self, cells: set[Cell] | list[Cell]) -> None:
        cells = {cell for cell in cells if self.gmap.passable(cell)}
        if not cells:
            return
        self.closures.append(Closure(set(cells)))
        self.closed_cells.update(cells)
        self.gmap.close_cells(cells)

    def open_corridor(self, cells: set[Cell] | list[Cell], robots: list[Robot] | None = None) -> None:
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
        self.exiting = {rid: cell for rid, cell in self.exiting.items() if cell in self.closed_cells}
        if not self.closed_cells:
            self.staged.clear()
        if robots is not None:
            for robot in robots:
                if robot.status == RobotStatus.WAITING and robot.last_blockers == ["closed"]:
                    if not self.closed_cells or robot.pos not in self.closed_cells:
                        robot.status = RobotStatus.IDLE
                        robot.wait_reason = None
                        robot.last_blockers = []

    def plan_all(self, robots: list[Robot], start_tick: int, strict: bool = True) -> None:
        self.last_dispositions = {}
        self._refresh_base_holds(robots, start_tick)
        self._clear_replanned_futures(robots, start_tick)
        affected = self._closure_affected(robots, start_tick)

        for robot in affected:
            self._release_future(robot, start_tick)
            self._plan_evacuation(robot, robots, start_tick, strict)

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
        # 候选机器人即将为自身规划;释放它们作为无任务空闲者留下的临时占位
        candidate_ids = {robot.robot_id for robot in ordered}
        self.reservations.holds = {
            cell: owner
            for cell, owner in self.reservations.holds.items()
            if not (owner in candidate_ids and cell not in self.reservations.permanent_holds)
        }
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
                robot.wait_reason = None
                self.pending.discard(robot.robot_id)
                self.reservations.reserve_path(robot.robot_id, result.path, start_tick)
                self._record(robot, "continue", "replanned", [], {})
                planned.append(robot)
            if len(planned) == len(attempt):
                return
            last_results = results
        for robot in ordered:
            result = last_results.get(robot.robot_id, PlanResult(SEARCH_LIMIT))
            if strict:
                continue
            self._classify_failure(robot, result, start_tick)

        if strict:
            results = {
                rid: res for rid, res in last_results.items()
                if any(r.robot_id == rid for r in ordered)
            }
            if results:
                statuses = {item.status for item in results.values()}
                message = "无可行安排" if statuses == {INFEASIBLE} else "尚未找到可行安排"
                raise CoordinationError(message, results)

    def _plan_evacuation(
        self, robot: Robot, robots: list[Robot], start_tick: int, strict: bool,
    ) -> None:
        """封闭段内机器人的退出安排:开放端优先,必要时临时停靠,不得抢占在途预约."""
        allowed = self.closed_cells if robot.pos in self.closed_cells else set()
        target = self._evacuation_target(robot.pos, start_tick, robot.robot_id)
        if target is None:
            robot.path = [robot.pos]
            robot.plan_start = start_tick
            robot.wait_reason = WaitReason.BLOCKED
            robot.last_blockers = self._local_blockers(robot, start_tick) or ["closed"]
            self.reservations.reserve_path(robot.robot_id, robot.path, start_tick, reserve_goal=False)
            self._record(robot, "wait", "no_exit", robot.last_blockers, {})
            if strict:
                raise CoordinationError(
                    "封闭通道内机器人无退出安排",
                    {robot.robot_id: PlanResult(INFEASIBLE, blockers={"closed"})},
                )
            self._terminate(robot, start_tick, reason="trapped", blockers=["closed"])
            return
        is_staging = target in self.staging_cells and target in self.closed_cells
        result = plan_path_result(
            self.gmap,
            self.reservations,
            robot.pos,
            target,
            start_tick,
            self.horizon,
            robot_id=robot.robot_id,
            allowed_closed=allowed | ({target} if target in self.closed_cells else set()),
        )
        if not result.found:
            robot.path = [robot.pos]
            robot.plan_start = start_tick
            robot.status = RobotStatus.WAITING
            robot.wait_reason = WaitReason.BLOCKED
            robot.last_blockers = sorted(result.blockers) or ["closed"]
            self.reservations.reserve_path(robot.robot_id, robot.path, start_tick, reserve_goal=False)
            self._record(robot, "wait", "exit_blocked", robot.last_blockers, {"exit": list(target)})
            if strict:
                raise CoordinationError("封闭通道内机器人无退出安排", {robot.robot_id: result})
            return
        robot.path = result.path
        robot.plan_start = start_tick
        robot.status = RobotStatus.WAITING
        robot.wait_reason = None
        robot.last_blockers = []
        self.exiting[robot.robot_id] = target
        if is_staging:
            self.staged.add(robot.robot_id)
        self.reservations.reserve_path(robot.robot_id, result.path, start_tick, reserve_goal=False)
        self._record(
            robot,
            "stage" if is_staging else "exit",
            "temporary_dock" if is_staging else "evacuate_to_open_end",
            [],
            {"exit": list(target)},
        )

    def _evacuation_target(self, start: Cell, tick: int, robot_id: str) -> Cell | None:
        """BFS 找最近的可用撤离点:开放端 > 临时停靠位;预约被占的位置不可用."""
        pending: list[tuple[Cell, int]] = [(start, 0)]
        seen = {start}
        best_open: tuple[int, Cell] | None = None
        best_stage: tuple[int, Cell] | None = None
        while pending:
            cell, dist = pending.pop(0)
            is_open = cell not in self.closed_cells
            is_stage = cell in self.staging_cells and not is_open
            if (is_open or is_stage) and self.reservations.is_free(cell, tick, owner=robot_id) \
                    and self.reservations.goal.get(cell) in (None, robot_id):
                if is_open:
                    candidate = (dist, cell)
                    if best_open is None or candidate < best_open:
                        best_open = candidate
                else:
                    candidate = (dist, cell)
                    if best_stage is None or candidate < best_stage:
                        best_stage = candidate
            if is_open:
                continue
            for nxt in self._physical_neighbors(cell):
                if nxt not in seen:
                    seen.add(nxt)
                    pending.append((nxt, dist + 1))
        chosen = best_open or best_stage
        return chosen[1] if chosen else None

    def _classify_failure(self, robot: Robot, result: PlanResult, start_tick: int) -> None:
        blockers = sorted(result.blockers) or self._local_blockers(robot, start_tick)
        self._release_future(robot, start_tick)
        existing_end = robot.path[-1] if robot.path else robot.pos
        robot.path = [robot.pos]
        robot.plan_start = start_tick
        if result.status == INFEASIBLE:
            self.reservations.reserve_path(
                robot.robot_id, robot.path, start_tick,
                reserve_goal=(existing_end == robot.goal),
            )
            self._terminate(robot, start_tick, reason="hard_blocked", blockers=blockers)
            return
        robot.status = RobotStatus.WAITING
        robot.last_blockers = blockers
        if result.status == DEADLINE_INFEASIBLE:
            robot.wait_reason = WaitReason.DEADLINE_INFEASIBLE
            self._record(robot, "wait", WaitReason.DEADLINE_INFEASIBLE, blockers,
                         {"deadline": robot.task.deadline if robot.task else None})
        elif result.status == SEARCH_LIMIT:
            robot.wait_reason = WaitReason.SEARCH_LIMIT
            if not blockers:
                blockers = [WaitReason.SEARCH_LIMIT]
                robot.last_blockers = blockers
            self._record(robot, "wait", WaitReason.SEARCH_LIMIT, blockers, {})
        else:
            robot.wait_reason = WaitReason.BLOCKED
            self._record(robot, "wait", WaitReason.BLOCKED, blockers, {})
        self.reservations.reserve_path(
            robot.robot_id, robot.path, start_tick,
            reserve_goal=(existing_end == robot.goal),
        )

    def _terminate(
        self, robot: Robot, tick: int, reason: str, blockers: list[str] | None = None,
    ) -> None:
        robot.status = RobotStatus.TERMINATED
        robot.wait_reason = None
        robot.end_reason = reason
        self.terminated.add(robot.robot_id)
        self.exiting.pop(robot.robot_id, None)
        self.staged.discard(robot.robot_id)
        if robot.task is not None:
            robot.task.end_tick = tick
            robot.task.end_reason = reason
            if reason in ("deadline_infeasible", "hard_blocked", "trapped", "expired"):
                robot.task.on_time = False
        self._release_future(robot, tick)
        self.reservations.release_hold(robot.robot_id)
        robot.path = [robot.pos]
        robot.plan_start = tick
        self.reservations.reserve_path(robot.robot_id, robot.path, tick, reserve_goal=False)
        self._record(robot, "terminate", reason, blockers or robot.last_blockers, {})

    def _record(
        self, robot: Robot, decision: str, reason: str,
        blockers: list[str], detail: dict,
    ) -> None:
        self.last_dispositions[robot.robot_id] = Disposition(
            robot_id=robot.robot_id,
            task_id=robot.task.task_id if robot.task else None,
            decision=decision,
            reason=reason,
            blockers=list(blockers or []),
            detail=dict(detail or {}),
        )

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
        self.reservations.permanent_holds = {
            cell: owner
            for cell, owner in self.reservations.permanent_holds.items()
            if owner in active_ids
        }
        self.reservations.sticky_holds = {
            cell: owner
            for cell, owner in self.reservations.sticky_holds.items()
            if owner in active_ids
        }
        for robot in robots:
            if robot.status in (RobotStatus.DONE, RobotStatus.TERMINATED):
                self.reservations.reserve_hold(robot.robot_id, robot.pos, permanent=True)
            elif robot.status == RobotStatus.PAUSED:
                self.reservations.reserve_hold(robot.robot_id, robot.pos, sticky=True)
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
            if robot.status == RobotStatus.TERMINATED:
                continue
            should_clear = robot.status in (
                RobotStatus.IDLE, RobotStatus.WAITING, RobotStatus.FAILED,
            )
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
                if robot.robot_id in self.staged and robot.pos not in self.closed_cells:
                    self.staged.discard(robot.robot_id)
                    robot.status = RobotStatus.IDLE
            return []
        affected: list[Robot] = []
        for robot in robots:
            if robot.status in (RobotStatus.DONE, RobotStatus.FAILED, RobotStatus.TERMINATED) or not robot.path:
                if robot.robot_id in self.staged and robot.pos in self.closed_cells \
                        and robot.status not in (
                            RobotStatus.DONE, RobotStatus.FAILED, RobotStatus.TERMINATED,
                            RobotStatus.PAUSED,
                        ):
                    affected.append(robot)
                elif (
                    robot.pos in self.closed_cells
                    and robot.status == RobotStatus.IDLE
                ):
                    affected.append(robot)
                continue
            if robot.pos in self.closed_cells and robot.status != RobotStatus.PAUSED:
                affected.append(robot)
        return affected

    def _candidates(self, robots: list[Robot], affected: list[Robot], strict: bool) -> list[Robot]:
        affected_ids = {r.robot_id for r in affected}
        out: list[Robot] = []
        for robot in robots:
            if not robot.goal or robot.status in (
                RobotStatus.DONE, RobotStatus.PAUSED, RobotStatus.FAILED, RobotStatus.TERMINATED,
            ):
                continue
            if robot.robot_id in self.exiting:
                continue
            if robot.status == RobotStatus.WAITING and robot.last_blockers == ["closed"]:
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
                if robot.pos in self.closed_cells:
                    # 撤到封闭段内的临时停靠位:停放等待重开,不视为恢复任务
                    robot.status = RobotStatus.WAITING
                    robot.wait_reason = WaitReason.BLOCKED
                    robot.last_blockers = ["closed"]
                    self.staged.add(robot.robot_id)
                    self._record(robot, "stage", "parked_until_reopen", ["closed"],
                                 {"dock": list(robot.pos)})
                else:
                    robot.status = RobotStatus.IDLE
                    self.staged.discard(robot.robot_id)
                    if self.closed_cells and robot.goal is not None and _static_unreachable(
                        self.gmap, robot.pos, robot.goal, set(),
                    ):
                        # 已撤到开放端,但原目标仍被未解除的封闭段隔开:等待重开
                        robot.status = RobotStatus.WAITING
                        robot.wait_reason = WaitReason.BLOCKED
                        robot.last_blockers = ["closed"]
                        robot.path = [robot.pos]
                        robot.plan_start = tick
                        self.reservations.reserve_path(
                            robot.robot_id, robot.path, tick, reserve_goal=False,
                        )
                        self._record(robot, "wait", "await_reopen", ["closed"], {})
                    else:
                        reached.append(robot)
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
                robot.wait_reason = None
                self.reservations.reserve_path(robot.robot_id, result.path, tick)
                self._record(robot, "continue", "replanned", [], {})
            else:
                self._classify_failure(robot, result, tick)

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

    def _physical_neighbors(self, cell: Cell) -> list[Cell]:
        x, y = cell
        out = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (x + dx, y + dy)
            if self.gmap.in_bounds(nxt) and nxt not in self.gmap.obstacles:
                out.append(nxt)
        return out
