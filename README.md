# 多机器人协同运行平台(共享通道协调)

## 结构
- `mrc/gridmap.py` 栅格地图,含单宽狭窄通道场景
- `mrc/robot.py` 机器人状态(IDLE/MOVING/PAUSED/DONE)与任务
- `mrc/reservation.py` 时空预约表:顶点 + 边(防对向换边) + 终点锁 + 暂停占路
- `mrc/planner.py` 时间窗 A*,返回可行路径、硬不可行或搜索未找到,并汇总占用者
- `mrc/coordinator.py` 保留在途有效预约;新任务只能在剩余时空资源中规划,
  支持优先级/截止时间、通道封闭撤离、重开恢复和换优先级重试
- `mrc/executor.py` tick 执行器,JSON Lines 记录分配、计划、封路、暂停、恢复、阻塞、截止时间和位置
- `mrc/replay.py` 逐时刻重放碰撞、对向换边、封闭区闯入、截止时间、完成情况和阻塞原因

## 语义
- 所有机器人共用一张预约表,不允许各算各的最短路。
- 暂停:占用格预约保留,不凭空释放;其余机器人绕开重排。
- 恢复:从当前时刻、当前位置重新协调,不插队。
- 封路:已在封闭段内的机器人先撤离到最近开放端;若无开放端,可使用登记的临时停靠位,
  但撤离路径逐格校验预约,不抢占任何在途预约;撤到开放端而目标仍被封闭段隔开时等待重开。
- 临时停靠:`Executor.add_staging_cells` 登记;停靠机器人在重开后恢复协调。
- 重开:清掉撤离临时计划,但继续保留其他仍有效预约。
- 等待原因三分类,不再混成单一等待状态:
  - `blocked`:暂时受阻(他人在途预约、暂停占位、封路等待重开),条件变化后可继续;
  - `deadline_infeasible`:不受截止约束的搜索证明能到达但必然错过截止时间;
  - `search_limit`:搜索预算用尽仍无结论,不能据此判定可行或不可行。
- 明确结束:`infeasible`(目标永久占位/静态不可达)立即 `TERMINATED`(hard_blocked);
  等待中的任务超过截止 tick 时终止(deadline_infeasible/expired);封路中无退出安排为 trapped。
  每次(重)规划为每个任务输出 `disposition`:continue/wait/exit/stage/terminate 及原因。
- 新任务:高优先级不能抢占在途预约;紧急任务同样遵守该约束。
- 逐时刻复核:每个 tick 先校验下一占用格的顶点/边/封闭状态,冲突即重排,再移动。
- 调度记录恢复:`Executor.resume_from_log(gmap, robots, log)` 重放日志恢复位置与终态;
  已 `TERMINATED` 的任务保持终止,再次分配只记录 `reject_reactivation`,不重新激活。
- 后续任务分配沿用同一执行记录文件。

## 运行
    python main.py            # 演示:会车 + 暂停/恢复 + 第二轮任务
    python -m pytest tests    # 测试
