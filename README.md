# 多机器人协同运行平台(共享通道协调)

## 结构
- `mrc/gridmap.py` 栅格地图,含单宽狭窄通道场景
- `mrc/robot.py` 机器人状态(IDLE/MOVING/WAITING/SEARCH_LIMIT/PAUSED/FAILED/DONE)与任务
- `mrc/reservation.py` 时空预约表:顶点 + 边(防对向换边) + 终点锁 + 暂停占路
- `mrc/planner.py` 时间窗 A*,返回可行路径、硬不可行或搜索未找到,并汇总占用者
- `mrc/coordinator.py` 保留在途有效预约;新任务只能在剩余时空资源中规划,
  支持优先级/截止时间、通道封闭撤离、重开恢复和换优先级重试
- `mrc/cargo.py` 运输物归属与交接承诺(promised/started/in_progress/completed/failed/cancelled)
- `mrc/executor.py` tick 执行器,JSON Lines 记录分配、计划、封路、暂停、恢复、阻塞、搜索预算、终止、截止时间和位置
- `mrc/replay.py` 逐时刻重放碰撞、对向换边、封闭区闯入、截止时间、完成情况和阻塞原因

## 语义
- 所有机器人共用一张预约表,不允许各算各的最短路。
- 暂停:占用格预约保留,不凭空释放;其余机器人绕开重排。
- 恢复:从当前时刻、当前位置重新协调,不插队。
- 封路:已在封闭段内的机器人先撤离到最近开放端,段外机器人不得进入。
- 重开:清掉撤离临时计划,但继续保留其他仍有效预约。
- 新任务:高优先级不能抢占在途预约;明确机器人/封路占用记为暂时等待,搜索预算耗尽记为 `SEARCH_LIMIT`,永久占位或静态不可达记为不可行。
- 截止时间:错过截止且无法挽回时终止任务,记录 `deadline_infeasible` 和阻塞原因。
- 持久判定:已判不可行的任务按任务 ID 保留,后续重算不会重新激活;新协调器可通过 `load_terminal_decisions()` 从既有日志恢复;日志中的 `task_decision` 给出 continue/exit/wait/search_pending/terminate。
- 封路:封闭段内机器人先撤离,必要时在出口临时停靠;原目标仍封闭时只等待,不抢占其他预约。
- 后续任务分配沿用同一执行记录文件。

## 交接语义
- 交接区由两个相邻占用槽组成,交出方与接收方各占一格;两条交接段与后续送货段
  全部进同一张预约表,不会因为“是交接”而绕过通道预约。
- 只有双方都实际到位、各自完成交接段占用后,运输物才转移归属;一方先到时物体仍
  只属于交出方,任何时刻都不会同时记在两个机器人名下。
- 承诺生命周期:`promised`(尚未开始)→ `started`(任一方沿交接段离开起点)
  → `in_progress`(一方已在交接槽等待)→ `completed`(双方到齐,归属转交)。
- 一方延迟时另一方可等待或撤离(占用不凭空释放);故障或任务取消时交接失败,
  物体归属保留在当前持有者,两段交接任务都终止且不复活。
- 只有 `promised` 的承诺可以取消或改换交接点;`started` 之后不可撤回。
  `compare_handovers()` 在预约表副本上 dry-run 原方案与替代方案,
  原先已经执行的路径和预约保留在日志中,不被重写。
- 紧急任务插入时用 `handover_policy()` 查询:未开始的承诺可调整,
  已开始的承诺只能等待/撤离,不能撤回。
- 重放逐时刻核对唯一归属、槽位避碰、交接完成时机以及终止任务不被重新指派。

### 交接 API
    executor.register_cargo("C1", owner="R1")
    executor.register_handover_zone("Z", carrier_slot=(5,1), receiver_slot=(6,1))
    executor.request_handover(
        "H1", cargo_id="C1", carrier="R1", receiver="R2", zone_id="Z",
        carrier_task_id="L1", receiver_task_id="L2",
        delivery_task_id="D1", delivery_goal=(11,1),
    )
    executor.handover_options("H1", "R1", evac_cell=(4,1))  # 等待/撤离/改换点
    executor.compare_handovers("H1")                        # 原方案 vs 替代区 dry-run
    executor.handover_policy("H1")                          # 紧急插入时的可调整性

## 运行
    python main.py            # 演示:会车 + 暂停/恢复 + 第二轮任务
    python -m pytest tests    # 测试
