# 多机器人协同运行平台(共享通道协调)

## 结构
- `mrc/gridmap.py` 栅格地图,含单宽狭窄通道场景
- `mrc/robot.py` 机器人状态(IDLE/MOVING/PAUSED/DONE)与任务
- `mrc/reservation.py` 时空预约表:顶点 + 边(防对向换边) + 终点永久预约
- `mrc/planner.py` 时间窗 A*,在 (格, 时刻) 空间绕开他人预约
- `mrc/coordinator.py` 共享预约表上的优先级规划;失败换优先级整体重排,
  处理迎面冲突与等待死结;暂停场景尽力规划(被堵者原地等待)
- `mrc/executor.py` tick 执行器,JSON Lines 执行记录(分配/计划/暂停/恢复/位置)
- `mrc/replay.py` 重放检查:碰撞与安全距离、对向换边、完成情况、饥饿

## 语义
- 所有机器人共用一张预约表,不允许各算各的最短路。
- 暂停:占用格预约保留,不凭空释放;其余机器人绕开重排。
- 恢复:从当前时刻、当前位置重新协调,不插队。
- 后续任务分配沿用同一执行记录文件。

## 运行
    python main.py            # 演示:会车 + 暂停/恢复 + 第二轮任务
    python -m pytest tests    # 测试
