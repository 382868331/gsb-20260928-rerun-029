# 多机器人协同运行平台基础实现

使用 Python 标准库实现图地图、机器人状态、共享通道调度、任务执行和日志重放。

## 核心语义

- GraphMap：维护节点、邻接边、BFS 最短路和狭窄通道。
- Coordinator：每个 tick 只允许机器人沿一条邻接边移动，逐机器人预留目标节点和移动边。
- Passage：通道按方向发放租约；同方向可成队通过，反方向等待，通道清空后释放租约。
- 死结处理：通道内机器人保持同一方向；入口冲突由等待计数和确定性机器人编号裁决，避免迎面进入后互锁。
- 公平性：等待越久优先级越高；重放中连续等待超过 starvation_limit 判定为饿死。
- 暂停恢复：暂停机器人继续占用当前节点；恢复后不能传送，也不能插入前方，只能重新参与调度。
- 执行记录：任务分配、移动、等待、完成和暂停写入同一份 JSON 记录。
- replay_log：仅依据地图和记录重放，检查重合碰撞、迎面换边、非法移动、暂停移动、完成位置、未完成任务和持续饥饿。

默认安全距离为图上 1 条边，即机器人不得重合；调用 replay_log(path, safe_distance=2) 可要求至少间隔一条边。

## 运行

演示：

    python examples/demo.py

测试：

    python -m unittest discover -s tests -v

演示生成 examples/execution_log.json。两轮任务使用同一个 Simulation、Coordinator 和 ExecutionLog；第一轮 r1 会停在通道内数 tick，用于验证占用保留、恢复顺序和后续任务分配。
