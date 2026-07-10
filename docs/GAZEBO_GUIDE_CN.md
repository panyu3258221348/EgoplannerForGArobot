# Gazebo 仿真配置与参数说明（中文）

## 1. 启动方式

```bash
# Gazebo 仿真（带深度相机建图）
roslaunch gazebo_sim ego_gazebo.launch

# 轻量仿真（360° 渲染，推荐验证算法）
roslaunch plan_manage run_in_sim.launch
```

RViz 中使用 **2D Nav Goal** 点击目标点，机器人在 Gazebo 中自动规划并执行轨迹。

---

## 2. 机器人模型参数（URDF）

| 参数 | 值 | 说明 |
|------|-----|------|
| 碰撞盒 | 0.5m × 0.4m × 0.1m | 长×宽×高 |
| base_link 位置 | 碰撞盒上方 0.10m | 着地时 base_link z=0.10 |
| 深度相机 | base_link 前 0.10m, 上 0.05m | FOV 87°, 640×480 |
| 初始位置 | x=6, y=6, z=0.5 | 重力拉落到 z≈0.10 |
| 重力 | 开启 | 着地时有物理碰撞 |

## 3. 关键参数对照

### 3.1 地图 & 传感器

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `map_size_x/y` | 100m | 地图 XY 尺寸 |
| `map_size_z` | 10m | 地图 Z 尺寸 |
| `obstacles_inflation` | 0.1m | 障碍物膨胀半径 |
| `local_update_range` | 5.5m | 局部占据图更新范围 |
| 点云采样数 | 1000 点/帧 | 深度相机降采样 |
| 滑窗帧数 | 10 帧 | 局部占据图保留 0.33s |
| `pose_type` | 2 (ODOMETRY) | 使用点云模式建图 |

### 3.2 规划器参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `max_vel` | 0.75 m/s | 最大速度 |
| `max_acc` | 1.0 m/s² | 最大加速度 |
| `replan_thresh` | 0.05m | 每走 5cm 重规划（≈15Hz） |
| `planning_horizon` | 7.5m | 规划前瞻距离 |

### 3.3 地空切换

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `ground_height` | 0.10m | 地面巡航高度（base_link z 值） |
| `ground_judge` | 0.30m | z > 0.3 切换到飞行模式 |
| `aerial_penalty` | 1.2 | 空中节点代价系数 |
| `flying_cost_base` | 1.0 | 起飞额外代价 |
| `barrier_max` | 0.5m | 可跨越障碍最大高度 |

### 3.4 优化器参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `dist0` | 0.7m | 控制点目标安全距离 |
| `lambda_height` | 2.0 | 高度约束权重 |
| `lambda_fitness` | 3.0 | refine 阶段路径匹配权重 |
| `lambda_collision` | 0.5 | 碰撞避免权重 |
| `lambda_smooth` | 1.0 | 光滑性权重 |
| `lambda_ground` | 0.1 | 地面吸引力权重 |
| `lambda_nonholo` | 0.01 | 地面非完整约束权重 |
| `max_height` | 2.0m | 飞行高度上限 |
| `L-BFGS g_epsilon` | 0.1 | 梯度收敛阈值（放宽防 -1008） |
| `L-BFGS mem_size` | 32 | 内存窗口（增强曲率近似） |

---

## 4. Bur 修复清单

| # | 问题 | 文件 | 修复 |
|---|------|------|------|
| 1 | `inf=1>>20` = 0，A* 启发式失效 | `dyn_a_star.h` | 改为 `1e20` |
| 2 | `raycast_num_` 为 char，6.4 秒溢出 | `grid_map.h/cpp` | 改为 int |
| 3 | `tf::TransformBroadcaster` 内存泄漏 | `gazebo_controller.cpp` | 改用 `unique_ptr` |
| 4 | `odom_` 无锁竞争 | `ground_sim.cpp` | 加 `std::mutex` |
| 5 | A* 析构时指针未初始化 | `dyn_a_star.cpp` | 初始化 `nullptr` + 空检查 |
| 6 | grid_map odom 话题硬编码 | `grid_map.cpp` | 参数化 |
| 7 | `getInflateOccupancy` 越界返回 -1 | `grid_map.h` | 改为返回 0 |
| 8 | EMERGENCY_STOP 不响应新目标 | `ego_replan_fsm.cpp` | 补上状态处理 |
| 9 | Bspline 发布代码重复 | `ego_replan_fsm.cpp` | 提取 `publishBspline()` |
| 10 | `calFeasibilityCost` #ifdef 死代码 | `bspline_optimizer.cpp` | 删除 200 行 |
| 11 | A* 允许地下路径 | `dyn_a_star.cpp` | nz < 0 直接跳过 |
| 12 | refine 只重试 1 次 | `bspline_optimizer.cpp` | 改重试 2 次 |
| 13 | 点云 5000→1000 降采样 | `gazebo_controller.cpp` | 减少存储 |
| 14 | 占据图每帧清空导致墙消失 | `grid_map.cpp` | 10 帧滑窗保留 |
| 15 | 地面点云被标记为障碍 | `grid_map.cpp` | 过滤 z<0.05 |
| 16 | L-BFGS -1008 频繁崩溃 | `bspline_optimizer.cpp` | g_epsilon 放宽, mem_size 增大 |

---

## 5. 常见问题

### Q: 机器人不飞？
检查 `aerial_penalty` 和 `flying_cost_base`。当前设为 1.2/1.0，飞行代价略高于地面。如果障碍密度不够，A* 会走地面绕行。增大障碍密度或降低飞行代价到 1.0/0.0。

### Q: -1008 错误频繁？
检查 `dist0` 是否太大（当前 0.7）。降低到 0.4-0.5，或增大 `g_epsilon`。

### Q: refine 阶段碰障碍？
增大 `lambda_fitness`（当前 3.0），让轨迹更贴近 A* 参考路径。

### Q: 地图中墙总消失？
`N_CLOUD_FRAMES` 太小。当前 10 帧 = 0.33s。增大到 20-30 帧延长保留。

### Q: 机器人穿地？
检查 `ground_height` 是否匹配 base_link 实际高度。当前 0.10 = 碰撞盒底刚好触地。A* 有 nz<0 硬过滤。
