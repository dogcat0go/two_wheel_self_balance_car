# 两轮自平衡车（ROS 2）

Gazebo 仿真 + 串级平衡控制（直立 PD + 速度 PI）+ `/cmd_vel` 运动与转向。

更详细的原理与调参见 `docs/` 目录，例如：

- `docs/cascaded_pid_theory.html` — 串级 PID 理论
- `docs/balance_velocity_tuning_guide.html` — 直立环 / 速度环调参
- `docs/motion_control_framework.html` — 速度与转向框架
- `docs/three_loop_cascade_and_position_tuning.html` — 三环串级与位置环调参

**斜坡仿真验收**见本文 [§5 斜坡测试](#5-斜坡仿真测试)。

---

## 环境准备

```bash
cd ~/ros2_projects/self_balance_car_class/two_wheel_self_balance_car
colcon build --symlink-install
source install/setup.bash
```

修改 Python 或 launch 后若已使用 `--symlink-install`，一般只需重新 `source install/setup.bash`；  
新增 `.py` 安装项或改 `CMakeLists.txt` 后需重新 `colcon build`。

---

## 1. 启动 Gazebo 仿真（支持暂停）

在**终端 1** 中：

```bash
source install/setup.bash
ros2 launch balance_car_description gazebo.launch.py \
  pause:=true \
  gui:=true \
  controller_type:=effort
```

| 参数 | 说明 |
|------|------|
| `pause:=true` | 启动后**物理仿真暂停**，便于先看清模型、再开控制 |
| `pause:=false` | 启动后立即仿真（默认行为） |
| `gui:=true` | 打开 Gazebo 图形界面（点 ▶ 可继续仿真） |
| `gui:=false` | 无界面，省 CPU；暂停后需用下面的 **service** 解除暂停 |
| `controller_type:=effort` | 力矩控制（平衡车推荐） |
| `world:=...` | 世界文件；默认 `empty.world`；斜坡见 [§5](#5-斜坡仿真测试) |
| `spawn_x/y/z/yaw` | 生成位姿；斜坡推荐值见 §5 |

### 解除暂停（开始仿真）

**有 GUI：** 点击 Gazebo 窗口工具栏的 **播放（▶）**。

**无 GUI（`gui:=false`）或习惯用命令行：**

```bash
# 先确认服务名（不同版本可能带 /gazebo 前缀）
ros2 service list | grep -i pause

# 常见名称（Gazebo Classic + gazebo_ros）：
ros2 service call /unpause_physics std_srvs/srv/Empty {}
# 若上面不存在，可试：
ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty {}
```

暂停仿真：

```bash
ros2 service call /pause_physics std_srvs/srv/Empty {}
```

> **说明：** 仿真处于暂停时，IMU、`/joint_states` 不会更新。请先完成 spawn 与控制器启动，再在合适时机 **unpause**，然后观察平衡。

---

## 2. 启动平衡控制器（使用非仿真时间）

在**终端 2** 中（等 Gazebo 与控制器 spawner 起来后再执行，或先执行再等 3 s 节点自启）：

```bash
source install/setup.bash
ros2 launch balance_car_control balance_controller.launch.py \
  enabled:=true \
  use_sim_time:=false
```

| 参数 | 说明 |
|------|------|
| `use_sim_time:=false` | 控制节点用**系统墙钟**，定时器约 200 Hz，不跟 `/clock` 变慢 |
| `enabled:=true` | 发布非零轮力矩；`false` 为仅观测、输出 0 |
| `use_sim_time:=true` | 与 Gazebo `/clock` 同步（RTF 低时控制会变慢） |

launch 内已有 **3 s 延迟** 再启动 `balance_controller_node`，避免 IMU / 控制器未就绪。

参数文件：`src/balance_car_control/config/balance_controller.yaml`（改 yaml 后重启 launch 即可）。

### 推荐启动顺序（带 pause）

1. 终端 1：`gazebo.launch.py` + `pause:=true`（斜坡时加 `world` / `spawn_x/z`，见 [§5.2](#52-启动与-12-相同流程)）
2. 终端 2：`balance_controller.launch.py` + `use_sim_time:=false` + `enabled:=true`
3. 确认无报错后：**unpause**（GUI ▶ 或 `unpause_physics` service）
4. 观察车体平衡，再发 `/cmd_vel`（平地见 §3，斜坡见 §5.3）

---

## 3. 转向与运动测试命令

`/cmd_vel` 为 **目标线速度 / 角速度**（不是加速度）。  
站立时不要长期发很小的 `angular.z`（会低于偏航激活阈值）；需要转向时请发 **≥ 0.1 rad/s** 的 `angular.z`。

### 3.1 仅原地转向

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

`angular.z`：逆时针为正（ROS 惯例）。转向太慢可在 yaml 中增大 `kp_yaw`、`max_turn_tau`。

### 3.2 仅前进（不转向）

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.3 前进 + 转向（弧线）

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

### 3.4 停止运动（松开发令或超时）

停止发布后约 **0.5 s**（`cmd_vel_timeout_sec`）目标速度自动归零，外环按站立平衡处理。

也可显式发零：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.5 常用查看

```bash
# 调试量：pitch、pitch_target、轮速、turn_tau 等
ros2 topic echo /balance_controller/debug

# 控制输出频率
ros2 topic hz /wheel_effort_controller/commands

# 仿真时钟（use_sim_time:=true 时才需关注）
ros2 topic hz /clock
```

---

## 4. 一键参考（复制用）

```bash
# 终端 1 — 仿真（先暂停）
source install/setup.bash
ros2 launch balance_car_description gazebo.launch.py pause:=true gui:=true controller_type:=effort

# 终端 2 — 平衡控制（墙钟）
source install/setup.bash
ros2 launch balance_car_control balance_controller.launch.py enabled:=true use_sim_time:=false

# 终端 3 — 解除暂停后开始转
ros2 service call /unpause_physics std_srvs/srv/Empty {}
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

斜坡一键命令见 [§5.6](#56-一键复制5-上坡前--上坡)。

---

## 5. 斜坡仿真测试

地形由 `scripts/generate_dual_slope_worlds.py` 生成，**上坡 → 岭台 → 下坡 → 平地** 一体赛道（内嵌于 world，不叠加 `ground_plane`）。  
修改坡角后需重新 build：

```bash
colcon build --packages-select balance_car_description --symlink-install
source install/setup.bash
```

可选：改脚本顶部 `GENTLE_CONFIG` / `PID_CONFIG` 的 `angle_deg`，或：

```bash
python3 src/balance_car_description/scripts/generate_dual_slope_worlds.py --gentle-angle-deg 6
colcon build --packages-select balance_car_description --symlink-install
```

### 5.1 选用哪个 world

| world 文件 | 坡角 | 岭台高度 | 适用场景 |
|------------|------|----------|----------|
| `gentle_slope.world` | 5° | ≈ 0.175 m | 三环串级入门：上坡、岭台站住、停车回位 |
| `balance_pid_test.world` | 8° | ≈ 0.281 m | 进阶：更大重力分量，检验 `kp_v` / `pitch_target_limit` |

Gazebo 彩色标记线（沿 Y 方向横条）：

| 颜色 | 含义 |
|------|------|
| 绿 | 上坡起点（x ≈ −3） |
| 黄 | 岭台（坡顶平台） |
| 蓝 | 下坡起点 |
| 红 | 下坡落点（接后平地） |

```
x=-6 ─ 前平地 ─ x=-3 上坡 ─ 岭台 ─ 下坡 ─ x=+5 后平地
              ▲ 推荐 spawn（上坡前）    ▲ 岭台 spawn
```

### 5.2 启动（与 §1、§2 相同流程）

**终端 1** — 仿真（示例：`gentle_slope`，上坡前 spawn）：

```bash
source install/setup.bash
ros2 launch balance_car_description gazebo.launch.py \
  pause:=true gui:=true controller_type:=effort \
  world:=$(ros2 pkg prefix balance_car_description)/share/balance_car_description/worlds/gentle_slope.world \
  spawn_x:=-4.5 spawn_y:=0.0 spawn_z:=0.034 spawn_yaw:=0.0
```

**终端 2** — 平衡控制（墙钟，与平地相同）：

```bash
source install/setup.bash
ros2 launch balance_car_control balance_controller.launch.py \
  enabled:=true use_sim_time:=false
```

**终端 3** — 解除暂停后再发 `/cmd_vel`（见 §5.3）。

换 **8° 双坡** 时只改 `world` 与岭台 `spawn_z`：

```bash
world:=$(ros2 pkg prefix balance_car_description)/share/balance_car_description/worlds/balance_pid_test.world
spawn_x:=-4.5 spawn_z:=0.034    # 上坡前（与 5° 相同）
# 岭台站立：spawn_x:=-0.5 spawn_z:=0.315
```

| 位置 | `spawn_x` | `spawn_z`（5° / 8°） | 说明 |
|------|-----------|----------------------|------|
| 上坡前平地 | `-4.5` | `0.034` / `0.034` | 默认测试起点，车头朝 **+X**（`spawn_yaw:=0`） |
| 岭台中心 | `-0.5` | `0.209` / `0.315` | 直接测坡顶站住、位置环 |
| 后平地 | `3.5` | `0.034` / `0.034` | 下坡落地后的验收 spawn（可选） |

轮心高度 ≈ **路面高度 + 0.0325**（轮半径）。坡角改动后请以对应 `.world` 文件头注释为准。

### 5.3 推荐测试顺序

按 **A → B → C → D → E** 做；5° 通过后再做 8° `balance_pid_test.world`。

**测试 A — 上坡前站立 + 爬上岭台（必做）**

1. 终端 3：`ros2 service call /unpause_physics std_srvs/srv/Empty {}`
2. 不发 cmd_vel，确认上坡前平地能站稳 **3～5 s**。
3. 小速度上坡（先 5° world）：

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

4. 观察能否过绿线、上岭台；到岭台后 **Ctrl-C 停发 cmd_vel**。
5. **验收**：`wheel_v`（debug[8]）趋近 0；坡上 `pitch`（debug[0]）有**稳定偏置**；不持续下滑。

**测试 B — 岭台直接站立**

1. 重启 Gazebo，`spawn_x:=-0.5`，`spawn_z:=0.209`（5°）或 `0.315`（8°）。
2. unpause 后不发 cmd_vel。
3. **验收**：`pitch_target`（debug[2]）非零且能稳住；`|wheel_v|` 小。

**测试 C — 岭台前进 + 停车回位（位置环）**

1. 在岭台 spawn（同 B），unpause 后小速度前进 **1～2 s**，再停发 cmd_vel。
2. **验收**：停车后无明显前后溜；debug[13] `x` 与 debug[14] `x_target` 收敛（见 §5.4）。

**测试 D — 下坡（进阶）**

1. 岭台 spawn，或测试 A 爬上岭台后停住。
2. 小负速度：

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

3. **验收**：不猛冲、不倒；落红线附近后 `wheel_v` 能收住。
4. 刹不住：略增 `ki_v`，检查 `pitch_target_limit` 是否顶死（`balance_controller.yaml`）。

**测试 E — 全程（可选）**

上坡前 spawn → 持续 `linear.x: 0.08` 直至过后平地 → 停发 cmd_vel，观察下坡段与后平地停车。用于检查三环在**连续变坡**下是否切换平滑。

### 5.4 观察与记录

```bash
ros2 topic echo /balance_controller/debug --field data
```

| debug 下标 | 量 | 斜坡上看什么 |
|------------|-----|----------------|
| 0 | pitch | 稳态倾角（坡角 + 重力补偿） |
| 2 | pitch_target | 是否顶在 `pitch_target_limit` |
| 8 | wheel_v | 持续非零 → 溜坡 |
| 13 | x | 轮位置积分 |
| 14 | x_target | 停车后应与 13 收敛 |
| 15 | velocity_setpoint | 位置环回位时短暂非零 |

录 bag 对比上坡 / 岭台 / 下坡 / 停车：

```bash
ros2 bag record /balance_controller/debug /imu/data /joint_states /wheel_effort_controller/commands
```

### 5.5 常见现象与调参

| 现象 | 可能原因 | 建议 |
|------|----------|------|
| 上坡中途倒 | 力矩饱和 | 查是否顶 `max_wheel_effort`；谨慎增大 `pitch_target_limit` |
| 岭台持续下滑 | 速度环积分不足 | 略增 `ki_v`；确认 `velocity_output_sign` |
| 岭台前后蹭 | 位置环过大 | 减小 `kp_x` |
| 停车仍溜 | 位置环未开 | `position_loop_enabled: true` |
| 一上坡就猛冲 | cmd_vel 过大 | `linear.x` 先用 `0.05～0.08` |
| 8° 过不了、5° 可以 | 坡太陡 | 先调好 5°，再单独加大 `kp_v` / `ki_v` |

参数文件：`src/balance_car_control/config/balance_controller.yaml`。

### 5.6 一键复制（5° 上坡前 → 上坡）

```bash
# 终端 1
source install/setup.bash
ros2 launch balance_car_description gazebo.launch.py pause:=true gui:=true \
  controller_type:=effort \
  world:=$(ros2 pkg prefix balance_car_description)/share/balance_car_description/worlds/gentle_slope.world \
  spawn_x:=-4.5 spawn_z:=0.034

# 终端 2
source install/setup.bash
ros2 launch balance_car_control balance_controller.launch.py enabled:=true use_sim_time:=false

# 终端 3
ros2 service call /unpause_physics std_srvs/srv/Empty {}
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 6. 故障简查

| 现象 | 可能原因 |
|------|----------|
| 控制器几乎不动 | 仿真仍 pause；或 `enabled:=false` |
| 控制很慢、日志稀疏 | 用了 `use_sim_time:=true` 且 `/clock` 很低 → 改用 `use_sim_time:=false` |
| 一转向就漂 / 倒 | `angular.z` 过小未过 `yaw_cmd_activate_threshold`；或 `kp_yaw` 过大 |
| 转向很慢 | 增大 `kp_yaw`、`max_turn_tau`（见 yaml 偏航环段） |
| `wheel_effort_controller` 未加载 | 重新 `colcon build` 并 `source`；确认 `controller_type:=effort` |
| 缓坡 spawn 后轮子悬空/陷地 | 对照 §5.2：`spawn_z` 平地 `0.034`，岭台 5° 用 `0.209`、8° 用 `0.315` |
| 地形与文档不一致 | 重新 `colcon build balance_car_description`（会重生成 world） |

---

## 7. 仓库结构（简要）

```
two_wheel_self_balance_car/
├── README.md                 # 本文件
├── docs/                     # HTML 说明文档
└── src/
    ├── balance_car_description/   # URDF、worlds（gentle_slope / balance_pid_test）、gazebo.launch.py
    └── balance_car_control/       # 平衡节点、yaml、balance_controller.launch.py
```
