# 两轮自平衡车（ROS 2）

Gazebo 仿真 + 串级平衡控制（直立 PD + 速度 PI）+ `/cmd_vel` 运动与转向。

更详细的原理与调参见 `docs/` 目录，例如：

- `docs/cascaded_pid_theory.html` — 串级 PID 理论
- `docs/balance_velocity_tuning_guide.html` — 直立环 / 速度环调参
- `docs/motion_control_framework.html` — 速度与转向框架

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

1. 终端 1：`gazebo.launch.py` + `pause:=true`
2. 终端 2：`balance_controller.launch.py` + `use_sim_time:=false` + `enabled:=true`
3. 确认无报错后：**unpause**（GUI ▶ 或 `unpause_physics` service）
4. 观察车体平衡，再发 `/cmd_vel`（见下节）

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

---

## 5. 故障简查

| 现象 | 可能原因 |
|------|----------|
| 控制器几乎不动 | 仿真仍 pause；或 `enabled:=false` |
| 控制很慢、日志稀疏 | 用了 `use_sim_time:=true` 且 `/clock` 很低 → 改用 `use_sim_time:=false` |
| 一转向就漂 / 倒 | `angular.z` 过小未过 `yaw_cmd_activate_threshold`；或 `kp_yaw` 过大 |
| 转向很慢 | 增大 `kp_yaw`、`max_turn_tau`（见 yaml 偏航环段） |
| `wheel_effort_controller` 未加载 | 重新 `colcon build` 并 `source`；确认 `controller_type:=effort` |

---

## 仓库结构（简要）

```
two_wheel_self_balance_car/
├── README.md                 # 本文件
├── docs/                     # HTML 说明文档
└── src/
    ├── balance_car_description/   # URDF、Gazebo world、gazebo.launch.py
    └── balance_car_control/       # 平衡节点、yaml、balance_controller.launch.py
```
