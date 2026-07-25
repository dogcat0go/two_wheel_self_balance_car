<!--
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-07-25 05:13:35
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-07-25 08:11:58
 * @FilePath: /two_wheel_self_balance_car/tools/ros_command.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# ROS 常用命令速查

> 先构建并 source：
>
> ```bash
> colcon build && source install/setup.bash
> ```

## 1. 启动斜坡仿真（Gazebo）

`gentle_slope.world` 斜坡世界，出生点在坡底前方：

```bash
ros2 launch balance_car_description gazebo.launch.py \
  pause:=true gui:=true controller_type:=effort \
  world:=$(ros2 pkg prefix balance_car_description)/share/balance_car_description/worlds/gentle_slope.world \
  spawn_x:=-4.5 spawn_y:=0.0 spawn_z:=0.034 spawn_yaw:=0.0
```

- `pause:=true` 起始暂停，摆好后再点 Gazebo 播放。
- `gui:=false` 关掉 gzclient，省 30-50% CPU（调参推荐）。
- `controller_type:=effort`（力矩）或 `velocity`（速度）。
- 出生点参考：坡底 `spawn_x:=-4.5 spawn_z:=0.034`；坡顶岭台 `gentle 0.209 / pid 0.315`。

其它世界（替换 `world:=` 即可）：`empty.world`、`balance_pid_test.world`、`imu_debug.world`、`mine_design.world`。

> `/clock` 频率已通过 `config/gazebo_params.yaml` 提到 1000Hz（gazebo_ros 默认仅 10Hz，
> 会把 `use_sim_time` 下的控制器定时器限到 10Hz）。改动已内置在 `gazebo.launch.py`。

## 2. 启动平衡控制器

LQR 全状态反馈（用 `config/balance_lqr.yaml`）：

```bash
ros2 launch balance_car_control balance_controller.launch.py \
  control_mode:=lqr use_sim_time:=false enabled:=true
```

级联 PID（默认 `config/balance_controller.yaml`）：

```bash
ros2 launch balance_car_control balance_controller.launch.py \
  control_mode:=pid use_sim_time:=true enabled:=true
```

- `enabled:=false` 只算不发力矩（干跑观察）。
- `config_file:=/abs/path.yaml` 手动指定配置，覆盖 `control_mode` 自动选择。
- 控制器与斜坡估计节点 `slope_dyn_obs` 一起延迟 3s 启动（等 Gazebo controllers 就绪）。

## 3. 角度（斜坡）估计节点

一维 KF 斜坡估计器（发布 `/slope/alpha` 与 `/slope/dyn_debug`）已随
`balance_controller.launch.py` 一起启动，无需单独运行。单独调试时：

```bash
ros2 run balance_car_control slope_dyn_obs_node --ros-args -p use_sim_time:=true
```

IMU 调试节点（可选）：

```bash
ros2 run balance_car_control imu_debug_node
```

## 4. rosbag 录制 / 回放

关键话题：
`/imu/data` `/joint_states` `/wheel_effort_controller/commands`
`/balance_controller/debug` `/slope/alpha` `/slope/dyn_debug` `/cmd_vel` `/clock`

录制（带仿真时钟）：

```bash
ros2 bag record -o slope_run \
  /clock /imu/data /joint_states /wheel_effort_controller/commands \
  /balance_controller/debug /slope/alpha /slope/dyn_debug /cmd_vel
```

录制所有话题：

```bash
ros2 bag record -a -o all_run
```

回放（`--clock` 让下游用仿真时间）：

```bash
ros2 bag play slope_run --clock
```

查看信息：

```bash
ros2 bag info slope_run
```

## 5. 手动发速度指令（前进/转向）

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```
