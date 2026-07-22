# 斜坡角度估计验证方案（前馈控制前提验证）

目标：在把斜坡角 α̂ 接入前馈（τ_ff = M·g·r·sin α̂）之前，先用一个**独立观测节点**
实现"动力学通道 + 运动学通道"双通道估计器，与 Gazebo 真值对比，定量回答两个问题：

1. **速度**：坡度阶跃后 α̂ 多久收敛（t_90、时延）
2. **准确度**：稳态误差、全程 RMSE

估计器不接入控制回路，现有 LQR / PID 行为完全不受影响。

---

## 1. 算法设计

### 1.1 动力学通道（全时有效，含 v = 0）

沿坡方向牛顿方程反解重力分量：

```
sin α̂_dyn = [ τ/r − m̃·v̇ − m_total·L·(θ̈·cos(θ̂+α̂) − θ̇²·sin(θ̂+α̂)) − c·v ] / (M·g)
```

| 量 | 来源 |
|---|---|
| τ | `/wheel_effort_controller/commands` 左右轮之和（effort 控制器直通，指令≈实际） |
| M = m_total, m_total·L, m̃ = m_total + J_spin/r² | 复用 `lqr_gain_design.py` 的 `load_xacro_properties` + `composite_inertia_about_axle`（即 a12、a22），**不重写建模** |
| r | `wheel_radius` = 0.0325 m |
| c | URDF 关节阻尼换算：c = 2 × 0.002 / r² ≈ 3.8 N·s/m |
| v, v̇ | 轮速 × r；一阶低通（alpha≈0.3）后差分 |
| θ̂, θ̇ | IMU 四元数 pitch、gyro y（同 `AttitudeEstimator`） |
| θ̈ | gyro y 差分 + 低通。该项量级小，第一版支持置零对比（参数开关） |

注意 α̂_dyn 右侧含 α̂ 本身（修正项），用上一拍融合值迭代即可。

### 1.2 运动学通道（瞬态有效，带门控）

IMU `linear_acceleration` 是车体系比力 f = a − g（静止时 z 轴读 +g）。
用 θ̂ 旋回世界系后取竖直分量：

```
f_wz = −sin θ̂ · f_x + cos θ̂ · f_z
sin α̂_kin = (f_wz − g) / v̇        仅当 |v̇| > 0.3 m/s²，否则保持上一拍
```

结果钳位到 ±α_max（默认 15°），防止除小数爆炸。

### 1.3 互补融合

```
α̂ += K_dyn · (α̂_dyn − α̂) · Δt/τ_α  +  1_kin · K_kin · (α̂_kin − α̂)
```

初始参数：τ_α = 0.5 s，K_dyn = 1.0，K_kin = 0.3，v̇ 门限 0.3 m/s²，α_max = 15°。
全部做成 ROS 参数，便于扫参。

### 1.4 符号约定（最容易出错处）

统一定义 **α > 0 = 上坡（车头抬高方向）**。实现后先做静态自检：
把车 spawn 在 gentle 5° 上坡段静止平衡，确认 α̂_dyn → +5°、f_wz 通道被门控挂起、
τ 稳态符号与"前馈需要正力矩顶住下滑"一致。三处符号（pitch、α、τ）对不上先改 sign 参数再跑动态实验。

---

## 2. 真值来源

- URDF 中给 `base_link` 增加 `libgazebo_ros_p3d` 插件，50 Hz 发布 `/ground_truth/odom`（仅调试用）。
- 赛道由 `generate_dual_slope_worlds.py` 生成，各段 x 区间写在 world 文件头部注释中，
  因此 **α_true 是世界系 x 的分段常值函数**：
  平地 0 → 上坡 +angle → 岭台 0 → 下坡 −angle → 平地 0。
- 分析脚本内置两套分段表（gentle 5° / pid_test 8°），按 bag 里的 x 查表生成真值序列。

---

## 3. 新增/改动文件

| 文件 | 内容 |
|---|---|
| `src/balance_car_control/src/slope_estimator.py` | 纯算法类 `SlopeEstimator`（仿 `AttitudeEstimator`，无 ROS 依赖，便于复用与离线回放） |
| `src/balance_car_control/src/slope_estimator_node.py` | 节点（仿 `imu_debug_node.py`）：订阅 `/imu/data`、`/joint_states`、`/wheel_effort_controller/commands`、`/ground_truth/odom`；200 Hz 发布 `/slope_estimator/debug` |
| `src/balance_car_control/scripts/analyze_slope_estimation.py` | 读 rosbag → 指标表 + α̂/α_dyn/α_kin/α_true 对比曲线 |
| `urdf/two_wheel_balance_65mm.urdf.xacro` | 加 p3d 真值插件 |
| `CMakeLists.txt` | 安装新脚本 |

`/slope_estimator/debug`（Float64MultiArray）排布：

```
[0] alpha_hat   [1] alpha_dyn   [2] alpha_kin   [3] kin_gate(0/1)
[4] v           [5] v_dot       [6] tau_total   [7] f_wz
[8] pitch       [9] x_world（来自 /ground_truth/odom，方便 rqt_plot 实时对照）
```

---

## 4. 实验矩阵

统一跑法：`gazebo.launch.py world:=<world> spawn_x:=-4.5` + `control_mode:=lqr enabled:=true`
+ `/cmd_vel` 定速，`ros2 bag record /slope_estimator/debug /ground_truth/odom`。

| # | 场景 | 验证点 |
|---|---|---|
| 1 | gentle 5°，v = 0.25 m/s 全程穿越 | 基准：上/下坡两次坡度阶跃响应 |
| 2 | pid_test 8°，v = 0.25 | 大角度、线性化误差 |
| 3 | gentle 5°，v = 0.10 / 0.40 | 速度敏感性（kin 通道信息量 ∝ v̇） |
| 4 | 直接 spawn 在上坡段静止（v ≈ 0） | **纯动力学通道**——"坡上停稳"前馈的关键前提 |
| 5 | empty.world 平地急加减速（cmd_vel 方波） | 误报测试：大 v̇ 瞬态是否把 α̂ 带偏 |

每组统计四个指标（真值切换点前后 0.2 s 剔除，避开轮子磕碰尖峰）：

- **t_90**：真值阶跃后 α̂ 首次进入 ±0.5° 带并保持 ≥ 0.5 s 的时间
- **稳态误差**：收敛后窗口内 mean(α̂ − α_true)
- **RMSE**：全程
- **时延**：α̂ 与 α_true 互相关峰值滞后

---

## 5. 验收标准（前馈前提是否成立）

前馈力矩 τ_ff = M·g·r·sin α̂，本车 M ≈ 0.66 kg、r = 0.0325 m，
**1° 估计误差仅折合 ≈ 0.004 N·m**（限幅 2.5 N·m 的 0.15%），因此精度要求不苛刻，重点在收敛速度与不发散：

| 指标 | 达标线 |
|---|---|
| 稳态误差 | < 1° |
| t_90（v = 0.25 m/s） | < 1.5 s |
| 静止坡上（场景 4） | 不发散，稳态误差 < 1.5° |
| 平地误报峰值（场景 5） | < 1° |

达标 → 把 α̂ 接入 `lqr_backend`（τ_ff 直接加到 `balance_tau`，K 不需要重设计，见 `前馈_LQR.md`）。
不达标 → 按顺序排查：① 调 τ_α / K_kin / 门限；② 查三处符号；③ 查 v̇ 滤波带宽（滞后 vs 噪声）。

---

## 6. 已知风险

- **坡道衔接磕碰**：上/下坡端点轮子撞击产生加速度尖峰，靠 kin 门控 + α_max 钳位兜底，指标统计剔除切换窗。
- **τ 用指令值**：忽略了 URDF 静摩擦 0.001 N·m，量级可忽略；若后续接真车需换成电流反馈。
- **v̇ 差分噪声**：低通截止过低会拖慢 kin 通道、抵消其"快"的价值，扫参时与 t_90 一起看。
