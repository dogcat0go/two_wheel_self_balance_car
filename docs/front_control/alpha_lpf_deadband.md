---
title: α̂ 入环前的死区与一阶低通
level: 1
order: 4
sections: true
---

# α̂ 入环前的死区与一阶低通

> TWIP 平衡车 · 控制集成卷 · 滤波基础
>
> 副标题：死区断环 + 一阶惯性低频化，以及 $3\tau\sim 5\tau$ 稳态时间

斜坡补偿里，$\hat{\alpha}$ 不能原样直通 $\theta_{\mathrm{eq}}$ / $\tau_{\mathrm{ff}}$（见 [把斜坡角接进 LQR](./lqr_slope_feedforward.md) 缺口 1）。控制器入口先做两件事：

1. **死区** — 平地小抖动归零，切断自激回路增益  
2. **一阶低通** — 压带宽，与快速平衡回路频带分离  

本节把这段实现背后的知识点说清楚，并用当前参数算一遍「多久跟到稳态」。

## 这块知识属于哪里 {#where}

| 知识点 | 通常归属 |
| --- | --- |
| 一阶惯性 $\tau\dot{y}+y=u$ | 信号与系统 / 经典控制（传递函数 $1/(\tau s+1)$） |
| 离散式 $y\leftarrow y+\frac{\mathrm{d}t}{\tau+\mathrm{d}t}(u-y)$ | 数字控制、嵌入式滤波 |
| 死区 | 非线性环节；工程里常作「零点附近熄火」 |
| LQR、状态空间 | **现代控制**（才大量用线性代数） |

结论：LQR 与前馈配对是现代控制应用；**死区 + 一阶低通是经典滤波与抗噪手段**，不必先啃完状态空间也能掌握。

## 死区：何时当零 {#deadband}

实现（`lqr_backend.py`）：

$$
\alpha_{\mathrm{db}} =
\begin{cases}
\hat{\alpha}, & |\hat{\alpha}|\ge \alpha_{\mathrm{db,min}} \\
0, & \text{否则}
\end{cases}
$$

当前默认：`slope_alpha_deadband = 0.015` rad $\approx 0.86^\circ$。

- 平地估计噪声典型落在死区内 → 补偿强制为 0，回路增益在零点附近断开  
- 真坡（如 $5^\circ\approx 0.087$ rad）远超死区 → 原样放行  

死区解决的是「该当零的小抖动别进环」；它**不**平滑跳变——那是低通的事。

## 一阶低通：连续形式 {#continuous}

把死区后的信号记为输入 $u=\alpha_{\mathrm{db}}$，滤波输出记为 $y=\hat{\alpha}_f$（代码里 `_alpha_f`）。连续时间一阶低通：

$$
\tau\,\dot{y} + y = u
\quad\Longleftrightarrow\quad
Y(s)=\frac{1}{\tau s+1}\,U(s)
$$

直觉：$y$ 以时间常数 $\tau$ **指数逼近** $u$，而不是瞬时跟上。

- $\tau$ 大 → 跟得慢、更平滑、带宽更低  
- $\tau\to 0$ → 直通  

当前默认：`slope_alpha_lpf_sec = 0.5` s，即 $\tau=0.5$。

## 离散写法怎么来的 {#discrete}

控制环每个周期步长为 $\mathrm{d}t$。对 $\tau\dot{y}+y=u$ 用**后向欧拉**离散：

$$
\tau\cdot\frac{y_k-y_{k-1}}{\mathrm{d}t} + y_k = u_k
$$

整理得：

$$
y_k = y_{k-1} + \underbrace{\frac{\mathrm{d}t}{\tau+\mathrm{d}t}}_{\alpha}\,(u_k - y_{k-1})
$$

或等价地：

$$
y_k = (1-\alpha)\,y_{k-1} + \alpha\,u_k,\qquad
\alpha=\frac{\mathrm{d}t}{\tau+\mathrm{d}t}\in(0,1)
$$

对应代码：

```python
self._alpha_f += (dt / (self.slope_alpha_lpf_sec + dt)) * (
    alpha_db - self._alpha_f
)
```

每一步只朝目标走「误差的一小截」——看起来像收敛，本质是**指数逼近**。对变 $\mathrm{d}t$ 友好、算量 $O(1)$，比滑动窗口更适合控制环。

$\tau\le 10^{-6}$ 或 $\mathrm{d}t\le 0$ 时实现退化为直通：$y\leftarrow u$。

## 例子：当前设计的 $3\tau\sim 5\tau$ {#settling}

一阶系统对阶跃输入，输出按 $1-e^{-t/\tau}$ 上升。工程上常记：

| 时间 | 相对终值约 | 含义 |
| --- | --- | --- |
| $1\tau$ | $63\%$ | 时间常数的定义点 |
| $3\tau$ | $95\%$ | 常说的「基本到位」 |
| $5\tau$ | $99\%$ | 近似稳态 |

对本车 $\tau=\texttt{slope\_alpha\_lpf\_sec}=0.5\,\mathrm{s}$：

| | 时间 |
| --- | --- |
| $3\tau$ | **1.5 s** |
| $5\tau$ | **2.5 s** |

含义：若上坡瞬间 $\alpha_{\mathrm{db}}$ 近似阶跃到真坡角，滤波后的 $\hat{\alpha}_f$（进而 $\theta_{\mathrm{eq}}$、$\tau_{\mathrm{ff}}$）大约 **1.5～2.5 s** 才跟到稳态。这与「补偿滞后约半秒量级、由快速 LQR 反馈临时兜底」一致——$\tau$ 本身是 0.5 s，全程到位要数个 $\tau$。

调参时把「$3\tau\sim 5\tau$」当成**可感知延迟预算**：

- 觉得上坡补偿太肉 → 略减小 $\tau$（别小到与平衡回路带宽粘在一起）  
- 觉得仍有慢抖/自激倾向 → 略增大 $\tau$，并确认死区够大  

## 和软启动的分工 {#vs-ramp}

三者叠在一起，但职责不同：

| 环节 | 参数 | 主要防什么 |
| --- | --- | --- |
| 死区 | `slope_alpha_deadband` | 平地噪声当假坡 |
| 一阶低通 | `slope_alpha_lpf_sec` | 高频入环、自激 |
| 软启动 | `slope_ff_ramp_sec`（默认 2.0 s） | 使能瞬间补偿从 0 渐入 |

低通的 $3\tau\sim 5\tau$ 刻画的是**信号跟踪**速度；`ramp` 刻画的是**增益渐入**。二者都会让上坡初期「力与姿态参考」来得慢，这段靠 LQR 反馈顶住。

## 以后怎么套用 {#template}

遇到「估计量有噪声 / 直通会自激 / 只关心慢变趋势」时，套三步：

1. 定输入 $u$、持久状态 $y$  
2. 定时间常数 $\tau$（秒），用 $3\tau\sim 5\tau$ 估跟稳时间是否可接受  
3. 每周期：$y \mathrel{+}= \dfrac{\mathrm{d}t}{\tau+\mathrm{d}t}\,(u-y)$；零点附近若需熄火，先对 $u$ 加死区  

一句话：**把「$\tau$ 秒跟上输入」的模拟惯性环节，按控制周期离散化**——这就是当前 $\hat{\alpha}$ 入环滤波的全部数学。
