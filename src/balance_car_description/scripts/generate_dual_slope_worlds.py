#!/usr/bin/env python3
"""根据顶部配置重新生成 gentle_slope.world / balance_pid_test.world。

几何关系（改 angle_deg 时会联动）:
  - dz = ramp_len * tan(angle)  → 岭台高度、下坡起点 z
  - 旋转薄盒的实际 x 端点 ≠ uphill_start_x + ramp_len（与角度有关）
  - 岭台水平长度 = 上坡顶 x ～ 下坡顶 x，由两端斜坡几何 + summit_len 共同决定
  - 前/后平地端点接上坡起点 x、下坡落点 x

用法:
  python3 scripts/generate_dual_slope_worlds.py [--gentle-angle-deg 6]
  colcon build --packages-select balance_car_description && source install/setup.bash
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

# ── 可调参数（不要单独写死 downhill_start_x，下坡起点由上坡+岭台长度推算）────
GENTLE_CONFIG = dict(
    angle_deg=5.0,
    ramp_len=2.0,
    summit_len=1.0,          # 岭台最小水平长度 (m)，车体站立区
    flat_approach_len=3.0,
    flat_exit_len=3.0,
    uphill_start_x=-3.0,
    track_width=4.0,
    plate_thickness=0.08,
    overlap=0.01,
)

PID_CONFIG = dict(
    angle_deg=8.0,
    ramp_len=2.0,
    summit_len=1.0,
    flat_approach_len=3.0,
    flat_exit_len=3.0,
    uphill_start_x=-3.0,
    track_width=4.0,
    plate_thickness=0.08,
    overlap=0.01,
)
# ─────────────────────────────────────────────────────────────────


@dataclass
class RampSegment:
    cx: float
    cz: float
    pitch: float
    size_x: float
    x_low: float
    x_high: float


@dataclass
class TrackLayout:
    up: RampSegment
    dn: RampSegment
    sum_cx: float
    sum_cz: float
    sum_sx: float
    sum_x0: float
    sum_x1: float
    flat_a_cx: float
    flat_a_sx: float
    flat_e_cx: float
    flat_e_sx: float
    z_top: float


@dataclass
class TrackGeom:
    angle_deg: float
    ramp_len: float
    summit_len: float
    flat_approach_len: float
    flat_exit_len: float
    uphill_start_x: float
    track_width: float
    plate_thickness: float
    overlap: float

    @property
    def angle_rad(self) -> float:
        return math.radians(self.angle_deg)

    @property
    def ramp_dz(self) -> float:
        return self.ramp_len * math.tan(self.angle_rad)

    @property
    def pitch_up(self) -> float:
        return -self.angle_rad

    @property
    def pitch_down(self) -> float:
        return self.angle_rad

    def ramp_segment(self, x_start: float, z_start: float, uphill: bool) -> RampSegment:
        pitch = self.pitch_up if uphill else self.pitch_down
        z_end = z_start + self.ramp_dz if uphill else z_start - self.ramp_dz
        size_x = self.ramp_len + 2.0 * self.overlap
        hx = size_x / 2.0
        hz = self.plate_thickness / 2.0
        cz = (z_start + z_end) / 2.0 - hz * math.cos(pitch)
        cx = x_start + hx * math.cos(pitch) - hz * math.sin(pitch)
        x_low = cx + (-hx * math.cos(pitch) + hz * math.sin(pitch))
        x_high = cx + (hx * math.cos(pitch) + hz * math.sin(pitch))
        if x_low > x_high:
            x_low, x_high = x_high, x_low
        return RampSegment(cx, cz, pitch, size_x, x_low, x_high)

    def ramp_fill_z(self, cz: float) -> float:
        return cz - self.plate_thickness / 2.0 - 0.07

    def flat_box(self, x_end: float, length: float) -> tuple[float, float]:
        cx = x_end - length / 2.0 - self.overlap / 2.0
        size_x = length + self.overlap
        return cx, size_x

    def compute_layout(self) -> TrackLayout:
        """按斜坡真实端点串联：上坡 → 岭台 → 下坡 → 平地。"""
        z_top = self.ramp_dz
        up = self.ramp_segment(self.uphill_start_x, 0.0, uphill=True)

        # 岭台：从上坡顶 x 起，至少 summit_len；下坡高侧对齐岭台末端
        sum_x0 = up.x_high - self.overlap / 2.0
        downhill_high_x = sum_x0 + self.summit_len + self.overlap / 2.0
        dn = self.ramp_segment(downhill_high_x, z_top, uphill=False)

        # 岭台实际长度覆盖 [上坡顶, 下坡顶]，随角度自动伸缩
        sum_x1 = dn.x_low + self.overlap / 2.0
        sum_len = sum_x1 - sum_x0
        if sum_len < 0.05:
            raise ValueError(
                f"summit_len={self.summit_len} 过小：上坡顶 x={up.x_high:.3f}，"
                f"下坡顶 x={dn.x_low:.3f}，请增大 summit_len 或 overlap"
            )

        sum_cx = (sum_x0 + sum_x1) / 2.0
        sum_sx = sum_len + self.overlap
        sum_cz = z_top - self.plate_thickness / 2.0

        flat_a_cx, flat_a_sx = self.flat_box(up.x_low, self.flat_approach_len)
        flat_e_cx, flat_e_sx = self.flat_box(dn.x_high + self.flat_exit_len, self.flat_exit_len)

        return TrackLayout(
            up=up,
            dn=dn,
            sum_cx=sum_cx,
            sum_cz=sum_cz,
            sum_sx=sum_sx,
            sum_x0=sum_x0,
            sum_x1=sum_x1,
            flat_a_cx=flat_a_cx,
            flat_a_sx=flat_a_sx,
            flat_e_cx=flat_e_cx,
            flat_e_sx=flat_e_sx,
            z_top=z_top,
        )


def _surface_block() -> str:
    return """          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <contact><ode><kp>1e6</kp><kd>100</kd><min_depth>0.001</min_depth></ode></contact>
          </surface>"""


def _fmt_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> str:
    return f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.6f} {yaw:.4f}"


def build_world(world_name: str, model_name: str, cfg: TrackGeom, comment_extra: str) -> str:
    lay = cfg.compute_layout()
    t = cfg.plate_thickness
    w = cfg.track_width
    dz = lay.z_top
    up, dn = lay.up, lay.dn
    flat_z = -t / 2.0
    spawn_summit_z = dz + 0.034
    spawn_summit_x = (lay.sum_x0 + lay.sum_x1) / 2.0

    return f"""<?xml version="1.0" ?>
<!-- AUTO-GENERATED by scripts/generate_dual_slope_worlds.py -->
<!--
  {comment_extra}

  【角度联动】angle={cfg.angle_deg}° → dz={dz:.4f}m
    岭台顶 z={dz:.4f}  实际岭台水平长度={lay.sum_sx - cfg.overlap:.4f}m (配置 summit_len>={cfg.summit_len})

  【x 向衔接】
    平地 {lay.flat_a_cx - lay.flat_a_sx/2:.2f} .. {up.x_low:.2f} (z=0)
    上坡 {up.x_low:.2f} .. {up.x_high:.2f}  (0 → {dz:.3f})
    岭台 {lay.sum_x0:.2f} .. {lay.sum_x1:.2f}  (z={dz:.3f})
    下坡 {dn.x_low:.2f} .. {dn.x_high:.2f}  ({dz:.3f} → 0)
    平地 {dn.x_high:.2f} .. {dn.x_high + cfg.flat_exit_len:.2f}

  spawn: 上坡前 x≈{lay.flat_a_cx:.1f} z=0.034 | 岭台 x≈{spawn_summit_x:.1f} z={spawn_summit_z:.3f}
-->
<sdf version="1.6">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <include>
      <uri>model://sun</uri>
    </include>

    <model name="{model_name}">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="track_link">
        <inertial>
          <mass>500</mass>
          <inertia>
            <ixx>500</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>500</iyy><iyz>0</iyz><izz>500</izz>
          </inertia>
        </inertial>

        <collision name="flat_approach_col">
          <pose>{_fmt_pose(lay.flat_a_cx, 0, flat_z, 0, 0, 0)}</pose>
          <geometry><box><size>{lay.flat_a_sx:.4f} {w:.1f} {t:.2f}</size></box></geometry>
{_surface_block()}
        </collision>
        <visual name="flat_approach_vis">
          <pose>{_fmt_pose(lay.flat_a_cx, 0, flat_z, 0, 0, 0)}</pose>
          <geometry><box><size>{lay.flat_a_sx:.4f} {w:.1f} {t:.2f}</size></box></geometry>
          <material><ambient>0.32 0.35 0.38 1</ambient><diffuse>0.42 0.45 0.48 1</diffuse></material>
        </visual>

        <collision name="ramp_up_col">
          <pose>{_fmt_pose(up.cx, 0, up.cz, 0, up.pitch, 0)}</pose>
          <geometry><box><size>{up.size_x:.4f} {w:.1f} {t:.2f}</size></box></geometry>
{_surface_block()}
        </collision>
        <visual name="ramp_up_surface_vis">
          <pose>{_fmt_pose(up.cx, 0, up.cz, 0, up.pitch, 0)}</pose>
          <geometry><box><size>{up.size_x:.4f} {w:.1f} {t:.2f}</size></box></geometry>
          <material><ambient>0.25 0.55 0.35 1</ambient><diffuse>0.30 0.65 0.40 1</diffuse></material>
        </visual>
        <visual name="ramp_up_fill_vis">
          <pose>{_fmt_pose(up.cx, 0, cfg.ramp_fill_z(up.cz), 0, up.pitch, 0)}</pose>
          <geometry><box><size>{up.size_x:.4f} {w:.1f} 0.22</size></box></geometry>
          <material><ambient>0.22 0.48 0.30 1</ambient><diffuse>0.28 0.55 0.35 1</diffuse></material>
        </visual>

        <collision name="summit_col">
          <pose>{_fmt_pose(lay.sum_cx, 0, lay.sum_cz, 0, 0, 0)}</pose>
          <geometry><box><size>{lay.sum_sx:.4f} {w:.1f} {t:.2f}</size></box></geometry>
{_surface_block()}
        </collision>
        <visual name="summit_vis">
          <pose>{_fmt_pose(lay.sum_cx, 0, lay.sum_cz, 0, 0, 0)}</pose>
          <geometry><box><size>{lay.sum_sx:.4f} {w:.1f} {t:.2f}</size></box></geometry>
          <material><ambient>0.38 0.40 0.44 1</ambient><diffuse>0.48 0.50 0.54 1</diffuse></material>
        </visual>

        <collision name="ramp_down_col">
          <pose>{_fmt_pose(dn.cx, 0, dn.cz, 0, dn.pitch, 0)}</pose>
          <geometry><box><size>{dn.size_x:.4f} {w:.1f} {t:.2f}</size></box></geometry>
{_surface_block()}
        </collision>
        <visual name="ramp_down_surface_vis">
          <pose>{_fmt_pose(dn.cx, 0, dn.cz, 0, dn.pitch, 0)}</pose>
          <geometry><box><size>{dn.size_x:.4f} {w:.1f} {t:.2f}</size></box></geometry>
          <material><ambient>0.55 0.38 0.22 1</ambient><diffuse>0.65 0.45 0.28 1</diffuse></material>
        </visual>
        <visual name="ramp_down_fill_vis">
          <pose>{_fmt_pose(dn.cx, 0, cfg.ramp_fill_z(dn.cz), 0, dn.pitch, 0)}</pose>
          <geometry><box><size>{dn.size_x:.4f} {w:.1f} 0.22</size></box></geometry>
          <material><ambient>0.48 0.32 0.18 1</ambient><diffuse>0.55 0.38 0.22 1</diffuse></material>
        </visual>

        <collision name="flat_exit_col">
          <pose>{_fmt_pose(lay.flat_e_cx, 0, flat_z, 0, 0, 0)}</pose>
          <geometry><box><size>{lay.flat_e_sx:.4f} {w:.1f} {t:.2f}</size></box></geometry>
{_surface_block()}
        </collision>
        <visual name="flat_exit_vis">
          <pose>{_fmt_pose(lay.flat_e_cx, 0, flat_z, 0, 0, 0)}</pose>
          <geometry><box><size>{lay.flat_e_sx:.4f} {w:.1f} {t:.2f}</size></box></geometry>
          <material><ambient>0.32 0.35 0.38 1</ambient><diffuse>0.42 0.45 0.48 1</diffuse></material>
        </visual>

        <visual name="mark_uphill_start">
          <pose>{_fmt_pose(up.x_low, 0, 0.002, 0, 0, 0)}</pose>
          <geometry><box><size>0.05 {w + 0.2:.1f} 0.004</size></box></geometry>
          <material><ambient>0.2 0.85 0.35 1</ambient><diffuse>0.25 0.95 0.40 1</diffuse></material>
        </visual>
        <visual name="mark_summit">
          <pose>{_fmt_pose(lay.sum_x0, 0, dz + 0.004, 0, 0, 0)}</pose>
          <geometry><box><size>0.05 {w + 0.2:.1f} 0.004</size></box></geometry>
          <material><ambient>1.0 0.85 0.1 1</ambient><diffuse>1.0 0.95 0.15 1</diffuse></material>
        </visual>
        <visual name="mark_downhill_start">
          <pose>{_fmt_pose(dn.x_low, 0, dz + 0.004, 0, 0, 0)}</pose>
          <geometry><box><size>0.05 {w + 0.2:.1f} 0.004</size></box></geometry>
          <material><ambient>0.15 0.45 0.95 1</ambient><diffuse>0.20 0.55 1.0 1</diffuse></material>
        </visual>
        <visual name="mark_downhill_end">
          <pose>{_fmt_pose(dn.x_high, 0, 0.002, 0, 0, 0)}</pose>
          <geometry><box><size>0.05 {w + 0.2:.1f} 0.004</size></box></geometry>
          <material><ambient>0.95 0.2 0.2 1</ambient><diffuse>1.0 0.25 0.25 1</diffuse></material>
        </visual>
      </link>
    </model>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-2.5 -3.5 1.4 0 0.3 1.2</pose>
      </camera>
    </gui>
  </world>
</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gentle-angle-deg", type=float, default=None)
    parser.add_argument("--pid-angle-deg", type=float, default=None)
    args = parser.parse_args()

    pkg = Path(__file__).resolve().parents[1]
    worlds = pkg / "worlds"

    gentle = GENTLE_CONFIG.copy()
    pid = PID_CONFIG.copy()
    if args.gentle_angle_deg is not None:
        gentle["angle_deg"] = args.gentle_angle_deg
    if args.pid_angle_deg is not None:
        pid["angle_deg"] = args.pid_angle_deg

    for name, cfg_dict, world_name, model_name, title in (
        ("gentle", gentle, "gentle_slope", "gentle_slope_track", "gentle_slope — 双坡缓坡"),
        ("pid", pid, "balance_pid_test", "pid_test_track", "balance_pid_test — 双坡 PID"),
    ):
        cfg = TrackGeom(**cfg_dict)
        lay = cfg.compute_layout()
        (worlds / f"{world_name}.world").write_text(
            build_world(world_name, model_name, cfg, title),
            encoding="utf-8",
        )
        print(
            f"[{name}] angle={cfg.angle_deg}° dz={lay.z_top:.4f}m | "
            f"上坡顶 x={lay.up.x_high:.3f} 岭台长={lay.sum_sx - cfg.overlap:.3f}m "
            f"下坡顶 x={lay.dn.x_low:.3f} 下坡底 x={lay.dn.x_high:.3f}"
        )


if __name__ == "__main__":
    main()
