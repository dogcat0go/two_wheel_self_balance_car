"""LQR 增益离线/在线共用：从 balance_lqr_tuning.yaml + URDF 计算 K。"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    from scipy.linalg import solve_discrete_are
    from scipy.signal import cont2discrete

    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

try:
    import yaml

    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False


@dataclass
class LqrDesignConfig:
    q_theta: float = 200.0
    q_theta_dot: float = 1.0
    q_x: float = 20.0
    q_x_dot: float = 1.0
    r_tau: float = 0.05
    dt: float = 0.005
    include_wheel_spin: bool = True
    gravity: float = 9.81
    xacro_path: Path | None = None
    runtime_yaml: Path | None = None
    pitch_linearization_rad: float = 0.0


def default_xacro_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        pkg = get_package_share_directory("balance_car_description")
        return Path(pkg) / "urdf" / "two_wheel_balance_65mm.urdf.xacro"
    except Exception:
        return (
            Path(__file__).resolve().parents[2]
            / "balance_car_description"
            / "urdf"
            / "two_wheel_balance_65mm.urdf.xacro"
        )


def default_tuning_yaml_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        pkg = get_package_share_directory("balance_car_control")
        return Path(pkg) / "config" / "balance_lqr_tuning.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / "balance_lqr_tuning.yaml"


def default_runtime_yaml_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        pkg = get_package_share_directory("balance_car_control")
        return Path(pkg) / "config" / "balance_lqr.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / "balance_lqr.yaml"


def _workspace_src_config(filename: str) -> Path | None:
    """install/share 运行时，尽量找到工作区 src 下同名 config 以便同步回写。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        share_pkg = Path(get_package_share_directory("balance_car_control"))
    except Exception:
        return None

    parts = share_pkg.parts
    if "install" not in parts:
        return None
    workspace = Path(*parts[: parts.index("install")])
    src_path = workspace / "src" / "balance_car_control" / "config" / filename
    return src_path if src_path.is_file() else None


def resolve_runtime_yaml_write_paths(
    design: LqrDesignConfig,
    tuning_path: Path | None = None,
) -> list[Path]:
    paths: list[Path] = []

    if design.runtime_yaml is not None:
        runtime = design.runtime_yaml.expanduser().resolve()
        if runtime.is_file():
            paths.append(runtime)

    default_runtime = default_runtime_yaml_path().resolve()
    if default_runtime.is_file() and default_runtime not in paths:
        paths.append(default_runtime)

    src_mirror = _workspace_src_config("balance_lqr.yaml")
    if src_mirror is not None:
        resolved = src_mirror.resolve()
        if resolved not in paths:
            paths.append(resolved)

    if not paths and tuning_path is not None:
        fallback = tuning_path.parent / "balance_lqr.yaml"
        if fallback.is_file():
            paths.append(fallback.resolve())

    return paths


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "on")
    return bool(value)


def _parse_scalar(raw: str) -> Any:
    text = raw.strip().strip('"').strip("'")
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    try:
        if "." in text or "e" in text.lower():
            return float(text)
        return int(text)
    except ValueError:
        return text


def _load_yaml_fallback(path: Path) -> dict[str, Any]:
    data: dict[str, Any] = {}
    in_design = False
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped == "lqr_design:":
            in_design = True
            continue
        if in_design and re.match(r"^[a-zA-Z_]", stripped) and not stripped.startswith(" "):
            if ":" in stripped and not stripped.endswith(":"):
                break
        if not in_design:
            continue
        match = re.match(r"^([a-zA-Z_][\w]*)\s*:\s*(.+)$", stripped)
        if match:
            data[match.group(1)] = _parse_scalar(match.group(2))
    return {"lqr_design": data}


def load_tuning_config(path: Path) -> LqrDesignConfig:
    if not path.is_file():
        raise FileNotFoundError(f"tuning yaml not found: {path}")

    if _HAS_YAML:
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    else:
        raw = _load_yaml_fallback(path)

    section = raw.get("lqr_design", raw)
    if not isinstance(section, dict):
        raise ValueError(f"invalid tuning yaml structure in {path}")

    control_rate = float(section.get("control_rate", 200.0))
    dt_explicit = float(section.get("dt", 0.0))
    dt = dt_explicit if dt_explicit > 0.0 else 1.0 / control_rate

    xacro_raw = section.get("xacro_path", "")
    xacro_path = Path(xacro_raw).expanduser() if str(xacro_raw).strip() else None

    runtime_raw = section.get("runtime_yaml", "balance_lqr.yaml")
    runtime_yaml = Path(runtime_raw)
    if not runtime_yaml.is_absolute():
        runtime_yaml = path.parent / runtime_yaml

    max_theta = float(section.get("max_theta_rad", 0.12))
    max_theta_dot = float(section.get("max_theta_dot", 3.0))
    max_x = float(section.get("max_x_m", 0.3))
    max_x_dot = float(section.get("max_x_dot_mps", 0.5))
    max_tau = float(section.get("max_tau", 0.12))

    return LqrDesignConfig(
        q_theta=float(section.get("q_scale_theta", 1.0)) / max_theta**2,
        q_theta_dot=float(section.get("q_scale_theta_dot", 1.0)) / max_theta_dot**2,
        q_x=float(section.get("q_scale_x", 1.0)) / max_x**2,
        q_x_dot=float(section.get("q_scale_x_dot", 1.0)) / max_x_dot**2,
        r_tau=float(section.get("r_scale", 1.0)) / max_tau**2,
        dt=dt,
        include_wheel_spin=_coerce_bool(section.get("include_wheel_spin", True)),
        gravity=float(section.get("gravity", 9.81)),
        xacro_path=xacro_path,
        runtime_yaml=runtime_yaml,
        pitch_linearization_rad=float(section.get("pitch_linearization_rad", 0.0)),
    )


def load_xacro_properties(xacro_path: Path) -> dict[str, float]:
    text = xacro_path.read_text(encoding="utf-8")
    props: dict[str, float] = {}
    simple = re.compile(
        r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"'
    )
    for match in simple.finditer(text):
        name, raw = match.group(1), match.group(2)
        try:
            props[name] = float(raw)
        except ValueError:
            pass
    return props


def composite_inertia_about_axle(props: dict[str, float]) -> tuple[float, float, float]:
    m_body = props["body_mass"]
    z_body = props["body_com_z"]
    l_body = props["body_length"]
    h_body = props["body_height"]
    i_body_com = m_body / 12.0 * (l_body**2 + h_body**2)
    i_body_axle = i_body_com + m_body * z_body**2

    parts = [
        (props["battery_mass"], props["battery_z"]),
        (props["board_mass"], props["board_z"]),
        (props["imu_mass"], props["imu_z"]),
        (props["motor_mass"], 0.0),
        (props["motor_mass"], 0.0),
        (props["wheel_mass"], 0.0),
        (props["wheel_mass"], 0.0),
    ]
    m_extra = sum(m for m, _ in parts)
    m_total = m_body + m_extra
    l_com = (m_body * z_body + sum(m * z for m, z in parts)) / m_total
    i_axle = i_body_axle + sum(m * z**2 for m, z in parts)
    return m_total, l_com, i_axle


def wheel_spin_inertia_total(props: dict[str, float]) -> float:
    m_w = props["wheel_mass"]
    r = props["wheel_radius"]
    return 2.0 * (0.5 * m_w * r**2)


def build_continuous_ab_from_props(
    props: dict[str, float],
    include_wheel_spin: bool,
    g: float = 9.81,
    pitch_linearization_rad: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, dict]:
    m_total, l_com, i_axle = composite_inertia_about_axle(props)
    r_wheel = props["wheel_radius"]
    j_spin = wheel_spin_inertia_total(props) if include_wheel_spin else 0.0

    a11 = m_total * l_com**2 + i_axle
    a12 = m_total * l_com
    a21 = a12
    a22 = m_total + j_spin / (r_wheel**2)
    det = a11 * a22 - a12 * a21

    grav_stiff = m_total * g * l_com * math.cos(pitch_linearization_rad)
    c11 = a22 * grav_stiff / det
    c12 = (-a22 - a12 / r_wheel) / det
    c21 = -a21 * grav_stiff / det
    c22 = (a21 + a11 / r_wheel) / det

    a_mat = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [c11, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [c21, 0.0, 0.0, 0.0],
        ]
    )
    b_mat = np.array([[0.0], [c12], [0.0], [c22]])

    meta = {
        "m_total": m_total,
        "l_com": l_com,
        "i_axle": i_axle,
        "r_wheel": r_wheel,
        "j_spin": j_spin,
        "m_eff_x": a22,
        "det": det,
        "g": g,
        "pitch_linearization_rad": pitch_linearization_rad,
        "grav_stiff": grav_stiff,
    }
    return a_mat, b_mat, meta


def discretize_ab(a_c: np.ndarray, b_c: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    if _HAS_SCIPY:
        a_d, b_d, _, _, _ = cont2discrete(
            (a_c, b_c, np.zeros((1, 4)), np.zeros((1, 1))), dt, method="zoh"
        )
        return a_d, b_d
    n = a_c.shape[0]
    return np.eye(n) + a_c * dt, b_c * dt


def solve_dare(a_d: np.ndarray, b_d: np.ndarray, q_mat: np.ndarray, r_mat: np.ndarray) -> np.ndarray:
    if _HAS_SCIPY:
        return solve_discrete_are(a_d, b_d, q_mat, r_mat)
    p_mat = q_mat.copy()
    for _ in range(500):
        s_mat = r_mat + b_d.T @ p_mat @ b_d
        k_gain = np.linalg.solve(s_mat, b_d.T @ p_mat @ a_d)
        p_next = q_mat + a_d.T @ p_mat @ (a_d - b_d @ k_gain)
        if np.max(np.abs(p_next - p_mat)) < 1e-12:
            break
        p_mat = p_next
    return p_mat


def compute_lqr_k(
    a_d: np.ndarray,
    b_d: np.ndarray,
    q_diag: np.ndarray,
    r_scalar: float,
) -> tuple[np.ndarray, np.ndarray]:
    q_mat = np.diag(q_diag)
    r_mat = np.array([[r_scalar]])
    p_mat = solve_dare(a_d, b_d, q_mat, r_mat)
    k_mat = np.linalg.solve(r_mat + b_d.T @ p_mat @ b_d, b_d.T @ p_mat @ a_d)
    return k_mat, p_mat


def compute_k_row_from_tuning(
    tuning_path: Path | None = None,
    xacro_path: Path | None = None,
) -> tuple[np.ndarray, LqrDesignConfig, Path]:
    """从 tuning yaml 计算 1×4 增益行。返回 (k_row, design, xacro_used)。"""
    tuning = tuning_path if tuning_path is not None else default_tuning_yaml_path()
    design = load_tuning_config(tuning)

    xacro = design.xacro_path or xacro_path or default_xacro_path()
    if not xacro.is_file():
        raise FileNotFoundError(f"URDF xacro not found: {xacro}")

    props = load_xacro_properties(xacro)
    required = [
        "wheel_radius", "wheel_mass", "body_mass", "body_com_z",
        "body_length", "body_height", "battery_mass", "battery_z",
        "board_mass", "board_z", "imu_mass", "imu_z", "motor_mass",
    ]
    missing = [k for k in required if k not in props]
    if missing:
        raise ValueError(f"Missing xacro properties: {missing}")

    a_c, b_c, _meta = build_continuous_ab_from_props(
        props,
        design.include_wheel_spin,
        design.gravity,
        design.pitch_linearization_rad,
    )
    a_d, b_d = discretize_ab(a_c, b_c, design.dt)
    q_diag = np.array([design.q_theta, design.q_theta_dot, design.q_x, design.q_x_dot])
    k_mat, _p_mat = compute_lqr_k(a_d, b_d, q_diag, design.r_tau)
    return k_mat[0], design, xacro


def write_yaml_gains(yaml_path: Path, k_row: np.ndarray) -> None:
    k = np.asarray(k_row, dtype=float).reshape(-1)
    if k.size < 4:
        raise ValueError(f"expected 4 gain values, got shape {np.asarray(k_row).shape}")

    text = yaml_path.read_text(encoding="utf-8")
    replacements = {
        "k_theta": k[0],
        "k_theta_dot": k[1],
        "k_x": k[2],
        "k_x_dot": k[3],
    }
    for key, val in replacements.items():
        text = re.sub(
            rf"^(\s*{key}:\s*)[-\d.eE+]+",
            rf"\g<1>{val:.8f}",
            text,
            count=1,
            flags=re.MULTILINE,
        )
    yaml_path.write_text(text, encoding="utf-8")


def write_yaml_gains_to_runtime_files(
    k_row: np.ndarray,
    design: LqrDesignConfig,
    tuning_path: Path | None = None,
) -> list[Path]:
    """将 K 写回 runtime yaml；返回实际写入的文件列表。"""
    written: list[Path] = []
    for yaml_path in resolve_runtime_yaml_write_paths(design, tuning_path):
        write_yaml_gains(yaml_path, k_row)
        written.append(yaml_path)
    if not written:
        raise FileNotFoundError("no balance_lqr.yaml found to write K gains")
    return written
