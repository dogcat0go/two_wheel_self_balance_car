# AGENTS.md

## Cursor Cloud specific instructions

This repo is a **ROS 2 Humble + Gazebo Classic** workspace (two-wheel self-balancing car).
The dev VM is **Ubuntu 24.04 (Noble)**, where ROS 2 Humble and **Gazebo Classic are not
available natively**. Therefore everything runs inside a **Docker container** based on
`osrf/ros:humble-desktop-full`. Standard build/run commands are in `README.md`; the notes
below are the non-obvious bits for this environment.

### One-time-per-session bring-up (the update script only installs Docker Engine)

The startup update script installs/configures Docker Engine only. Each session you must:

```bash
# 1. Start the Docker daemon (systemd is not available in the VM)
sudo dockerd > /tmp/dockerd.log 2>&1 &   # wait ~8s; verify: sudo docker info

# 2. Pull the base image if not already present (first session only; ~4 GB)
sudo docker pull osrf/ros:humble-desktop-full

# 3. (Re)create the container: host network + X11 + workspace mount
sudo docker rm -f balance_car 2>/dev/null
sudo docker run -d --name balance_car --privileged --network host \
  -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /workspace:/workspace -w /workspace \
  osrf/ros:humble-desktop-full sleep infinity

# 4. Install ROS deps INSIDE the container (base image lacks some of them).
#    rosdep covers most; effort_controllers is NOT pulled by rosdep but the
#    README default `controller_type:=effort` needs it, so install it explicitly.
sudo docker exec balance_car bash -lc "apt-get update -qq && rosdep update --rosdistro humble && \
  cd /workspace && rosdep install --from-paths src --ignore-src -r -y && \
  apt-get install -y ros-humble-effort-controllers"

# 5. Build (per README)
sudo docker exec balance_car bash -lc "source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install"
```

Run any ROS command as: `sudo docker exec balance_car bash -lc "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && <cmd>"`.

### Lint / test

`colcon test` runs `ament_flake8` / `ament_pep257` only (no unit tests exist). It currently
reports many **pre-existing** `pep257` failures (Chinese docstrings not ending with a period,
etc.) — these are code-style issues in the repo, not environment problems.

### Running the simulation — critical ordering caveat

A two-wheel car is unstable and falls within ~1 s if physics runs before the balance
controller is ready. Do **not** launch with `pause:=false`. Use the README's paused sequence
with a timed unpause:

1. `gazebo.launch.py pause:=true gui:=false controller_type:=effort`
2. wait ~8-10 s, then `balance_controller.launch.py enabled:=true use_sim_time:=false`
3. within the controller-spawner retry window (~30 s of gazebo launch), call
   `/unpause_physics`. Under `pause:=true` the `controller_manager` uses sim time and cannot
   activate controllers until you unpause, so unpause reasonably soon.

Verify balancing from **ground truth**, not the GUI (see next note):
`gz model -m two_wheel_balance_65mm -p` → last three numbers are roll/pitch/yaw; pitch ≈ 0
(and `z` ≈ 0.0315) means upright. The controller log also prints `pitch=...deg`.

### Gazebo GUI (gzclient) rendering is unreliable here — trust physics/data instead

The VM only has software OpenGL (llvmpipe). `gzclient` connects and pose messages flow, but
its **rendered view does not reliably reflect the true physics state** (it can show the robot
lying flat while `gz model -p` / the IMU confirm it is upright and balancing). For evidence,
prefer authoritative sources: `gz model -p`, `/imu/data`, and `/balance_controller/debug`
(e.g. plot `pitch`, `turn_tau`, wheel `cmd`). To make gzclient reachable at all, run
`DISPLAY=:1 xhost +` on the host once.
