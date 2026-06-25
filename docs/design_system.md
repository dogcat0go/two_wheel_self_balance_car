<!--
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-06-19 23:26:34
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-06-19 23:26:49
 * @FilePath: /two_wheel_self_balance_car/docs/design_system.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
```mermaid
flowchart TB
    subgraph APP["应用层 balance_car_control"]
        BC["balance_controller_node<br/>200 Hz 定时器"]
        EST["AttitudeEstimator"]
        PD["BalancePD + 外环"]
        MIX["WheelMixer + SafetyLimiter"]
        BC --> EST --> PD --> MIX
    end

    subgraph ROS2["ROS2 话题层"]
        T_IMU["/imu/data<br/>sensor_msgs/Imu"]
        T_JS["/joint_states<br/>sensor_msgs/JointState"]
        T_CMD["/wheel_effort_controller/commands<br/>std_msgs/Float64MultiArray"]
        T_CV["/cmd_vel<br/>geometry_msgs/Twist"]
        T_DBG["/balance_controller/debug"]
    end

    subgraph R2C["ros2_control 层 (balance_car_description)"]
        CM["controller_manager<br/>250 Hz"]
        JSB["joint_state_broadcaster"]
        WEC["wheel_effort_controller<br/>JointGroupEffortController"]
        IMB["imu_sensor_broadcaster<br/>（已 spawn，当前未用）"]
        HW["GazeboSystem 硬件接口<br/>left/right_wheel_joint"]
    end

    subgraph GZ["Gazebo 仿真"]
        PHY["物理引擎"]
        IMU_GZ["Gazebo IMU 传感器<br/>libgazebo_ros_imu_sensor"]
        WHEEL["左右轮关节力矩/速度"]
    end

    T_CV --> BC
    IMU_GZ --> T_IMU --> BC
    JSB --> T_JS --> BC
    MIX --> T_CMD --> WEC
    BC --> T_DBG

    WEC --> CM --> HW --> WHEEL --> PHY
    JSB --> CM
    HW --> PHY
    PHY --> HW
    PHY --> IMU_GZ
    PHY --> WHEEL

```