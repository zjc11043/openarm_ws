# OpenArm 双臂仿真与手眼标定项目

- 本项目实现了基于ROS 2和MuJoCo的OpenArm双臂机器人仿真，并完成了从视觉感知、手眼标定到抓取规划执行的全流程验证。
- 本md文件作大体介绍，更加详细请见[README_details](README_details.md)。
- 实验效果视频链接：[实验视频](https://www.bilibili.com/video/BV1GFrfBXE6n/?vd_source=c28f58e982478639c258972af4986800)

---

## 1. 依赖环境

### 1.1 基础系统
*   **Ubuntu**: 22.04
*   **ROS 2**: Humble 发行版
    *   已安装工具：`colcon`, `rosdep`, `rviz2`, `MoveIt 2`

### 1.2 仿真与驱动
*   **MuJoCo**: 3.4.0
*   **关键ROS 2包**:
    *   `mujoco_ros2` (Woolfrey/mujoco_ros2)
    *   `mujoco_ros2_control` (moveit/mujoco_ros2_control)
    *   `openarm_ros2` (含bringup, MoveIt配置)
    *   `openarm_mujoco` (MJCF场景文件)
    *   `openarm_description` (机器人URDF模型)
    *   `openarm_mujoco_viewer` (自定义MuJoCo Viewer)

### 1.3 编译工作区
在工作区根目录执行以下命令：
```bash
cd ~/openarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-ignore openarm_hardware
source install/setup.bash
```

---

## 2. 运行指南

### 2.1 启动基础仿真与规划环境 (MuJoCo + MoveIt)
此命令同时启动MuJoCo物理仿真、MoveIt运动规划和RViz可视化界面。
```bash
source ~/openarm_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch openarm_bringup openarm_bimanual_mujoco_moveit.launch.py
```

**操作**：
1.  在RViz的MotionPlanning面板中选择规划组（如`left_arm`）。
2.  设置目标位姿或关节角。
3.  点击**Plan and Execute**，机械臂将在RViz和MuJoCo窗口中同步运动。

### 2.2 启动全流程抓取验证（一键启动）
此命令将按顺序自动启动仿真、视觉、规划、抓取等所有模块，并执行完整的抓取流程。
```bash
source ~/openarm_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch openarm_grasp_planner full_integration.launch.py
```

### 2.3 （备选）分步启动与调试
如需调试，可依次执行以下命令：
```bash
# 1. 启动仿真核心
ros2 launch openarm_bringup openarm_bimanual_mujoco_moveit.launch.py
# 2. 启动虚拟相机接口
ros2 run openarm_vision virtual_camera
# 3. 启动视觉检测模块
ros2 run openarm_vision banana_detector
# 4. 启动抓取规划与夹爪控制模块
ros2 run openarm_grasp_planner grasp_planner
ros2 run openarm_grasp_planner gripper_controller
# 5. 触发抓取流程
ros2 run openarm_grasp_planner full_verification
```

---

## 3. 系统架构与数据流

系统各模块通过ROS 2话题、服务、动作和TF进行通信，构成以下数据流：
```
┌─────────────────┐
│  MuJoCo 仿真    │
│  + MoveIt       │
└────────┬────────┘
         │ (关节状态 /joint_states)
┌────────▼────────┐
│  虚拟相机模块    │──> 发布RGB与深度图
│ virtual_camera   │
└────────┬────────┘
         │
┌────────▼────────┐
│  视觉检测模块    │──> 发布香蕉的TF坐标 `banana_target`
│ banana_detector │
└────────┬────────┘
         │
┌────────▼────────┐
│  抓取规划模块    │<── 接收规划请求
│ grasp_planner   │──> 发送关节轨迹给控制器
└────────┬────────┘
         │
┌────────▼────────┐
│  夹爪控制模块    │<── 接收开合指令
│gripper_controller│──> 发送夹爪动作指令
└─────────────────┘
```

**核心模块说明**:
*   **`virtual_camera.py`**：桥接MuJoCo渲染引擎，发布仿真图像。
*   **`banana_detector.py`**：处理图像，检测香蕉并计算其在基座坐标系下的位姿。
*   **`grasp_planner.py`**：基于目标位姿，规划并执行抓取轨迹。
*   **`gripper_controller.py`**：提供控制夹爪开合的服务。
*   **`full_verification.py`**：协调各模块，自动化执行完整抓取任务。

---

## 4. 坐标系说明

### 4.1 主要坐标系
*   **`openarm_body_link0`**：机器人底座中心，作为全局参考坐标系。
*   **`openarm_left_link0`/`openarm_right_link0`**：左/右机械臂的基座坐标系。
*   **`d435_optical_frame`**：深度相机的光学中心坐标系。
*   **`banana_target`**：由视觉模块发布的、香蕉在`openarm_body_link0`坐标系下的目标位置。

### 4.2 坐标变换流程
1.  **感知定位**：`banana_detector`根据相机内参和相机-基座的固定变换（从`v10.urdf.xacro`读取），将像素坐标转换到`openarm_body_link0`下，并发布为`banana_target` TF帧。
2.  **抓取规划**：`grasp_planner`通过TF树，自动将`banana_target`的位姿转换到左臂基座坐标系`openarm_left_link0`下，用于运动规划。
3.  **执行**：规划出的关节轨迹通过`ros2_control`发送给仿真器执行。

---

## 5. 实验数据结果

### 5.1 物理参数与手眼标定
基于机器人描述文件`v10.urdf.xacro`中的真实几何参数进行标定：
*   **相机安装位置 (XYZ)**: `[0.20, 0.0, 0.80]` 米
*   **相机安装姿态 (RPY)**: `[3.14, 1.57, 0]` 弧度
由此构建的相机到基座的变换矩阵如下：

| 变换矩阵 | X轴 | Y轴 | Z轴 | 平移 |
| :--- | :---: | :---: | :---: | :---: |
| **行1** | 0.000 | 1.000 | 0.000 | 0.200 |
| **行2** | 1.000 | 0.000 | 0.000 | 0.000 |
| **行3** | 0.000 | 0.000 | -1.000 | 0.800 |
| **行4** | 0.000 | 0.000 | 0.000 | 1.000 |

### 5.2 定位精度
当香蕉位于桌面中心时，`banana_detector`实时解算出的位姿为：
*   **平移**: `[0.350, -0.001, 0.350]` (单位：米，相对于`openarm_body_link0`)
*   **旋转**: `[0.0, 0.0, 0.0, 1.0]` (四元数)

### 5.3 实验结论
1.  **物理闭环准确**：解算出的X方向坐标0.350米，严格符合相机高度、俯仰角与目标深度(0.35m)的几何关系，验证了基于硬件参数的标定准确性。
2.  **系统功能完整**：全流程系统能稳定完成“视觉检测 -> 坐标解算 -> 运动规划 -> 夹爪抓取”的自动化流程。
3.  **定位稳定**：通过滤波处理，视觉定位输出波动小于1mm，满足抓取精度要求。
