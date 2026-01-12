import os

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """Unified launch: MuJoCo + MoveIt + Vision + Grasp Planning + Verification.
    
    启动顺序时间线：
    - 0秒: MuJoCo + MoveIt 基础环境启动（包含 MuJoCo 仿真、控制器、MoveIt、RViz）
      - 2秒: joint_state_broadcaster 启动
      - 3秒: arm controllers 启动
      - 3.5秒: gripper controllers 启动（关键！）
    - 5秒: 虚拟相机启动（提供图像数据）
    - 8秒: 视觉检测模块和抓取规划模块启动
    - 12秒: 夹爪控制器和全流程验证系统启动（确保控制器已完全就绪）
    """

    use_sim_time = {"use_sim_time": True}

    # === 0. 启动 MuJoCo + MoveIt 基础环境 ===
    # 包含 MuJoCo 仿真、控制器、MoveIt 和 RViz
    # 注意：openarm_bimanual_mujoco_moveit.launch.py 内部已设置 use_sim_time=True
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openarm_bringup"),
                "launch",
                "openarm_bimanual_mujoco_moveit.launch.py"
            )
        ),
        # 注意：目标 launch 文件不接受 launch 参数，但内部已正确设置 use_sim_time
    )

    # === 1. Virtual camera module (provides image data for vision) ===
    virtual_camera_node = Node(
        package="openarm_vision",
        executable="virtual_camera",
        name="virtual_camera",
        output="screen",
        parameters=[use_sim_time],
    )

    # === 2. Vision perception module ===
    vision_node = Node(
        package="openarm_vision",
        executable="banana_detector",
        name="banana_detector",
        output="screen",
        parameters=[use_sim_time],
    )

    # === 3. Grasp planning module ===
    grasp_planner_node = Node(
        package="openarm_grasp_planner",
        executable="grasp_planner",
        name="grasp_planner",
        output="screen",
        parameters=[use_sim_time],
    )

    # === 4. Gripper control module ===
    gripper_controller_node = Node(
        package="openarm_grasp_planner",
        executable="gripper_controller",
        name="gripper_controller",
        output="screen",
        parameters=[use_sim_time],
    )

    # === 5. Full verification system ===
    full_verification_node = Node(
        package="openarm_grasp_planner",
        executable="full_verification",
        name="full_verification",
        output="screen",
        parameters=[use_sim_time],
    )

    # === 6. Execution order ===
    # 启动顺序说明：
    # - bringup_launch 立即启动，内部会在 3.5 秒后启动 gripper controllers
    # - 5 秒后启动虚拟相机（提供图像数据）
    # - 8 秒后启动视觉检测和抓取规划模块（此时基础环境和相机已稳定）
    # - 12 秒后启动夹爪控制器和全流程验证系统（确保 gripper controllers 已就绪）
    return LaunchDescription(
        [
            # 首先启动 MuJoCo + MoveIt 基础环境
            bringup_launch,
            
            # 延迟启动虚拟相机，确保基础环境已启动
            TimerAction(
                period=5.0,  # 给 MuJoCo 基础环境时间启动
                actions=[virtual_camera_node],
            ),
            
            # 延迟启动视觉模块和抓取规划模块，确保基础环境和相机已完全启动
            # 注意：move_group 需要更多时间启动，所以延迟到 15 秒
            TimerAction(
                period=15.0,  # 给 move_group 更多时间启动（move_group 通常在 5-10 秒后完全就绪）
                actions=[vision_node, grasp_planner_node],
            ),
            
            # 再延迟启动夹爪控制器和全流程验证系统，确保控制器已就绪
            TimerAction(
                period=12.0,  # 确保 gripper controllers（3.5秒启动）已完全就绪
                actions=[gripper_controller_node, full_verification_node],
            ),
        ]
    )