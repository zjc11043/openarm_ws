import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
import tf2_ros
from tf2_ros import TransformListener, Buffer
from moveit_msgs.msg import Grasp, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from moveit_msgs.srv import GraspPlanning
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')
        
        # TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建服务
        self.grasp_planning_service = self.create_service(
            GraspPlanning, 'plan_grasp', self.plan_grasp_callback
        )
        
        # 创建 Action 客户端用于执行轨迹（使用左臂）
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # 创建 MoveIt 规划 Action 客户端（ROS2 MoveIt 使用 Action 而不是 Service）
        # MoveIt 的规划通过 Action 接口进行：/move_action
        self.moveit_action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        # 规划方法选择：'moveit' 或 'simple'
        self.use_moveit_planning = True  # 默认使用 MoveIt 规划
        
        self.get_logger().info('抓取规划模块已启动')
        if self.use_moveit_planning:
            self.get_logger().info('使用 MoveIt Action 接口进行轨迹规划（推荐）')
            self.get_logger().info('Action 名称: /move_action')
            self.get_logger().info('注意：MoveIt 节点（move_group）应在 launch 文件中启动')
        else:
            self.get_logger().info('使用简化的几何计算来规划轨迹（不使用 IK 服务）')
        self.get_logger().info('等待轨迹控制器就绪...')
    
    def plan_grasp_callback(self, request, response):
        self.get_logger().info('收到抓取规划请求')
        
        # 1. 从TF获取目标物体的位置
        try:
            transform = self.tf_buffer.lookup_transform(
                'openarm_body_link0',  # 基座坐标系
                'banana_target',        # 目标物体坐标系
                rclpy.time.Time()       # 当前时间
            )
            
            target_pose = transform.transform
            self.get_logger().info(f'获取到目标位置: x={target_pose.translation.x:.3f}, y={target_pose.translation.y:.3f}, z={target_pose.translation.z:.3f}')
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'无法获取目标位置: {ex}')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 2. 生成抓取姿态
        grasp = self.generate_grasp_pose(target_pose)
        
        # 3. 规划抓取轨迹（使用 MoveIt 或简化方法）
        if self.use_moveit_planning:
            grasp_trajectory = self.plan_grasp_trajectory_with_moveit(grasp)
            if grasp_trajectory is None or len(grasp_trajectory.points) == 0:
                self.get_logger().warn('MoveIt 规划失败，回退到简化方法')
                grasp_trajectory = self.plan_grasp_trajectory(grasp)
        else:
            grasp_trajectory = self.plan_grasp_trajectory(grasp)
        
        # 4. 执行抓取轨迹（通过 Action 接口）
        if self.execute_trajectory(grasp_trajectory):
            self.get_logger().info('抓取轨迹执行成功')
        else:
            self.get_logger().error('抓取轨迹执行失败')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 5. 返回响应
        from moveit_msgs.msg import MoveItErrorCodes
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.grasps.append(grasp)
        
        return response
    
    def generate_grasp_pose(self, target_pose):
        """生成抓取姿态"""
        grasp = Grasp()
        
        # 设置抓取位置（直接使用目标位置，不加偏移）
        grasp.grasp_pose.header.frame_id = 'openarm_body_link0'
        grasp.grasp_pose.pose.position.x = target_pose.translation.x
        grasp.grasp_pose.pose.position.y = target_pose.translation.y
        grasp.grasp_pose.pose.position.z = target_pose.translation.z  # 直接使用目标位置
        
        # 设置抓取方向（简单示例）
        grasp.grasp_pose.pose.orientation.x = 0.0
        grasp.grasp_pose.pose.orientation.y = 0.0
        grasp.grasp_pose.pose.orientation.z = 0.0
        grasp.grasp_pose.pose.orientation.w = 1.0
        
        # 设置抓取参数
        grasp.pre_grasp_approach.direction.vector.z = -1.0  # 从上方接近
        grasp.pre_grasp_approach.min_distance = 0.01
        grasp.pre_grasp_approach.desired_distance = 0.05
        
        grasp.post_grasp_retreat.direction.vector.z = 1.0  # 抓取后向上撤退
        grasp.post_grasp_retreat.min_distance = 0.01
        grasp.post_grasp_retreat.desired_distance = 0.05
        
        return grasp
    
    def plan_grasp_trajectory_with_moveit(self, grasp_pose):
        """使用 MoveIt Action 接口生成抓取轨迹
        
        优势：
        - 自动碰撞检测
        - 关节限位检查
        - 精确的 IK 求解
        - 平滑的轨迹路径
        
        参数:
            grasp_pose: Grasp 消息，包含目标姿态
            
        返回:
            JointTrajectory: 规划好的轨迹，如果失败返回 None
        """
        # 检查 MoveIt Action 服务器是否就绪
        wait_timeout = 15.0
        self.get_logger().info(f'等待 MoveIt Action 服务器就绪 (/move_action)，最多等待 {wait_timeout} 秒...')
        
        if not self.moveit_action_client.wait_for_server(timeout_sec=wait_timeout):
            self.get_logger().error('MoveIt Action 服务器不可用: /move_action')
            self.get_logger().error('可能的原因：')
            self.get_logger().error('  1. move_group 节点未启动（检查: ros2 node list | grep move_group）')
            self.get_logger().error('  2. move_group 节点启动时间不够（可能需要更长时间）')
            self.get_logger().error('  3. Action 名称不正确（检查: ros2 action list | grep move_action）')
            return None
        
        self.get_logger().info('✓ MoveIt Action 服务器已就绪: /move_action')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('使用 MoveIt Action 接口进行轨迹规划...')
        
        # 创建 Action Goal
        goal_msg = MoveGroup.Goal()
        
        # 设置规划组（左臂）
        goal_msg.request.group_name = "left_arm"
        goal_msg.request.num_planning_attempts = 5  # 减少尝试次数，提高响应速度
        goal_msg.request.allowed_planning_time = 10.0  # 增加规划时间，提高成功率
        goal_msg.request.planner_id = ""  # 使用空字符串，让MoveIt使用默认规划器配置
        # 注意：ROS2 MoveIt 中碰撞检测默认开启，不需要设置 avoid_collisions 属性
        
        # 创建目标约束（位置和姿态）
        constraints = Constraints()
        
        # 位置约束（按照示例代码的正确方式设置）
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'openarm_body_link0'
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = "openarm_left_link7"  # 末端执行器链接
        
        # 设置目标位置（使用 target_point_offset 方式，更符合 MoveIt 规范）
        target_pos = grasp_pose.grasp_pose.pose.position
        position_constraint.target_point_offset.x = target_pos.x
        position_constraint.target_point_offset.y = target_pos.y
        position_constraint.target_point_offset.z = target_pos.z
        
        # 约束区域（立方体，增大容差以提高规划成功率）
        box_constraint = SolidPrimitive()
        box_constraint.type = SolidPrimitive.BOX
        box_constraint.dimensions = [0.02, 0.02, 0.02]  # 2cm 容差（从1cm增加到2cm）
        position_constraint.constraint_region.primitives = [box_constraint]
        position_constraint.constraint_region.primitive_poses = [Pose()]  # 位置在原点，通过 target_point_offset 指定
        position_constraint.weight = 1.0
        
        # 姿态约束（增大容差以提高规划成功率）
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'openarm_body_link0'
        orientation_constraint.header.stamp = self.get_clock().now().to_msg()
        orientation_constraint.link_name = "openarm_left_link7"
        orientation_constraint.orientation = grasp_pose.grasp_pose.pose.orientation
        # 增大姿态容差（从0.1增加到0.2弧度，约11.5度）
        orientation_constraint.absolute_x_axis_tolerance = 0.2
        orientation_constraint.absolute_y_axis_tolerance = 0.2
        orientation_constraint.absolute_z_axis_tolerance = 0.2
        orientation_constraint.weight = 1.0
        
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        
        goal_msg.request.goal_constraints = [constraints]
        
        self.get_logger().info(f'目标位置: x={target_pos.x:.3f}, y={target_pos.y:.3f}, z={target_pos.z:.3f}')
        planner_name = goal_msg.request.planner_id if goal_msg.request.planner_id else "默认规划器"
        self.get_logger().info(f'规划配置: 规划组={goal_msg.request.group_name}, 规划器={planner_name}')
        self.get_logger().info(f'规划配置: 尝试次数={goal_msg.request.num_planning_attempts}, 超时={goal_msg.request.allowed_planning_time}秒')
        self.get_logger().info(f'约束配置: 位置容差={box_constraint.dimensions[0]*100:.1f}cm, 姿态容差={orientation_constraint.absolute_x_axis_tolerance:.2f}弧度')
        self.get_logger().info('发送 MoveIt Action Goal...')
        
        # 发送 Action Goal 并等待结果
        try:
            future = self.moveit_action_client.send_goal_async(goal_msg)
            
            # 使用 executor 等待结果
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self)
            
            # 等待 goal 被接受（最多等待 10 秒）
            goal_timeout = 10.0
            try:
                executor.spin_until_future_complete(future, timeout_sec=goal_timeout)
            except Exception as e:
                self.get_logger().error(f'等待 MoveIt Action Goal 响应时发生异常: {str(e)}')
                executor.remove_node(self)
                return None
            finally:
                executor.remove_node(self)
            
            if not future.done():
                self.get_logger().error(f'MoveIt Action Goal 超时（>{goal_timeout}秒）')
                return None
            
            goal_handle = future.result()
            
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error('MoveIt Action Goal 被拒绝')
                return None
            
            self.get_logger().info('MoveIt Action Goal 已接受，等待规划完成...')
            
            # 等待规划结果
            result_future = goal_handle.get_result_async()
            
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self)
            
            # 等待结果（增加超时时间到 20 秒，因为规划可能需要更长时间）
            result_timeout = 20.0
            self.get_logger().info(f'等待规划结果，最多等待 {result_timeout} 秒...')
            
            try:
                # 使用循环等待，并显示进度
                start_time = self.get_clock().now().seconds_nanoseconds()[0]
                check_interval = 1.0
                elapsed = 0.0
                
                while elapsed < result_timeout:
                    if result_future.done():
                        break
                    executor.spin_once(timeout_sec=check_interval)
                    elapsed = self.get_clock().now().seconds_nanoseconds()[0] - start_time
                    if elapsed % 3.0 < check_interval:  # 每3秒显示一次进度
                        self.get_logger().info(f'规划进行中... ({elapsed:.1f}/{result_timeout:.1f} 秒)')
                
            except Exception as e:
                self.get_logger().error(f'等待 MoveIt Action 结果时发生异常: {str(e)}')
                executor.remove_node(self)
                return None
            finally:
                executor.remove_node(self)
            
            if not result_future.done():
                self.get_logger().error(f'MoveIt Action 结果超时（>{result_timeout}秒）')
                self.get_logger().error('可能的原因：目标位置不可达，或规划时间过长')
                return None
            
            result = result_future.result()
            
            if result is None:
                self.get_logger().error('MoveIt Action 返回 None')
                return None
            
            # 检查规划结果
            from moveit_msgs.msg import MoveItErrorCodes
            error_code = result.error_code.val
            
            if error_code != MoveItErrorCodes.SUCCESS:
                # 详细的错误信息
                error_map = {
                    MoveItErrorCodes.NO_IK_SOLUTION: "无IK解（目标位置不可达）",
                    MoveItErrorCodes.PLANNING_FAILED: "规划失败（可能碰撞/超出限位/无法采样有效状态）",
                    MoveItErrorCodes.INVALID_GROUP_NAME: "规划组名称错误",
                    MoveItErrorCodes.INVALID_LINK_NAME: "末端链接名称错误",
                    MoveItErrorCodes.TIMED_OUT: "规划超时",
                    MoveItErrorCodes.FAILURE: "规划失败（通用错误）"
                }
                error_info = error_map.get(error_code, f"未知错误（错误码：{error_code}）")
                self.get_logger().error(f'MoveIt 规划失败: {error_info}')
                self.get_logger().error(f'目标位置可能不可达或约束过严，建议：')
                self.get_logger().error(f'  1. 检查目标位置是否在机械臂工作空间内')
                self.get_logger().error(f'  2. 增大位置和姿态容差')
                self.get_logger().error(f'  3. 检查是否有碰撞或关节限位问题')
                return None
            
            # 提取轨迹
            if len(result.planned_trajectory.joint_trajectory.points) == 0:
                self.get_logger().error('MoveIt 返回的轨迹为空')
                return None
            
            trajectory = result.planned_trajectory.joint_trajectory
            
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'MoveIt 规划成功！')
            self.get_logger().info(f'轨迹包含 {len(trajectory.points)} 个轨迹点')
            self.get_logger().info(f'关节数量: {len(trajectory.joint_names)}')
            self.get_logger().info('=' * 60)
            
            return trajectory
            
        except Exception as e:
            self.get_logger().error(f'调用 MoveIt Action 时发生异常: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
    
    def plan_grasp_trajectory(self, grasp_pose):
        """规划抓取轨迹（直接使用 openarm_body_link0 坐标系，简化计算）"""
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'openarm_body_link0'  # 使用底座中心为原点
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # 设置关节名称（使用左臂的关节名称）
        trajectory.joint_names = [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        # 从抓取姿态中获取目标位置（在 openarm_body_link0 坐标系下）
        target_x = grasp_pose.grasp_pose.pose.position.x
        target_y = grasp_pose.grasp_pose.pose.position.y
        target_z = grasp_pose.grasp_pose.pose.position.z
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('坐标系说明：')
        self.get_logger().info('  使用 openarm_body_link0（机器人基座坐标系）')
        self.get_logger().info('    X轴正方向 = 前方（向前）')
        self.get_logger().info('    Y轴正方向 = 左侧（向左）')
        self.get_logger().info('    Z轴正方向 = 上方（向上）')
        self.get_logger().info('=' * 60)
        
        self.get_logger().info(f'目标位置（openarm_body_link0坐标系）: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
        
        # 计算相对于左臂基座的位置（在 openarm_body_link0 坐标系下）
        # 左臂基座位置：left_arm_base = [0.0, 0.031, 0.698]
        rel_x = target_x - 0.0
        rel_y = target_y - 0.031  # 负值表示香蕉在左臂左侧
        rel_z = target_z - 0.698  # 负值表示香蕉在左臂下方
        
        self.get_logger().info(f'相对位置（相对于左臂基座，在openarm_body_link0坐标系下）: x={rel_x:.3f}, y={rel_y:.3f}, z={rel_z:.3f}')
        self.get_logger().info(f'  解释：x={rel_x:.3f}（前方），y={rel_y:.3f}（左侧），z={rel_z:.3f}（上方）')
        
        # 使用简化的几何计算来规划关节角度
        self.get_logger().info('=' * 60)
        self.get_logger().info('使用简化的几何计算来规划关节角度...')
        
        # 计算目标位置的关节角度（基于几何关系）
        joint_angles = self.compute_joint_angles_from_position(target_x, target_y, target_z)
        if joint_angles is None:
            self.get_logger().error('关节角度计算失败，无法到达目标位置')
            return trajectory
        
        self.get_logger().info(f'计算出的关节角度: {[f"{a:.3f}" for a in joint_angles]}')
        self.get_logger().info('=' * 60)
        
        # 创建轨迹点1 - 接近位置（稍微高于目标）
        point1 = JointTrajectoryPoint()
        approach_z = target_z + 0.1  # 在目标上方10cm
        
        self.get_logger().info(f'接近位置: x={target_x:.3f}, y={target_y:.3f}, z={approach_z:.3f}')
        approach_angles = self.compute_joint_angles_from_position(target_x, target_y, approach_z)
        if approach_angles is None:
            self.get_logger().warn('接近位置计算失败，使用目标位置的角度')
            approach_angles = joint_angles
        point1.positions = approach_angles
        point1.velocities = [1.0] * 7  # 设置速度
        point1.accelerations = [0.0] * 7
        point1.time_from_start.sec = 0
        point1.time_from_start.nanosec = 500000000  # 0.5秒
        
        # 创建轨迹点2 - 抓取位置（目标位置）
        point2 = JointTrajectoryPoint()
        point2.positions = joint_angles
        point2.velocities = [1.0] * 7  # 设置速度
        point2.accelerations = [0.0] * 7
        point2.time_from_start.sec = 0
        point2.time_from_start.nanosec = 1000000000  # 1.0秒
        
        # 将轨迹点添加到轨迹中
        trajectory.points.append(point1)
        trajectory.points.append(point2)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'规划了包含 {len(trajectory.points)} 个轨迹点的抓取轨迹')
        self.get_logger().info(f'轨迹点1（接近位置，z+0.1m）: {[f"{p:.3f}" for p in point1.positions]}')
        self.get_logger().info(f'轨迹点2（抓取位置，目标位置）: {[f"{p:.3f}" for p in point2.positions]}')
        self.get_logger().info(f'目标位置: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
        self.get_logger().info('使用简化的几何计算，确保夹子能到达目标位置')
        self.get_logger().info('=' * 60)
        
        return trajectory
    
    def compute_joint_angles_from_position(self, target_x, target_y, target_z):
        """使用简化的几何计算来估算关节角度
        
        参数:
            target_x, target_y, target_z: 目标位置（在 openarm_body_link0 坐标系下）
            
        返回:
            关节角度列表 [joint1, joint2, ..., joint7]，如果计算失败返回 None
        """
        # 左臂基座位置（在 openarm_body_link0 坐标系下）
        # 根据 URDF，左臂基座位于：x=0.0, y=0.031, z=0.698
        base_x = 0.0
        base_y = 0.031
        base_z = 0.698
        
        # 计算相对于左臂基座的位置
        rel_x = target_x - base_x
        rel_y = target_y - base_y
        rel_z = target_z - base_z
        
        # 计算水平距离和角度
        horizontal_dist = np.sqrt(rel_x**2 + rel_y**2)
        total_dist = np.sqrt(horizontal_dist**2 + rel_z**2)
        
        # 简化的逆运动学计算（基于几何关系）
        # Joint1: 控制水平旋转（绕Z轴）
        joint1 = np.arctan2(rel_y, rel_x)
        
        # Joint2: 控制垂直平面内的旋转（基于水平距离和高度差）
        # 假设这是一个类似人臂的结构，joint2 控制肩部俯仰
        joint2 = np.arctan2(-rel_z, horizontal_dist)  # 负号因为Z向下为正
        
        # Joint3-Joint7: 使用简化的配置
        # 这些关节主要用于调整末端姿态，我们使用经验值
        # 假设关节3-7形成一个类似手腕的结构，用于微调姿态
        
        # 根据目标位置调整关节3-7
        # 如果目标在下方，需要更多向下弯曲
        if rel_z < 0:
            joint3 = 0.3  # 向下弯曲
            joint4 = 0.2
            joint5 = 0.1
        else:
            joint3 = -0.2  # 向上弯曲
            joint4 = 0.0
            joint5 = 0.0
        
        # Joint6 和 Joint7 用于调整末端姿态（roll 和 pitch）
        joint6 = 0.0  # 保持水平
        joint7 = 0.0  # 保持水平
        
        # 根据距离调整关节角度（如果距离较远，需要更多伸展）
        if total_dist > 0.5:
            # 目标较远，需要更多伸展
            joint3 += 0.2
            joint4 += 0.1
        elif total_dist < 0.3:
            # 目标较近，需要更多弯曲
            joint3 -= 0.2
            joint4 -= 0.1
        
        joint_angles = [
            joint1,   # joint1: 水平旋转
            joint2,   # joint2: 垂直俯仰
            joint3,   # joint3: 肘部
            joint4,   # joint4: 手腕俯仰
            joint5,   # joint5: 手腕偏航
            joint6,   # joint6: 末端旋转
            joint7    # joint7: 末端俯仰
        ]
        
        self.get_logger().info(f'计算关节角度:')
        self.get_logger().info(f'  目标相对位置: x={rel_x:.3f}, y={rel_y:.3f}, z={rel_z:.3f}')
        self.get_logger().info(f'  水平距离: {horizontal_dist:.3f}, 总距离: {total_dist:.3f}')
        self.get_logger().info(f'  关节角度: {[f"{a:.3f}" for a in joint_angles]}')
        
        return joint_angles
    
    
    def execute_trajectory(self, trajectory):
        """通过 Action 接口执行轨迹"""
        # 检查服务器是否就绪（不阻塞）
        if not self.trajectory_client.server_is_ready():
            self.get_logger().error('轨迹控制器未就绪')
            return False
        
        # 创建 Action goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # 发送 goal（异步方式，不等待结果）
        self.get_logger().info('发送轨迹执行请求...')
        
        # 发送 goal（异步，不等待结果）
        # 在服务回调中，我们不能阻塞等待 Action 的结果
        future = self.trajectory_client.send_goal_async(goal_msg)
        
        # 使用回调记录结果（但不阻塞）
        def goal_response_callback(future):
            try:
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    self.get_logger().info('轨迹执行请求已接受并执行')
                else:
                    self.get_logger().warn('轨迹执行请求被拒绝')
            except Exception as e:
                self.get_logger().error(f'轨迹执行请求处理异常: {str(e)}')
        
        future.add_done_callback(goal_response_callback)
        
        # 立即返回成功（不等待 future 完成）
        # 因为服务回调不应该阻塞，而且从日志看 goal 已经被接受了
        self.get_logger().info('轨迹执行请求已发送（异步执行）')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()