import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')
        
        # 夹爪状态参数（根据 SRDF 定义：0.0=闭合，0.060=更大打开）
        # 注意：位置值表示手指分开的距离，0.0 表示完全闭合，0.060 表示完全打开
        self.gripper_open_position = 0.060   # 夹爪打开位置（手指分开）
        # 稳定防滑模式：适当减小闭合距离，但不过度压入，避免弹飞
        # 香蕉碰撞胶囊半径为 0.02，这里设置略大于 0.02，主要依靠高摩擦防滑
        self.gripper_closed_position = 0.03  # 稍紧的闭合目标位置，兼顾防滑与稳定
        self.gripper_max_effort = 70.0  # 适中最大抓取力，减小接触瞬间冲击
        
        # 创建Action客户端用于控制夹爪（双机械臂版本）
        self.left_gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/left_gripper_controller/gripper_cmd'  # 使用正确的主题名称和命名空间
        )
        
        self.right_gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/right_gripper_controller/gripper_cmd'  # 使用正确的主题名称和命名空间
        )
        
        # 创建服务用于控制夹爪开合
        self.gripper_service = self.create_service(
            SetBool, 
            'control_gripper', 
            self.control_gripper_callback
        )
        
        # 订阅关节状态获取夹爪当前位置
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        self.current_left_gripper_position = None
        self.current_right_gripper_position = None
        self.get_logger().info('夹爪控制模块已启动（双机械臂版本）')
        self.get_logger().info(f'左夹爪 Action 话题: /left_gripper_controller/gripper_cmd')
        self.get_logger().info(f'右夹爪 Action 话题: /right_gripper_controller/gripper_cmd')
        self.get_logger().info('等待夹爪控制器启动...')
    
    def joint_state_callback(self, msg):
        """获取夹爪当前位置"""
        try:
            # 获取左夹爪位置
            index = msg.name.index('openarm_left_finger_joint1')
            self.current_left_gripper_position = msg.position[index]
        except ValueError:
            pass
            
        try:
            # 获取右夹爪位置
            index = msg.name.index('openarm_right_finger_joint1')
            self.current_right_gripper_position = msg.position[index]
        except ValueError:
            pass
    
    def control_gripper_callback(self, request, response):
        """处理夹爪控制服务请求（只控制左夹爪）"""
        self.get_logger().info(f'收到夹爪控制请求: {"闭合" if request.data else "打开"}')
        
        # 设置目标位置
        target_position = self.gripper_closed_position if request.data else self.gripper_open_position

        # 若为闭合命令，采用“多段闭合”策略，减小瞬时速度和冲击
        if request.data:
            from threading import Thread

            def staged_close():
                # 多段目标：从当前/打开位，依次缩小步长再到最终闭合值
                # 这里使用 0.05  -> gripper_closed_position
                import time
                self.get_logger().info('开始多段闭合：阶段 1 -> 0.05')
                self.send_gripper_command(self.left_gripper_client, 0.05, "左")
                time.sleep(0.4)
                self.get_logger().info('多段闭合：阶段 2 -> 0.045')
                self.send_gripper_command(self.left_gripper_client, 0.045, "左")
                time.sleep(0.4)
                self.get_logger().info(f'多段闭合：阶段 3 -> 最终 {self.gripper_closed_position:.3f}')
                self.send_gripper_command(self.left_gripper_client, self.gripper_closed_position, "左")

            Thread(target=staged_close, daemon=True).start()
            left_result = True
        else:
            # 打开时仍然一次性命令到打开位置即可
            left_result = self.send_gripper_command(self.left_gripper_client, target_position, "左")
        
        if left_result:
            response.success = True
            response.message = f'左夹爪已{"闭合" if request.data else "打开"}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f'左夹爪{"闭合" if request.data else "打开"}失败'
            self.get_logger().error(response.message)
        
        return response
    
    def send_gripper_command(self, client, position, arm_side):
        """发送夹爪控制命令（只控制左夹爪）"""
        # 检查Action服务器是否就绪（不阻塞）
        if not client.server_is_ready():
            self.get_logger().error(f'{arm_side}夹爪控制服务器未就绪')
            self.get_logger().error(f'请确保 MuJoCo 和控制器已启动，Action 话题: {client._action_name}')
            return False
        
        self.get_logger().info(f'{arm_side}夹爪控制服务器已就绪')
        
        # 创建Action目标
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = self.gripper_max_effort
        
        # 发送请求（异步方式，不等待结果）
        self.get_logger().info(f'发送{arm_side}夹爪控制命令: 位置={position}, 最大力={self.gripper_max_effort}')
        self.get_logger().info(f'注意：位置值含义 - 0.0=闭合（手指靠拢），0.044=打开（手指分开）')
        try:
            # 发送 goal（异步，不等待结果）
            # 在服务回调中，我们不能阻塞等待 Action 的结果
            # 只要服务器就绪，就发送命令并假设成功
            future = client.send_goal_async(goal_msg)
        
            # 使用回调记录结果（但不阻塞）
            def goal_response_callback(future):
                try:
                    goal_handle = future.result()
                    if goal_handle and goal_handle.accepted:
                        self.get_logger().info(f'{arm_side}夹爪控制命令已被控制器接受，开始执行...')

                        # 在执行完成后再获取一次结果（完全异步，不阻塞 service 回调）
                        result_future = goal_handle.get_result_async()

                        def result_callback(result_future):
                            try:
                                result = result_future.result().result
                                # Action 返回的最终位置
                                self.get_logger().info(
                                    f'{arm_side}夹爪动作执行完成，Action 返回位置: {result.position:.4f}'
                                )

                                # 结合 /joint_states 中记录的当前位置再打印一次（如果可用）
                                if arm_side == "左" and self.current_left_gripper_position is not None:
                                    self.get_logger().info(
                                        f'执行完成后当前左夹爪 /joint_states 位置: {self.current_left_gripper_position:.4f}'
                                    )
                            except Exception as e:
                                self.get_logger().error(f'{arm_side}夹爪结果回调处理异常: {str(e)}')

                        result_future.add_done_callback(result_callback)
                    else:
                        self.get_logger().warn(f'{arm_side}夹爪控制命令被拒绝')
                except Exception as e:
                    self.get_logger().error(f'{arm_side}夹爪控制命令处理异常: {str(e)}')
            
            future.add_done_callback(goal_response_callback)
            
            # 立即返回成功（不等待 future 完成）
            # 因为服务回调不应该阻塞，而且从日志看 goal 已经被接受了
            self.get_logger().info(f'{arm_side}夹爪控制命令已发送（异步执行）')
            return True
            
        except Exception as e:
            self.get_logger().error(f'发送{arm_side}夹爪控制命令时发生异常: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()