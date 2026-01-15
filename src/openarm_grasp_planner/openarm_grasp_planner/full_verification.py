import rclpy
from rclpy.node import Node
import time
import threading
from std_srvs.srv import SetBool, Trigger
from moveit_msgs.srv import GraspPlanning
import tf2_ros

class FullVerificationSystem(Node):
    def __init__(self):
        super().__init__('full_verification_system')
        
        # TF 监听器，用于检查 banana_target frame 是否存在
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 服务客户端
        self.grasp_planner_client = self.create_client(GraspPlanning, 'plan_grasp')
        self.gripper_control_client = self.create_client(SetBool, 'control_gripper')
        # 抓取后抬起机械臂的服务客户端
        self.lift_client = self.create_client(Trigger, 'lift_after_grasp')
        
        # 等待服务可用
        while not self.grasp_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('抓取规划服务不可用，等待中...')
        
        while not self.gripper_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('夹爪控制服务不可用，等待中...')

        while not self.lift_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('抬起机械臂服务不可用，等待中...')
        
        self.get_logger().info('全流程实验验证系统已启动')
        self.get_logger().info('等待视觉模块定位目标物体...')
    
    def wait_for_target_detection(self, timeout_sec=30.0):
        """等待视觉模块检测到目标物体（banana_target frame 可用）"""
        self.get_logger().info('等待视觉模块检测目标物体...')
        start_time = time.time()
        check_count = 0
        
        while (time.time() - start_time) < timeout_sec:
            try:
                # 尝试查找 banana_target frame
                # 使用当前时间，允许一些时间容差
                transform = self.tf_buffer.lookup_transform(
                    'openarm_body_link0',
                    'banana_target',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.get_logger().info(f'目标物体已检测到! 位置: x={transform.transform.translation.x:.3f}, '
                                     f'y={transform.transform.translation.y:.3f}, '
                                     f'z={transform.transform.translation.z:.3f}')
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Frame 还不存在，继续等待
                check_count += 1
                if check_count % 10 == 0:  # 每5秒打印一次
                    elapsed = time.time() - start_time
                    self.get_logger().info(f'仍在等待目标物体检测... (已等待 {elapsed:.1f}秒)')
                time.sleep(0.5)
                continue
        
        self.get_logger().error(f'等待目标物体检测超时（{timeout_sec}秒）')
        self.get_logger().error('请确保：')
        self.get_logger().error('1. 虚拟相机已启动（ros2 run openarm_vision virtual_camera）')
        self.get_logger().error('2. 场景中存在香蕉物体')
        self.get_logger().error('3. 相机能够看到香蕉（黄色物体）')
        return False
    
    def run_full_verification(self):
        """执行完整的抓取验证流程"""
        self.get_logger().info('=== 开始全流程抓取验证 ===')
        
        try:
            # 步骤0: 等待视觉模块检测到目标物体
            self.get_logger().info('步骤0: 等待视觉模块检测目标物体')
            if not self.wait_for_target_detection(timeout_sec=30.0):
                self.get_logger().error('无法检测到目标物体，终止流程')
                return False
            
            # 步骤1: 打开夹爪
            self.get_logger().info('步骤1: 打开夹爪')
            self.control_gripper(False)  # False表示打开夹爪
            time.sleep(2)
            
            # 步骤2: 请求抓取规划
            self.get_logger().info('步骤2: 请求抓取规划')
            grasp_result = self.request_grasp_planning()
            
            if not grasp_result:
                self.get_logger().error('抓取规划失败，终止流程')
                return False
            
            # 步骤3: 执行抓取动作（这里假设机械臂会自动执行发布的轨迹）
            self.get_logger().info('步骤3: 执行抓取动作')
            self.get_logger().info('注意：请确保机械臂控制器已启动并订阅grasp_trajectory话题')
            time.sleep(4)  # 等待机械臂完成抓取动作
            
            # 步骤4: 闭合夹爪
            self.get_logger().info('步骤4: 闭合夹爪')
            self.control_gripper(True)  # True表示闭合夹爪
            time.sleep(15)
            
            # 步骤5: 抓取完成，提升机械臂
            self.get_logger().info('步骤5: 抓取完成，提升机械臂')
            if not self.lift_arm():
                self.get_logger().error('提升机械臂失败')
                return False
            # 给机械臂一些时间完成抬起动作
            time.sleep(5)
            
            # 步骤6: 打开夹手放下香蕉
            self.get_logger().info('步骤6: 打开夹手放下香蕉')
            self.control_gripper(False)  # False表示打开夹爪
            time.sleep(2)
            
            self.get_logger().info('=== 全流程抓取验证完成 ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'全流程验证失败: {e}')
            return False

    def lift_arm(self):
        """调用抬起机械臂服务。"""
        req = Trigger.Request()

        future = self.lift_client.call_async(req)

        # 使用单独的executor避免spin冲突
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(future)
        executor.remove_node(self)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('提升机械臂成功')
                return True
            else:
                self.get_logger().error(f'提升机械臂失败: {future.result().message}')
                return False
        else:
            self.get_logger().error('抬起机械臂服务调用失败')
            return False
    
    def request_grasp_planning(self):
        """请求抓取规划"""
        req = GraspPlanning.Request()
        # 这里可以根据需要设置请求参数
        
        future = self.grasp_planner_client.call_async(req)
        
        # 使用单独的executor避免spin冲突
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(future)
        executor.remove_node(self)
        
        if future.result() is not None:
            from moveit_msgs.msg import MoveItErrorCodes
            if future.result().error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('抓取规划成功')
                return True
            else:
                self.get_logger().error(f'抓取规划失败，错误码: {future.result().error_code.val}')
                return False
        else:
            self.get_logger().error('抓取规划服务调用失败')
            return False
    
    def control_gripper(self, close):
        """控制夹爪开合"""
        req = SetBool.Request()
        req.data = close
        
        future = self.gripper_control_client.call_async(req)
        
        # 使用单独的executor避免spin冲突
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(future)
        executor.remove_node(self)
        
        if future.result() is not None:
            if future.result().success:
                action = "闭合" if close else "打开"
                self.get_logger().info(f'夹爪{action}成功')
                return True
            else:
                action = "闭合" if close else "打开"
                self.get_logger().error(f'夹爪{action}失败: {future.result().message}')
                return False
        else:
            action = "闭合" if close else "打开"
            self.get_logger().error(f'夹爪{action}服务调用失败')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    # 创建全流程验证系统节点
    verification_system = FullVerificationSystem()
    
    # 创建一个线程来执行验证流程，避免阻塞spin
    verification_thread = threading.Thread(target=verification_system.run_full_verification)
    verification_thread.daemon = True
    verification_thread.start()
    
    # 启动节点
    try:
        rclpy.spin(verification_system)
    except KeyboardInterrupt:
        pass
    finally:
        verification_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()