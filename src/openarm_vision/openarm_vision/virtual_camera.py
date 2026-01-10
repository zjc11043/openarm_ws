import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformListener, Buffer
import numpy as np
from cv_bridge import CvBridge
import cv2

class VirtualCamera(Node):
    def __init__(self):
        super().__init__('virtual_camera')
        self.bridge = CvBridge()
        
        # 话题发布
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        
        # TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 相机内参配置
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0
        self.width = 640
        self.height = 480
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Virtual Camera Started')

    def timer_callback(self):
        # 初始化背景 (灰色) 和 深度 (2米)
        img_color = np.full((self.height, self.width, 3), 50, dtype=np.uint8)
        img_depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
        now = self.get_clock().now().to_msg()
        
        try:
            # 查询 TF: 香蕉相对于相机的位置
            t = self.tf_buffer.lookup_transform(
                'd435_optical_frame', 
                'banana',             
                rclpy.time.Time())
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            
            # 如果物体在相机前方 (z > 0.01)
            if z > 0.01: 
                # 3D 投影到 2D 像素坐标
                u = int((x * self.fx / z) + self.cx)
                v = int((y * self.fy / z) + self.cy)
                
                # 计算显示大小 (距离越近越大)
                box_size = int((0.05 * self.fx) / z)
                r = max(5, int(box_size / 2))
                
                # 在图像范围内绘制
                if 0 <= u < self.width and 0 <= v < self.height:
                    # 绘制黄色方块代表香蕉
                    cv2.rectangle(img_color, (u-r, v-r), (u+r, v+r), (0, 255, 255), -1)
                    # 更新深度图对应区域
                    img_depth[v-r:v+r, u-r:u+r] = z
                else:
                    self.get_logger().debug(f'Object out of view: u={u}, v={v}')
            
        except Exception as e:
            # 仅在调试时关注 TF 错误，避免刷屏
            pass

        # 发布消息
        self.color_pub.publish(self.bridge.cv2_to_imgmsg(img_color, 'bgr8', header=None))
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(img_depth, '32FC1'))
        
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = "d435_optical_frame"
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
