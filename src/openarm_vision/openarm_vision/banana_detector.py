#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Point
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge

class BananaDetector(Node):
    def __init__(self):
        super().__init__('banana_detector')
        
        # 初始化工具
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)

        # 订阅话题
        self.create_subscription(Image, '/camera/color/image_raw', self.process, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        self.depth = None
        self.k = None

        # 将检测到的香蕉中心点在基座坐标系下沿 -Z 方向平移一点，
        # 让 banana_target 更接近香蕉底部（单位：米，可根据模型半径微调）
        self.banana_z_offset = 0.03
        
        # 黄色阈值 (根据环境光照微调)
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([35, 255, 255])

        # 位置变化检测：记录上次发布的位置，只有位置变化超过阈值才发布
        self.last_published_position = None
        self.position_change_threshold = 0.01  # 位置变化阈值（米），超过此值才发布新位置

        self.get_logger().info('Banana Detector: Started (Clean TF Mode)')

    def depth_callback(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            pass

    def info_callback(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)

    def pixel_to_optical(self, u, v, z):
        # 像素坐标 -> 光学坐标系
        fx = self.k[0, 0]
        fy = self.k[1, 1]
        cx = self.k[0, 2]
        cy = self.k[1, 2]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z

    def process(self, msg):
        if self.depth is None or self.k is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        # 1. 颜色识别
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                M = cv2.moments(c)
                if M["m00"] == 0: return
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # 2. 获取深度 (取中值滤波)
                depth_crop = self.depth[max(0, cy-2):min(cy+3, self.depth.shape[0]), 
                                        max(0, cx-2):min(cx+3, self.depth.shape[1])]
                if depth_crop.size == 0: return
                z_raw = float(np.nanmedian(depth_crop))

                if np.isnan(z_raw) or z_raw < 0.1:
                    return

                # 3. 计算相机光学坐标系下的点
                x_opt, y_opt, z_opt = self.pixel_to_optical(cx, cy, z_raw)

                # 4. TF 变换：从相机 -> 机器人基座
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'openarm_body_link0', # 目标坐标系
                        'd435_optical_frame', # 源坐标系
                        rclpy.time.Time())
                    
                    pt_optical = PointStamped()
                    pt_optical.header.frame_id = 'd435_optical_frame'
                    pt_optical.header.stamp = msg.header.stamp
                    pt_optical.point.x = x_opt
                    pt_optical.point.y = y_opt
                    pt_optical.point.z = z_opt

                    # 自动变换核心
                    pt_base = do_transform_point(pt_optical, trans).point

                    # 将坐标在基座坐标系下沿 -Z 方向平移，
                    # 把目标从香蕉几何中心下移到更接近底部的位置
                    pt_base.z -= self.banana_z_offset

                    # 5. 检查位置是否发生显著变化
                    should_publish = False
                    if self.last_published_position is None:
                        # 首次检测到，直接发布
                        should_publish = True
                    else:
                        # 计算位置变化距离
                        dx = pt_base.x - self.last_published_position[0]
                        dy = pt_base.y - self.last_published_position[1]
                        dz = pt_base.z - self.last_published_position[2]
                        distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                        
                        if distance > self.position_change_threshold:
                            should_publish = True
                    
                    if should_publish:
                        # 更新记录的位置
                        self.last_published_position = [pt_base.x, pt_base.y, pt_base.z]
                        
                        # 输出结果 (直接使用，不加负号，不减高度)
                        self.get_logger().info(
                            f' Result: X={pt_base.x:.3f}, Y={pt_base.y:.3f}, Z={pt_base.z:.3f}'
                        )

                        # 6. 发布坐标用于可视化
                        t = TransformStamped()
                        t.header.stamp = msg.header.stamp
                        t.header.frame_id = 'openarm_body_link0'
                        t.child_frame_id = 'banana_target'
                        t.transform.translation.x = pt_base.x
                        t.transform.translation.y = pt_base.y
                        t.transform.translation.z = pt_base.z
                        t.transform.rotation.w = 1.0
                        self.tf_pub.sendTransform(t)

                except Exception as e:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = BananaDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()