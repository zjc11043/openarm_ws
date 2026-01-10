import rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge

class BananaDetector(Node):
    def __init__(self):
        super().__init__('banana_detector')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)

        # 合并订阅
        self.create_subscription(Image, '/camera/color/image_raw', self.process, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', lambda m: setattr(self, 'depth', self.bridge.imgmsg_to_cv2(m, '32FC1')), 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: setattr(self, 'k', np.array(m.k).reshape(3,3)), 10)
        
        self.depth = self.k = None
        self.get_logger().info('Banana Detector: Simplified Mode')

    def process(self, msg):
        if self.depth is None or self.k is None: return

        # 视觉识别 (HSV)
        hsv = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if cnts and cv2.contourArea(max(cnts, key=cv2.contourArea)) > 150:
            M = cv2.moments(max(cnts, key=cv2.contourArea))
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            
            # 获取中值深度
            z_cam = np.nanmedian(self.depth[cy-2:cy+3, cx-2:cx+3])
            if np.isnan(z_cam) or z_cam < 0.2: return

            # 投影到相机坐标系
            x_cam = (cx - self.k[0,2]) * z_cam / self.k[0,0]
            y_cam = (cy - self.k[1,2]) * z_cam / self.k[1,1]

            try:
                # 坐标变换与补正逻辑
                trans = self.tf_buffer.lookup_transform('openarm_body_link0', 'd435_optical_frame', rclpy.time.Time())
                p = do_transform_point(PointStamped(header=msg.header, point=Point(x=x_cam, y=float(y_cam), z=float(z_cam))), trans).point
                
                # 几何补正逻辑 (如果计算结果偏离桌子，则手动投影)
                fx = p.x if p.x > 0.1 else 0.04 + (z_cam * 0.96)
                fz = p.z if p.x > 0.1 else 0.62 - (z_cam * 0.26)

                self.get_logger().info(f'Target: [{fx:.3f}, {p.y:.3f}, {fz:.3f}]')
                
                # 发布 TF
                t = TransformStamped(header=msg.header)
                t.header.frame_id, t.child_frame_id = 'openarm_body_link0', 'banana_target'
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = fx, p.y, fz
                t.transform.rotation.w = 1.0
                self.tf_pub.sendTransform(t)
            except: pass

from geometry_msgs.msg import Point # 补充导入
def main():
    rclpy.init(); rclpy.spin(BananaDetector()); rclpy.shutdown()

if __name__ == '__main__':
    main()
