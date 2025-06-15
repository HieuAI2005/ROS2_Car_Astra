import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from astra_camera_driver.astra_camera import Camera

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(
            Image, 
            'camera/image_raw', 
            10
        )
        self.cam = Camera()
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1/30, self.publish_frame)
    
    def publish_frame(self):
        _, rgb = self.cam.get_depth_and_color()
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding = 'bgr8')
        self.publisher.publish(msg)
        self.get_logger().info('Done 1 frame')
    
    def destroy_node(self):
        self.cam.unload()
        super().destroy_node()

def main(args = None):
    rclpy.init(args = args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()