import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            'yolov8n/detection_img',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
    
    def listener_callback(self, msg):
        self.get_logger().info('Oke')
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        cv2.imshow('Camera', frame)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args = args)      #Initialize ROS2
    node = ImageSubscriber()     #Create Node
    try:
        rclpy.spin(node)         #Keep node running, listen for callbacks
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()         #Shutdown ROS2 after node stops