import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2  
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolov8_detect')
        self.declare_parameter('model_path', 'best_yolov8n_2.pt')
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLOv8 model from: {self.model_path}')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Start')
        self.publisher_image = self.create_publisher(
            Image, 
            'yolov8n/detection_img', 
            10
        )
        self.get_logger().info('Publishing to /yolov8/detection_img')
    
    def image_callback(self, msg):
        self.get_logger().debug('Received image message.')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return
        
        results = self.model(cv_image, verbose = False)
        self.get_logger().info(f'Detected {len(results[0].boxes)} objects')
        frame = cv_image.copy()
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'Person: {conf:.2f}'
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        try:
            image_out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_image.publish(image_out)
        except Exception as e:
            self.get_logger().error(f'CvBridge Error (publish): {e}')

def main(args = None):
    rclpy.init(args = args)
    yolo = YoloDetector()
    rclpy.spin(yolo)
    yolo.destroy_node()
    rclpy.shutdown()
