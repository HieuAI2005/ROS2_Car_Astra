from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():

    yolov8_detect_dir = get_package_share_directory('yolo_detection')
    model_path = os.path.join(yolov8_detect_dir, 'models', 'best_yolov8n_2.pt')

    return LaunchDescription([
        Node(
            package='astra_camera_driver',
            executable='camera_publisher',
            name='camera_publisher_node',
            output='screen',
        ),
        Node(
            package='yolo_detection',
            executable='yolo_detect',
            name='yolov8_detect_node',
            output='screen',
            parameters=[
                {'model_path': model_path}
            ]
        ),
        Node(
            package='astra_camera_driver',
            executable='camera_subscriber',
            name='camera_subcriber_node',
            output='screen'
        )
    ])
