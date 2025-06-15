# ROS2_Car_Astra
1. **Install ROS2 Jazzy Jalisco**
    ```
    Link: https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html
    ```
2. **Create project, setup environment ros2 and python**
    ```
    source /opt/ros/jazzy/setup.bash
    mkdir ros2_ws
    cd /ros2_ws/
    python3 -m venv venv
    ```
3. **Build packages**
    ```
    mkdir src
    cd /src
    ros2 pkg create ...
    ```
4. **Using Astra_camera and yolov8n (pre-trained)**
    Download your sdk from here 
    ```
    https://www.orbbec.com/developers/openni-sdk/
    ```
    Pre-install 
    ```
    sudo apt-get install build-essential freeglut3 freeglut3-dev
    ```
    Extract sdk zip file, chose your sdk for your os, extract one more, open rules file.
    ```
    bash install.sh
    ```
    Install openni and copy file lib to your project
    ```
    pip install openni
    ```
    Install yolov8, ...
    ```
    pip install ultralytics 
    ```
5. **Run your project**
    Using colcon 
    ```
    colcon build
    ```
    Run 
    ```
    ros2 launch astra_camera_driver camera_driver_launch.py
    ```
