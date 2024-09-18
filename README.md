# Husky GPS Follower 

This workspace has a src folder and dependencies for navigating the Husky robot using GPS in simulation.

## [Package and setup for GPS](https://docs.clearpathrobotics.com/docs/ros/config/yaml/sensors/gps)
```bash
# Install build dependencies.
sudo apt-get install build-essential pkg-config cmake doxygen check
# Clone driver to home directory
cd ~
git clone https://github.com/swift-nav/libsbp.git
# Checkout latest supported branch
cd libsbp
git checkout e149901e63ddcdb0d818adcd8f8e4dbd0e2738d6
# Update submodules
cd c
git submodule update --init --recursive
# Build and install
mkdir build; cd build
cmake ../
make
sudo make install
```

## [Installation #1: clearpath](https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc)
```bash
sudo apt install ros-humble-clearpath-desktop
mkdir ~/clearpath/
cd ~/clearpath/
```
Download robot.yaml into 'clearpath' folder from this [repo](https://github.com/hyeseon-irosol/husky_config/tree/main/clearpath)
```bash
source /opt/ros/humble/setup.bash
ros2 run clearpath_generator_common generate_bash -s ~/clearpath
# Add the following line to your ~/.bashrc file to automatically source the generated setup.bash file in new terminals:
source ~/clearpath/setup.bash
```

## [Installation #2: clearpath_ws](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install)
```bash
cd ~/clearpath_ws/
git clone https://github.com/hyeseon-irosol/husky_gps_follower.git
colcon build
# Add the following line to your ~/.bashrc file to automatically source the generated setup.bash file in new terminals:
source ~/clearpath_ws/install/setup.bash
```

## Navigation node setting
Requirement for node
```bash
pip install pyproj
sudo apt-get install ros-humble-tf-transformations
```
Package for node & Editing the nav2.launch file
```bash
sudo apt-get update
sudo apt-get install ros-humble-clearpath-nav2-demos
cd /opt/ros/humble/share/clearpath_nav2_demos/launch
sudo nano nav2.launch.py
```

1. Add "Node" in the line for actions
```bash
from launch_ros.actions import PushRosNamespace, SetRemap, Node
```

2. Copy this line to packages list
```bash
pkg_gps_package = get_package_share_directory('nav2_gps_waypoint_follower_demo')
```

3. Copy these lines to the GroupAction
```bash
Node(
        package='nav2_gps_waypoint_follower_demo',
        executable='gps_waypoint_follower',  # Replace with your actual executable name
        name='sensor_data_subscriber',
        output='screen',
        )
```
## [Running SLAM and Navigation using Simulator](https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation_demos/nav2)

1. Launch the simulation:
    ```bash
    ros2 launch clearpath_gz simulation.launch.py
    ```
    Click the active button on the left-bottom side.
   ![alt text](https://github.com/hyeseon-irosol/husky_gps_follower/blob/main/gazebo_image.png?raw=true)

3. Launch RViz:
    ```bash
    ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0284
    ```

4. Launch Localization:
    ```bash
    ros2 launch clearpath_nav2_demos localization.launch.py setup_path:=$HOME/clearpath/
    ```
    Use the 2D Pose Estimate tool in RViz to set the initial pose of the robot.
   ![alt text](https://github.com/hyeseon-irosol/husky_gps_follower/blob/main/pose_est.png?raw=true)

6. Launch Nav2 with GPS follower node:
    ```bash
    ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/
    ```
   ![alt text](https://github.com/hyeseon-irosol/husky_gps_follower/blob/main/gps_nav.mp4?raw=true)

