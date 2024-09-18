# Husky GPS Follower 

This workspace has a src folder and dependencies for navigating the Husky robot using GPS in simulation.

## [Installation #1: clearpath](https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc)
```bash
$ sudo apt install ros-humble-clearpath-desktop
$ mkdir ~/clearpath/
$ cd ~/clearpath/
```
Download robot.yaml into 'clearpath' folder from this [repo](https://github.com/hyeseon-irosol/husky_config/tree/main/clearpath)
```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run clearpath_generator_common generate_bash -s ~/clearpath
$ source ~/clearpath/setup.bash
```

## [Installation #2: clearpath_ws](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install)
```bash
$ cd ~/clearpath_ws/
$ git clone https://github.com/hyeseon-irosol/husky_gps_follower.git
$ colcon build
```

## Navigation node setting
```bash
$ sudo apt-get update
$ sudo apt-get install ros-humble-clearpath-nav2-demos
$ cd /opt/ros/humble/share/clearpath_nav2_demos/launch
$ sudo nano nav2.launch.py
```
Copy these lines to the GroupAction

"Node(
        package='nav2_gps_waypoint_follower_demo',
        executable='gps_waypoint_follower',  # Replace with your actual executable name
        name='sensor_data_subscriber',
        output='screen',
        )"

## Running SLAM and Navigation using Simulator

1. Launch the simulation:
    ```bash
    $ ros2 launch clearpath_gz simulation.launch.py
    ```

2. Launch RViz:
    ```bash
    $ ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0284
    ```

3. Launch Localization:
    ```bash
    $ ros2 launch clearpath_nav2_demos localization.launch.py setup_path:=$HOME/clearpath/
    ```

4. Launch Nav2 with GPS follower node:
    ```bash
    $ ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/
    ```
