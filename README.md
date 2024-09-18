# Husky GPS Follower 

This workspace has src folder and dependencies for navigating the Husky robot using GPS in simulation.

## Installation
```bash
$ sudo apt install ros-humble-clearpath-desktop
$ mkdir ~/clearpath/
$ cd ~/clearpath/
```
Download robot.yaml into clearpath folder from this [repo](https://github.com/hyeseon-irosol/husky_config/tree/main/clearpath)

```bash
$ cd ~/clearpath_ws/
$ git clone https://github.com/hyeseon-irosol/husky_gps_follower.git
$ colcon build
```

## Navigation node setting
```bash
$ cd /opt/ros/humble/share/clearpath_nav2_demos/launch
$ sudo nano nav2.launch.py
```
Copy these lines to the GroupAction
Node(
        package='nav2_gps_waypoint_follower_demo',
        executable='gps_waypoint_follower',  # Replace with your actual executable name
        name='sensor_data_subscriber',
        output='screen',
        )

## How to use the packages from [Tinker-Twins](https://github.com/Tinker-Twins/Husky)

1. Keyboard Teleoperation:
    ```bash
    $ roslaunch husky_gazebo husky_playpen.launch
    $ roslaunch husky_control teleop_keyboard.launch
    ```

2. Map-Less Navigation:
    ```bash
    $ roslaunch husky_gazebo husky_playpen.launch
    $ roslaunch husky_viz view_robot.launch
    $ roslaunch husky_navigation map_less_navigation.launch
    ```

3. Simultaneous Localization And Mapping (SLAM):
    ```bash
    $ roslaunch husky_gazebo husky_playpen.launch
    $ roslaunch husky_viz view_robot.launch
    $ roslaunch husky_navigation gmapping.launch
    $ roslaunch husky_control teleop_keyboard.launch
    ```
    To save generated map to current working directory, run:
    ```bash
    $ rosrun map_server map_saver -f <filename>
    ```

4. Adaptive Monte Carlo Localization (AMCL):
    ```bash
    $ roslaunch husky_gazebo husky_playpen.launch
    $ roslaunch husky_viz view_robot.launch
    $ roslaunch husky_navigation amcl.launch
    $ roslaunch husky_control teleop_keyboard.launch
    ```

5. Map-Based Navigation:
    ```bash
    $ roslaunch husky_gazebo husky_playpen.launch
    $ roslaunch husky_viz view_robot.launch
    $ roslaunch husky_navigation map_based_navigation.launch
    ```
