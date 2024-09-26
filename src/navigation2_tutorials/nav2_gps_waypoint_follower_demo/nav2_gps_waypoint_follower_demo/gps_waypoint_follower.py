import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped, TransformStamped, PoseWithCovarianceStamped, PoseStamped, Quaternion
from pyproj import CRS, Transformer
from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import math
import tf_transformations

# Define projections
wgs84 = CRS('epsg:4326')  # WGS84 (GPS coordinates)
utm = CRS('epsg:32723')   # UTM Zone 33N (adjust based on your location) / MTU utm = CRS('epsg:32616') (UTM Zone 16T)

# Create a transformer object
transformer = Transformer.from_crs(wgs84, utm, always_xy=True)

class SensorDataSubscriber(Node):

    utm_x = 0.0  # Initialize UTM coordinates
    utm_y = 0.0
    dist_x = 0.0
    dist_y = 0.0
    current_yaw = 0.0

    def __init__(self):
        super().__init__('sensor_data_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.utm_x = 0.0  # Initialize UTM coordinates
        #self.utm_y = 0.0
        #self.dist_x = 0.0
        #self.dist_y = 0.0
        #self.current_yaw = 0.0

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/a200_0284/amcl_pose',
            self.amcl_callback,
            10
        )

        # Subscriber for GPS data
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/a200_0284/sensors/gps_0/fix',
            self.gps_callback,
            10
        )

        # Subscriber for Odometry data
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/a200_0284/platform/odom',
            self.odom_callback,
            10
        )

        # Subscriber for IMU data
        """self.imu_subscription = self.create_subscription(
            Imu,
            '/a200_0284/sensors/imu_0/data',
            self.imu_callback,
            10
        )"""

        self.initpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/a200_0284/initialpose',
            self.initpose_callback,
            10
        )

        # Initialize BasicNavigator
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Set up goal poses
        #self.goal_poses = []
        # self.setup_goals()

        # Flag for starting navigation after receiving valid GPS data
        #self.start_navigation()
        self.navigation_started = False

        # TF2 listener
        """self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self.broadcast_utm_frame)

    def broadcast_utm_frame(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'utm'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        #self.get_logger().info('Broadcasting UTM frame.')"""

    def gps_to_utm(self, lat, lon):
        x, y = transformer.transform(lon, lat)
        return x, y

    """def transform_to_robot_local_frame(self, utm_x, utm_y):
        if not hasattr(self, 'robot_pose'):
            self.get_logger().warn('Robot pose not available yet.')
            return

        # Create a PointStamped object with UTM coordinates
        utm_point = PointStamped()
        utm_point.header.frame_id = 'utm'  # UTM frame
        utm_point.header.stamp = self.get_clock().now().to_msg()
        utm_point.point.x = utm_x
        utm_point.point.y = utm_y
        utm_point.point.z = 0.0

        try:
            # Transform UTM coordinates to robot local frame
            transform = self.tf_buffer.lookup_transform('base_link', 'utm', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            local_point = tf2_geometry_msgs.do_transform_point(utm_point, transform)
            self.get_logger().info(f'Local coordinates: X: {local_point.point.x}, Y: {local_point.point.y}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Transform error: {e}')"""

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose  # Store robot pose
        #self.get_logger().info(f'Received Odometry data: Position - X: {msg.pose.pose.position.x}, Y: {msg.pose.pose.position.y}, Z: {msg.pose.pose.position.z}')

    """def imu_callback(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        # Use tf2 to convert quaternion to Euler angles
        try:
            
            euler = tf_transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw= euler[2]  # Yaw angle in radians
            SensorDataSubscriber.current_yaw =yaw

            #self.get_logger().info(f'Received IMU data: Roll: {roll}, Pitch: {pitch}, Yaw: {SensorDataSubscriber.current_yaw}')
        except TransformException as e:
            self.get_logger().error(f'Failed to transform: {e}')"""

    def amcl_callback(self, msg):
        #self.amcl_pose = msg.pose.pose
        
        self.get_logger().info(f'ANYTHING')
        amcl_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        try:
            
            euler = tf_transformations.euler_from_quaternion(amcl_quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw= euler[2]  # Yaw angle in radians
            SensorDataSubscriber.current_yaw =yaw

            self.get_logger().info(f'Received AMCL data: Roll: {roll}, Pitch: {pitch}, Yaw: {SensorDataSubscriber.current_yaw}')
        except TransformException as e:
            self.get_logger().error(f'Failed to transform: {e}')

    """def utm_goal(self):
        # Example UTM coordinates for goals
        utm_goal1 = (684251.6534255196, 7456825.068300178)  # Replace with your UTM coordinates
        # utm_goal2 = (500500, 4649000)  # Replace with your UTM coordinates
        # utm_goal3 = (501000, 4648000)  # Replace with your UTM coordinates"""


    """def utm_to_dist(self, target_utm):
        self.dist = self.calculate_dist(self.utm_x, self.utm_y, target_utm, self.current_yaw)
        self.get_logger().info(f'Dist: x={self.dist[0]}, y={self.dist[1]}')

        # Transform UTM coordinates to robot local frame
        #self.transform_to_robot_local_frame(utm_x, utm_y)
        #self.get_logger().info(f'Converted Local coordinates: X: {local_x}, Y: {local_y}')

        self.goal_poses.append(self.utm_to_dist(*utm_goal1))
        # self.goal_poses.append(self.utm_to_pose(*utm_goal2))
        # self.goal_poses.append(self.utm_to_pose(*utm_goal3))"""

    def initpose_callback(self, msg):
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        self.get_logger().info(f'Init Position: x={position.x}, y={position.y}, z={position.z}')
        self.get_logger().info(f'Init Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')

#######################################

    """def utm_to_pose(self, dist, frame_id='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = dist[0]
        pose.pose.position.y = dist[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose"""

    def calculate_dist(self, goal_utm):
        angle = SensorDataSubscriber.current_yaw
        self.get_logger().info(f'Current_utm: x={SensorDataSubscriber.utm_x}, y={SensorDataSubscriber.utm_y}')
        self.get_logger().info(f'Goal_utm: x={goal_utm[0]}, y={goal_utm[1]}')
        self.get_logger().info(f'Current_yaw: {SensorDataSubscriber.current_yaw}')
        SensorDataSubscriber.dist_x = (goal_utm[0]-SensorDataSubscriber.utm_x)*math.cos(angle) + (goal_utm[1]-SensorDataSubscriber.utm_y)*math.sin(angle)
        SensorDataSubscriber.dist_y = -(goal_utm[0]-SensorDataSubscriber.utm_x)*math.sin(angle) + (goal_utm[1]-SensorDataSubscriber.utm_y)*math.cos(angle)

    def gps_callback(self, msg):
        # Convert GPS coordinates to UTM
        SensorDataSubscriber.utm_x, SensorDataSubscriber.utm_y = self.gps_to_utm(msg.latitude, msg.longitude)
        self.get_logger().info(f'Received GPS data: Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}')
        
        self.get_logger().info(f'Converted UTM coordinates: X: {SensorDataSubscriber.utm_x}, Y: {SensorDataSubscriber.utm_y}')
        # Start navigation after receiving the first valid GPS data
        if not hasattr(self, 'navigation_started') or not self.navigation_started:
            self.start_navigation()
            self.navigation_started = True

    def set_goal_by_gps(self, goal_lat, goal_lon, goal_poses):
        # Convert the GPS goal to UTM coordinates
        goal_utm_x, goal_utm_y = self.gps_to_utm(goal_lat, goal_lon)
        self.get_logger().info(f'Goal GPS: lat={goal_lat}, lon={goal_lon}')
        self.get_logger().info(f'Converted Goal UTM coordinates: X: {goal_utm_x}, Y: {goal_utm_y}')

        # Calculate the distance to the goal based on the current UTM position
        self.calculate_dist((goal_utm_x, goal_utm_y))
        self.get_logger().info(f'goal x= {SensorDataSubscriber.dist_x}, goal y={SensorDataSubscriber.dist_y}')

        # Prepare PoseStamped for the current goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = SensorDataSubscriber.dist_x
        goal_pose.pose.position.y = SensorDataSubscriber.dist_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # Add the current goal pose to the goal_poses list
        goal_poses.append(goal_pose)

    def start_navigation(self):
        # List of GPS coordinates (latitude, longitude) for multiple goals
        waypoints = [
              # Example goal 1
            (-22.986628459702754, -43.20239925274351),  # Example goal 2
            (-22.98650994296294,  -43.20241170571052)   # Example goal 3
        ]

        # Create a list to store goal poses
        goal_poses = []

        # Iterate through each waypoint and set the goal using GPS coordinates
        for lat, lon in waypoints:
            self.set_goal_by_gps(lat, lon, goal_poses)

        # Start following the waypoints
        self.navigator.followWaypoints(goal_poses)
        self.get_logger().info('Starting navigation to multiple waypoints.')

#######################################

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataSubscriber()
    while rclpy.ok():
        rclpy.spin(node)
        feedback = node.navigator.getFeedback()
        if feedback:
            node.get_logger().info(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(node.goal_poses)}')
            node.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters.')

            now = node.navigator.get_clock().now()
            if now - node.navigator.get_clock().now() > Duration(seconds=600.0):
                node.navigator.cancelTask()

        if node.navigator.isTaskComplete():
            result = node.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                node.get_logger().info('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                node.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                node.get_logger().info('Goal failed!')
            else:
                node.get_logger().info('Goal has an invalid return status!')
            break

    node.destroy_node()
    rclpy.shutdown()
    #######################
if __name__ == '__main__':
    main()