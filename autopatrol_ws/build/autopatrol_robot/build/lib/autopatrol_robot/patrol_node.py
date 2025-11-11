# Ch 7.5.2
import rclpy
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy.time
from tf2_ros import TransformListener, Buffer
import math
from autopatrol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion [x, y, z, w]"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    
    return [qx, qy, qz, qw]

class PatrolNode(BasicNavigator):
    def __init__(self):
        super().__init__('patrol_node')
        
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, math.pi/2])

        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value

        self.buffer_ = Buffer()
        self.broadcaster_ = TransformListener(self.buffer_, self)
        self.speech_client_ = self.create_client(SpeechText, 'speech_text')

        self.declare_parameter('img_save_path', '')
        self.img_save_path_ = self.get_parameter('img_save_path').value
        self.cv_bridge_ = CvBridge()
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(Image, '/camera/image_raw', self.img_callback, 1)
        
        # Track start time and point index for image naming
        self.start_time_ = None  # Will be set to float in main()
        self.current_point_index_ = 0

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def init_robot_pose(self):
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()

    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for i in range(int(len(self.target_points_) / 3)):
            x = self.target_points_[i * 3]
            y = self.target_points_[i * 3 + 1]
            yaw = self.target_points_[i * 3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f'Target point {i}: x= {x}, y= {y}, yaw= {yaw}')
        return points

    def nav_to_pose(self, target_point):
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback is not None:
                self.get_logger().info(f'Remaining distance: {feedback.distance_remaining:.2f} meters')
            time.sleep(0.5)
        result = self.getResult()
        self.get_logger().info(f'Navigation result: {result}')

    def get_current_pose(self):
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform('map', 'base_footprint', 
                                                    rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                self.get_logger().info(f"Translation: {transform.translation}")
                return transform
            except Exception as e:
                self.get_logger().warn(f"Failed to obtain TF, cause: {str(e)}")

    def speech_text(self, text):
      while not self.speech_client_.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Service not available, waiting again...')
      request = SpeechText.Request()
      request.text = text
      future = self.speech_client_.call_async(request)
      rclpy.spin_until_future_complete(self, future)
      if future.result() is not None:
        response = future.result()
        if response.result == True:
          self.get_logger().info(f'Speech text: {text}')
        else:
          self.get_logger().error(f'Failed to speech text: {text}')
      else:
        self.get_logger().error('Failed to call service')


    def img_callback(self, img_msg):
        self.latest_img_ = img_msg

    def save_img(self, point_index):
        if self.latest_img_ is not None:
            cv_image = self.cv_bridge_.imgmsg_to_cv2(self.latest_img_, 'bgr8')
            
            # Default to media folder in workspace, or use configured path
            if self.img_save_path_:
                save_path = self.img_save_path_
            else:
                # Use get_package_share_directory to find workspace root
                # Package share dir: autopatrol_ws/install/autopatrol_robot/share/autopatrol_robot
                # Go up 4 levels to reach autopatrol_ws/
                try:
                    package_share_dir = get_package_share_directory('autopatrol_robot')
                    workspace_root = os.path.abspath(os.path.join(package_share_dir, '../../../..'))
                    save_path = os.path.join(workspace_root, 'media')
                    self.get_logger().info(f'Calculated save path: {save_path} (from package share: {package_share_dir})')
                except Exception as e:
                    # Fallback: use current file location
                    self.get_logger().warn(f'Failed to get package share directory: {e}, using fallback')
                    current_file = os.path.abspath(__file__)
                    workspace_root = os.path.abspath(os.path.join(os.path.dirname(current_file), '../../../..'))
                    save_path = os.path.join(workspace_root, 'media')
                    self.get_logger().info(f'Using fallback save path: {save_path}')
            
            # Ensure save directory exists
            if save_path and not os.path.exists(save_path):
                self.get_logger().info(f'Creating media directory: {save_path}')
                os.makedirs(save_path, exist_ok=True)
            
            # Calculate relative time from start
            if self.start_time_ is not None:
                current_time = time.time()
                relative_time = current_time - self.start_time_
                
                # Format filename with point index and relative timestamp
                filename = os.path.join(save_path, f'time_{int(relative_time)}s_point_{point_index}.png')
                self.get_logger().info(f'Attempting to save image to: {filename}')
                success = cv2.imwrite(filename, cv_image)
                if success:
                    self.get_logger().info(f'Image saved successfully to: {filename}')
                else:
                    self.get_logger().error(f'Failed to save image to: {filename}')
            else:
                self.get_logger().warn('Start time not set, cannot calculate relative time')
        else:
            self.get_logger().warn('No image available to save')

def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speech_text('Intializing Robot Pose')
    patrol.init_robot_pose()
    patrol.speech_text('Robot Pose Initialized')

    # Record start time for relative timestamp calculation
    patrol.start_time_ = time.time()

    while rclpy.ok():
        points = patrol.get_target_points()
        for point_index, point in enumerate(points):
            x, y, yaw = point
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speech_text(f'Navigating to point {x}, {y}')
            patrol.nav_to_pose(target_pose)
            patrol.speech_text(f'Arrived at point {x}, {y}, Saving image...')
            patrol.save_img(point_index)
            patrol.speech_text(f'Image saved')

    rclpy.shutdown()