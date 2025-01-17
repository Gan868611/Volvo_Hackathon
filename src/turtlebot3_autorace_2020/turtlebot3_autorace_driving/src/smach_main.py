#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import os
import yaml
import requests
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import UInt8
from turtlebot3_autorace_msgs.msg import MovingParam
from enum import Enum
import threading

ws_name = "volvo_hackathon_ws"
TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')

# Helper function to check proximity to a location
def is_near_location(current_pose, target_location):
    dx = current_pose.position.x - target_location['x']
    dy = current_pose.position.y - target_location['y']
    distance = (dx**2 + dy**2)**0.5
    return distance < target_location['radius']

# Define state StartState
class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['proceed_to_localization'])
        self.required_topics = [
            # '/cmd_vel',
            # '/odom',
            # '/scan',  # Assuming a LIDAR or depth sensor topic
            # '/signs_detected',
            # '/gantry_status'
        ]
        self.required_services = [
            # '/map',  # SLAM map service, for example
        ]
        self.initialization_complete = False

    def check_topics(self):
        """Verify that required topics are active."""
        rospy.loginfo("Checking required topics...")
        active_topics = [topic[0] for topic in rospy.get_published_topics()]
        for topic in self.required_topics:
            if topic not in active_topics:
                rospy.logwarn(f"Required topic '{topic}' is not active!")
                return False
        rospy.loginfo("All required topics are active.")
        return True

    def check_services(self):
        """Verify that required services are available."""
        rospy.loginfo("Checking required services...")
        for service in self.required_services:
            try:
                rospy.wait_for_service(service, timeout=5)
                rospy.loginfo(f"Service '{service}' is available.")
            except rospy.ROSException:
                rospy.logwarn(f"Required service '{service}' is not available!")
                return False
        return True

    def execute(self, userdata):
        rospy.loginfo("Executing StartState...")

        # Perform system checks
        topics_ok = self.check_topics()
        services_ok = self.check_services()

        if not (topics_ok and services_ok):
            rospy.logerr("System initialization failed! Retrying...")
            rospy.sleep(5)  # Retry delay
            return self.execute(userdata)  # Retry the state

        rospy.loginfo("System initialization complete. Ready to proceed.")
        self.initialization_complete = True

        rospy.sleep(2)  # Simulate additional setup time if necessary
        return 'proceed_to_localization'
    
class LocalizationState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['localized', 'failed'])
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)
        # Set the fixed path for the coordinate folder
        self.coordinate_folder = os.path.expanduser(f"~/catkin_ws/coordinate")

    def read_initial_pose(self, filename):
        """Reads the pose from a YAML file."""
        try:
            filepath = os.path.join(self.coordinate_folder, filename)
            rospy.loginfo(f"Reading initial pose from {filepath}")
            with open(filepath, 'r') as file:
                pose_data = yaml.safe_load(file)
                return pose_data
        except Exception as e:
            rospy.logerr(f"Failed to read initial pose from {filename}: {e}")
            return None

    def execute(self, userdata):
        rospy.loginfo("Executing LocalizationState...")

        # Retrieve the desired starting position (fixed position for testing)
        position_param = rospy.get_param('~start_position', 1)  # Defaults to parking_1 if not set
        filename = f"parking_{position_param}.yaml"

        # Read the pose data from the file
        pose_data = self.read_initial_pose(filename)
        if not pose_data:
            rospy.logerr(f"Could not load pose data for {filename}.")
            return 'failed'

        # Create a PoseWithCovarianceStamped message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"

        # Set the position and orientation
        initial_pose.pose.pose.position.x = pose_data['pose']['pose']['position']['x']
        initial_pose.pose.pose.position.y = pose_data['pose']['pose']['position']['y']
        initial_pose.pose.pose.position.z = pose_data['pose']['pose']['position']['z']
        initial_pose.pose.pose.orientation.x = pose_data['pose']['pose']['orientation']['x']
        initial_pose.pose.pose.orientation.y = pose_data['pose']['pose']['orientation']['y']
        initial_pose.pose.pose.orientation.z = pose_data['pose']['pose']['orientation']['z']
        initial_pose.pose.pose.orientation.w = pose_data['pose']['pose']['orientation']['w']
        rospy.loginfo(initial_pose.pose.pose)

        # Set covariance
        initial_pose.pose.covariance = pose_data['pose']['covariance']

        # Publish the initial pose
        rospy.loginfo("Publishing initial pose...")
        for _ in range(10):  # Publish multiple times to ensure the message is received
            self.initial_pose_pub.publish(initial_pose)
            rospy.sleep(0.1)

        # Stop the robot 20x0.1seconds
        twist = Twist()
        twist.linear.x = 0.0
        for _ in range(20): 
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

        rospy.loginfo("Localization complete.")
        return 'localized'

class NavigateToBeforeGantry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_at_before_gantry', 'navigation_failed'])
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size=10)
        self.sub_moving_complete = rospy.Subscriber('/control/moving/complete', UInt8, self.moving_complete_callback, queue_size=1)
        self.is_moving_complete = False

        # Define movement configurations for parking positions 
        self.movement_config = {
            1: [
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.5},
                {'type': TypeOfMoving.right.value, 'angular': 90, 'linear': 0},
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.25},
            ],
            2: [
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.5},
                {'type': TypeOfMoving.right.value, 'angular': 90, 'linear': 0},
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.5},
            ],
            3: [
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.22},
                {'type': TypeOfMoving.left.value, 'angular': 90, 'linear': 0},
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.40},
            ],
            4: [
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.22},
                {'type': TypeOfMoving.left.value, 'angular': 90, 'linear': 0},
                {'type': TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.19},
            ],
        }

    def moving_complete_callback(self, data):
        """Callback to update moving completion status."""
        self.is_moving_complete = True

    def wait_for_completion(self):
        """Wait until the robot completes the current movement."""
        rate = rospy.Rate(10)  # 10 Hz
        timeout = rospy.Time.now() + rospy.Duration(15)  # 15-second timeout
        while not rospy.is_shutdown():
            if self.is_moving_complete:
                self.is_moving_complete = False
                return True
            if rospy.Time.now() > timeout:
                rospy.logwarn("Movement timed out!")
                return False
            rate.sleep()

    def execute(self, userdata):
        rospy.loginfo("Navigating to before gantry using Robotis moving package...")

        # Retrieve the start position
        position_param = rospy.get_param('~start_position', 1)  # Defaults to parking_1 if not set
        movements = self.movement_config.get(position_param)

        if not movements:
            rospy.logerr(f"No movement configuration found for parking position {position_param}")
            return 'navigation_failed'

        # Execute movements sequentially
        for idx, move in enumerate(movements):
            rospy.loginfo(f"Step {idx + 1}: Executing movement {move}")
            msg_moving = MovingParam()
            msg_moving.moving_type = move['type']
            msg_moving.moving_value_angular = move['angular']
            msg_moving.moving_value_linear = move['linear']
            self.pub_moving.publish(msg_moving)

            if not self.wait_for_completion():
                rospy.logerr(f"Failed at step {idx + 1} for parking position {position_param}")
                return 'navigation_failed'

            rospy.sleep(0.5)  # Optional pause between movements

        rospy.loginfo("Successfully navigated to before gantry.")
        return 'arrived_at_before_gantry'

class HandleNavigationFailure(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retry_navigation', 'abort_task'])

    def execute(self, userdata):
        rospy.logwarn("Navigation failed! Deciding next steps...")
        # Logic to decide whether to retry or abort
        retry = True  # Placeholder decision
        if retry:
            rospy.loginfo("Retrying navigation...")
            return 'retry_navigation'
        else:
            rospy.logerr("Aborting the task due to navigation failure.")
            return 'abort_task'

class HandleNavigationFailure(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retry_navigation', 'abort_task'])

    def execute(self, userdata):
        rospy.logwarn("Navigation failed! Deciding next steps...")
        # Logic to decide whether to retry or abort
        retry = True  # Placeholder decision
        if retry:
            rospy.loginfo("Retrying navigation...")
            return 'retry_navigation'
        else:
            rospy.logerr("Aborting the task due to navigation failure.")
            return 'abort_task'

class GantryInteractionExit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['proceed_to_direction_detection'])
        rospy.sleep(1)
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size=10)
        self.sub_moving_complete = rospy.Subscriber('/control/moving/complete', UInt8, self.moving_complete_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=1)
        self.is_moving_complete = False

        # Arduino device IP address for gantry control
        self.arduino_ip = "10.183.111.239"

    def moving_complete_callback(self, data):
        """Callback to update moving completion status."""
        self.is_moving_complete = True

    def wait_for_completion(self):
        """Wait until the robot completes the current movement."""
        rate = rospy.Rate(10)  # 10 Hz
        timeout = rospy.Time.now() + rospy.Duration(15)  # 15-second timeout
        while not rospy.is_shutdown():
            print("is_moving_complete:",  self.is_moving_complete)
            if self.is_moving_complete:
                self.is_moving_complete = False
                return True
            if rospy.Time.now() > timeout:
                rospy.logwarn("Movement timed out!")
                return False
            rate.sleep()

    def send_command(self, command):
        """Send HTTP command to Arduino for gantry control while maintaining zero velocity."""
        url = f"http://{self.arduino_ip}/{command}"
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        rospy.loginfo("Sending command to Arduino and maintaining zero velocity.")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            try:
                # Send the HTTP request (with a short timeout)
                response = requests.get(url, timeout=0.5)
                if response.status_code == 200:
                    rospy.loginfo(f"Gantry command '{command}' successful: {response.text}")
                    return  # Exit after a successful response
                else:
                    rospy.logerr(f"Gantry command '{command}' failed with status code: {response.status_code}")
            except requests.exceptions.RequestException:
                # Log the error but continue trying
                rospy.logwarn("Error or timeout while sending command. Retrying...")

            # Publish zero velocity to prevent timeout in `twist_mux`
            self.cmd_vel_pub.publish(twist)

            # Exit if the timeout is exceeded
            if rospy.Time.now() - start_time > rospy.Duration(5):
                rospy.logerr("Timeout waiting for Arduino response.")
                break

            rate.sleep()

    def execute(self, userdata):
        rospy.loginfo("Executing GantryInteractionExit...")
        self.is_moving_complete = False

        # Step 1: Open the gantry
        rospy.loginfo("Sending command to open the gantry...")
        #self.send_command("open")

         # Wait for the gantry to open (adjust if needed)
        # Stop the robot 20x0.1seconds
        # twist = Twist()
        # twist.linear.x = 0.0
        # for _ in range(10): 
        #     self.cmd_vel_pub.publish(twist)
        #     rospy.sleep(0.1)
        # rospy.sleep(0.2)
        
        # Step 2: Move forward through the gantry
        rospy.loginfo("Moving forward through the gantry.")
        msg_moving = MovingParam()
        msg_moving.moving_type = TypeOfMoving.forward.value
        msg_moving.moving_value_angular = 0
        msg_moving.moving_value_linear = 0.18  # Move forward 50 cm
        self.pub_moving.publish(msg_moving)
        if not self.wait_for_completion():
            rospy.logerr("Failed to move forward through the gantry.")
            return 'proceed_to_lane'

        rospy.sleep(0.5)  # Optional pause after movement

        # Step 3: Close the gantry
        rospy.loginfo("Sending command to close the gantry...")
        #self.send_command("close")
        # Wait for the gantry to close (adjust if needed)
        # Stop the robot 20x0.1seconds
        # twist = Twist()
        # twist.linear.x = 0.0
        # for _ in range(10): 
        #     self.cmd_vel_pub.publish(twist)
        #     rospy.sleep(0.1)

        rospy.loginfo("Exiting gantry. Proceeding to direction detection.")
        return 'proceed_to_direction_detection'

class DirectionDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_completed'], output_keys=['turn_direction_out'])
        self.sign_detected = None
        rospy.Subscriber('/cv_node/direction_detected', String, self.sign_callback, queue_size=1)
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size=10)
        self.sub_moving_complete = rospy.Subscriber('/control/moving/complete', UInt8, self.moving_complete_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)
        self.is_moving_complete = False

    def moving_complete_callback(self, data):
        self.is_moving_complete = True

    def wait_for_completion(self):
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(15)
        while not rospy.is_shutdown():
            if self.is_moving_complete:
                self.is_moving_complete = False
                return True
            if rospy.Time.now() > timeout:
                rospy.logwarn("Movement timed out!")
                return False
            rate.sleep()

    def sign_callback(self, msg):
        self.sign_detected = msg.data  # E.g., "turn_left" or "turn_right"

    def execute(self, userdata):
        self.is_moving_complete = False
        rospy.loginfo("Executing state: DirectionDetection")
        rate = rospy.Rate(10)
        # Stop the robot 20x0.1seconds
        twist = Twist()
        twist.linear.x = 0.0
        for _ in range(20): 
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

        # Wait until a sign is detected
        rospy.loginfo("Waiting for direction detection...")
        while not rospy.is_shutdown() and not self.sign_detected:
            # Stop the robot 20x0.1seconds
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)

        rospy.loginfo(f"Direction detected: {self.sign_detected}")

        if self.sign_detected == "left":
            userdata.turn_direction_out = "left"
            rospy.loginfo("Executing left turn sequence.")

            # Step 1: Move forward
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.forward.value
            msg_moving.moving_value_angular = 0
            msg_moving.moving_value_linear = 0.15
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'turn_completed'

            rospy.sleep(0.5)

            # Step 2: Turn left
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.left.value
            msg_moving.moving_value_angular = 90
            msg_moving.moving_value_linear = 0
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'turn_completed'

            rospy.sleep(0.5)

            # Step 3: Move forward
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.forward.value
            msg_moving.moving_value_angular = 0
            msg_moving.moving_value_linear = 0.25
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'turn_completed'

        elif self.sign_detected == "right":
            userdata.turn_direction_out = "right"
            rospy.loginfo("Executing right turn sequence.")

            # Step 1: Move forward
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.forward.value
            msg_moving.moving_value_angular = 0
            msg_moving.moving_value_linear = 0.4
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'turn_completed'

            rospy.sleep(0.5)

            # Step 2: Turn right
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.right.value
            msg_moving.moving_value_angular = 90
            msg_moving.moving_value_linear = 0
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'turn_completed'

            rospy.sleep(0.5)

            # Step 3: Move forward
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.forward.value
            msg_moving.moving_value_angular = 0
            msg_moving.moving_value_linear = 0.25
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'turn_completed'

        rospy.loginfo("Turn completed.")
        return 'turn_completed'

class LaneFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['car_detected', 'pedestrian_detected', 'roundabout_detected', 'triangle_detected', 
                                             'stop_sign_detected', 'completed_track'],
                             input_keys=['turn_direction'])  # Receives 'left' or 'right'

        # Flags for detected events
        self.car_detected = False
        self.pedestrian_detected = False
        self.stop_sign_detected = False
        self.stop_sign_last_handled_time = rospy.Time(0)  # Initialize to time 0
        self.stop_sign_cooldown = rospy.Duration(10)  # Cool-down period (10 seconds)
        self.current_pose = None

        # Throttle pose updates
        self.last_pose_update_time = rospy.Time.now()
        self.pose_update_interval = rospy.Duration(0.5)  # Throttle to 2 Hz

        # Subscribers
        rospy.Subscriber('/cv_node/car_detected', String, self.car_callback, queue_size=1)
        rospy.Subscriber('/cv_node/pedestrian_detected', String, self.pedestrian_callback, queue_size=1)
        rospy.Subscriber('/cv_node/stop_sign_detected', String, self.sign_callback, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Define locations for both directions
        self.locations_left = {
            'car_park': {'x': 6.0, 'y': 2.0, 'radius': 0.5},
            'roundabout': {'x': 3.0, 'y': 4.0, 'radius': 0.5},
            'triangle': {'x': -0.12997154959514903, 'y': -1.7598585083217464, 'radius': 0.1},
        }

        self.locations_right = {
            'car_park': {'x': 2.0, 'y': 6.0, 'radius': 0.5},
            'roundabout': {'x': 4.0, 'y': 3.0, 'radius': 0.5},
            'triangle': {'x': 3.5, 'y': 4.0, 'radius': 0.5},
        }

    def car_callback(self, msg):
        if msg.data == "car":
            self.car_detected = True

    def pedestrian_callback(self, msg):
        if msg.data == "pedestrian":
            self.pedestrian_detected = True

    def sign_callback(self, msg):
        if msg.data == "stop_sign":
            self.stop_sign_detected = True

    def pose_callback(self, msg):
        # Throttle pose updates
        current_time = rospy.Time.now()
        if (current_time - self.last_pose_update_time) > self.pose_update_interval:
            self.current_pose = msg.pose.pose
            self.last_pose_update_time = current_time

    def check_proximity(self, location):
        """Check if the robot is near a specific location."""
        if self.current_pose is None:
            return False
        return is_near_location(self.current_pose, location)

    def execute(self, userdata):
        rospy.loginfo("Executing state: LaneFollowing")
        rate = rospy.Rate(10)

        # Get the direction from the previous state
        turn_direction = userdata.turn_direction
        rospy.loginfo(f"Turn direction received: {turn_direction}")

        # Use appropriate locations based on the turn direction
        if turn_direction == "left":
            locations = self.locations_left
        elif turn_direction == "right":
            locations = self.locations_right
        else:
            rospy.logerr("Invalid turn direction received!")
            return 'completed_track'

        while not rospy.is_shutdown():
            # Check for detections
            if self.car_detected:
                rospy.loginfo("Car detected! Transitioning to ObstacleDetection.")
                self.car_detected = False
                return 'car_detected'

            if self.pedestrian_detected:
                rospy.loginfo("Pedestrian detected! Transitioning to StopForPedestrian.")
                self.pedestrian_detected = False
                return 'pedestrian_detected'

            # Stop Sign Handling
            current_time = rospy.Time.now()
            if self.stop_sign_detected:
                if (current_time - self.stop_sign_last_handled_time) > self.stop_sign_cooldown:
                    rospy.loginfo("Stop sign detected! Transitioning to StopSignHandling.")
                    self.stop_sign_last_handled_time = current_time  # Update the handled time
                    self.stop_sign_detected = False
                    return 'stop_sign_detected'
                else:
                    rospy.loginfo("Ignoring stop sign due to cool-down period.")

            # Check proximity to predefined locations
            if self.check_proximity(locations['roundabout']):
                rospy.loginfo("Approaching roundabout.")
                return 'roundabout_detected'

            if self.check_proximity(locations['triangle']):
                rospy.loginfo("Approaching triangle.")
                return 'triangle_detected'

            if self.check_proximity(locations['car_park']):
                rospy.loginfo("Track completed. Transitioning to ReturnToCarPark.")
                return 'completed_track'

            rate.sleep()

        rospy.loginfo("Completed lane following.")
        return 'completed_track'

class StopSignHandling(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_lane_following'])
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Executing state: StopSignHandling")
        
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Robot stopped at stop sign.")

        # Wait for 3 seconds (or as needed)
        # Stop the robot 30x0.1seconds
        twist = Twist()
        twist.linear.x = 0.0
        for _ in range(40): 
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

        # Resume lane following
        rospy.loginfo("Resuming lane following.")
        return 'continue_lane_following'

# Define state StopForPedestrian
class StopForPedestrian(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_lane_following'])
        self.pedestrian_detected = False
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/cv_node/pedestrian_detected', String, self.pedestrian_callback)

    def pedestrian_callback(self, msg):
        # Update pedestrian detection status
        if msg.data == "pedestrian":
            self.pedestrian_detected = True
        else:
            self.pedestrian_detected = False

    def execute(self, userdata):
        rospy.loginfo("Executing state: StopForPedestrian")

        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Robot stopped for pedestrian.")

        # Wait until the pedestrian is no longer detected
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            if not self.pedestrian_detected:
                rospy.loginfo("Pedestrian has crossed. Resuming lane following.")
                return 'continue_lane_following'
            rospy.loginfo("Waiting for pedestrian to cross...")
            rate.sleep()

# Define state ObstacleDetection
class ObstacleDetection(smach.State):
    def __init__(self):
        #smach.State.__init__(self, outcomes=['overtake', 'proceed'])
        smach.State.__init__(self, outcomes=['continue_lane_following'])
        self.obstacle_detected = False

        rospy.Subscriber('/cv_node/car_detected', String, self.obstacle_callback)
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)

    def obstacle_callback(self, msg):
        if msg.data == "car":
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def execute(self, userdata):
        rospy.loginfo("Executing state: ObstacleDetection")
        #rospy.sleep(2)

        # if self.obstacle_detected:
        #     rospy.loginfo("Obstacle detected! Initiating overtaking.")
        #     self.obstacle_detected = False  # Reset flag
        #     return 'overtake'
        # else:
        #     rospy.loginfo("No obstacles detected. Proceeding.")
        #     return 'proceed'
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Robot stopped for car.")

        # Wait until the pedestrian is no longer detected
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            if not self.obstacle_detected:
                rospy.loginfo("Car is gone. Resuming lane following.")
                return 'continue_lane_following'
            rospy.loginfo("Waiting for car...")
            rate.sleep()

# Define state Overtaking
class Overtaking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_lane_following'])
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Executing state: Overtaking")
        twist = Twist()

        # Turn left to overtake
        rospy.loginfo("Turning left to overtake...")
        twist.angular.z = -0.5  # Simulate turning right
        twist.linear.x = 0.3
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(3)

        # Return to lane
        rospy.loginfo("Returning to lane...")
        twist.angular.z = 0.5  # Simulate turning back to lane
        twist.linear.x = 0.3
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(3)

        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Overtaking complete.")
        return 'continue_lane_following'


# Define state RoundaboutNavigation
class RoundaboutNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['roundabout_complete'],
                             input_keys=['turn_direction'])  # Receives 'left' or 'right'
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size=10)
        self.sub_moving_complete = rospy.Subscriber('/control/moving/complete', UInt8, self.moving_complete_callback, queue_size=1)
        self.is_moving_complete = False

    def moving_complete_callback(self, data):
        """Callback to update moving completion status."""
        self.is_moving_complete = True

    def wait_for_completion(self):
        """Wait until the robot completes the current movement."""
        rate = rospy.Rate(10)  # 10 Hz
        timeout = rospy.Time.now() + rospy.Duration(15)  # 15-second timeout
        while not rospy.is_shutdown():
            if self.is_moving_complete:
                self.is_moving_complete = False
                return True
            if rospy.Time.now() > timeout:
                rospy.logwarn("Movement timed out!")
                return False
            rate.sleep()

    def execute(self, userdata):
        rospy.loginfo("Executing state: RoundaboutNavigation")

        # Get the turn direction from userdata
        turn_direction = userdata.turn_direction
        rospy.loginfo(f"Received turn direction: {turn_direction}")

        if turn_direction == "left":
            # Movement for left direction
            rospy.loginfo("Navigating roundabout for left turn.")
            # Move forward in the roundabout for left turn
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.forward.value
            msg_moving.moving_value_angular = 0
            msg_moving.moving_value_linear = 0.5  # Adjust distance for left roundabout
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'roundabout_complete'

            rospy.sleep(1)

        elif turn_direction == "right":
            # Movement for right direction
            rospy.loginfo("Navigating roundabout for right turn.")
            # Move forward in the roundabout for right turn
            msg_moving = MovingParam()
            msg_moving.moving_type = TypeOfMoving.forward.value
            msg_moving.moving_value_linear = 0.7  # Adjust distance for right roundabout
            self.pub_moving.publish(msg_moving)
            if not self.wait_for_completion():
                return 'roundabout_complete'

            rospy.sleep(1)

        rospy.loginfo("Roundabout navigation complete.")
        return 'roundabout_complete'

    
# Define state RoundaboutNavigation
class TriangleNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['triangle_complete'])
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Executing state: TriangleNavigation")

        # # Navigate through the roundabout
        # twist = Twist()
        # twist.linear.x = 0.2
        # twist.angular.z = 0.4  # Simulate navigating a curve
        # rospy.loginfo("Navigating through the triangle...")
        # self.cmd_vel_pub.publish(twist)
        # rospy.sleep(5)

        # # Stop the robot
        # self.cmd_vel_pub.publish(Twist())

        # twist = Twist()
        # twist.linear.x = 0.0
        # for _ in range(40): 
        #     self.cmd_vel_pub.publish(twist)
        #     rospy.sleep(0.1)
        # rospy.loginfo("Exited the triangle.")
        return 'triangle_complete'

# Define state ReturnToCarPark
class ReturnToCarPark(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_at_parking', 'navigation_failed'])
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.navigation_status = None

    def status_callback(self, msg):
        # Check the latest goal status
        if len(msg.status_list) > 0:
            self.navigation_status = msg.status_list[-1].status

    def execute(self, userdata):
        rospy.loginfo("Navigating back to the car park using move_base...")

        # Define the car park location (modify as needed)
        car_park_pose = PoseStamped()
        car_park_pose.header.frame_id = "map"  # Use the map frame
        car_park_pose.header.stamp = rospy.Time.now()
        car_park_pose.pose.position.x = 6.0  # Example car park x-coordinate
        car_park_pose.pose.position.y = 2.0  # Example car park y-coordinate
        car_park_pose.pose.orientation.w = 1.0  # Facing forward

        # Publish the goal
        rospy.loginfo("Sending goal to move_base...")
        self.goal_pub.publish(car_park_pose)

        # Wait for the goal to be reached or failed
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.navigation_status == 3:  # Goal reached
                rospy.loginfo("Successfully reached the car park gantry.")
                return 'arrived_at_parking'
            elif self.navigation_status in [4, 5]:  # Goal aborted or rejected
                rospy.logwarn("Navigation to the car park failed.")
                return 'navigation_failed'

            rate.sleep()

class GantryInteractionEnter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['park_car'])
        self.gantry_pub = rospy.Publisher('/gantry_control', String, queue_size=10)
        self.gantry_status_sub = rospy.Subscriber('/gantry_status', String, self.status_callback)
        self.cmd_vel_pub = rospy.Publisher('/smach_cmd_vel', Twist, queue_size=10)
        self.gantry_opened = False

    def status_callback(self, msg):
        if msg.data == "opened":
            self.gantry_opened = True

    def execute(self, userdata):
        rospy.loginfo("Executing GantryInteractionEnter...")

        # Signal to open the gantry
        self.gantry_pub.publish("open")
        rate = rospy.Rate(10)

        # Wait for the gantry to open
        while not self.gantry_opened:
            rospy.loginfo("Waiting for gantry to open...")
            rate.sleep()

        rospy.loginfo("Gantry open. Proceeding through.")

        # Move forward through the gantry
        twist = Twist()
        twist.linear.x = 0.3
        for _ in range(30):  # Move forward for 3 seconds
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

        # Stop the robot
        self.cmd_vel_pub.publish(Twist())

        # Signal to close the gantry
        rospy.loginfo("Signaling gantry to close.")
        self.gantry_pub.publish("close")
        rospy.sleep(2)  # Simulate gantry closing

        rospy.loginfo("Entering car park. Proceeding to park the car.")
        return 'park_car'

# Define state ParkInSlot
class ParkInSlot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['task_complete'])

    def execute(self, userdata):
        rospy.loginfo("Parking in designated slot...")
        rospy.sleep(5)
        rospy.loginfo("Parking complete.")
        return 'task_complete'

# Main function
def main():
    rospy.init_node('smach_main')

    sm = smach.StateMachine(outcomes=['task_complete'])

    # Define userdata for the state machine
    sm.userdata.turn_direction = None  # Initialize the variable to store turn direction

    with sm:
        # Start state transitions to NavigateToGantry
        smach.StateMachine.add('StartState', StartState(), transitions={'proceed_to_localization': 'LocalizationState'})

        smach.StateMachine.add('LocalizationState', LocalizationState(),
                       transitions={'localized': 'NavigateToBeforeGantry',
                                    'failed': 'task_complete'})

        # Navigate to the gantry using move_base
        smach.StateMachine.add('NavigateToBeforeGantry', NavigateToBeforeGantry(),
                            transitions={'arrived_at_before_gantry': 'GantryInteractionExit',
                                            'navigation_failed': 'HandleNavigationFailure'})

        # Handle navigation failures
        smach.StateMachine.add('HandleNavigationFailure', HandleNavigationFailure(),
                            transitions={'retry_navigation': 'NavigateToBeforeGantry',
                                            'abort_task': 'task_complete'})

        # Interact with the gantry when exiting the car park
        smach.StateMachine.add('GantryInteractionExit', GantryInteractionExit(),
                            transitions={'proceed_to_direction_detection': 'DirectionDetection'})
        
        # Detect direction and turn after exiting the car park
        smach.StateMachine.add('DirectionDetection',
                           DirectionDetection(),
                           transitions={'turn_completed': 'LaneFollowing'},
                           remapping={'turn_direction_out': 'turn_direction'})  # Pass direction to LaneFollowing

        # LaneFollowing handles general navigation and transitions to other states based on events
        smach.StateMachine.add('LaneFollowing',
                           LaneFollowing(),
                           transitions={'car_detected': 'ObstacleDetection',
                                        'pedestrian_detected': 'StopForPedestrian',
                                        'stop_sign_detected': 'StopSignHandling',
                                        'roundabout_detected': 'RoundaboutNavigation',
                                        'triangle_detected': 'TriangleNavigation',
                                        'completed_track': 'ReturnToCarPark'},
                           remapping={'turn_direction': 'turn_direction'})  # Receive direction from DirectionDetection

        # Handle pedestrians crossing
        smach.StateMachine.add('StopForPedestrian', StopForPedestrian(),
                            transitions={'continue_lane_following': 'LaneFollowing'})

        # Handle obstacles and decide whether to overtake
        smach.StateMachine.add('ObstacleDetection', ObstacleDetection(),
                            # transitions={'overtake': 'Overtaking',
                            #                 'proceed': 'LaneFollowing'})
                            transitions={'continue_lane_following': 'LaneFollowing'})

        # Perform overtaking maneuver
        # smach.StateMachine.add('Overtaking', Overtaking(),
        #                     transitions={'continue_lane_following': 'LaneFollowing'})

        # Handle stop signs
        smach.StateMachine.add('StopSignHandling', StopSignHandling(),
                            transitions={'continue_lane_following': 'LaneFollowing'})

        # Navigate through the roundabout
        smach.StateMachine.add('RoundaboutNavigation',
                       RoundaboutNavigation(),
                       transitions={'roundabout_complete': 'LaneFollowing'},
                       remapping={'turn_direction': 'turn_direction'})  # Pass direction from previous state

        
        # Navigate through the triangle
        smach.StateMachine.add('TriangleNavigation', TriangleNavigation(),
                            transitions={'triangle_complete': 'LaneFollowing'})

        # Return to the car park
        smach.StateMachine.add('ReturnToCarPark', ReturnToCarPark(),
                            transitions={'arrived_at_parking': 'GantryInteractionEnter',
                                            'navigation_failed': 'HandleNavigationFailure'})

        # Interact with the gantry when entering the car park
        smach.StateMachine.add('GantryInteractionEnter', GantryInteractionEnter(),
                            transitions={'park_car': 'ParkInSlot'})

        # Park the robot in the car park
        smach.StateMachine.add('ParkInSlot', ParkInSlot(),
                            transitions={'task_complete': 'task_complete'})


    # Attach introspection server to visualize the state machine
    sis = smach_ros.IntrospectionServer('smach_viewer', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    # Stop the introspection server after execution
    sis.stop()

    rospy.spin()

if __name__ == '__main__':
    main()