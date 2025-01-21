#!/usr/bin/env python3

import rospy
import math
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Bool, String
from geometry_msgs.msg import Twist
from turtlebot3_autorace_msgs.msg import MovingParam
from std_msgs.msg import String

class StuckHandlerNode:
    def __init__(self):
        rospy.init_node('stuck_handler')

        # Constants for moving types
        self.MOVING_TYPE_IDLE = 1
        self.MOVING_TYPE_LEFT = 2
        self.MOVING_TYPE_RIGHT = 3
        self.MOVING_TYPE_FORWARD = 4
        self.MOVING_TYPE_BACKWARD = 5

        # Publisher for unstuck movements
        self.unstuck_pub = rospy.Publisher('/control/moving/state', MovingParam, queue_size=10)
        self.right_park_pub = rospy.Publisher('/parking_right_start', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/lane_det_started', Bool, self.lane_detection_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/control/moving/complete', UInt8, self.moving_complete_callback, queue_size=1)
        rospy.Subscriber('/starting_direction', String, self.direction_callback)

        self.is_moving_complete = False
        self.check_stuck_enabled = False  # Flag to enable or disable stuck checking
        self.start_processing = False  # Flag to start processing after lane detection

        # Tracking movement and states
        self.last_move_time = rospy.Time.now()
        self.is_stuck = False
        self.current_state = 1  # Start at state 1
        self.initial_position = None
        self.initial_orientation = None
        self.linear_accumulated = 0
        self.angular_accumulated = 0
        self.pending_movements = []  # Queue to handle multi-step movements
        self.published = False
        self.odom_count = 0
        self.executing = False
        self.starting_direction = None

        # Stuck detection threshold
        self.STUCK_DURATION = rospy.Duration(3)

        # Timer to check stuck condition
        rospy.Timer(rospy.Duration(0.5), self.check_stuck_status)
        rospy.Timer(rospy.Duration(0.4), self.execute_pending_movement)

        rospy.loginfo("Stuck Handling Node Initialized. Waiting for lane detection to start...")

    def lane_detection_callback(self, msg):
        """Callback function for lane detection start signal."""
        if msg.data:  # Check if the message indicates the lane detection has started
            if self.starting_direction is not None:
                if not self.start_processing:
                    rospy.loginfo("Lane detection started, initializing node...")
                    time.sleep(20)
                    self.start_processing = True
                rospy.loginfo("Stuck checking resumed.")
                self.check_stuck_enabled = True
                self.last_move_time = rospy.Time.now()
            else:
                rospy.loginfo("No starting direction.")
        elif not msg.data:
            rospy.loginfo("Stuck checking paused.")
            self.check_stuck_enabled = False

    def odom_callback(self, msg):
        """Track movement based on odometry data."""
        if not self.start_processing:
            return

        self.odom_count += 1
        if self.odom_count % 2 == 0:
            self.odom_count = 0
            return
    
        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position
            self.initial_orientation = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            return

        # Compute linear displacement
        dx = msg.pose.pose.position.x - self.initial_position.x
        dy = msg.pose.pose.position.y - self.initial_position.y
        self.linear_accumulated = math.sqrt(dx**2 + dy**2)

        # Compute angular displacement
        current_orientation = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.angular_accumulated = current_orientation - self.initial_orientation
        delta_angle = current_orientation - self.initial_orientation
        self.angular_accumulated = math.atan2(math.sin(delta_angle), math.cos(delta_angle))
    
        rospy.loginfo(f"Current State: {self.current_state}, Linear: {self.linear_accumulated}, Angular: {self.angular_accumulated}")

        self.determine_state()

    def cmd_vel_callback(self, msg):
        """Track movement but don't check stuck here."""
        current_time = rospy.Time.now()
        self.last_move_time = current_time

    def direction_callback(self, msg):
        """Get starting direction"""
        self.starting_direction = msg.data
        self.current_state = 1

    def get_yaw_from_quaternion(self, orientation):
        """Convert quaternion to yaw (angle)."""
        import tf.transformations
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]  # Yaw angle (rotation around Z-axis)

    def check_stuck_status(self, event):
        """Check if the robot is stuck periodically."""
        if not self.start_processing or not self.check_stuck_enabled:
            return

        if not self.executing: # don't check if it is stuck during movement execution
            current_time = rospy.Time.now()
            if current_time - self.last_move_time > self.STUCK_DURATION:
                if not self.is_stuck:
                    rospy.loginfo("Robot detected as stuck.")
                self.is_stuck = True
                self.unstuck()
            else:
                self.is_stuck = False

    def moving_complete_callback(self, data):
        """Callback to update moving completion status."""
        self.is_moving_complete = True

    def determine_state(self):
        """Sequentially determine the robot's current state based on accumulated movements."""
        if self.check_stuck_enabled == False:
            return
        
        if self.starting_direction == 'left':
            if self.current_state == 1 and self.angular_accumulated >= 2.65: # Until U-turn
                self.switch_to_next_state(2)

            elif self.current_state == 2 and self.angular_accumulated <= -1.5: # Until Triangle Sharp right turn
                self.switch_to_next_state(3)

            elif self.current_state == 3 and self.linear_accumulated >= 0.6: # Towards triangle exit
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.35))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 90, 0))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))
                self.switch_to_next_state(6)
            
            elif self.current_state == 6 and self.angular_accumulated >= 0.78:
                self.switch_to_next_state(4)

            elif self.current_state == 4 and self.linear_accumulated >= 0.3: # only start checking when robot is in roundabout
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.45))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 45, 0))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.5))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 55, 0))
                # if self.angular_accumulated <= 0: # robot failed to exit roundabout
                #     self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 90, 0))
                # elif self.angular_accumulated >= 1.57: # robot manage to exit roundabout
                self.switch_to_next_state(5)

            elif self.current_state == 5 and self.linear_accumulated >= 10: # after roundabout exit towards sharp right turn
                self.switch_to_next_state(None)  # All states achieved
                self.start_processing = False # Stop processing
                self.starting_direction = None

        elif self.starting_direction == 'right':
            if self.current_state == 1 and self.linear_accumulated >= 1.9: # after the twist and turns
                if self.angular_accumulated >= -1.6 and self.angular_accumulated <= -1.5:
                    self.switch_to_next_state(11)
            
            if self.current_state == 11 and self.linear_accumulated >= 1:
                if self.angular_accumulated >= -1.6 and self.angular_accumulated <= -1.5:
                    self.switch_to_next_state(2)

            elif self.current_state == 2 and self.angular_accumulated >= 0.78: # Until Triangle Sharp right turn
                self.switch_to_next_state(22)

            elif self.current_state == 22 and self.linear_accumulated >= 0.03:
                self.switch_to_next_state(4)
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.36))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_RIGHT, 50, 0))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.36))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 35, 0))

            # elif self.current_state == 3 and self.linear_accumulated >= 0.5: # Towards roundabout entry
            #     if self.angular_accumulated <= -0.17:
            #         self.switch_to_next_state(4)

            elif self.current_state == 4 and self.linear_accumulated >= 1.43: # robot in roundabout
                if self.angular_accumulated <= -3: # robot failed to exit roundabout
                    self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.4))
                    self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 40, 0))
                    self.switch_to_next_state(55)
                # elif self.angular_accumulated <= -2.35: # robot manage to exit roundabout
                #     self.switch_to_next_state(5)

            elif self.current_state == 55 and self.angular_accumulated >= 0.6: 
                self.switch_to_next_state(5)

            elif self.current_state == 5 and self.linear_accumulated >= 0.1: # exit roundabout
                self.switch_to_next_state(6)
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.5))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 90, 0))
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_FORWARD, 0, 0.25))

            elif self.current_state == 6 and self.angular_accumulated <= -1.3: #towards u turn exit
                self.switch_to_next_state(7)

            elif self.current_state == 7 and self.linear_accumulated >= 1.85: # towards parking
                self.right_park_pub.publish('start')
                self.switch_to_next_state(None)  # All states achieved
                self.start_processing = False # Stop processing
                self.starting_direction = None


    def switch_to_next_state(self, next_state):
        """Switch to the next state and reset accumulated distances."""
        rospy.loginfo(f"Switching from state {self.current_state} to state {next_state}, Linear: {self.linear_accumulated}, Angular: {self.angular_accumulated}")
        self.current_state = next_state
        self.linear_accumulated = 0
        self.angular_accumulated = 0
        self.initial_position = None
        self.initial_orientation = None

    def unstuck(self):
        """Publish unstuck movement based on the detected state in sequence."""
        if self.current_state is None:
            rospy.loginfo("All states achieved. No unstuck action required.")
            return
        
        if self.starting_direction == 'left':
            if self.current_state == 1:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))

            elif self.current_state == 2:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_RIGHT, 45, 0))

            elif self.current_state == 3:  # Triangle exit left turn
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_BACKWARD, 0, 0.05))

            elif self.current_state == 4:  # Inside roundabout
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))

            # elif self.current_state == 5:  # Until sharp right turn
                # self.pending_movements.append(self.create_movement(self.MOVING_TYPE_RIGHT, 30, 0))

        elif self.starting_direction == 'right':
            if self.current_state == 1:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))

            elif self.current_state == 2:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 60, 0))

            elif self.current_state == 3:  
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))

            elif self.current_state == 4:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))

            elif self.current_state == 5:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))

            elif self.current_state == 6:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_RIGHT, 20, 0))

            elif self.current_state == 7:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_LEFT, 10, 0))

            elif self.current_state == 8:
                self.pending_movements.append(self.create_movement(self.MOVING_TYPE_RIGHT, 10, 0))

    def create_movement(self, moving_type, angular, linear):
        """Helper function to create movement commands."""
        cmd = MovingParam()
        cmd.moving_type = moving_type
        cmd.moving_value_angular = angular
        cmd.moving_value_linear = linear
        return cmd
    
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
    
    def execute_pending_movement(self, event):

        if self.executing:
            return

        if not self.pending_movements:
            # no pending movement
            return

        rospy.loginfo("Starting correction")

        self.executing = True
        self.is_moving_complete = False

        # Execute movements sequentially
        for idx, move in enumerate(self.pending_movements):
            rospy.loginfo(f"Step {idx + 1}: Executing movement {move}")
            self.is_moving_complete = False
            self.unstuck_pub.publish(move)

            if not self.wait_for_completion():
                rospy.logerr(f"Failed at step {idx + 1}")
            self.is_moving_complete = False

            rospy.sleep(0.5)  # Optional pause between movements

        self.pending_movements.clear()

        rospy.loginfo("Successfully carry out correction")
        
        self.executing = False

if __name__ == '__main__':
    try:
        node = StuckHandlerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
