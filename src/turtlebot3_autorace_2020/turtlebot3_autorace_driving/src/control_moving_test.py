#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from turtlebot3_autorace_msgs.msg import MovingParam
from enum import Enum
from std_msgs.msg import UInt8

class ControlMovingTestNode:
    def __init__(self):
    
        # Publisher for moving commands
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size=1)
        
        # Subscriber to monitor completion
        self.sub_moving_complete = rospy.Subscriber('/control/moving/complete', UInt8, self.moving_complete_callback, queue_size=1)

        self.is_moving_complete = False
        self.TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')

        # Movement sequence
        self.movements = [
            {'type': self.TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.22},
            {'type': self.TypeOfMoving.left.value, 'angular': 90, 'linear': 0},
            {'type': self.TypeOfMoving.forward.value, 'angular': 0, 'linear': 0.22}
        ]

        # self.execute_movements()

    def moving_complete_callback(self, msg):
        self.is_moving_complete = True

    def execute_movements(self):
        rospy.loginfo("Starting movement sequence...")
        rospy.sleep(1)
        # rospy.loginfo("exiting parking")

        for movement in self.movements:
            rospy.loginfo(f"Executing movement: {movement}")

            # Publish movement command
            msg = MovingParam()
            msg.moving_type = movement['type']
            msg.moving_value_angular = movement['angular']
            msg.moving_value_linear = movement['linear']
            self.pub_moving.publish(msg)

            while True:
                rospy.loginfo("moving")
                if self.is_moving_complete == True:
                    break
            self.is_moving_complete = False


            rospy.sleep(1)


            # Wait for completion
            # self.wait_for_completion()

        rospy.loginfo("Movement sequence complete.")

    def wait_for_completion(self):
        rate = rospy.Rate(10)  # 10 Hz
        timeout = rospy.Time.now() + rospy.Duration(15)  # 15 seconds timeout

        while not rospy.is_shutdown():
            if self.is_moving_complete:
                self.is_moving_complete = False
                rospy.loginfo("Movement completed.")
                return

            if rospy.Time.now() > timeout:
                rospy.logwarn("Movement timed out.")
                return

            rate.sleep()
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_moving_test_node')
    try:
        node = ControlMovingTestNode()
        node.execute_movements()
        node.main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ControlMovingTestNode terminated.")
    