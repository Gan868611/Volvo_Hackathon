#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class PedestrianSafetyNode:
    def __init__(self):
        rospy.init_node('pedestrian_safety_node')

        # Subscriber to detect pedestrians
        self.pedestrian_sub = rospy.Subscriber('/cv_node/pedestrian_detected', String, self.pedestrian_callback)
        
        # Publisher to the safety command velocity topic
        self.safety_cmd_vel_pub = rospy.Publisher('/safety_cmd_vel', Twist, queue_size=1)
        
        self.pedestrian_detected = False
        self.stop_duration = 2  # Time to stop when a pedestrian is detected (seconds)

    def pedestrian_callback(self, msg):
        if msg.data == "pedestrian":
            rospy.loginfo("Pedestrian detected! Publishing stop command.")
            self.pedestrian_detected = True
            self.publish_stop_cmd()

    def publish_stop_cmd(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        end_time = rospy.Time.now() + rospy.Duration(self.stop_duration)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.safety_cmd_vel_pub.publish(stop_cmd)
            rospy.sleep(0.1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PedestrianSafetyNode()
    node.run()
