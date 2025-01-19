#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np
import cv2
import tensorflow_hub as hub

# Load the MobileNet Model (DeepLabV3)
class MobileNetLaneFollower:
    def __init__(self):
        self.model = hub.load("https://tfhub.dev/tensorflow/deeplabv3-mobilenet-v2/1")
        self.bridge = CvBridge()
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('mobile_net_lane_follower', anonymous=True)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform lane segmentation
            lane_mask = self.segment_lanes(cv_image)

            # Compute velocity commands
            self.follow_lane(lane_mask)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def segment_lanes(self, image):
        # Preprocess the image for MobileNet
        input_image = cv2.resize(image, (513, 513))
        input_tensor = tf.convert_to_tensor(input_image, dtype=tf.float32)
        input_tensor = tf.expand_dims(input_tensor, 0) / 255.0  # Normalize to [0, 1]

        # Perform segmentation
        output = self.model(input_tensor)
        mask = tf.argmax(output['logits'], axis=-1)
        mask = tf.squeeze(mask).numpy()

        # Resize the mask to the original image size
        lane_mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
        return lane_mask

    def follow_lane(self, lane_mask):
        # Find the lane center in the lower half of the mask
        height, width = lane_mask.shape
        lane_center_x = self.get_lane_center(lane_mask, width, height)

        # Compute offset from image center
        center_offset = (width // 2) - lane_center_x

        # Proportional control for steering
        angular_z = -0.005 * center_offset
        linear_x = 0.2  # Constant forward speed

        # Publish velocity commands
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.velocity_pub.publish(twist)

    def get_lane_center(self, lane_mask, width, height):
        # Average x-coordinates of white pixels in the lower half of the image
        lane_pixels = np.where(lane_mask[height // 2:, :] > 0)
        if len(lane_pixels[1]) > 0:
            return int(np.mean(lane_pixels[1]))
        else:
            return width // 2  # Default to center if no lane detected

if __name__ == '__main__':
    try:
        follower = MobileNetLaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
