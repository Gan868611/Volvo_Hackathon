#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ContrastEnhancerNode:
    def __init__(self):
        rospy.init_node("contrast_enhancer", anonymous=True)
        
        # Updated topic names
        self.image_sub = rospy.Subscriber("/camera/image_projected_compensated", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/contrast_image_projected_compensated", Image, queue_size=10)
        
        self.bridge = CvBridge()

    def adjust_hsv_ranges(self, hsv):
        """
        Dynamically adjust HSV ranges for yellow, blue, and white colors based on brightness.
        """
        # Calculate average brightness from the V channel
        brightness = np.mean(hsv[:, :, 2])

        # Base thresholds
        base_yellow_lower = np.array([15, 40, 40])  # Relaxed lower bounds
        base_yellow_upper = np.array([40, 255, 255])

        base_blue_lower = np.array([85, 40, 40])
        base_blue_upper = np.array([140, 255, 255])

        base_white_lower = np.array([0, 0, 200])
        base_white_upper = np.array([180, 30, 255])

        # print("brightness:",brightness )

        # Adjust based on brightness
        if brightness < 90:  # Dark environment
            delta_s, delta_v = 40, 40
        elif brightness > 200:  # Bright environment
            delta_s, delta_v = -20, -20
        else:  # Normal lighting
            delta_s, delta_v = 0, 0

        # Adjust thresholds
        yellow_lower = base_yellow_lower + np.array([0, delta_s, delta_v])
        yellow_upper = base_yellow_upper + np.array([0, 0, delta_v])

        blue_lower = base_blue_lower + np.array([0, delta_s, delta_v])
        blue_upper = base_blue_upper

        white_lower = base_white_lower + np.array([0, 0, delta_v])
        white_upper = base_white_upper

        # Clip to valid range
        yellow_lower = np.clip(yellow_lower, 0, 255)
        yellow_upper = np.clip(yellow_upper, 0, 255)
        blue_lower = np.clip(blue_lower, 0, 255)
        blue_upper = np.clip(blue_upper, 0, 255)
        white_lower = np.clip(white_lower, 0, 255)
        white_upper = np.clip(white_upper, 0, 255)

        return (yellow_lower, yellow_upper), (blue_lower, blue_upper), (white_lower, white_upper)

    def enhance_contrast(self, image):
        """
        Enhance contrast for yellow, blue, and white colors in the image.
        """
        # Convert the image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Apply Gaussian Blur to smooth the image
        hsv_blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

        # Get dynamically adjusted HSV ranges
        (yellow_lower, yellow_upper), (blue_lower, blue_upper), (white_lower, white_upper) = self.adjust_hsv_ranges(hsv_blurred)

        # Create masks for each color
        yellow_mask = cv2.inRange(hsv_blurred, yellow_lower, yellow_upper)
        blue_mask = cv2.inRange(hsv_blurred, blue_lower, blue_upper)
        white_mask = cv2.inRange(hsv_blurred, white_lower, white_upper)

        # Apply morphological operations to refine the yellow mask
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=1)

        # Combine masks
        combined_mask = cv2.bitwise_or(yellow_mask, blue_mask)
        combined_mask = cv2.bitwise_or(combined_mask, white_mask)

        # Apply the mask to the original image
        masked_image = cv2.bitwise_and(image, image, mask=combined_mask)

        return masked_image

    def image_callback(self, msg):
        """
        Callback function for processing images.
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Enhance contrast
            enhanced_image = self.enhance_contrast(cv_image)
            
            # Convert back to ROS Image message and publish
            enhanced_msg = self.bridge.cv2_to_imgmsg(enhanced_image, "bgr8")
            self.image_pub.publish(enhanced_msg)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = ContrastEnhancerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
