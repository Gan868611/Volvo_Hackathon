#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import os

def compressed_image_publisher():
    # Initialize the ROS node
    rospy.init_node('compressed_image_publisher', anonymous=True)
    
    # Create a publisher for the topic /camera/image/compressed
    image_pub = rospy.Publisher('/camera/image_rect_color/compressed', CompressedImage, queue_size=10)
    
    # Set the publishing rate to 1 Hz
    rate = rospy.Rate(1)  # 1 Hz
    
    # Load the image file
    image_dir = "/home/volvo2/catkin_ws/src/lane_following/images/"  # Image directory
    image_prefix = "output_"
    image_extension = ".jpg"
    image_count = 84  # Number of images to loop through (000 to 124)

    image_index = 11  # Start at the first image

    while not rospy.is_shutdown():
        try:
            # Encode the OpenCV image as JPEG
            image_filename = f"{image_prefix}{image_index:03d}{image_extension}"
            image_path = os.path.join(image_dir, image_filename)
            
            # Load the image file
            cv_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logerr(f"Failed to load image: {image_path}")
                image_index = (image_index + 1) % image_count  # Move to the next image
                continue
            _, encoded_image = cv2.imencode('.jpg', cv_image)
            
            # Create a CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = rospy.Time.now()  # Add a timestamp
            compressed_msg.format = "jpeg"  # Specify the format as JPEG
            compressed_msg.data = encoded_image.tobytes()  # Add the compressed image data
            
            # Publish the CompressedImage message
            image_pub.publish(compressed_msg)
            
            rospy.loginfo("Published compressed image to /camera/image/compressed")
            image_index = (image_index + 1) % image_count  
            
            # Sleep to maintain the publishing rate
            rate.sleep()
        
        except rospy.ROSInterruptException:
            rospy.loginfo("Compressed image publisher node interrupted.")
            break

if __name__ == '__main__':
    try:
        compressed_image_publisher()
    except rospy.ROSInterruptException:
        pass
