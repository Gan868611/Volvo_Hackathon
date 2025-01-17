#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
from torchvision import transforms, models
from PIL import Image as PILImage
from io import BytesIO
import numpy as np
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

# ROS node for image classification
class DirectionClassifierNode:
    def __init__(self):
        # Load the trained model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('volvo')  # Replace with your package name
        model_path = os.path.join(package_path, 'resnet18_image_classification_3.pth')
        
        num_classes = 2 
        self.model = models.resnet18(pretrained=False)
        self.model.fc = torch.nn.Linear(self.model.fc.in_features, num_classes)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model = self.model.to(self.device)
        self.model.eval()

        # Define the transform for input images
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),  # Resize to match ResNet input
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Standard normalization
        ])

        self.class_names = ['Left', 'Right'] 

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Set up ROS topics
        self.image_sub = rospy.Subscriber("/camera/rgb/image_throttled", Image, self.image_callback)
        self.result_pub = rospy.Publisher("/direction_result", String, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert OpenCV image to PIL format
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            cv2.imshow("image",cv_image)
            cv2.waitKey(1)
            # Perform inference
            result = self.classify_image(pil_image)
            # Publish result
            self.result_pub.publish(result)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Unhandled exception in image callback: {e}")

    def classify_image(self, pil_image):
        # Preprocess the image
        input_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)

        # Make predictions
        with torch.no_grad():
            output = self.model(input_tensor)
            probabilities = torch.nn.functional.softmax(output[0], dim=0)
            predicted_class = torch.argmax(probabilities).item()

        # Map predicted class to class name
        predicted_class_name = self.class_names[predicted_class]
        rospy.loginfo(f"Predicted Class: {predicted_class_name}")
        rospy.loginfo(f"Class Probabilities: {probabilities}")
        return predicted_class_name

if __name__ == "__main__":
    rospy.init_node("image_classifier_node", anonymous=True)
    classifier_node = DirectionClassifierNode()
    rospy.loginfo("Direction classifier node is running...")
    rospy.spin()
