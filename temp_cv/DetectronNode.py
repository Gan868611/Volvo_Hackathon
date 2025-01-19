#!/usr/bin/env python3

import rospy
import torch
import cv2
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
from detectron2 import model_zoo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Detectron2Node:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("detectron2_node", anonymous=True)

        # Set up subscribers and publishers
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.result_pub = rospy.Publisher("/cv_node/pedestrian_detected", String, queue_size=10)
        self.bridge = CvBridge()

        # Load Detectron2 configuration and model
        self.cfg = get_cfg()
        self.cfg.merge_from_file(
            model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        )
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(
            "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"
        )
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # Confidence threshold
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 80  # Number of classes in COCO dataset
        self.cfg.MODEL.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

        self.predictor = DefaultPredictor(self.cfg)
        rospy.loginfo("Detectron2 node initialized.")

    def image_callback(self, data):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # Run object detection
            outputs = self.predictor(cv_image)

            # Extract instances and filter by target classes (e.g., cars and pedestrians)
            instances = outputs["instances"].to("cpu")
            pred_classes = instances.pred_classes
            pred_boxes = instances.pred_boxes
            target_classes = [0, 2]  # COCO IDs for cars and pedestrians
            # target_indices = [
            #     i for i, cls in enumerate(pred_classes) if cls.item() in target_classes
            # ]
            # detected_objects = [
            #     f"Class: {pred_classes[i].item()}, Box: {pred_boxes[i].tensor.numpy()}"
            #     for i in target_indices
            # ]
            # Check if class index 2 exists
            if 0 in pred_classes.tolist():  # Convert tensor to list for the `in` check
                rospy.loginfo("Class index 0 is detected in the predictions.")
                self.result_pub.publish("pedestrian")
            else:
                rospy.loginfo("Class index 0 is not detected in the predictions.")
            

            # Visualize and show the results
            v = Visualizer(
                cv_image[:, :, ::-1],
                MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]),
                scale=1.2,
            )
            out = v.draw_instance_predictions(instances)
            result_image = out.get_image()[:, :, ::-1]

            # Display the detection result (optional)
            # cv2.imshow("Detections", result_image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = Detectron2Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
