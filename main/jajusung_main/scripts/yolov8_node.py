#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import message_filters
from threading import Thread, Lock
import time

class YOLOv8Node:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_ros_node', anonymous=True)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to the image topics
        self.left_image_sub = message_filters.Subscriber('/cameraLeft/usb_cam1/image_raw', Image, queue_size=1)
        self.right_image_sub = message_filters.Subscriber('/cameraRight/usb_cam2/image_raw', Image, queue_size=1)

        # Synchronize both image streams
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_image_sub, self.right_image_sub], queue_size=1, slop=0.05)
        self.ts.registerCallback(self.callback)

        # Publisher for the combined image
        self.publisher = rospy.Publisher('cone_result', Image, queue_size=1)

        # Shared variables for the latest images
        self.latest_images = None
        self.new_image_available = False
        self.lock = Lock()

        # Load the YOLO model
        self.model = YOLO('./best.pt')

        # Start the inference thread
        self.inference_thread = Thread(target=self.run_inference)
        self.inference_thread.start()

        rospy.loginfo("YOLOv8 Node initialized.")

    def callback(self, left_msg, right_msg):
        # Convert ROS Image messages to OpenCV images
        left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

        # Update the shared variable with the latest images
        with self.lock:
            self.latest_images = [left_image, right_image]
            self.new_image_available = True

    def run_inference(self):
        bridge = CvBridge()
        rospy.loginfo("Inference thread started.")

        while not rospy.is_shutdown():
            # Wait for a new image to be available
            if self.new_image_available:
                with self.lock:
                    batch_images = self.latest_images.copy()
                    self.new_image_available = False

                # Perform YOLO inference
                start_time = time.time()
                results = self.model(batch_images)
                inference_time = time.time() - start_time
                rospy.loginfo(f"YOLO inference time: {inference_time * 1000:.2f} ms")
                class_colors = [
                    (255, 0, 0),  # Class 0: Blue
                    (0, 255, 255)  # Class 1: Yellow
                ]

                # Process the YOLO results and generate mask images
                mask_images = []
                for i, result in enumerate(results):
                    if hasattr(result, 'masks') and result.masks is not None:
                        masks = result.masks.data.cpu().numpy()
                        class_ids = result.boxes.cls.cpu().numpy()
                        mask_img = np.zeros_like(batch_images[i])

                        for j in range(len(masks)):
                            class_id = int(class_ids[j])
                            # For the left image (i == 0), only process class 0 (Blue)
                            # For the right image (i == 1), only process class 1 (Yellow)
                            if (i == 0 and class_id == 0) or (i == 1 and class_id == 1):
                                binary_mask = (masks[j] * 255).astype(np.uint8)
                                color = class_colors[class_id]
                                colored_mask = np.zeros_like(batch_images[i])
                                colored_mask[binary_mask > 0] = color
                                # Overlay the colored mask onto the original mask image
                                mask_img = cv2.addWeighted(mask_img, 1.0, colored_mask, 0.5, 0)
                        mask_images.append(mask_img)
                    else:
                        # If no mask is found, create a blank mask
                        mask_images.append(np.zeros_like(batch_images[i]))

                # Combine the masks (concatenate horizontally)
                if len(mask_images) == 2:
                    combined_image = np.concatenate(mask_images, axis=1)

                    # Convert to ROS Image message
                    mask_image_msg = bridge.cv2_to_imgmsg(combined_image, encoding='bgr8')

                    # Publish the combined mask image
                    rospy.loginfo("Publishing combined mask image.")
                    self.publisher.publish(mask_image_msg)
            else:
                # No new image available, sleep briefly to avoid busy waiting
                rospy.sleep(0.005)  # Sleep for 5 milliseconds

    def shutdown(self):
        self.inference_thread.join()

if __name__ == '__main__':
    try:
        # Create and start the ROS node
        yolo_node = YOLOv8Node()

        rospy.spin()

        # Ensure the inference thread terminates when the main process is finished
        yolo_node.shutdown()

    except rospy.ROSInterruptException:
        pass
