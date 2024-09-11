import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YOLOv8Node:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_ros_node', anonymous=True)

        # Load the YOLOv8 model
        self.model = YOLO('/home/heven/Downloads/best.pt')  # Update with your model path

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to the /usb_cam/image_raw topic
        self.left_image_sub = message_filters.Subscriber('/cameraLeft/usb_cam1/image_raw', Image)
        self.right_image_sub = message_filters.Subscriber('/cameraRight/usb_cam2/image_raw', Image)
        
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        self.image_pub = rospy.Publisher('cone_result', Image, queue_size = 10)

    def callback(self, left_msg, right_msg):
        # Convert ROS Image message to OpenCV image
        left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        
        # Perform inference
        batch_images = [left_image, right_image]
        results = self.model(batch_images)
        
        # Store mask images for both left and right
        mask_images = []
        
        # Process each result
        for i, result in enumerate(results):
            if result and hasattr(result, 'masks') and result.masks is not None:
                masks = result.masks.data
                class_ids = list(result.boxes.cls.cpu().numpy())
                mask_img = np.zeros_like(batch_images[i])

                class_colors = [
                    (255, 0, 0),
                    (0, 255, 255)
                ]

                for j in range(len(masks)):
                    binary_mask = masks[j].cpu().numpy().astype(np.uint8) * 255
                    color = class_colors[int(class_ids[j])]
                    colored_mask = np.zeros_like(batch_images[i])
                    colored_mask[binary_mask > 0] = color
                    mask_img = cv2.addWeighted(mask_img, 1.0, colored_mask, 0.5, 0)

                mask_images.append(mask_img)
            else:
                rospy.logwarn("No segmentation masks found in YOLOv8 results.")

        # Concatenate mask images horizontally and convert to a ROS Image message, then publish
        if len(mask_images) == 2:
            combined_image = np.concatenate(mask_images, axis=1)
            mask_image_msg = self.bridge.cv2_to_imgmsg(combined_image, encoding='bgr8')
            self.combined_mask_pub.publish(mask_image_msg)
            cv2.imshow('YOLOv8 Detection Combined', combined_image)
            cv2.waitKey(1)
        
       

if __name__ == '__main__':
    try:
        yolo_node = YOLOv8Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
