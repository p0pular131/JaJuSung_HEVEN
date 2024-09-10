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
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self.image_pub = rospy.Publisher('cone_result', Image, queue_size = 10)

    def callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Perform inference
        results = self.model(cv_image)    

        # Check if results are not empty and handle the result
        if results:
            result = results[0]

            # Check if segmentation masks are available
            if hasattr(result, 'masks') and result.masks is not None:
                # Extract the masks from the result
                masks = result.masks.data
                class_ids = list(result.boxes.cls.cpu().numpy())
                mask_img = np.zeros_like(cv_image)

                class_colors = [
                    (255,0,0),
                    (0,255,255)
                ]

                # Iterate through all masks and color them according to their class
                for i in range(len(masks)):
                    binary_mask = masks[i].cpu().numpy().astype(np.uint8) * 255  # Convert mask to uint8 type
                    color = class_colors[int(class_ids[i])]  # Get the color for the current class

                    # Create a color mask using the binary mask and the class color
                    colored_mask = np.zeros_like(cv_image)
                    colored_mask[binary_mask > 0] = color  # Apply color where the mask is present

                    # Overlay the colored mask on the main mask image
                    mask_img = cv2.addWeighted(mask_img, 1.0, colored_mask, 0.5, 0)  # Blend with some transparency


                # Convert the mask image to a ROS Image message and publish it
                mask_image_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding='bgr8')
                self.image_pub.publish(mask_image_msg)

            else:
                rospy.logwarn("No segmentation masks found in YOLOv8 results.")
        
            # Render boxes on the image
            annotated_frame = result.plot()  # Use `plot` method instead of `render`

            # Display the result using OpenCV
            cv2.imshow('YOLOv8 Detection', annotated_frame)
            cv2.waitKey(1)
        else:
            rospy.logwarn("No results returned from YOLOv8.")

if __name__ == '__main__':
    try:
        yolo_node = YOLOv8Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()