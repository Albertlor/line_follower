import rclpy
import cv2
import math
import numpy as np
from rclpy.node import Node
from example_interfaces.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Line_Detector_Node(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        self.sensor_center_position=1.5
        self.base_length=0.21 #in meter
        self.chord_length_error=0
        self.angular_error_str=String()
        
        # Subscribing to the camera image topics
        self.subscriber_cam0 = self.create_subscription(
            Image,
            'camera_sensor_0/image_raw',
            self.camera_callback_0,
            10
        )

        self.publisher_ = self.create_publisher(String, "error_state", 10)
        self.timer_ = self.create_timer(0.5, self.publish_to_error_state)

    def publish_to_error_state(self):
        self.publisher_.publish(self.angular_error_str)

    def camera_callback_0(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        roi1 = cv_image[87:163,72:100]
        roi2 = cv_image[87:163,120:148]
        roi3 = cv_image[87:163,168:196]
        roi4 = cv_image[87:163,216:244]
        roi_list=[
            roi1,
            roi2,
            roi3,
            roi4
        ]

        # Get the average intensity of each ROI and the overall average
        _, average_intensity_list = self.get_average_intensity(roi_list)
        v0,v1,v2,v3 = average_intensity_list

        # Calculate the sensor current position and the angular error
        sensor_position = 3 - (0*v0 + 1*v1 + 2*v2 + 3*v3) // (v0+v1+v2+v3)
        print(f"Cam0:{v0}, Cam1:{v1}, Cam2:{v2}, Cam3:{v3}",flush=True)
        self.chord_length_error = (self.sensor_center_position-sensor_position) * 0.012 #0.012 is half of the distance between two sensors
        angular_error = math.atan2(self.chord_length_error,self.base_length/2)

        # Publish angular_error to a topic
        self.angular_error_str.data = f"{angular_error}"
        
        self.display_image(roi_list, "Camera")

    def display_image(self, msg, window_name):
        roi1,roi2,roi3,roi4 = msg
        # Define the border color and thickness
        border_color = (255, 255, 255)  # White border
        border_thickness = 4

        # Add a border around each ROI
        roi1_with_border = cv2.copyMakeBorder(roi1, top=border_thickness, bottom=border_thickness,
                                            left=border_thickness, right=border_thickness, 
                                            borderType=cv2.BORDER_CONSTANT, value=border_color)
        roi2_with_border = cv2.copyMakeBorder(roi2, top=border_thickness, bottom=border_thickness,
                                            left=border_thickness, right=border_thickness, 
                                            borderType=cv2.BORDER_CONSTANT, value=border_color)
        roi3_with_border = cv2.copyMakeBorder(roi3, top=border_thickness, bottom=border_thickness,
                                            left=border_thickness, right=border_thickness, 
                                            borderType=cv2.BORDER_CONSTANT, value=border_color)
        roi4_with_border = cv2.copyMakeBorder(roi4, top=border_thickness, bottom=border_thickness,
                                            left=border_thickness, right=border_thickness, 
                                            borderType=cv2.BORDER_CONSTANT, value=border_color)

        # Stack the ROIs with borders vertically
        new_cv_image = np.hstack((roi1_with_border, roi2_with_border, roi3_with_border, roi4_with_border))
        
        cv2.imshow(window_name+' 1-2-3-4', new_cv_image)
        cv2.waitKey(1)

    def get_average_intensity(self, image_list):
        grayscale_image_list=[]
        average_intensity_list=[]
        try:
            for image in image_list:
                # Convert the image to grayscale
                grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                grayscale_image_list.append(grayscale_image)

                # Calculate the average pixel intensity
                average_intensity = np.mean(grayscale_image)
                average_intensity_list.append(average_intensity)
        except Exception as e:
            print(e,flush=True)

        return grayscale_image_list, average_intensity_list

def main(args=None):
    rclpy.init(args=args)
    node = Line_Detector_Node()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()