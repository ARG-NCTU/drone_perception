#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32

class BarTracker:
    """ 
    BarTracker is a ROS node that processes images from a drone's camera to detect a blue target (bar),
    publishes the center coordinates of the target, and calculates the distance to the target using depth information.

    Subscriptions:
    - /drone/front_camera/rgb/image_raw/compressed: Compressed RGB image from the drone's front camera.
    - /drone/front_camera/depth/image_raw/compressedDepth: Compressed depth image from the drone's front camera.

    Publications:
    - /bar_tracker/image_processed: Compressed RGB image with annotations (detected bar and crosshair).
    - /target/center: Point message containing the (cx, cy) coordinates of the detected bar's center.
    - /target/distance: Float32 message containing the distance from the camera to the detected bar.
    - /object_in_view_success: Bool message indicating whether the bar is detected in the current frame.
    """

    def __init__(self):
        rospy.init_node('bar_tracker')
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber('/drone/front_camera/rgb/image_raw/compressed', CompressedImage, self.image_callback)
        self.depth_sub = rospy.Subscriber('/drone/front_camera/depth/image_raw/compressedDepth', CompressedImage, self.depth_callback)
        self.image_pub = rospy.Publisher('/bar_tracker/image_processed/compressed', CompressedImage, queue_size=1)
        self.center_pub = rospy.Publisher('/target/center', Point, queue_size=1)
        self.distance_pub = rospy.Publisher('/target/distance', Float32, queue_size=1)
        self.condition_pub = rospy.Publisher('/find_target_object_success', Bool, queue_size=1)

        self.object_state = False
        self.cx = None
        self.cy = None
        self.depth_image = None

    def depth_callback(self, msg):
        """
        Callback function for the depth image topic.

        Converts the ROS CompressedImage message to an OpenCV image.

        :param msg: CompressedImage message from the depth camera.
        """
        try:
            # Convert ROS CompressedImage to OpenCV image
            # print("Depth recieve")
            depth_header_size = 12 #humanly reomve the header
            raw_data = msg.data[depth_header_size:]
            np_arr = np.frombuffer(raw_data, np.uint8)
            self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            # print(self.depth_image.shape)
            
        except Exception as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        """
        Callback function for the RGB image topic.

        Processes the image to detect a blue bar, computes its center coordinates,
        and calculates the distance to the bar using depth information. Publishes
        the processed image, center coordinates, distance, and detection status.

        :param msg: CompressedImage message from the RGB camera.
        """
        try:
            # Convert ROS CompressedImage to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert BGR image to HSV for better color filtering
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define the range for detecting color in HSV space
            lower_blue = np.array([90, 100, 100])
            upper_blue = np.array([120, 255, 255])

            # Create a binary mask for the color
            mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

            # Find contours in the mask
            _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Find the largest contour (assumed to be the bar)
            if contours:
                bar_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(bar_contour)
                if h >= w:
                    M = cv2.moments(bar_contour)
                    if M['m00'] != 0:
                        self.cx = int(M['m10'] / M['m00'])
                        self.cy = int(M['m01'] / M['m00'])

                        # Publish the center coordinates
                        center_point = Point()
                        center_point.x = self.cx
                        center_point.y = self.cy
                        self.center_pub.publish(center_point)
                        self.object_state = True
                        
                        # Draw a red circle at the center of the bar
                        cv2.circle(cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)

                        # Draw a rectangle around the bar
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                        # Label the bar
                        label = "Bar"
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.5
                        font_thickness = 2
                        text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]
                        label_pos = (x + w // 2 - text_size[0] // 2, y - 10)
                        cv2.putText(cv_image, label, label_pos, font, font_scale, (0, 255, 0), font_thickness)

                        # Get the distance from the depth image
                        if self.depth_image is not None:
                            distance = self.depth_image[self.cy, self.cx]
                            self.distance_pub.publish(Float32(distance))
                        else:
                            print("Depth is none")

                # Draw crosshair
                # image_center_x = cv_image.shape[1] // 2
                # image_center_y = cv_image.shape[0] // 2
                # crosshair_radius = 20  # Radius of the circle at the center
                # crosshair_length = 30  # Length of the lines extending from the circle
                # crosshair_offset = 10

                # # Draw the non-filled green circle at the center of the image
                # cv2.circle(cv_image, (image_center_x, image_center_y), crosshair_radius, (0, 100, 0), 2)
                
                # # Draw four lines starting from the boundary of the circle
                # cv2.line(cv_image, (image_center_x, image_center_y - crosshair_radius + crosshair_offset), 
                #          (image_center_x, image_center_y - crosshair_radius - crosshair_length + crosshair_offset), (0, 100, 0), 2)
                # cv2.line(cv_image, (image_center_x, image_center_y + crosshair_radius - crosshair_offset), 
                #          (image_center_x, image_center_y + crosshair_radius + crosshair_length - crosshair_offset), (0, 100, 0), 2)
                # cv2.line(cv_image, (image_center_x - crosshair_radius + crosshair_offset, image_center_y), 
                #          (image_center_x - crosshair_radius - crosshair_length + crosshair_offset, image_center_y), (0, 100, 0), 2)
                # cv2.line(cv_image, (image_center_x + crosshair_radius - crosshair_offset, image_center_y), 
                #          (image_center_x + crosshair_radius + crosshair_length - crosshair_offset, image_center_y), (0, 100, 0), 2)

                # Convert the processed image back to ROS CompressedImage
                _, compressed_image = cv2.imencode('.jpg', cv_image)
                processed_image_msg = CompressedImage()
                processed_image_msg.header = msg.header
                processed_image_msg.format = "jpeg"
                processed_image_msg.data = np.array(compressed_image).tobytes()

                # Publish the processed image
                self.image_pub.publish(processed_image_msg)

                condition = Bool()
                condition.data = self.object_state
                self.condition_pub.publish(condition)

                # Reset object state // find object is one time event
                # self.object_state = False

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        bar_tracker = BarTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
