#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class BallTracker:
    def __init__(self):
        rospy.init_node("ball_tracker")
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera_front/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        self.image_pub = rospy.Publisher("/ball_tracker/image_processed/compressed", CompressedImage, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/target/error", Twist, queue_size=1)
        self.condition_pub = rospy.Publisher("/object_in_view_success", Bool, queue_size=1)

        self.object_state = False

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert BGR image to HSV for better color filtering
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define the range for detecting red color in HSV space
            lower_red1 = np.array([0, 100, 80])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([350, 100, 80])
            upper_red2 = np.array([360, 255, 255])

            # Create a binary mask for the red color
            mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            # Find contours in the mask
            _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Find the largest contour (assumed to be the ball)
            if len(contours) > 0:
                ball_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(ball_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                # Calculate the error (center of the image - center of the ball)
                    image_center_x = cv_image.shape[1] / 2
                    image_center_y = cv_image.shape[0] / 2
                    error_x = image_center_x - cx
                    error_y = image_center_y - cy
                    # Visual servoing to keep the ball at the center of the image
                    twist = Twist()
                    twist.linear.z = error_y * 0.1
                    twist.linear.y = error_x * 0.1
                # Publish the twist command
                    self.cmd_vel_pub.publish(twist)
                    self.object_state = True
                    rospy.loginfo_throttle(1, "x: {}, y: {}".format(error_x, error_y))
                # print("error_x: ", error_x)
                # print("error_y: ", error_y)

                # cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)

                x, y, w, h = cv2.boundingRect(ball_contour)
                cv2.rectangle(
                    cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2
                )  # The rectangle will be green and have a thickness of 2

                label = "Ball"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.4
                font_thickness = 1
                text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]
                cv2.rectangle(
                    cv_image, (x, y - text_size[1] - 5), (x + text_size[0], y), (255, 0, 0), -1
                )  # Background rectangle for the label

                cv2.putText(
                    cv_image, label, (x, y - 5), font, font_scale, (255, 255, 255), font_thickness, lineType=cv2.LINE_AA
                )

            # crosshair

            image_center_x = cv_image.shape[1] // 2  # Integer division for center x-coordinate
            image_center_y = cv_image.shape[0] // 2  # Integer division for center y-coordinate
            # length = 20  # Length of crosshair lines, you can adjust this value
            # color = (0, 255, 0)  # Color of the crosshair (white in this case)
            # thickness = 2  # Thickness of the crosshair lines

            # # Horizontal line of the crosshair
            # cv2.line(cv_image, (image_center_x - length, image_center_y), (image_center_x + length, image_center_y), color, thickness)

            # # Vertical line of the crosshair
            # cv2.line(cv_image, (image_center_x, image_center_y - length), (image_center_x, image_center_y + length), color, thickness)
            # cv2.circle(cv_image, (image_center_x, image_center_y), 9, (0, 255, 0), 2)

            crosshair_radius = 20  # Radius of the circle at the center
            crosshair_length = 30  # Length of the lines extending from the circle
            crosshair_offset = 10

            # Draw the non-filled green circle at the center of the image
            cv2.circle(cv_image, (int(image_center_x), int(image_center_y)), crosshair_radius, (0, 100, 0), 2)

            # Draw four lines starting from the boundary of the circle
            cv2.line(
                cv_image,
                (int(image_center_x), int(image_center_y) - crosshair_radius + crosshair_offset),
                (int(image_center_x), int(image_center_y) - crosshair_radius - crosshair_length + crosshair_offset),
                (0, 100, 0),
                2,
            )
            cv2.line(
                cv_image,
                (int(image_center_x), int(image_center_y) + crosshair_radius - crosshair_offset),
                (int(image_center_x), int(image_center_y) + crosshair_radius + crosshair_length - crosshair_offset),
                (0, 100, 0),
                2,
            )
            cv2.line(
                cv_image,
                (int(image_center_x) - crosshair_radius + crosshair_offset, int(image_center_y)),
                (int(image_center_x) - crosshair_radius - crosshair_length + crosshair_offset, int(image_center_y)),
                (0, 100, 0),
                2,
            )
            cv2.line(
                cv_image,
                (int(image_center_x) + crosshair_radius - crosshair_offset, int(image_center_y)),
                (int(image_center_x) + crosshair_radius + crosshair_length - crosshair_offset, int(image_center_y)),
                (0, 100, 0),
                2,
            )

            # Convert the processed image back to ROS Image
            ret, encoded_image = cv2.imencode('.jpg', cv_image)
            
            processed_image_msg = CompressedImage()
            processed_image_msg.header = msg.header
            processed_image_msg.format = "jpeg"
            processed_image_msg.data = np.array(encoded_image).tobytes()

            # Publish the processed compressed image
            self.image_pub.publish(processed_image_msg)
            # processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            # processed_image_msg.header = msg.header

            # Publish the processed image
            # self.image_pub.publish(processed_image_msg)

            condition = Bool()
            condition.data = self.object_state
            self.condition_pub.publish(condition)

            # Reset object state
            self.object_state = False

        except Exception as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        ball_tracker = BallTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
