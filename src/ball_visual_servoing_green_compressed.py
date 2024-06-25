#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from behavior_tree_msgs.msg import Active, Status
from cv_bridge import CvBridge
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

HELIPAD_COLLISION = "wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__helipad_plate_collision_collision"
CYLINDER_COLLISION = "wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__cylinder_collision_collision_1"

FAIL = 0
RUNNING = 1
SUCCESS = 2


class BallTracker:
    def __init__(self):
        rospy.init_node("ball_tracker")
        self.bridge = CvBridge()

        self.mavros_cmd_command = rospy.ServiceProxy("/drone/kill", Trigger)

        self.image_sub = rospy.Subscriber(
            "/camera_front/rgb/image_raw/compressed", CompressedImage, self.image_callback
        )
        self.image_pub = rospy.Publisher("/ball_tracker/image_processed/compressed", CompressedImage, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/target/error", Twist, queue_size=1)
        self.condition_pub = rospy.Publisher("/object_in_view_success", Bool, queue_size=1)

        self.twist = Twist()
        self.object_state = False
        self.target_height = 2.5
        self.descent_speed = -0.5
        self.ascent_speed = 1.0
        self.collide = False
        self.current_height = 0.0
        self.down = True
        self.up = False
        self.goal = 0.0
        self.active = False

        self.drone_move_forward_up_status_pub = rospy.Publisher("/drone_vs_land_status", Status, queue_size=10)

        self.pose_sub = rospy.Subscriber("/pozyx_simulation/drone/pose/ground_truth", PoseStamped, self.pose_callback)
        self.bumper_sub = rospy.Subscriber("/drone/bumper_states", ContactsState, self.bumper_callback)
        rospy.Subscriber("/drone_vs_land_active", Active, self.active_callback)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert BGR image to HSV for better color filtering
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define the range for detecting red color in HSV space
            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])

            # Create a binary mask for the red color
            mask = cv2.inRange(hsv_image, lower_green, upper_green)

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
                    image_center_x = cv_image.shape[1] / 2 + 10
                    image_center_y = cv_image.shape[0] / 2 + 60
                    error_x = image_center_x - cx
                    error_y = image_center_y - cy
                    # Visual servoing to keep the ball at the center of the image
                    self.twist.linear.x = np.clip(error_y * 0.005, -0.5, 0.5)
                    self.twist.linear.y = np.clip(error_x * 0.005, -0.5, 0.5)
                    # Publish the twist command
                    self.cmd_vel_pub.publish(self.twist)
                    self.object_state = True
                    rospy.loginfo_throttle(0.1, "x: {}, y: {}".format(self.twist.linear.x, self.twist.linear.y))
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
            image_center_y = cv_image.shape[0] // 2 + 60  # Integer division for center y-coordinate
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
            ret, encoded_image = cv2.imencode(".jpg", cv_image)

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

    def pose_callback(self, msg):
        self.current_height = msg.pose.position.z
        if self.current_height > self.target_height:
            self.down = True

    def bumper_callback(self, msg):
        for state in msg.states:
            if state.collision2_name == HELIPAD_COLLISION:
                print("Helipad collision")
                self.down = False

            if state.collision2_name == CYLINDER_COLLISION:
                print("Cylinder collision")
                self.mavros_cmd_command()
                self.flying = False
                return

    def active_callback(self, msg):
        self.active = msg.active
        status = Status()
        status.id = msg.id
        status.status = SUCCESS
        self.drone_move_forward_up_status_pub.publish(status)
        if self.active:
            if self.down:
                self.twist.linear.z = -0.5
            else:
                self.twist.linear.z = 1.0
        else:
            self.twist.linear.z = 0.0


if __name__ == "__main__":
    try:
        ball_tracker = BallTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
