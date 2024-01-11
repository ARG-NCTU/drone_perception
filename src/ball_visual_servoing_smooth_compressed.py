#!/usr/bin/env python
import cv2
import cv_bridge
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class DroneImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.Subscriber("/drone/front_camera/rgb/image_raw/compressed", CompressedImage, self.camera_callback)
        self.image_pub = rospy.Publisher("/ball_tracker/image_processed/compressed", CompressedImage, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/target/error", Twist, queue_size=1)
        self.object_in_view_pub = rospy.Publisher("/object_in_view_success", Bool, queue_size=1)
        self.result_pub = rospy.Publisher("/drone/perception/detect_result/compressed", CompressedImage, queue_size=1)

        rospy.loginfo("Drone image processor node initialized")

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            processed_image, detect_result, contours = self.detect_red_circle(cv_image)

            # Draw crosshair in the center of the image
            self.draw_crosshair(processed_image)

            # Publish the processed image with the crosshair
            self.publish_images(processed_image, detect_result)
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def detect_red_circle(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 80])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        mask = cv2.GaussianBlur(mask, (9, 9), 0)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        offset_x, offset_y = 0, 0

        min_area_threshold = 100
        if contours:
            contours = [c for c in contours if cv2.contourArea(c) > min_area_threshold]

            merged_rect = None
            for contour in contours:
                rect = cv2.boundingRect(contour)
                if merged_rect is None:
                    merged_rect = rect
                else:
                    merged_rect = self.merge_rect(merged_rect, rect)

            if merged_rect:
                x, y, w, h = merged_rect
                self.draw_bounding_box_with_label(img, merged_rect, "Target")
                cx, cy = x + w // 2, y + h // 2
                image_center_x = img.shape[1] // 2
                image_center_y = img.shape[0] // 2

                offset_x = image_center_x - cx
                offset_y = image_center_y - cy

                self.publish_object_in_view(True)
            else:
                self.publish_object_in_view(False)
        else:
            self.publish_object_in_view(False)

        self.publish_error_twist(offset_x, offset_y)
        rospy.loginfo_throttle(1, "Offset x: {}, Offset y: {}".format(offset_x, offset_y))
        return img, mask, contours

    def merge_rect(self, rect1, rect2):
        x1, y1, w1, h1 = rect1
        x2, y2, w2, h2 = rect2
        x = min(x1, x2)
        y = min(y1, y2)
        w = max(x1 + w1, x2 + w2) - x
        h = max(y1 + h1, y2 + h2) - y
        return (x, y, w, h)

    def draw_crosshair(self, img):
        image_center_x = img.shape[1] // 2
        image_center_y = img.shape[0] // 2
        crosshair_radius = 20  # Radius of the circle at the center
        crosshair_length = 30  # Length of the lines extending from the circle
        crosshair_offset = 10

        # Draw the non-filled circle at the center of the image
        cv2.circle(img, (image_center_x, image_center_y), crosshair_radius, (0, 100, 0), 2)

        # Draw four lines starting from the boundary of the circle
        cv2.line(
            img,
            (image_center_x, image_center_y - crosshair_radius + crosshair_offset),
            (image_center_x, image_center_y - crosshair_radius - crosshair_length + crosshair_offset),
            (0, 100, 0),
            2,
        )
        cv2.line(
            img,
            (image_center_x, image_center_y + crosshair_radius - crosshair_offset),
            (image_center_x, image_center_y + crosshair_radius + crosshair_length - crosshair_offset),
            (0, 100, 0),
            2,
        )
        cv2.line(
            img,
            (image_center_x - crosshair_radius + crosshair_offset, image_center_y),
            (image_center_x - crosshair_radius - crosshair_length + crosshair_offset, image_center_y),
            (0, 100, 0),
            2,
        )
        cv2.line(
            img,
            (image_center_x + crosshair_radius - crosshair_offset, image_center_y),
            (image_center_x + crosshair_radius + crosshair_length - crosshair_offset, image_center_y),
            (0, 100, 0),
            2,
        )

    def draw_bounding_box_with_label(self, img, rect, label):
        x, y, w, h = rect
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Label the bounding box
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        font_thickness = 1

        # Get the size of the text box
        text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]

        # Draw background rectangle for the label
        cv2.rectangle(img, (x, y - text_size[1] - 5), (x + text_size[0], y), (255, 0, 0), -1)

        # Put the text on the image
        cv2.putText(img, label, (x, y - 5), font, font_scale, (255, 255, 255), font_thickness, lineType=cv2.LINE_AA)


    def publish_images(self, processed_image, detect_result):
        try:
            self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(processed_image))
            self.result_pub.publish(self.bridge.cv2_to_compressed_imgmsg(detect_result))
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def publish_error_twist(self, x, y):
        error_twist = Twist()
        error_twist.linear.y = x * 0.1
        error_twist.linear.z = y * 0.1
        self.cmd_vel_pub.publish(error_twist)

    def publish_object_in_view(self, status):
        condition = Bool()
        condition.data = status
        self.object_in_view_pub.publish(condition)


if __name__ == "__main__":
    rospy.init_node("drone_image_processor")
    processor = DroneImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
