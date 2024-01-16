#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, LaserScan


class LaserImageProcessor:
    def __init__(self):
        rospy.init_node("laser_image_processor", anonymous=True)

        self.bridge = CvBridge()

        self.laser_sub = rospy.Subscriber("/RL/scan", LaserScan, self.laser_callback)
        self.image_sub = rospy.Subscriber(
            "/drone/perception/detect_result/compressed", CompressedImage, self.image_callback
        )

        self.laser_pub = rospy.Publisher("/RL/scan_label", LaserScan, queue_size=10)

        self.current_scan = None
        self.downsampled_and_padded = None

    def laser_callback(self, data):
        self.current_scan = data
        self.process_data()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            _, binary_image = cv2.threshold(cv_image, 1, 255, cv2.THRESH_BINARY)

            column_presence = np.zeros(cv_image.shape[1], dtype=np.int)
            for i in range(cv_image.shape[1]):
                if np.any(binary_image[:, i] == 255):
                    column_presence[i] = 1

            self.downsampled_and_padded = self.downsample_and_pad(np.flip(column_presence, 0))
        except CvBridgeError as e:
            rospy.logerr(e)

    def downsample_and_pad(self, array, target_length=58, final_length=241):
        downsampled = np.array(
            [
                np.max(array[i : i + len(array) // target_length])
                for i in range(0, len(array), len(array) // target_length)
            ]
        )
        pad_length = (final_length - target_length) // 2
        padded_array = np.pad(downsampled, (pad_length, pad_length), "constant")
        return padded_array[:final_length]

    def process_data(self):
        if self.current_scan is not None and self.downsampled_and_padded is not None:
            self.current_scan.intensities = self.downsampled_and_padded.tolist()
            self.laser_pub.publish(self.current_scan)
            self.current_scan = None


if __name__ == "__main__":
    try:
        processor = LaserImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
