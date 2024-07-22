#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage


class CollisionPrevention:
    """
    A class to implement collision prevention using depth images from a drone's front camera.

    Attributes:
    threshold_distance (float): The distance threshold for collision detection.
    percentage_threshold (float): The percentage threshold of pixels below the distance threshold to trigger a collision warning.

    Methods:
    depth_callback(msg): Callback function to process the incoming compressed depth image.
    check_collision(event): Timer callback function to check for potential collisions based on the depth image.
    """

    def __init__(self, threshold_distance, percentage_threshold):
        """
        Initialize the CollisionPrevention class.

        Parameters:
        threshold_distance (float): The distance threshold for collision detection.
        percentage_threshold (float): The percentage threshold of pixels below the distance threshold to trigger a collision warning.
        """
        self.depth_sub = rospy.Subscriber('/drone/front_camera/depth/image_raw/compressedDepth', CompressedImage, self.depth_callback)
        self.condition_pub = rospy.Publisher('/collision_prevention_success', Bool, queue_size=1)

        self.depth_image = None
        self.collision_safe_state = True
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_collision)
        self.threshold_distance = threshold_distance * 1000 # Convert unit to millimeters
        self.percentage_threshold = percentage_threshold

    def depth_callback(self, msg):
        """
        Callback function to process the incoming compressed depth image.

        Parameters:
        msg (CompressedImage): The incoming depth image message.
        """
        try:
            # Convert ROS CompressedImage to OpenCV image
            depth_header_size = 12  # Remove the header
            raw_data = msg.data[depth_header_size:]
            np_arr = np.frombuffer(raw_data, np.uint8)
            self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        except Exception as e:
            rospy.logerr(e)

    def check_collision(self, event):
        """
        Timer callback function to check for potential collisions based on the depth image.

        Parameters:
        event (rospy.TimerEvent): The timer event.
        """
        if self.depth_image is not None:
            # Calculate the number of pixels that smaller the threshold condition (will cause collision)

            num_pixels = np.sum(self.depth_image < self.threshold_distance)
    
            # Calculate the total number of pixels in the depth image
            total_pixels = self.depth_image.size
            
            # Calculate the percentage of pixels that meet the threshold condition
            percentage = (float(num_pixels) / total_pixels) * 100
            print(percentage)
            # Determine collision state
            if percentage > self.percentage_threshold:# if the percentage is larger than threshold, it means it is very likely to collision
                self.collision_safe_state = False
            else:
                self.collision_safe_state = True
            self.condition_pub.publish(data=self.collision_safe_state)
            rospy.loginfo("Collision prevention: %s", "Safe" if self.collision_safe_state else "Unsafe")
            rospy.loginfo("dangerous percentage: %.2f%%", percentage)
        else:
            rospy.logwarn("Depth image not received")


if __name__ == '__main__':
    rospy.init_node('collision_prevention_node', anonymous=False)
    threshold_distance = rospy.get_param('/collision_prevention/threshold_distance', 2.5)
    percentage_threshold = rospy.get_param('/collision_prevention/percentage_threshold', 50.0)
    CollisionPrevention(threshold_distance=threshold_distance, percentage_threshold=percentage_threshold)
    rospy.spin()
