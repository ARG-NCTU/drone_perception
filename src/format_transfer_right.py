#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def depth_image_callback(msg):
    bridge = CvBridge()
    
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    
    # Convert depth values to 16-bit unsigned integer
    depth_image = (cv_image * 1000).astype(np.uint16)
    
    # Create a new ROS Image message with the converted depth image
    try:
        converted_msg = bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    
    # Publish the converted depth image
    converted_depth_pub.publish(converted_msg)

if __name__ == '__main__':
    rospy.init_node('depth_image_converter_right')
    
    # Create a ROS publisher for the converted depth image
    converted_depth_pub = rospy.Publisher('/camera_right/depth/image_new', Image, queue_size=1)
    
    # Create a ROS subscriber for the original depth image
    depth_image_sub = rospy.Subscriber('/camera_right/depth/image_raw', Image, depth_image_callback)
    
    rospy.spin()

