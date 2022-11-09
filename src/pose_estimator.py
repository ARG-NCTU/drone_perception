#!/usr/bin/env python

import numpy as np
import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
import math
import tf

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped

from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
import message_filters

from darknet_ros_msgs.msg import BoundingBoxes


class Estimator():
    def __init__(self):
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0
        self.frame_id = "none"
        self.cnt = 0
        ## for opencv
        self.cv_bridge = CvBridge()

        ## tf publisher
        # self.tf_pub = tf.TransformBroadcaster()

        # Publisher & Subscriber

        rospy.Subscriber("desired_image/info", CameraInfo, self.camInfoCallback)

        sub_depth = message_filters.Subscriber("desired_image/depth", Image)
        sub_boxed = message_filters.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes)
        ats = ApproximateTimeSynchronizer((sub_depth, sub_boxed), queue_size = 1, slop = 0.1)
        ats.registerCallback(self.syncCallback)

        # rospy.Subscriber("/desired_image/depth", Image, self.depthImageCallback)
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.boxesCallback)
        
        self.pub_pose = rospy.Publisher("docking_pose", PoseStamped, queue_size=1)


    def camInfoCallback(self, msg):
        self.fx = msg.P[0]
        self.fy = msg.P[5]
        self.cx = msg.P[2]
        self.cy = msg.P[6]
        self.frame_id = msg.header.frame_id
        # print("camera_from: ", self.frame_id)
    
    def boxesCallback(self, msg):
        # id: 0 = G, 1 = R, 2 = W, 3 = H
        print("source: ", msg.image_header.frame_id)


    def syncCallback(self, msg_depth, msg_boxes):
        print("darknet: ", msg_boxes.image_header.frame_id)
        print("depth  : ", msg_depth.header.frame_id)
        cv_depth = self.cv_bridge.imgmsg_to_cv2(msg_depth, "16UC1")
        for i in range(len(msg_boxes.bounding_boxes)):
            # print("xmin: ", msg_boxes.bounding_boxes[i].xmin)
            # print("ymin: ", msg_boxes.bounding_boxes[i].ymin)
            # print("xmax: ", msg_boxes.bounding_boxes[i].xmax)
            # print("ymax: ", msg_boxes.bounding_boxes[i].ymax)
            # print("class: ", msg_boxes.bounding_boxes[i].id)
            x = (msg_boxes.bounding_boxes[i].xmin + msg_boxes.bounding_boxes[i].xmax)/2
            y = (msg_boxes.bounding_boxes[i].ymin + msg_boxes.bounding_boxes[i].ymax)/2
            zc = cv_depth[int(y), int(x)]/1000.0
            print("x, y:", x, y)
            print("zc", zc)
            rx, ry, rz = self.getXYZ(x/1.0 , y/1.0, zc/1.0)
            if(rx != 0):
                # posest = PoseStamped()
                # posest.pose.position.x = rx
                # posest.pose.position.y = ry
                # posest.pose.position.z = rz
                # posest.header = depth_img.header
                # posest.header.frame_id = "/aaa"
                print("camera_pose: ", rx, ry, rz)
                tf_pub = tf.TransformBroadcaster()
                tf_pub.sendTransform((rx, ry, rz), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), "object", "zedm_forward_left_camera_frame")
                # try:
                #     posest = self.listener.transformPose("base_link", posest)
                # except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #     rospy.loginfo("faile to catch tf %s 2 %s" % ("/map", "/zed_mid_link"))    
                #     return
                # self.pub_pose.publish(posest)

    def bbboxesCallback(self, msg):
        depth_img = msg.depth_image
        cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_img, "16UC1")

        redbb = BoundingBox()
        redbb.probability = 0
        greenbb = BoundingBox()
        greenbb.probability = 0
        bluebb = BoundingBox()
        bluebb.probability = 0
        # print("num:", msg.count)
        for i in range(msg.count):
            if(msg.bounding_boxes[i].Class == "Red_clone"):
                if(msg.bounding_boxes[i].probability > redbb.probability):
                    redbb = msg.bounding_boxes[i]
            elif(msg.bounding_boxes[i].Class == "Green_clone"):
                if(msg.bounding_boxes[i].probability > greenbb.probability):
                    greenbb = msg.bounding_boxes[i]
            elif(msg.bounding_boxes[i].Class == "Blue_clone"):
                if(msg.bounding_boxes[i].probability > bluebb.probability):
                    bluebb = msg.bounding_boxes[i]

        if(self.color_dock=="Red_clone" and redbb.probability>0):
            x = (redbb.xmin + redbb.xmax)/2
            y = (redbb.ymin + redbb.ymax)/2
            zc = cv_depth[int(y), int(x)]/1000.0
            print("x, y:", x, y)
            print("zc", zc)
            rx, ry, rz = self.getXYZ(x/1.0 , y/1.0, zc/1.0)
            if(rx != 0):
                posest = PoseStamped()
                posest.pose.position.x = rx
                posest.pose.position.y = ry
                posest.pose.position.z = rz
                posest.header = depth_img.header
                posest.header.frame_id = "/zed_mid_link"
                print("camera_pose: ", rx, ry, rx)
                try:
                    posest = self.listener.transformPose("base_link", posest)
                except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("faile to catch tf %s 2 %s" % ("/map", "/zed_mid_link"))    
                    return
                self.pub_pose.publish(posest)

        elif(self.color_dock=="Green_clone" and greenbb.probability>0):
            x = (greenbb.xmin + greenbb.xmax)/2
            y = (greenbb.ymin + greenbb.ymax)/2
            zc = cv_depth[int(y), int(x)]
            rx, ry, rz = self.getXYZ(x/1.0 , y/1.0, zc/1.0)
            if(rx != 0):
                posest = PoseStamped()
                posest.pose.position.x = rx
                posest.pose.position.y = ry
                posest.pose.position.z = rz
                posest.header = depth_img.header
                posest.header.frame_id = "/zed_mid_link"
                try:
                    posest = self.listener.transformPose("/map", posest)
                except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("faile to catch tf %s 2 %s" % ("/map", "/zed_mid_link"))    
                    return
                self.pub_pose.publish(posest)
        
        elif(self.color_dock=="Blue_clone" and bluebb.probability>0):
            x = (bluebb.xmin + bluebb.xmax)/2
            y = (bluebb.ymin + bluebb.ymax)/2
            zc = cv_depth[int(y), int(x)]
            rx, ry, rz = self.getXYZ(x/1.0 , y/1.0, zc/1.0)
            if(rx != 0):
                posest = PoseStamped()
                posest.pose.position.x = rx
                posest.pose.position.y = ry
                posest.pose.position.z = rz
                posest.header = depth_img.header
                posest.header.frame_id = "/zed_mid_link"
                try:
                    posest = self.listener.transformPose("/map", posest)
                except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("faile to catch tf %s 2 %s" % ("/map", "/zed_mid_link"))    
                    return
                self.pub_pose.publish(posest)

    def getXYZ(self, x, y, zc):
        x = float(x)
        y = float(y)
        zc = float(zc)
        inv_fx = 1.0/self.fx
        inv_fy = 1.0/self.fy
        x = (x - self.cx) * zc * inv_fx
        y = (y - self.cy) * zc * inv_fy
        return zc, -1*x, -1*y



if __name__ == '__main__':
    rospy.init_node('bb_to_zed_node',anonymous=False)
    zed_detection = Estimator()
    rospy.spin()
