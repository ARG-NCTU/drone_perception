#!/usr/bin/env python

import rospy
import numpy as np
import tf
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices

class PoseCalculator:
    """
    PoseCalculator is a ROS node that subscribes to the /target/distance and /drone/front_camera/depth/camera_info topics
    to calculate the pose (xc, yc, zc) of the detected target based on its center coordinates and depth information.
    It then transforms the target position to the drone's baselink and the arm's baselink frames for navigation and grasping.

    Subscriptions:
    - /target/distance: Float32 message containing the distance from the camera to the detected target.
    - /target/center: Point message containing the (cx, cy) coordinates of the detected target's center.
    - /drone/front_camera/depth/camera_info: CameraInfo message containing the camera intrinsic parameters.

    Publications:
    - /target/pose/camera_frame: PoseStamped message containing the calculated (xc, yc, zc) pose of the target in the camera frame.
    - /target/pose/drone_baselink: PoseStamped message containing the target pose transformed to the drone's baselink frame.
    - /target/pose/arm_baselink: PoseStamped message containing the target pose transformed to the arm's baselink frame.
    """

    def __init__(self):
        rospy.init_node('pose_calculator')
        
        self.distance_sub = rospy.Subscriber('/target/distance', Float32, self.distance_callback)
        self.center_sub = rospy.Subscriber('/target/center', Point, self.center_callback)
        self.camera_info_sub = rospy.Subscriber('/drone/front_camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        self.pose_pub_camera_frame = rospy.Publisher('/target/pose/camera_frame', PoseStamped, queue_size=1)
        self.pose_pub_drone_baselink = rospy.Publisher('/target/pose/drone_baselink', PoseStamped, queue_size=1)
        self.pose_pub_arm_baselink = rospy.Publisher('/target/pose/arm_baselink', PoseStamped, queue_size=1)

        self.distance = None
        self.cx = None
        self.cy = None
        self.fx = None
        self.fy = None
        self.camera_cx = None
        self.camera_cy = None

        self.distance_lock = False
        self.center_lock = False

        self.listener = tf.TransformListener()

        # Get the source frame from the parameter server
        self.source_frame = rospy.get_param('~source_frame', 'drone/front_camera_optical_link')

        # Cache the transforms to the target frames
        self.transform_camera_to_drone = None
        self.transform_camera_to_arm = None
        self.cache_transforms()

        # Create a timer to call calculate_pose at 5Hz
        rospy.Timer(rospy.Duration(1.0 / 5), self.calculate_pose)

    def cache_transforms(self):
        """
        Cache the static transforms from the source frame to the target frames.
        """
        try:
            # Wait for the transformations
            self.listener.waitForTransform('drone/base_link', self.source_frame, rospy.Time(), rospy.Duration(4.0))
            self.listener.waitForTransform('wx200/base_link', self.source_frame, rospy.Time(), rospy.Duration(4.0))

            # Get the transformations
            (trans_drone, rot_drone) = self.listener.lookupTransform('drone/base_link', self.source_frame, rospy.Time(0))
            (trans_arm, rot_arm) = self.listener.lookupTransform('wx200/base_link', self.source_frame, rospy.Time(0))

            # Convert to transformation matrices
            self.transform_camera_to_drone = concatenate_matrices(translation_matrix(trans_drone), quaternion_matrix(rot_drone))
            self.transform_camera_to_arm = concatenate_matrices(translation_matrix(trans_arm), quaternion_matrix(rot_arm))

            rospy.loginfo("Transforms cached successfully")
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error caching transforms: %s", e)

    def distance_callback(self, msg):
        """
        Callback function for the /target/distance topic.

        :param msg: Float32 message containing the distance from the camera to the target.
        """
        self.distance = msg.data
        self.distance_lock = True

    def center_callback(self, msg):
        """
        Callback function for the /target/center topic.

        :param msg: Point message containing the (cx, cy) coordinates of the target's center.
        """
        self.cx = msg.x
        self.cy = msg.y
        self.center_lock = True

    def camera_info_callback(self, msg):
        """
        Callback function for the /drone/front_camera/depth/camera_info topic.

        Extracts the camera intrinsic parameters (fx, fy, cx, cy) from the CameraInfo message.

        :param msg: CameraInfo message containing the camera intrinsic parameters.
        """
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.camera_cx = msg.K[2]
        self.camera_cy = msg.K[5]

    def calculate_pose(self, event):
        """
        Calculates the pose (xc, yc, zc) of the target based on the distance, center coordinates,
        and camera intrinsic parameters. Publishes the calculated pose as a PoseStamped message.
        Transforms the target pose to the drone's baselink and arm's baselink frames and publishes them as PoseStamped messages.
        """
        if self.distance_lock and self.center_lock and self.fx is not None and self.fy is not None:
            zc = self.distance * 0.001 # Convert distance from mm to meters
            xc = (self.cx - self.camera_cx) * zc / self.fx
            yc = (self.cy - self.camera_cy) * zc / self.fy
            
            pose_camera_frame = PoseStamped()
            pose_camera_frame.header.frame_id = self.source_frame
            pose_camera_frame.header.stamp = rospy.Time.now()
            pose_camera_frame.pose.position.x = xc
            pose_camera_frame.pose.position.y = yc
            pose_camera_frame.pose.position.z = zc
            pose_camera_frame.pose.orientation.w = 1.0  # No rotation

            self.pose_pub_camera_frame.publish(pose_camera_frame)

            # Transform to drone baselink frame
            self.transform_and_publish_pose(self.transform_camera_to_drone, 'drone/base_link', pose_camera_frame, self.pose_pub_drone_baselink)

            # Transform to arm baselink frame
            self.transform_and_publish_pose(self.transform_camera_to_arm, 'wx200/base_link', pose_camera_frame, self.pose_pub_arm_baselink)

            # Reset the locks
            self.distance_lock = False
            self.center_lock = False

    def transform_and_publish_pose(self, transform_matrix, target_frame, pose_camera_frame, publisher):
        """
        Transforms the pose from the camera frame to the specified target frame and publishes it.

        :param transform_matrix: The transformation matrix from the camera frame to the target frame.
        :param target_frame: The target frame to transform the pose to.
        :param pose_camera_frame: The PoseStamped message in the camera frame.
        :param publisher: The ROS publisher to publish the transformed pose.
        """
        try:
            # Create the pose in the target frame
            camera_pose = np.array([pose_camera_frame.pose.position.x, pose_camera_frame.pose.position.y, pose_camera_frame.pose.position.z, 1.0])
            target_pose = np.dot(transform_matrix, camera_pose)
            pose_transformed = PoseStamped()
            pose_transformed.header.frame_id = target_frame
            pose_transformed.header.stamp = pose_camera_frame.header.stamp
            pose_transformed.pose.position.x = target_pose[0]
            pose_transformed.pose.position.y = target_pose[1]
            pose_transformed.pose.position.z = target_pose[2]
            pose_transformed.pose.orientation.w = 1.0
            # Publish the transformed pose
            publisher.publish(pose_transformed)

        except Exception as e:
            rospy.logerr("Error in transform_and_publish_pose: %s", e)

if __name__ == '__main__':
    try:
        pose_calculator = PoseCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
