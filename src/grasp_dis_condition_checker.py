#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import math

class GraspConditionChecker:
    def __init__(self):
        rospy.init_node('grasp_condition_checker')
        
        # Read the distance threshold from the parameter server
        self.target_distance_threshold = rospy.get_param('~distance_threshold', 0.5)

        self.pose_subscriber = rospy.Subscriber('/target/pose/arm_baselink', PoseStamped, self.pose_callback)
        self.condition_publisher = rospy.Publisher('/object_in_grasp_distance_success', Bool, queue_size=1)
        
        self.current_distance = None

        # Create a timer to check the condition at 5Hz
        rospy.Timer(rospy.Duration(1.0 / 5), self.check_condition)

    def pose_callback(self, msg):
        """
        Callback function for the /target/pose/arm_baselink topic.

        :param msg: PoseStamped message containing the pose of the target in the arm's baselink frame.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.current_distance = math.sqrt(x**2 + y**2 + z**2)

    def check_condition(self, event):
        """
        Checks if the current distance is less than the threshold and publishes the condition.
        """
        if self.current_distance is not None:
            in_grasp_distance = self.current_distance < self.target_distance_threshold
            self.condition_publisher.publish(Bool(data=in_grasp_distance))

if __name__ == '__main__':
    try:
        GraspConditionChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
