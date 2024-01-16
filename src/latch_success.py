#! /usr/bin/env python
import rospy
import tf2_ros
from std_msgs.msg import Bool


class FramePositionChecker:
    def __init__(self, target_frame, reference_frame):
        self.target_frame = target_frame
        self.reference_frame = reference_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.condition_met_time = None  # Timestamp when the condition is first met
        self.condition_hold_duration = rospy.Duration(1.0)  # Duration for which the condition must hold
        self.condition_currently_met = False  # Current state of the condition
        
        self.condition_pub = rospy.Publisher("latch_success_success", Bool, queue_size=1)
        
        # Set up a ROS timer to check the position at a specified interval (e.g., every 0.1 seconds)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_position)

    def check_position(self, event):
        if self.is_frame_in_front():
            if self.condition_met_time is None:
                self.condition_met_time = rospy.Time.now()
            elif rospy.Time.now() - self.condition_met_time >= self.condition_hold_duration:
                self.condition_currently_met = True
        else:
            self.condition_met_time = None
            self.condition_currently_met = False

        if self.condition_currently_met:
            rospy.loginfo("True")
            self.condition_pub.publish(True)
        else:
            rospy.loginfo("False")
            self.condition_pub.publish(False)

    def is_frame_in_front(self):
        try:
            # Wait for the transform to become available
            trans = self.tf_buffer.lookup_transform(
                self.reference_frame, self.target_frame, rospy.Time(0), rospy.Duration(1.0)
            )

            # Check if the x coordinate of the target frame is greater than that of the reference frame
            return trans.transform.translation.x > 0.2

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF2 exception: %s", e)
            return False


if __name__ == "__main__":
    rospy.init_node("frame_position_checker")
    checker = FramePositionChecker("drone/horizontal_rod_head_link", "wamv/red_torus_link")

    # Spin to keep the script for exiting
    rospy.spin()
