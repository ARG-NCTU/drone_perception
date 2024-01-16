#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class ReadytoLatch:
    def __init__(self):
        self.error_subscriber = rospy.Subscriber("/drone/controller/target/error", Twist, self.error_callback)

        self.condition_met_time = None  # Timestamp when the condition is first met
        self.condition_hold_duration = rospy.Duration(5.0)  # Duration for which the condition must hold
        self.condition_currently_met = False  # Current state of the condition

        self.auto_mode_subscriber = rospy.Subscriber("/enable_auto_mode_success", Bool, self.auto_mode_callback)
        self.condition_pub = rospy.Publisher("ready_to_latch_success", Bool, queue_size=1)

        # Set up a ROS timer to check the condition at a specified interval (e.g., every 0.1 seconds)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_condition)

    def auto_mode_callback(self, msg):
        # Reset condition if auto mode is disabled
        if not msg.data:
            self.condition_currently_met = False
            self.condition_met_time = None

    def error_callback(self, msg):
        if not self.condition_currently_met:  # Check condition only if it's not already met
            if self.check_ready_to_latch(msg):
                if self.condition_met_time is None:
                    self.condition_met_time = rospy.Time.now()
                elif rospy.Time.now() - self.condition_met_time >= self.condition_hold_duration:
                    self.condition_currently_met = True
            else:
                self.condition_met_time = None

    def check_ready_to_latch(self, msg):
        if abs(msg.linear.x) > 0.2:
            return False
        if abs(msg.linear.y) > 0.2:
            return False
        if abs(msg.linear.z) > 10:
            return False
        if abs(msg.angular.z) > 10:
            return False
        return True

    def publish_condition(self, event):
        self.condition_pub.publish(self.condition_currently_met)
        rospy.loginfo("Status: {}".format(self.condition_currently_met))


if __name__ == "__main__":
    rospy.init_node("ready_to_latch_success")
    ready_to_latch = ReadytoLatch()
    rospy.spin()
