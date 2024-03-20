#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class MavrosVIOBridge:
    def __init__(self):
        rospy.loginfo("[MavrosVIOBridge] Subscribing to Odometry messages")
        rospy.init_node('odom_tf_publisher', anonymous=True)

        self.odom_subscriber = rospy.Subscriber('/mso_estimator/odometry', Odometry, self.odom_cb)
        self.mavros_vio_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

    def odom_cb(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = -msg.pose.pose.position.x
        pose_msg.pose.position.y = -msg.pose.pose.position.y
        pose_msg.pose.position.z = msg.pose.pose.position.z
        pose_msg.pose.orientation = msg.pose.pose.orientation
        self.mavros_vio_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        odom_tf_publisher = MavrosVIOBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass