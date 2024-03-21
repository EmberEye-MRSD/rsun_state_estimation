#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from collections import deque  # Import deque for implementing the queue

class GroundTruthVisualizer:
    def __init__(self):
        rospy.init_node('pose_history_visualizer', anonymous=True)

        # Determine the topic and message type based on user-set parameter
        self.flag_odom_input = rospy.get_param('~odom_input', False)
        # Get the queue size from parameters, defaulting to 100 if not set
        self.queue_size = rospy.get_param('~queue_size', 100)

        self.gt_viz_pub = rospy.Publisher('groundtruth_pose_hist', Path, queue_size=10)
        self.robot_viz_pub = rospy.Publisher('robot_pose_hist', Path, queue_size=10)

        if self.flag_odom_input:
            rospy.loginfo("[GroundTruthVisualizer] Subscribing to Odometry messages")
            self.pose_subscriber = rospy.Subscriber('/mocap/pose', Odometry, self.gt_odom_cb)
        else:
            rospy.loginfo("[GroundTruthVisualizer] Subscribing to PoseStamped messages")
            self.pose_subscriber = rospy.Subscriber('/mocap/pose', PoseStamped, self.gt_pose_cb)

        self.robot_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.robot_pose_cb)

        self.gt_path = Path()
        self.robot_path = Path()
        self.initialize_paths()

        # Initialize pose history queues
        self.gt_poses = deque(maxlen=self.queue_size)
        self.robot_poses = deque(maxlen=self.queue_size)

    def initialize_paths(self):
        self.gt_path.header.frame_id = "map"
        self.robot_path.header.frame_id = "map"

    def gt_odom_cb(self, msg):
        # print(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        # print(msg.pose.pose)
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        pose.header = msg.header
        self.gt_poses.append(pose)
        self.update_path(self.gt_path, self.gt_poses)

    def gt_pose_cb(self, msg):
        self.gt_poses.append(msg)
        self.update_path(self.gt_path, self.gt_poses)

    def robot_pose_cb(self, msg):
        self.robot_poses.append(msg)
        self.update_path(self.robot_path, self.robot_poses)
        self.publish_paths()

    def update_path(self, path, poses):
        path.poses = list(poses)  # Update the path with the current pose history

    def publish_paths(self):
        # Update the timestamp and publish the paths
        self.gt_path.header.stamp = rospy.Time.now()
        self.gt_viz_pub.publish(self.gt_path)
        self.robot_path.header.stamp = rospy.Time.now()
        self.robot_viz_pub.publish(self.robot_path)

if __name__ == '__main__':
    try:
        visualizer = GroundTruthVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
