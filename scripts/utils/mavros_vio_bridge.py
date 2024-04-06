#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from collections import deque
from sensor_msgs.msg import Imu

from mavros_msgs.msg import State, CompanionProcessStatus
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, SetModeRequest, SetModeResponse


class MavrosVIOBridge:
    def __init__(self):
        rospy.loginfo("[MavrosVIOBridge] Subscribing to Odometry messages")
        rospy.init_node('odom_tf_publisher', anonymous=True)

        self.use_pre_int = rospy.get_param("~use_pre_int", True)
        self.send_to_mav = rospy.get_param("~send_to_mav", True)

        # failsafe parameters
        self.max_vel = rospy.get_param("~max_vel", 5.0)
        self.max_accel = rospy.get_param("~max_accel", 5.0)
        self.imu_diff_scale = rospy.get_param("~imu_diff_scale", 1.25)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.vel_deque = deque(maxlen=3)  # Stores recent velocities for smoothing
        self.accel_deque = deque(maxlen=3)  # Stores recent accelerations for smoothing

        # Previous state for velocity and acceleration calculation
        self.prev_time = rospy.Time.now()
        self.prev_pos = np.array([0.0, 0.0, 0.0])
        self.prev_vel = np.array([0.0, 0.0, 0.0])
        
        self.flag_got_imu = False
        self.latest_imu_data = None
        self.fail_count = 0
        self.flag_failed = False
        self.mav_state = None

        
        self.pose_pub = rospy.Publisher('/mso_estimator/pose_transformed', PoseStamped, queue_size=1)
        self.pose_imu_pub = rospy.Publisher('/mso_estimator/pose_imu_transformed', PoseStamped, queue_size=1)            
        self.mav_pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/mavros/companion_process/status', CompanionProcessStatus, queue_size=1)
        
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_cb)
        self.status_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.odom_imu_sub = rospy.Subscriber('/mso_estimator/imu_propagate', Odometry, self.odom_imu_cb)
        self.odom_sub = rospy.Subscriber('/mso_estimator/odometry', Odometry, self.odom_cb)
        
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.fs_timer = rospy.Timer(rospy.Duration(0.01), self.failsafe_cb)



    def odom_cb(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = -msg.pose.pose.position.x
        pose_msg.pose.position.y = -msg.pose.pose.position.y
        pose_msg.pose.position.z = msg.pose.pose.position.z
        pose_msg.pose.orientation = msg.pose.pose.orientation
        self.pose_pub.publish(pose_msg)
        if not self.use_pre_int and self.send_to_mav:
            self.mav_pose_pub.publish(pose_msg)
    
    def imu_cb(self, msg):
        if not self.flag_got_imu:
            self.flag_got_imu = True
        self.latest_imu_data = msg

    
    def odom_imu_cb(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = -msg.pose.pose.position.x
        pose_msg.pose.position.y = -msg.pose.pose.position.y
        pose_msg.pose.position.z = msg.pose.pose.position.z
        pose_msg.pose.orientation = msg.pose.pose.orientation

        # Calculate current velocity
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec() if self.prev_time else 0.1  # Prevent division by zero
        curr_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        odom_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        vel = odom_vel

        # Update deque and calculate smoothed velocity
        self.vel_deque.append(vel)
        smoothed_vel = np.mean(self.vel_deque, axis=0)

        # Check velocity against threshold
        # if np.linalg.norm(smoothed_vel) > self.max_vel:
        #     rospy.logwarn("Velocity exceeds threshold: {}".format(np.linalg.norm(smoothed_vel)))

        # Calculate acceleration
        accel = (smoothed_vel - self.prev_vel) / dt if dt > 0 else np.array([0.0, 0.0, 0.0])

        # Update deque and calculate smoothed acceleration
        self.accel_deque.append(accel)
        smoothed_accel = np.mean(self.accel_deque, axis=0)

        # Check acceleration against threshold
        if self.flag_got_imu:
            imu_accel = np.array([self.latest_imu_data.linear_acceleration.x, self.latest_imu_data.linear_acceleration.y, self.latest_imu_data.linear_acceleration.z])
            imu_accel = imu_accel - np.array([0.0, 0.0, 9.81])

            if np.linalg.norm(smoothed_accel) > 1.50*np.linalg.norm(imu_accel) and np.linalg.norm(smoothed_vel) > 1.0:
                self.flag_failed = True
                print("VIO FAILING!")
        
        if not self.flag_failed and self.send_to_mav and self.use_pre_int:
            self.mav_pose_pub.publish(pose_msg)

        # Prepare for next iteration
        self.prev_time = current_time
        self.prev_pos = curr_pos
        self.prev_vel = smoothed_vel
    
    def state_cb(self, msg):
        self.mav_state = msg

    def failsafe_cb(self, event):
        status_msg = CompanionProcessStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.component = status_msg.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        status_msg.state = status_msg.MAV_STATE_ACTIVE

        if self.flag_failed:
            status_msg.state = status_msg.MAV_STATE_FLIGHT_TERMINATION
            # rospy.logwarn("VIO FAILING!")
            # self.mav_state = State()
            if self.mav_state is not None:
                if (self.mav_state.mode == self.mav_state.MODE_PX4_POSITION
                    and self.mav_state.mode != self.mav_state.MODE_PX4_ALTITUDE):
                    self.set_mode(self.mav_state.MODE_PX4_ALTITUDE)
                    rospy.logwarn("Landing")

        self.status_pub.publish(status_msg)
    
    def set_mode(self, mode_id):
        # Mode change service client
        modeReq = SetModeRequest()
        modeResp = SetModeResponse()
        
        modeReq.custom_mode = mode_id
        modeReq.base_mode = 0
        
        # Request mode change
        try:
            rospy.loginfo_throttle(2.0,'[MavrosAPI] Attempting flight mode change to '+ mode_id)
            modeResp = self.set_mode_srv(modeReq)
            return modeResp.mode_sent 
        except rospy.ServiceException as e:
            rospy.logerr('[MavrosAPI] Mode change to '+ mode_id +' failed! %s'%e)
            return False

if __name__ == '__main__':
    try:
        odom_tf_publisher = MavrosVIOBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass