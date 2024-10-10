#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

class OdomTransformer
{
public:
    OdomTransformer(ros::NodeHandle& nh)
    {
        // Load the static transform values from parameters
        nh.param<std::string>("parent_frame", parent_frame_, "map");
        nh.param<std::string>("child_frame", child_frame_, "base_frame");

        nh.param<double>("x", x_, 0.0);
        nh.param<double>("y", y_, 0.0);
        nh.param<double>("z", z_, 0.0);
        nh.param<double>("roll", roll_, 0.0);
        nh.param<double>("pitch", pitch_, 0.0);
        nh.param<double>("yaw", yaw_, 0.0);

        // Subscribe to the odometry topic
        odom_sub_ = nh.subscribe("/vins_estimator/odometry", 10, &OdomTransformer::odomCallback, this);

        // Publisher for transformed odometry data
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/rsun/odometry", 10);

        // Broadcast a static transform
        broadcastStaticTransform();
    }
    
private:
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    tf2_ros::TransformBroadcaster dynamic_broadcaster_;

    std::string parent_frame_, child_frame_;
    double x_, y_, z_, roll_, pitch_, yaw_;

    void broadcastStaticTransform()
    {
        geometry_msgs::TransformStamped static_transform;

        static_transform.header.stamp = ros::Time::now();
        static_transform.header.frame_id = parent_frame_;
        static_transform.child_frame_id = child_frame_;

        // Set the static transform (translation)
        static_transform.transform.translation.x = x_;
        static_transform.transform.translation.y = y_;
        static_transform.transform.translation.z = z_;

        // Set the static transform (rotation)
        tf2::Quaternion quat;
        quat.setRPY(roll_, pitch_, yaw_);
        static_transform.transform.rotation.x = quat.x();
        static_transform.transform.rotation.y = quat.y();
        static_transform.transform.rotation.z = quat.z();
        static_transform.transform.rotation.w = quat.w();

        // Broadcast the static transform
        static_broadcaster_.sendTransform(static_transform);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        try
        {
            // Create a transformed odometry message based on the input
            nav_msgs::Odometry transformed_odom = *msg;
            transformed_odom.header.frame_id = parent_frame_;  // Set the new frame ID

            // Apply translation to the original position
            transformed_odom.pose.pose.position.x += x_;
            transformed_odom.pose.pose.position.y += y_;
            transformed_odom.pose.pose.position.z += z_;

            // Create a tf2::Vector3 for the translated position
            tf2::Vector3 translated_position(
                transformed_odom.pose.pose.position.x,
                transformed_odom.pose.pose.position.y,
                transformed_odom.pose.pose.position.z);
            
            // Create a rotation matrix from Euler angles
            Eigen::Matrix3d rotation_matrix;

            // Set the rotation matrix using roll, pitch, yaw
            rotation_matrix = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX());

            // Rotate the position
            Eigen::Vector3d new_position(translated_position.x(), translated_position.y(), translated_position.z());
            Eigen::Vector3d rotated_position = rotation_matrix * new_position;

            // Set the new position after rotation
            transformed_odom.pose.pose.position.x = rotated_position.x();
            transformed_odom.pose.pose.position.y = rotated_position.y();
            transformed_odom.pose.pose.position.z = rotated_position.z();

            // Create a quaternion for the combined orientation
            tf2::Quaternion q_combined;
            q_combined.setRPY(roll_, pitch_, yaw_);  // Set the combined orientation

            // Normalize the combined quaternion
            q_combined.normalize();

            // Set the transformed orientation
            transformed_odom.pose.pose.orientation = tf2::toMsg(q_combined);  // Convert back to geometry_msgs::Quaternion

            // Publish the transformed odometry data
            odom_pub_.publish(transformed_odom);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_transformer_node");
    ros::NodeHandle nh;

    OdomTransformer odomTransformer(nh);

    ros::spin();
    return 0;
}
