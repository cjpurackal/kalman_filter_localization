#include "ros/ros.h"
#include "ekf.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <chrono>

class InputSubscriber
{
public:	
	ros::Subscriber init_pose_sub, imu_sub, odom_sub, gnss_sub ;
	InputSubscriber(std::string init_pose_topic, std::string imu_topic, std::string odom_topic, std::string gnss_topic, ros::NodeHandle nh, EKFEstimator& ekf);
	void init_pose_callback(const geometry_msgs::PoseStamped msg);
	geometry_msgs::PoseStamped getPose();
	void predictUpdate(const sensor_msgs::Imu imu_msg);
	void measurementUpdate( const geometry_msgs::PoseStamped pose_msg, const Eigen::Vector3d variance);
	void imu_callback(const sensor_msgs::Imu msg);
	void odom_callback(const nav_msgs::Odometry msg);
	void gnss_callback(geometry_msgs::PoseStamped msg);
	void broadcastPose();
  	bool initial_pose_recieved_{false}, use_odom_{false}, use_gnss_{false};
private:
	std::string robot_frame_id_, reference_frame_id_;
	geometry_msgs::PoseStamped current_pose_;
  	EKFEstimator& ekf_;
 	tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener tfListener;
    ros::Time current_stamp_;
    geometry_msgs::PoseStamped current_pose_odom_;
    Eigen::Matrix4d previous_odom_mat_{Eigen::Matrix4d::Identity()};
    Eigen::Vector3d var_odom_, var_gnss_;
	ros::Publisher current_pose_pub_;

	enum STATE
	{
		X  = 0, Y = 1, Z = 2,
		VX = 3, VY = 4, VZ = 5,
		QX = 6, QY = 7, QZ = 8, QW = 9,
	};


};