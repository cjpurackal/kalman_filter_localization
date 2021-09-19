#include "ros/ros.h"
#include "ekf.h"
#include "sensor_msgs/Imu.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <chrono>

class InputSubscriber
{
public:	
	ros::Subscriber init_pose_sub, imu_sub, odom_sub, gnss_sub ;
	InputSubscriber(std::string init_pose_topic, std::string imu_topic, std::string odom_topic, std::string gnss_topic, ros::NodeHandle nh, EKFEstimator& ekf);
	void init_pose_callback(const geometry_msgs::PoseStamped msg);
	geometry_msgs::PoseStamped getPose();
	void imu_callback(const sensor_msgs::Imu msg);
	// void odom_callback();
	// void gnss_callback();
private:
	std::string robot_frame_id_;
	geometry_msgs::PoseStamped current_pose_;
  	bool initial_pose_recieved_{false};
  	EKFEstimator& ekf_;
 	tf2_ros::Buffer tfbuffer_;

	enum STATE
	{
		X  = 0, Y = 1, Z = 2,
		VX = 3, VY = 4, VZ = 5,
		QX = 6, QY = 7, QZ = 8, QW = 9,
	};


};