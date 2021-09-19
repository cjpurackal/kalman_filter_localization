#include "InputSubscriber.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "kf_node");
	ros::NodeHandle nh; 
	EKFEstimator ekf;
	InputSubscriber isub("/initial_pose" ,"/imu" ,"/odom" ,"/gnss_pose", nh, ekf);
	geometry_msgs::PoseStamped pose;
	while (ros::ok())
	{
		pose = isub.getPose();
		ros::spinOnce();
	}
	return 0;
}