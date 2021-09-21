#include "InputSubscriber.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "kf_node");
	ros::NodeHandle nh; 
	EKFEstimator ekf;
	InputSubscriber isub("/initial_pose" ,"/X1/imu/data" ,"/X1/vodom" ,"/gnss_pose", nh, ekf);
	geometry_msgs::PoseStamped pose;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(ekf.getNumState());
	x(0) = 0;
	x(1) = 0;
	x(2) = 0;
	x(6) = 0;
	x(7) = 0;
	x(8) = 0;
	x(9) = 0;
	ekf.setInitialX(x);
	isub.initial_pose_recieved_ = true;
	isub.use_gnss_ = false;
	isub.use_odom_ = true;
	while (ros::ok())
	{
		// pose = isub.getPose();
		isub.broadcastPose();
		ros::spinOnce();
	}
	return 0;
}