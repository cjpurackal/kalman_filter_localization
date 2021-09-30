#include "InputSubscriber.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "kf_node");
	ros::NodeHandle nh; 
	EKFEstimator ekf;
	InputSubscriber isub("/initial_pose" ,/*"/X1/imu/data"*/"/imu" ,"/X1/vodom" ,"/gnss_pose", nh, ekf);
	geometry_msgs::PoseStamped pose;
	float var_imu_w_ = 0.01;
	float var_imu_acc = 0.01;

    Eigen::VectorXd x = Eigen::VectorXd::Zero(ekf.getNumState());
	x(0) = 0;
	x(1) = 0;
	x(2) = 10;
	x(6) = 0;
	x(7) = 0;
	x(8) = 1;
	x(9) = 0;
	ekf.setInitialX(x);
	ekf.setVarImuGyro(var_imu_w_);
 	ekf.setVarImuAcc(var_imu_acc);

	isub.initial_pose_recieved_ = true;
	isub.use_gnss_ = true;
	isub.use_odom_ = false;
	while (ros::ok())
	{
		// pose = isub.getPose();
		isub.broadcastPose();
		ros::spinOnce();
	}
	return 0;
}