#include "InputSubscriber.h"

InputSubscriber::InputSubscriber(std::string init_pose_topic, std::string imu_topic, std::string odom_topic, std::string gnss_topic, ros::NodeHandle nh, EKFEstimator& ekf)
:ekf_(ekf),
tfListener(tfbuffer_),
var_odom_(0,0,0),
var_gnss_(0.1,0.1,0.1),
odom_trans(0,0,0)
{
	init_pose_sub = nh.subscribe(init_pose_topic, 1, &InputSubscriber::init_pose_callback, this);
	imu_sub = nh.subscribe(imu_topic, 1, &InputSubscriber::imu_callback, this);
  robot_frame_id_ = "base_link";
  // reference_frame_id_ = "world";
  reference_frame_id_ = "map";

	odom_sub = nh.subscribe(odom_topic, 1, &InputSubscriber::odom_callback, this);
	gnss_sub = nh.subscribe(gnss_topic, 1, &InputSubscriber::gnss_callback, this);
  current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("fused_pose", 10);

}

void InputSubscriber::init_pose_callback(const geometry_msgs::PoseStamped msg)
{
    std::cout << "initial pose callback" << std::endl;
    initial_pose_recieved_ = true;
    current_pose_ = msg;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(ekf_.getNumState());
    x(STATE::X) = current_pose_.pose.position.x;
    x(STATE::Y) = current_pose_.pose.position.y;
    x(STATE::Z) = current_pose_.pose.position.z;
    x(STATE::QX) = current_pose_.pose.orientation.x;
    x(STATE::QY) = current_pose_.pose.orientation.y;
    x(STATE::QZ) = current_pose_.pose.orientation.z;
    x(STATE::QW) = current_pose_.pose.orientation.w;
    ekf_.setInitialX(x);
}


void InputSubscriber::imu_callback(const sensor_msgs::Imu msg)
{
      if (initial_pose_recieved_) 
      {
        sensor_msgs::Imu transformed_msg;
        try 
        {
          // std::cout<<"inside imu"<<std::endl;
          geometry_msgs::Vector3Stamped acc_in, acc_out, w_in, w_out;
          acc_in.vector.x = msg.linear_acceleration.x;
          acc_in.vector.y = msg.linear_acceleration.y;
          acc_in.vector.z = msg.linear_acceleration.z;
          w_in.vector.x = msg.angular_velocity.x;
          w_in.vector.y = msg.angular_velocity.y;
          w_in.vector.z = msg.angular_velocity.z;
          // tf2::TimePoint time_point = tf2::TimePoint(
          //   std::chrono::seconds(msg->header.stamp.sec) +
          //   std::chrono::nanoseconds(msg->header.stamp.nanosec));
          // ros::Time time_point = ros::Time( msg.header.stamp.sec, msg.header.stamp.nsec);
          //   const geometry_msgs::TransformStamped transform =
          //     tfbuffer_.lookupTransform(
          //     robot_frame_id_,
          //     msg.header.frame_id,
          //     time_point);
          //   tf2::doTransform(acc_in, acc_out, transform);
          //   tf2::doTransform(w_in, w_out, transform);
          
          transformed_msg.header.stamp = msg.header.stamp;
          transformed_msg.angular_velocity.x = /*w_out*/w_in.vector.x;
          transformed_msg.angular_velocity.y = /*w_out*/w_in.vector.y;
          transformed_msg.angular_velocity.z = /*w_out*/w_in.vector.z;
          transformed_msg.linear_acceleration.x = /*acc_out*/acc_in.vector.x;
          transformed_msg.linear_acceleration.y = /*acc_out*/acc_in.vector.y;
          transformed_msg.linear_acceleration.z = /*acc_out*/acc_in.vector.z;
          // std::cout<<"transformed_msg.header.stamp = "<<transformed_msg.header.stamp<<std::endl<<"transformed_msg.angular_velocity.x = "<<transformed_msg.angular_velocity.x<<std::endl<<"transformed_msg.angular_velocity.y = "<<transformed_msg.angular_velocity.y<<std::endl<<"transformed_msg.angular_velocity.z = "<<transformed_msg.angular_velocity.z<<std::endl<<"transformed_msg.linear_acceleration.x = "<<transformed_msg.linear_acceleration.x<<std::endl<<"transformed_msg.linear_acceleration.y = "<<transformed_msg.linear_acceleration.y<<std::endl<<"transformed_msg.linear_acceleration.z = "<<transformed_msg.linear_acceleration.z<<std::endl;

          predictUpdate(transformed_msg);
        }
        catch (tf2::TransformException & e) 
        {
          ROS_ERROR("imu_callback exception: %s", e.what());
          return;
        }
      }
 }


geometry_msgs::PoseStamped InputSubscriber::getPose()
{
	return current_pose_;
}

void InputSubscriber::odom_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
{

      // ekf_.printState();
      if (initial_pose_recieved_ && use_odom_) 
      {
        Eigen::Affine3d affine;
        tf2::fromMsg(msg.pose.pose, affine);
        Eigen::Matrix4d odom_mat = affine.matrix();
        if (previous_odom_mat_ == Eigen::Matrix4d::Identity()) {
          current_pose_odom_ = current_pose_;
          previous_odom_mat_ = odom_mat;
          return;
        }
        Eigen::Affine3d current_affine;
        tf2::fromMsg(current_pose_odom_.pose, current_affine);
        Eigen::Matrix4d current_trans = current_affine.matrix();
        // std::cout<<current_trans<<std::endl;
        current_trans = current_trans * previous_odom_mat_.inverse() * odom_mat;


        geometry_msgs::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = current_trans(0, 3);
        pose.pose.position.y = current_trans(1, 3);
        pose.pose.position.z = current_trans(2, 3);

        // odom_trans(0) = current_trans(0, 3);
        // odom_trans(1) = current_trans(1, 3);
        // odom_trans(2) = current_trans(2, 3);

        var_odom_(0) = msg.pose.covariance[0];
        var_odom_(1) = msg.pose.covariance[7];
        var_odom_(2) = msg.pose.covariance[13];

        
        measurementUpdate(pose, var_odom_);
        current_pose_odom_ = current_pose_;
        previous_odom_mat_ = odom_mat;
      }
	
}

void InputSubscriber::gnss_callback(geometry_msgs::PoseStamped msg)
{
      if (initial_pose_recieved_ && use_gnss_) {
        measurementUpdate(msg, var_gnss_);
      }
}

	
void InputSubscriber::predictUpdate(const sensor_msgs::Imu imu_msg)
{
  current_stamp_ = imu_msg.header.stamp;

  double current_time_imu = imu_msg.header.stamp.sec +
    imu_msg.header.stamp.nsec * 1e-9;
  Eigen::Vector3d gyro = Eigen::Vector3d(
    imu_msg.angular_velocity.x,
    imu_msg.angular_velocity.y,
    imu_msg.angular_velocity.z);
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d(
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z);

  ekf_.predictionUpdate(current_time_imu, gyro, linear_acceleration);
}

void InputSubscriber::measurementUpdate( const geometry_msgs::PoseStamped pose_msg, const Eigen::Vector3d variance)
{
  current_stamp_ = pose_msg.header.stamp;
  Eigen::Vector3d y = Eigen::Vector3d(pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);


  ekf_.observationUpdate(y, variance);
}

void InputSubscriber::broadcastPose()
{
  if (initial_pose_recieved_) {
    auto x = ekf_.getX();
    current_pose_.header.stamp = current_stamp_;
    current_pose_.header.frame_id = reference_frame_id_;
    current_pose_.pose.position.x = x(STATE::X);
    current_pose_.pose.position.y = x(STATE::Y);
    current_pose_.pose.position.z = x(STATE::Z);
    current_pose_.pose.orientation.x = x(STATE::QX);
    current_pose_.pose.orientation.y = x(STATE::QY);
    current_pose_.pose.orientation.z = x(STATE::QZ);
    current_pose_.pose.orientation.w = x(STATE::QW);
    current_pose_pub_.publish(current_pose_);
  }
}