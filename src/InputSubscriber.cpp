#include "InputSubscriber.h"

InputSubscriber::InputSubscriber(std::string init_pose_topic, std::string imu_topic, std::string odom_topic, std::string gnss_topic, ros::NodeHandle nh, EKFEstimator& ekf)
:ekf_(ekf)
{
	init_pose_sub = nh.subscribe(init_pose_topic, 1, &InputSubscriber::init_pose_callback, this);
	imu_sub = nh.subscribe(imu_topic, 1, &InputSubscriber::imu_callback, this);
  robot_frame_id_ = "base_link";
	// odom_sub = nh.subscribe(odom_topic, 1, &InputSubscriber::odom_callback, this);
	// gnss_sub = nh.subscribe(gnss_topic, 1, &InputSubscriber::gnss_callback, this);

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
		      ros::Time time_point = ros::Time( msg.header.stamp.sec, msg.header.stamp.nsec);
          const geometry_msgs::TransformStamped transform =
            tfbuffer_.lookupTransform(
            robot_frame_id_,
            msg.header.frame_id,
            time_point);
          tf2::doTransform(acc_in, acc_out, transform);
          tf2::doTransform(w_in, w_out, transform);
          transformed_msg.header.stamp = msg.header.stamp;
          transformed_msg.angular_velocity.x = w_out.vector.x;
          transformed_msg.angular_velocity.y = w_out.vector.y;
          transformed_msg.angular_velocity.z = w_out.vector.z;
          transformed_msg.linear_acceleration.x = acc_out.vector.x;
          transformed_msg.linear_acceleration.y = acc_out.vector.y;
          transformed_msg.linear_acceleration.z = acc_out.vector.z;
          // predictUpdate(transformed_msg);
        }
        catch (tf2::TransformException & e) 
        {
          // RCLCPP_ERROR(this->get_logger(), "%s", e.what());
          std::cout<<"ERROR :"<<e.what();
          return;
        }
      }
 }


geometry_msgs::PoseStamped InputSubscriber::getPose()
{
	return current_pose_;
}

// void InputSubscriber::imu_callback()
// {
	
// }

// void InputSubscriber::odom_callback()
// {
	
// }

// void InputSubscriber::gnss_callback()
// {
	
// }

