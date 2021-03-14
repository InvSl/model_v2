#include <model/controller.h>

Controller::Controller(ros::NodeHandle &nh) {
	nh_ = &nh;
	client_ = nh_->serviceClient<model::TakePose>("take_pose");

	pub_ = nh.advertise<geometry_msgs::Twist>("slave/cmd_vel", 100);
	
	while (pub_.getNumSubscribers() < 1) {
	    ros::WallDuration sleep_t(0.5);    
	    sleep_t.sleep();
	}
}

// ---------------------------------------------------------------
// -----| BLOCK: Localization |-----------------------------------
// ---------------------------------------------------------------
geometry_msgs::Pose Controller::TakeCurrentPose()
{
	client_.call(srv_);
	return srv_.response.pose;
}

double Controller::TakeCurrentYaw()
{
	geometry_msgs::Pose pose = TakeCurrentPose();
	tf::Quaternion q(
		pose.orientation.x,
		pose.orientation.y,
		pose.orientation.z,
		pose.orientation.w);

	double roll, pitch, yaw;
	((tf::Matrix3x3)q).getRPY(roll, pitch, yaw);
	ROS_INFO_STREAM("Yaw: " << yaw);
	return yaw;
}

Point<double> Controller::TakeCurrentPosition()	
{
	geometry_msgs::Pose pose = TakeCurrentPose();
	return Point<double>(pose.position.x, pose.position.y);
}

// ---------------------------------------------------------------
// -----| BLOCK: Controls |---------------------------------------
// ---------------------------------------------------------------
void Controller::MoveForward(double target_theta)
{
	geometry_msgs::Twist twist;

	double yaw = TakeCurrentYaw();
	twist.linear.x = abs(cos(yaw)) * 0.8;
	twist.linear.y = abs(sin(yaw)) * 0.8;

	// angular acc
	double theta = target_theta - yaw;
	twist.angular.z = theta * 0.1;

	pub_.publish(twist);
	ros::spinOnce();
}

void Controller::Rotation(double theta)
{
	geometry_msgs::Twist twist;
	twist.angular.z = theta;

	pub_.publish(twist);
	ros::spinOnce();
}

void Controller::Stop()
{
	geometry_msgs::Twist twist;

	pub_.publish(twist);
	ros::spinOnce();
}

// ---------------------------------------------------------------
// -----| BLOCK: Controller |-------------------------------------
// ---------------------------------------------------------------
void Controller::MovePath(vector<Point<double>> &path)
{
	for(int i = 0; i < path.size(); i++)
	{
		MoveToPoint(path[i]);
		ros::Duration(1.0).sleep();	
	}

}

void Controller::MoveToPoint(Point<double> &B)
{
	Point<double> A = TakeCurrentPosition();
	
	double yaw = TakeCurrentYaw();
	double target_angle = std::atan2(B.y - A.y, B.x - A.x);
	double theta = target_angle - yaw;

	// Rotaton
	while(abs(theta) > 10e-3)
	{
		Rotation(theta);
		yaw = TakeCurrentYaw();
		theta = target_angle - yaw;
	}

	Stop();

	// Movement
	while(Euclid(A, B) > 10e-2)
	{
		A = TakeCurrentPosition();
		target_angle = std::atan2(B.y - A.y, B.x - A.x);
		MoveForward(target_angle);
	}
	
	Stop();
	
	ros::Duration(1.0).sleep();		
}