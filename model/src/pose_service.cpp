#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <model/TakePose.h>

using namespace std;

class PoseService
{
public:
	PoseService(ros::NodeHandle& nh)
	{
		sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &PoseService::callback, this);
		server_ = nh.advertiseService("take_pose", &PoseService::takePose, this);

	}

	void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
	{
		last_msg_ = msg;
		ROS_INFO_STREAM("Pose has gotten!");
	}


	bool takePose(model::TakePose::Request& req,
				 model::TakePose::Response& res)
	{
		geometry_msgs::PoseWithCovarianceStampedConstPtr p = last_msg_;
		if(!p) return false;

		res.pose = last_msg_->pose.pose;
		ROS_INFO_STREAM("Pose gaven.");

		return true;
	}

	
	ros::Subscriber sub_;
	geometry_msgs::PoseWithCovarianceStampedConstPtr last_msg_;
	ros::ServiceServer server_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "PoseSerivce_node");
	ros::NodeHandle nh;

	PoseService srv(nh);
	ROS_INFO("PoseService node starting");

	ros::spin();
}

