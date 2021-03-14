#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <model/TakeMap.h>
#include <typeinfo>
#include <vector>
using namespace std;

class MapService
{
public:
	MapService(ros::NodeHandle& nh)
	{
		sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &MapService::callback, this);
		server_ = nh.advertiseService("take_map", &MapService::takeMap, this);
	}

	void callback(const nav_msgs::OccupancyGridConstPtr& msg)
	{
		last_msg_ = msg;
		ROS_INFO_STREAM("Map has gotten!");
	}


	bool takeMap(model::TakeMap::Request& req,
				 model::TakeMap::Response& res)
	{
		nav_msgs::OccupancyGridConstPtr p = last_msg_;
		if(!p) return false;

		res.map = *last_msg_;
		

		// reshaping
		std::vector<int8_t> new_map;
		for(int i = 1900; i < 2100; i++)
			for(int j = 1970; j < 2170; j++)
				new_map.push_back(res.map.data[i*4000 + j]);

		res.map.data = new_map;
		res.map.info.width = 200;
		res.map.info.height = 200;

		return true;
	}

	
	ros::Subscriber sub_;
	nav_msgs::OccupancyGridConstPtr last_msg_;
	ros::ServiceServer server_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "MapService_node");
	ros::NodeHandle nh;

	MapService MapService(nh);
	ROS_INFO("MapService node starting");

	ros::spin();
}

