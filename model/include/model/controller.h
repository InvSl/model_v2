#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

#include <model/mapper.h>
#include <model/TakePose.h>

using namespace std;

double Euclid(Point<double> &A, Point<double> &B) {
	return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

class Controller
{
public:

	Controller(ros::NodeHandle &nh);

	// ---------------------------------------------------------------
	// -----| BLOCK: Localization |-----------------------------------
	// ---------------------------------------------------------------
	geometry_msgs::Pose TakeCurrentPose();
	double TakeCurrentYaw();	
	Point<double> TakeCurrentPosition();

	// ---------------------------------------------------------------
	// -----| BLOCK: Controls |---------------------------------------
	// ---------------------------------------------------------------
	void MoveForward(double target_theta);
	void Rotation(double theta);
	void Stop();

	// ---------------------------------------------------------------
	// -----| BLOCK: Controller |-------------------------------------
	// ---------------------------------------------------------------
	void MovePath(vector<Point<double>> &path);
	void MoveToPoint(Point<double> &B);


	ros::NodeHandle *nh_;
	ros::Publisher pub_;
	ros::ServiceClient client_;
	model::TakePose srv_;
};