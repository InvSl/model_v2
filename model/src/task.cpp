#include <model/controller.h>
#include <vector>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "task_script_node");
	ros::NodeHandle nh;

	Controller controller(nh);
	Mapper mapper(nh);
	mapper.Load();

	Point<double> A = controller.TakeCurrentPosition();
	Point<double> B(4, 4); // finish;

	vector<Point<double>> path = mapper.FindPath(A, B);

	controller.MovePath(path);

	ros::Duration(1.0).sleep();
	
	return 0;
}