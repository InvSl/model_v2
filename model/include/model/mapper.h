#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <model/TakeMap.h>
#include <model/TakePose.h>
#include <vector>

using namespace std;

template <class T>
struct Point {
	T x, y;

	Point(T x, T y){
		this->x = x;
		this->y = y;
	}
};

class Mapper
{
private:

	ros::NodeHandle *nh_;
	int8_t** data;
	int width, height;
	vector<Point<int>> nodes;

public:

	// ---------------------------------------------------------------
 	// -----| BLOCK: Initialiation |----------------------------------
 	// ---------------------------------------------------------------
	Mapper(ros::NodeHandle &nh);
	~Mapper();
	void Load();
	void Clear();
	void Reload();

 	// ---------------------------------------------------------------
 	// -----| BLOCK: Drawing |----------------------------------------
 	// ---------------------------------------------------------------
    void DrawCarcass();
    int DrawNode(Point<int> p);
    void DrawHPath(int i, int j1, int j2);
	void DrawVPath(int i1, int i2, int j);

	// ---------------------------------------------------------------
	// -----| BLOCK: Graph Constracting |-----------------------------
	// ---------------------------------------------------------------
	int** MakeGraph();
	int CountCost(Point<int> &p1, Point<int> &p2);

	// ---------------------------------------------------------------
	// -----| BLOCK: Transforms |-------------------------------------
	// ---------------------------------------------------------------
	Point<int> CoordsToIndexPoint(Point<double> p);
	Point<double> IndexToCoordsPoint(Point<int> p);

	// ---------------------------------------------------------------
	// -----| BLOCK: Pathfinder |-------------------------------------
	// ---------------------------------------------------------------
	vector<Point<double>> FindPath(Point<double> &p1, Point<double> &p2);

	// ---------------------------------------------------------------
	// -----| BLOCK: Debug |------------------------------------------
	// ---------------------------------------------------------------
	void PrintMap();
	void PrintNodes();
	void Test(Point<double> p1, Point<double> p2);
};
