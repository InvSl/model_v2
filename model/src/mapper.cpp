#include <model/mapper.h>


// ---------------------------------------------------------------
// -----| BLOCK: Initialiation |----------------------------------
// ---------------------------------------------------------------
Mapper::Mapper(ros::NodeHandle &nh)
{
	nh_ = &nh;
}

Mapper::~Mapper() {
	this->Clear();
}

void Mapper::Load()
{
	ros::ServiceClient client = nh_->serviceClient<model::TakeMap>("take_map");
	model::TakeMap srv;
	client.call(srv);
	nav_msgs::OccupancyGrid map = srv.response.map;


	this->width = map.info.width,
	this->height = map.info.height;

	this->data = new int8_t*[width];
	int8_t value;
	for(int i = 0; i < width; i++)
	{
		data[i] = new int8_t[height];
		for(int j = 0; j < height; j++)
		{
			value = map.data[i*width + j];
			if ((int)value != 0) data[i][j] = -1;
			else data[i][j] = 0;		
		}
	}
}

void Mapper::Reload()
{
	this->Clear();
	this->Load();
}
void Mapper::Clear()
{
	for(int i = 0; i < width; i++)
		delete[] data[i];
	delete[] data;
}

void Mapper::PrintMap()
{
	bool val;
	for(int i = 199; i >= 0; i--)
	{
		for(int j = 130; j < 200; j++)
			std::cout << (int)data[i][j] << " ";
		std::cout << std::endl;
	}
}

void Mapper::PrintNodes()
{
	for (int i = 0; i < nodes.size(); i++)
		ROS_INFO_STREAM("Node " << i << ":" << nodes[i].x << ' ' << nodes[i].y);
}
	

// ---------------------------------------------------------------
// -----| BLOCK: Drawing |----------------------------------------
// ---------------------------------------------------------------

// -1 - obstacle
//  0 - empty
//  1 - path
//  2 - node
void Mapper::DrawCarcass()
{
	// TODO: better make carcass nodes on the edges of obstacles
	int a = 30, b = 170;
	data[a][a] = 2;
	nodes.push_back(Point<int>(a, a));

	data[a][b] = 2;
	nodes.push_back(Point<int>(a, b));

	data[b][a] = 2;
	nodes.push_back(Point<int>(b, a));

	data[b][b] = 2;
	nodes.push_back(Point<int>(b, b));

	DrawHPath(a, a, b);
	DrawHPath(b, a, b);
	DrawVPath(a, b, a);
	DrawVPath(a, b, b);
}

int Mapper::DrawNode(Point<int> p)
{
	int x = p.x, y = p.y;

	// if node already exists
	if (data[x][y] == 2) return -1;

	bool on_path = data[x][y] == 1;
	data[x][y] = 2;
	nodes.push_back(Point<int>(x, y));
	int node_index = nodes.size() - 1;

	// if node was created on path
	if (on_path) return node_index;

	int8_t value;
	
	for(int i = x + 1;; i++) // look top
	{
		value = (int)data[i][y];

		if(value == -1) break;
		if(value == 0) continue;
		if(value == 1 || value == 2) {
			data[i][y] = 2; 
			nodes.push_back(Point<int>(i, y));
			DrawVPath(x, i, y); 
			break; 
		}
	}
	
	for(int i = x - 1;; i--) // look bottom
	{
		value = (int)data[i][y];

		if(value == -1) break;
		if(value == 0) continue;
		if(value == 1 || value == 2) {
			data[i][y] = 2; 
			DrawVPath(i, x, y); 
			nodes.push_back(Point<int>(i, y));
			break; 
		}		
	}
	
	for(int j = y - 1;; j--) // look left
	{
		value = (int)data[x][j];

		if(value == -1) break;
		if(value == 0) continue;
		if(value == 1 || value == 2) { 
			data[x][j] = 2; 
			DrawHPath(x, j, y); 
			nodes.push_back(Point<int>(x, j));
			break; 
		}
	}

	for(int j = y + 1;; j++) // look right
	{
		value = (int)data[x][j];

		if(value == -1) break;
		if(value == 0) continue;
		if(value == 1 || value == 2) { 
			data[x][j] = 2; 
			DrawHPath(x, y, j); 
			nodes.push_back(Point<int>(x, j));
			break; 
		}
	}

	return node_index;
}

// Not including extreme points -----
void Mapper::DrawHPath(int i, int j1, int j2) // NOTE: should be private
{
	// if (j1 > j2) swap(j1, j2);
	for(int j = j1 + 1; j < j2; j++)
		data[i][j] = 1;
}

void Mapper::DrawVPath(int i1, int i2, int j) // NOTE: should be private
{
	// if (i1 > i2) swap(i1, i2);
	for(int i = i1 + 1; i < i2; i++)
		data[i][j] = 1;
}


// ---------------------------------------------------------------
// -----| BLOCK: Graph Constracting |-----------------------------
// ---------------------------------------------------------------
int** Mapper::MakeGraph()
{
	int size = nodes.size();
	int** graph = new int*[size];
	for(int i = 0; i < size; i++)
		graph[i] = new int[size];

	for(int i = 0; i < size; i++)
		for(int j = i + 1; j < size; j++)
			graph[i][j] = graph[j][i] = CountCost(nodes[i], nodes[j]);

	for(int k = 0; k < size; k++)
		graph[k][k] = INT_MAX;


	return graph;

}

// NOTE: or just with the diff in abs value
int Mapper::CountCost(Point<int> &p1, Point<int> &p2) // NOTE: should be private
{
	int cost = 0;
	int min, max;
	if (p1.x == p2.x) // for horizontal path
	{
		if(p1.y > p2.y) { min = p2.y; max = p1.y; }
		else 			{ min = p1.y; max = p2.y; }
	
		for(int j = min; j < max; j++)
		{
			if ((int)data[p1.x][j] == -1) return INT_MAX;
			cost++;
		}
	}

	if (p1.y == p2.y) // for vertical path
	{
		if(p1.x > p2.x) { min = p2.x; max = p1.x; }
		else			{ min = p1.x; max = p2.x; }

		for(int i = min; i < max; i++)
		{
			if ((int)data[i][p1.y] == -1) return INT_MAX;
			cost++;
		}

	}

	if (cost == 0) return INT_MAX;
	else return cost;
}

// ---------------------------------------------------------------
// -----| BLOCK: Transforms |-------------------------------------
// ---------------------------------------------------------------
Point<int> Mapper::CoordsToIndexPoint(Point<double> p)
{
	int j = (p.x + 1.5) * 20;
	int i = (p.y + 5) * 20;

	// ROS_INFO("Translate Coords: (%f;%f) to [%i][%i]", p.x, p.y, i, j);
	return Point<int>(i, j);
}
Point<double> Mapper::IndexToCoordsPoint(Point<int> p)
{
	double y = (double)p.x / 20 - 5;
	double x = (double)p.y / 20 - 1.5;

	// ROS_INFO("Translate Index: [%i][%i] to (%f,%f)", p.x, p.y, x, y);
	return Point<double>(x, y);
}


// ---------------------------------------------------------------
// -----| BLOCK: Pathfinder |-------------------------------------
// ---------------------------------------------------------------
vector<Point<double>> Mapper::FindPath(Point<double> &p1, Point<double> &p2)
{
	this->DrawCarcass();

	int i_start = DrawNode(CoordsToIndexPoint(p1));
	int i_finish = DrawNode(CoordsToIndexPoint(p2));
	int n = nodes.size();

	int** graph = MakeGraph();

	// Dijkstra
	vector<int> path_len(n, INT_MAX);
	vector<int> prev_node(n); 
	vector<bool> was_here(n, false);

	path_len[i_start] = 0;

	while(true)
	{
		int i = -1;

		for(int j = 0; j < n; j++)
			if(!was_here[j] && (i == -1 || path_len[j] < path_len[i]))
				i = j;

		if (i == -1)
			break;

		was_here[i] = true;

		for(int j = 0; j < n; j++)
		{
			if (graph[i][j] == INT_MAX) continue;
			int len = graph[i][j];

			if (path_len[i] + len < path_len[j]){
				path_len[j] = path_len[i] + len;
				prev_node[j] = i;
			}
		}
	}

	/* DEBUG
	for(int i = 0; i < n; i++)
		ROS_INFO("Node %u. Shortest path: %u. Previous node: %u", i, path_len[i], prev_node[i]);

	ROS_INFO_STREAM(" --------------------------- ");
	this->PrintNodes();
	*/
	
	vector<Point<double>> path;
	for(int i = i_finish; i != i_start; i = prev_node[i])
		path.push_back(IndexToCoordsPoint(nodes[i]));
	path.push_back(IndexToCoordsPoint(nodes[i_start]));

	reverse(path.begin(), path.end());

	/* 
	for(int i = 0; i < path.size(); i++)
		ROS_INFO("%i. x = %f. y = %f", i, path[i].x, path[i].y);
	*/

	
	for(int i = 0; i < n; i++)
		delete[] graph[i];
	delete[] graph;

	return path;
}
