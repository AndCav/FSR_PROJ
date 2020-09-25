#include "ros/ros.h"
#include "navigate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "boost/thread.hpp"
#include <iostream>
#include <random>
#define Width 384
#define Height 384
#define Xzero 200
#define Yzero 184
#define step 0.02


//Create class for map collision checking
class Grid{
public:
	Grid(); //costruttore
	void topic_callback(nav_msgs::OccupancyGrid data);
	bool no_collision(double x,double y);
	bool its_on(){return ison;}
	bool get_value(double x,double y);
	
private:
ros::NodeHandle _nh;
ros::Subscriber _topic_sub;
void get_position(int &pos_x,int &pos_y,double x, double y);

bool ison;
int cell[Width][Height];	
};







//RRT Graph
class RRT{
 public:
	RRT();
	RRT(const Vertex v);
	RRT(int initsize){nodes_.resize(initsize);}
	bool add_node(const Vertex v);
	void iter(int SorG);
	int closest(const Vertex v);
	bool find(const Vertex v);
	void run(Vertex source,Vertex goal);
	bool freepath(Grid map, Vertex v1, Vertex v2);
	void connect();
	void A_star_routine();
	//information functions
	int get_size(){//size_=nodes_.size();
	return size_;}
	std::vector<std::shared_ptr<Vertex>> getnodes(){return nodes_;}
	
	
	//variables
	std::vector<std::shared_ptr<Vertex>> nodes_;
	int size_;
  private:
  	std::allocator<Vertex> alloc;
  	std::vector<std::shared_ptr<Vertex>> Source_nodes;
  	std::vector<std::shared_ptr<Vertex>> Goal_nodes;
  	Vertex source;
  	Vertex goal;
  	bool fromSource;
  	bool fromGoal;
  	Grid map;
  	int MAX_ITER;
  	double MAX_LENGTH;


};



