#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include "boost/thread.hpp"
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <random>
#define Width 384
#define Height 384
#define Xzero 200
#define Yzero 184

#define MAX_ITER 1000


//Create class for map collision checking
class Grid{
public:
	Grid(); //costruttore
	void topic_callback(nav_msgs::OccupancyGrid data);
	bool no_collision(double x,double y);
	bool its_on(){return ison;}
	
private:
ros::NodeHandle _nh;
ros::Subscriber _topic_sub;
void get_position(int &pos_x,int &pos_y,double x, double y);

bool ison;
int cell[Width][Height];	
};

Grid::Grid(){
_topic_sub = _nh.subscribe("/costmap/costmap/costmap", 10, &Grid::topic_callback,this);
 ison=false;
}

void Grid::topic_callback(nav_msgs::OccupancyGrid data){

for (int y = 0; y < Height; y++){
for (int x = 0; x < Width; x++){
if(data.data[y * Width + x]==-1) cell[383-y][x] = 100;
}
}
ison=true;
}

void Grid::get_position(int &pos_x,int &pos_y,double x, double y){
pos_x= Xzero + int(x/0.05);
pos_y= Yzero - int(y/0.05);
}


bool Grid::no_collision(double x,double y){
int pos_x; int pos_y;
if((x>9)||(x<-9)||(y>9)||(y<-9)) return false;
get_position(pos_x,pos_y,x,y);

if((cell[pos_y][pos_x]>60)||(cell[pos_y][pos_x]==-1)) return false ;
else return true ;
}



/////CREATING CLASS VERTEX 

class Vertex {

 public:

  Vertex() {
    x = 0;
    y = 0;
    visited=false;
    links_.resize(0);
    cost_.resize(0);
   // heur_cost_.resize(0);
  };

  Vertex(double newx, double newy) {
    x = newx;
    y = newy;
  }

  bool operator==(const Vertex &vert2) {
    return ((vert2.x == x) && (vert2.y == y));
  }

  bool operator!=(const Vertex &vert2) {
    return ((vert2.x != x) || (vert2.y != y));
  }
  
  double operator-(const Vertex &vert2){
  return std::sqrt(std::pow(x - vert2.x , 2) + std::pow(y - vert2.y , 2));
  }
  void copy(const Vertex &vert2){
  x=vert2.x;
  y=vert2.y;
  visited=vert2.visited;
//  links_=vert2.links_;
//  cost_=vert2.cost_;
  heur_cost_=vert2.heur_cost_;

  }
  
  //information functions
  bool is_visited(){ return visited;}
  bool set_visited(const bool b){visited=b;}
  int numberoflinks(){return links_.size();}
  bool add_link(const std::shared_ptr<Vertex> v);
  std::vector<std::shared_ptr<Vertex>> get_links(){return links_;}
  double calc_heur(const Vertex& vert2){
  heur_cost_=std::sqrt(std::pow(x - vert2.x , 2) + std::pow(y - vert2.y , 2));
  return heur_cost_;
  }
  double get_heur(){return heur_cost_;}
  double set_total_cost(const double cost){return total_cost=cost;}
  
  
  //variables
  double x;
  double y;
  bool visited;
  std::vector<std::shared_ptr<Vertex>> links_;
  std::vector<double> cost_;
  double heur_cost_;
  double total_cost;
  std::shared_ptr<Vertex> parent_;
};




//RRT Graph
class RRT{
 public:
	RRT();
	RRT(const Vertex v);
	RRT(int initsize){nodes_.resize(initsize);}
	bool add_node(const Vertex v);
	void iter();
	int closest(const Vertex v);
	bool find(const Vertex v);
	void run();
	
	//information functions
	int get_size(){//size_=nodes_.size();
	return size_;}
	std::vector<std::shared_ptr<Vertex>> getnodes(){return nodes_;}
	
	
	//variables
	std::vector<std::shared_ptr<Vertex>> nodes_;
	int size_;
  private:
  	std::allocator<int> alloc;


};



