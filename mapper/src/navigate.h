#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <tf/tf.h>

#define MAX_VEL 0.15

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
  }

  Vertex(double newx, double newy);

  bool operator==(const Vertex &vert2) {
    return ((vert2.x == x) && (vert2.y == y));
  }

  bool operator!=(const Vertex &vert2) {
    return ((vert2.x != x) || (vert2.y != y));
  }
  
  double operator-(const Vertex &vert2){
  double result=std::sqrt(std::pow(x - vert2.x , 2) + std::pow(y - vert2.y , 2));
  return result;
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


class NAVO {
    public:
        NAVO();
        
        void move(double z_ang_vel, double x_vel);
        void moveWheel(double z_ang_vel, double x_vel);
        double get_x(){ return _x;}
        double get_y(){ return _y;}
        double get_yaw(){ return _yaw;}
//        bool newValue(){bool newVal=_updated; _updated=false; return newVal;}
	void Post_Reg(std::vector<Vertex> path, double k1,double k2, double k3);
	void IO_FBL(std::vector<Vertex> path);
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _laser_sub;
        ros::Publisher _cmd_vel_pub;
        ros::Publisher _cmd_rightWheel_pub;
        ros::Publisher _cmd_leftWheel_pub;
        double _yaw;
        Vertex position;
        double _x;
        double _y;
        void odom_cb( nav_msgs::OdometryConstPtr );
        bool _updated;
        void plot();
        //study variables
        ros::Publisher xd_pub;
        ros::Publisher yd_pub;
        ros::Publisher v_pub;
        ros::Publisher w_pub;
        ros::Publisher x_pub;
        ros::Publisher y_pub;
        ros::Publisher y1_pub;
        ros::Publisher y2_pub;
        ros::Publisher err_pub;
        ros::Publisher err_y_pub;
        std::vector<double> xd_plot;
        std::vector<double> yd_plot;
        std::vector<double> x_plot;
        std::vector<double> y_plot;        
        std::vector<double> y1_plot;        
        std::vector<double> y2_plot;     
        std::vector<double> v_plot;        
        std::vector<double> w_plot;
        
};

