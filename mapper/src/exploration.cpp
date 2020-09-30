#include "ros/ros.h"
#include "rrt.h"

void load_param( double & p, double def, std::string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
 // std::cout << name << ": " << "\t" << p << std::endl;
}
void load_param( bool & p, bool def, std::string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
 // std::cout << name << ": " << "\t" << p << std::endl;
}


int main (int argc,char** argv) {
ros::init(argc,argv,"exploration" ) ;
double x_i;
double y_i;
double x_f;
double y_f;
bool FBL;
sleep(2);

load_param(x_i, 0, "x_initial");
load_param(y_i, 0, "y_initial");
load_param(x_f, 3, "x_dest");
load_param(y_f,-3, "y_dest");
load_param(FBL,true, "FBL");
Vertex qinit(x_i,y_i);
if(FBL==true) qinit.x+=0.2;
Vertex qdest(x_f,y_f);
RRT rrt;

rrt.run(qinit,qdest);

return 0;
}
