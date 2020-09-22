#include "ros/ros.h"
#include "mapper.h"

int main (int argc,char** argv) {
ros::init(argc,argv,"exploration" ) ;
Vertex qinit(0,0);
Vertex qdest(-5,-3);
RRT rrt;

rrt.run(qinit,qdest);

return 0;
}
