#include "ros/ros.h"
#include "mapper.h"

int main (int argc,char** argv) {
ros::init(argc,argv,"exploration" ) ;
Vertex qinit(-6,6.5);
Vertex qdest(0,0);
RRT rrt;

rrt.run(qinit,qdest);

return 0;
}
