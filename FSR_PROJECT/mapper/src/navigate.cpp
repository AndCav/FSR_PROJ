#include "navigate.h"
#define pi 3.14

Vertex::Vertex(double newx, double newy) {
    x = newx;
    y = newy;
  }
bool Vertex::add_link(const std::shared_ptr<Vertex> v){
	int i=0;
	bool found=false;
	while((i<links_.size())&&(!found)){
	if(links_[i]==v){std::cout<<"\nlink already exists"; found=true; }
	i++;	
	}
	if(!found){
	links_.push_back(v);
	double costo = std::sqrt(std::pow(x - v->x , 2) + std::pow(y - v->y , 2));
	cost_.push_back(costo);
	
//	std::cout<<"\n "<<pos<<" added link from: "<<this->x<<" "<<this->y<<"to this:"<<v.x<<" "<<v.y;
	return true;
	}
	else return false;
}


void NAVO::odom_cb( nav_msgs::OdometryConstPtr odom ) {
    
    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _yaw);

    position.x = odom->pose.pose.position.x;
    position.y = odom->pose.pose.position.y;
   // _updated=true;
    
}


void NAVO::laser_cb( sensor_msgs::LaserScanConstPtr ) {
}

NAVO::NAVO(){
        _laser_sub = _nh.subscribe("p3dx_1/laser/scan",0,&NAVO::laser_cb,this);
        _odom_sub = _nh.subscribe("p3dx_1/odom", 0, &NAVO::odom_cb, this);
        _cmd_vel_pub = _nh.advertise< geometry_msgs::Twist>("p3dx_1/cmd_vel", 0);
        _updated=false;
}

void NAVO::tripto(std::vector<Vertex> path){
int iteration=path.size()-1;
double gamma=0.0001;
double delta=0.0001;
double ep_x=0;
double ep_y=0;
double k1=1;
double k2=1;
double k3=0.1;

double v=0;
double w=0;

ros::Rate r(100);
int ro=1;
int c=0;
Vertex nextpose=path[iteration];
double des_yaw=0;

while((position-path[0]>0.05)){
	ro=nextpose-position;
	if(ro<0.05){
	std::cout<<"\n another point reached \n";
		iteration--;
		if(iteration>=0) nextpose=path[iteration];
		ep_x=nextpose.x-position.x;
		ep_y=nextpose.y-position.y;
		des_yaw=atan2(ep_y,ep_x);
		ro=nextpose-position;
	}
	ep_x=nextpose.x-position.x;
	ep_y=nextpose.y-position.y;
	if(ep_x==0||ep_y==0){gamma=0;delta=0;}
	else{
		gamma=(atan2(ep_y,ep_x) - _yaw);
		delta=atan2(ep_y,ep_x) - des_yaw;
	}
	v=k1*ro*cos(gamma);
	w=k2*gamma+k1*((sin(gamma)*cos(gamma))/gamma)*(gamma+k3*delta);
	if(v>0.5) v=0.5;
	if(w>0.2) w=0.2;
	move(w,v);
	r.sleep();
	}
	v=0;
	w=0;
	move(w,v);
}

void NAVO::move(double z_ang_vel, double x_vel){
	geometry_msgs::Twist cmd;
        cmd.angular.x=0;
        cmd.angular.y=0;
        cmd.angular.z = z_ang_vel;
        cmd.linear.x = x_vel;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        _cmd_vel_pub.publish(cmd);
}


