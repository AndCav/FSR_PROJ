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
    _updated=true;
    
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
double k1=0.1;
double k2=0.3;
double k3=0.2;

double v=0;
double w=0;

ros::Rate r(100);
double ro=1;
int c=0;
Vertex nextpose=path[iteration];
double des_yaw=0;

while((position-path[0]>0.0)){
if(_updated){
	_updated=false;
	ro=nextpose-position;
	std::cout<<"\n ro "<<ro;
	if(ro<=0.05){
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
		gamma=(atan2(ep_y,ep_x) - _yaw);
		delta=atan2(ep_y,ep_x) - des_yaw +pi;
		v=k1*ro*cos(gamma);
		if((gamma==0)||(delta==0)) w=0;
		else w=k2*gamma+k1*((sin(gamma)*cos(gamma))/gamma)*(gamma+k3*delta);
		
	//if((v>1)||(w>1)) std::cout<<"\n thee velocities are "<<v<<" "<<w<<std::endl;
	move(w,v);
	r.sleep();
	
	}
	}
	std::cout<<"\nfinished";
	v=0;
	w=0;
	move(w,v);
	while(!_updated){}
	double finalerror;
	finalerror=position-path[0];
	std::cout <<"\n finalerror :"<<finalerror;
	std::cout<<"\nfinished";
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

void NAVO::IO_FBL(std::vector<Vertex> pathInv){
std::vector<double> y1d;
std::vector<double> y2d;
std::vector<double> y1d_dot;
std::vector<double> y2d_dot;
double Ts=0.01;
double b=0.1;
double distance=0;
double necTime=0;
double vel=MAX_VEL*0.4;
int nstep;
double cat_x;
double cat_y;
double STEP=0;
double xstep=0;
double ystep=0;
int actualplace=0;
std::vector<Vertex> path;
for(int i=pathInv.size()-1;i>=0;i--){
path.push_back(pathInv[i]);
}
std::cout<<"\n new path";
for(int i=0;i<path.size();i++){
std::cout<<"\n ["<<path[i].x<<","<<path[i].y<<"]";
}
//generate trajectory
for(int i=1;i<path.size();i++){
	
	distance=path[i]-path[i-1];
	necTime=distance/vel;
	nstep=round(necTime/Ts);
	STEP=distance/nstep;
	cat_x=path[i].x-path[i-1].x;
	cat_y=path[i].y-path[i-1].y;
	double ang = atan2(cat_y, cat_x);
	xstep=STEP*cos(ang);
	ystep=STEP*sin(ang);
	
	double nextx=path[i-1].x;
	double nexty=path[i-1].y;
	for(int j=0;j<nstep;j++){
		y1d.push_back(nextx+j*xstep);
		y2d.push_back(nexty+j*ystep);
	}
}
y1d.push_back(path[path.size()-1].x);
y2d.push_back(path[path.size()-1].y);
y1d_dot.push_back(0);
y2d_dot.push_back(0);
double derivative;
for(int i=1;i<y1d.size();i++){
derivative=(y1d[i]-y1d[i-1])/Ts;
y1d_dot.push_back(derivative);


derivative=(y2d[i]-y2d[i-1])/Ts;
y2d_dot.push_back(derivative);

}

int time=0;
ros::Rate r(100);
double y1=0;
double y2=0;
double k1=2;
double k2=1.5;
double u1;
double u2;
double v;
double w;
std::cout<<"\n \n ********** \n";
while((position-path[path.size()-1])>0.05){
	
std::cout<<"\n time"<<time<<"["<<position.x<<","<<position.y<<"]";
std::cout<<"\n time"<<time<<"["<<y1d[time]<<","<<y2d[time]<<"]"<<"["<<y1<<","<<y2<<"]";
	y1=position.x+b*cos(_yaw);
	y2=position.y+b*sin(_yaw);
	u1=y1d_dot[time]+k1*(y1d[time]-y1);
	u2=y2d_dot[time]+k2*(y2d[time]-y2);
	v=u1*cos(_yaw) + u2*sin(_yaw);
	w=(-u2*sin(_yaw) + u2*cos(_yaw))/b;
	move(w,v);
	if(time<(y1d.size())){	
		time++;
		}
	r.sleep();
	
	}

move(0,0);
}
