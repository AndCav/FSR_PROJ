#include "navigate.h"
#define pi 3.14159
#define twopi 6.28318

Vertex::Vertex(double newx, double newy) {
    x = newx;
    y = newy;
  }
bool Vertex::add_link(const std::shared_ptr<Vertex> v){
	int i=0;
	bool found=false;
	while((i<links_.size())&&(!found)){
	if(*links_[i]==*v){std::cout<<"\nlink already exists"; found=true; }
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


/*

struct Des_Traj{
std::vector<double> &x_d;
std::vector<double> &y_d;
std::vector<double> &teta_d;
std::vector<double> &d_x_d;
std::vector<double> &d_y_d;
std::vector<double> &d_teta_d;
std::vector<double> &dd_x_d;
std::vector<double> &dd_y_d;
std::vector<double> &dd_teta_d;

}
void Cart_polynomials(Vertex qi,Vertex qf, double yaw_in,double yaw_fin, double k, double T, double Ts,std::vector<double> &x_d,std::vector<double> &y_d,std::vector<double> &teta_d){
double a_x=k*cos(yaw_in)-3*qf.x;
double a_y=k*sin(yaw_in)-3*qf.y;
double a_x=k*cos(yaw_in)+3*qi.x;
double a_y=k*sin(yaw_in)+3*qi.y;

// construct s
double s_buff;
double sd_buff;
double buff_x;
double buff_y;

x_d.resize(0);
y_d.resize(0);
teta_d.resize(0);
x_d.push_back(qi.x);
y_d.push_back(qi.y);
for(int i=1;i< T*Ts;i++){
s_buff=s[i];
sd_buff=sd[i];

buff_x=pow(s_buff,3)*qf.x -pow((s_buff-1),3)*qi.x+ a_x*pow(s_buff,2)*(s-1)+b_x*s_buff*pow((s-1),2);
buff_y=pow(s_buff,3)*qf.y -pow((s_buff-1),3)*qi.y+ a_y*pow(s_buff,2)*(s-1)+b_y*s_buff*pow((s-1),2);
x_d.resize(0);
x_d.push_back(buff_x);
y_d.push_back(buff_y);
if(((buff_x==0)&&(buff_x==0))
teta_d.push_back(atan2(
}



}

*/

void NAVO::odom_cb( nav_msgs::OdometryConstPtr odom ) {
    
    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _yaw);

    position.x = odom->pose.pose.position.x;
    position.y = odom->pose.pose.position.y;
    _updated=true;
    
}



NAVO::NAVO(){
        _odom_sub = _nh.subscribe("tb3_0/odom", 0, &NAVO::odom_cb, this);
        _cmd_vel_pub = _nh.advertise< geometry_msgs::Twist>("tb3_0/cmd_vel", 0);
       // _cmd_rightWheel_pub=_nh.advertise< std_msgs::Float64>("p3dx_1/right_wheel_velocity_controller/command", 1);
        //_cmd_leftWheel_pub=_nh.advertise< std_msgs::Float64>("tb3_0/left_wheel_velocity_controller/command", 1);
        _updated=false;
}

void NAVO::tripto(std::vector<Vertex> path, double k1,double k2, double k3){
int iteration=path.size()-1;
double gamma=0.0001;
double delta=0.0001;
double ep_x=0;
double ep_y=0;
//double k1=0.2;
//double k2=1.2;
//double k3=0.8;

double v=0;
double w=0;
double w_l;
double w_r;

ros::Rate r(100);
double ro=1;
int c=0;
Vertex nextpose=path[iteration];
double des_yaw=0;
double act_yaw=_yaw;

while(ros::ok){
if(_updated){
	act_yaw=_yaw;
	_updated=false;
	ro=nextpose-position;
	std::cout<<"\n ro "<<ro;
	if(ro<=0.05){
		iteration--;
		if(iteration>=0) {nextpose=path[iteration];
		ep_x=nextpose.x-position.x;
		ep_y=nextpose.y-position.y;
		des_yaw=atan2(ep_y,ep_x);
		ro=nextpose-position;}
		
		
	}
	ep_x=nextpose.x-position.x;
	ep_y=nextpose.y-position.y;
	
	//if(des_yaw-act_yaw){ v=0; w=0.*(des_yaw-act_yaw);}
	//else{
		gamma=(atan2(ep_y,ep_x) - act_yaw);
		delta=(atan2(ep_y,ep_x) - des_yaw) ;
		v=k1*ro*cos(gamma);
		if((gamma<0.05)&&(gamma>-0.05)) w=k2*gamma+k1*(gamma+k3*delta);
		else w=k2*gamma+k1*((sin(gamma)*cos(gamma))/gamma)*(gamma+k3*delta);
	//}	
	//if((v>1)||(w>1)) std::cout<<"\n thee velocities are "<<v<<" "<<w<<std::endl;
	
	
	move(w,v);
	std::cout<<" v,w:"<<v<<" "<<w;
//	moveWheel(w,v);
	r.sleep();
	
	}
	}
	std::cout<<"\nfinished";
	
	move(-w,-v);
	v=0;
	w=0;
	r.sleep();
	move(w,v);
	
	move(w,v);
	
	move(w,v);
	
	move(w,v);
	
	move(w,v);
	
	while(!_updated){}
//	moveWheel(w,v);
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


void NAVO::moveWheel(double z_ang_vel, double x_vel){
	std_msgs::Float64 Left_cmd_wheel;
	std_msgs::Float64 Right_cmd_wheel;
	//r=0.09 d=0.314
	//w_l=r/v-(dw/(2r))
	//w_r=r*v+(dw/(2r))
	double buffL=x_vel/0.09 + 1.7541*z_ang_vel;
	double buffR=x_vel/0.09 - 1.7541*z_ang_vel;
	Left_cmd_wheel.data =3*buffL;
	Right_cmd_wheel.data=3*buffR;
	_cmd_rightWheel_pub.publish(Right_cmd_wheel);
	_cmd_leftWheel_pub.publish(Left_cmd_wheel);
	
}

void NAVO::IO_FBL(std::vector<Vertex> pathInv){
std::vector<double> y1d;
std::vector<double> y2d;
std::vector<double> y1d_dot;
std::vector<double> y2d_dot;
double Ts=0.01;
double b=0.2;
double distance=0;
double necTime=0;
double vel=MAX_VEL;
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
	nstep=ceil(necTime/Ts);
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
int end_path=path.size()-1;
bool traj=true;
while((position-path[end_path])>0.2){
//std::cout<<"\n time"<<time<<"["<<position.x<<","<<position.y<<"]";
//std::cout<<"\n time"<<time<<"["<<y1d[time]<<","<<y2d[time]<<"]"<<"["<<y1<<","<<y2<<"]";
	y1=position.x+b*cos(_yaw);
	y2=position.y+b*sin(_yaw);
	u1=y1d_dot[time]+k1*(y1d[time]-y1);
	u2=y2d_dot[time]+k2*(y2d[time]-y2);
	v=u1*cos(_yaw) + u2*sin(_yaw);
	w=(-u1*sin(_yaw) + u2*cos(_yaw))/b;
	
	if(v>0.22){std::cout<<"\n VELOCITÀ CURVATURA: "<<v;v=0.22; }
	if((w>2.84)||(w<-2.84)){std::cout<<"\n tokyo Fast and Furious :"<<w;}
	move(w,v);
	
	std::cout<<"\n error e1:"<<(y1d[time]-position.x)<<" e2: "<<(y2d[time]-position.y);
	if(time<(y1d.size())&&traj){	
		time++;
		}
	
	if(time>=(y1d.size())){time--; traj=false;
	//std::cout<<" \n ******** \n fine traiettoria valori di v_des: "<<y1d_dot[time]<<" "<<y2d_dot[time];
	}
	r.sleep();
	
	}
std::vector<Vertex> path_last;
path_last.push_back(path[end_path]);
tripto(path_last,0.8,1.2,1.5);
move(0,0);
}
