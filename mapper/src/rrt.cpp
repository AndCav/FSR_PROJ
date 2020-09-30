#include "rrt.h"



Grid::Grid(){
_topic_sub = _nh.subscribe("/costmap/costmap/costmap", 10, &Grid::topic_callback,this);
 ison=false;
}



void Grid::topic_callback(nav_msgs::OccupancyGrid data){

for (int y = 0; y < Height; y++){
for (int x = 0; x < Width; x++){
if(cell[Height-1-y][x]=data.data[y * Width + x]<0){cell[Height-1-y][x]=data.data[y * Width + x]=100;}
else{cell[Height-1-y][x]=data.data[y * Width + x];}
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

if(cell[pos_y][pos_x]<20){ //std::cout<<"\n free "<<cell[pos_y][pos_x]; 
							return true ;}
else {return false;}
}

bool Grid::get_value(double x,double y){
int pos_x; int pos_y;
if((x>9)||(x<-9)||(y>9)||(y<-9)) return false;
get_position(pos_x,pos_y,x,y);
if(cell[pos_y][pos_x]<20){
std::cout<<"\n free "<<cell[pos_y][pos_x]; 
							return true ;}
else {return false;}
}

RRT::RRT(){
	marker_id=0;
	nodes_.resize(0);
	size_=0;
	MAX_ITER=50;
	MAX_LENGTH=1;
	step=0.05;
	fromSource=false;
	fromGoal=false;
	vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1500 );
}

RRT::RRT(const Vertex v){
	
	marker_id=0;
	nodes_.resize(0);
	MAX_ITER=50;
	MAX_LENGTH=1;
	step=0.05;
	nodes_.push_back(std::make_shared<Vertex>(v.x,v.y));
	size_=1;
	fromSource=false;
	fromGoal=false;
	vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1500 );

}

bool RRT::find(const Vertex v){
bool found=false;
int f=0;
while((!found)&&(f<nodes_.size())){
	if(*nodes_[f]==v) found=true;
	f++;
	}
return found;
}

bool RRT::add_node(const Vertex v){
if(find(v)) {//std::cout<<"\n already exist";
 return false;
}
else{	nodes_.push_back(std::make_shared<Vertex>(v.x,v.y));
	size_++;
	//std::cout<<"\nNODE ADDED";
	return true;
	}

}



//****************
int FIND(const Vertex v,std::vector<std::shared_ptr<Vertex>> &nodes){
bool found=false;
int f=0;
int index=-1000;
while((!found)&&(f<nodes.size())){
	if(*nodes[f]==v) {found=true;index=f;}
	f++;
	}
return index;
}

bool ADD_NODE(const Vertex v, std::vector<std::shared_ptr<Vertex>> &nodes){
if(FIND(v,nodes)>=0) {
//std::cout<<"\n already exist";
 return false;
}
else{	
	nodes.push_back(std::make_shared<Vertex>(v.x,v.y));
	//std::cout<<"\nNODE ADDED";
	return true;
	}

}
int CLOSEST(const Vertex v,std::vector<std::shared_ptr<Vertex>> &nodes){
double max_dist=100;
double buff=0;
int index=0;
for(int c=0;c<nodes.size();c++){
	buff=(*nodes[c] - v);
	if(buff<max_dist){max_dist=buff; index=c;}
}
return index;
}

//********************************
int RRT::closest(const Vertex v){
double max_dist=100;
double buff=0;
int index=0;
for(int c=0;c<nodes_.size();c++){
	buff=(*nodes_[c] - v);
	if(buff<max_dist){max_dist=buff; index=c;}
}
return index;
}


bool RRT::createMarker(const Vertex v1,const Vertex v2, std::string from){

visualization_msgs::Marker marker;
visualization_msgs::Marker marker_line;

if(from=="source"){
marker.color.a = 1.0;
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
marker.ns = "Source";
marker_line.color.a = 1.0;
marker_line.color.r = 1.0;
marker_line.color.g = 1.0;
marker_line.color.b = 0.0;

marker_line.ns = "Source";
}
else if(from=="goal"){
marker.color.a = 1.0;
marker.color.r = 0.0;
marker.color.g = 0.0;
marker.color.b = 1.0;
marker.ns = "Goal";
marker_line.color.r = 0.0;
marker_line.color.g = 1.0;
marker_line.color.b = 1.0;
marker_line.ns = "Goal";
}
else{
marker.color.r = 255;
marker.color.g = 0;
marker.color.b = 0;
marker.color.a = 1.0;
marker.ns = "connect";
marker_line.color.r = 255;
marker_line.color.g = 0;
marker_line.color.b = 0;
marker_line.color.a = 1.0;
marker_line.ns = "connect";
}
marker.header.frame_id = "map";
marker_id++;
marker.header.stamp = ros::Time();
marker.id = marker_id;
marker_id++;
marker.type =2;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = v1.x;
marker.pose.position.y = v1.y;
marker.pose.position.z = 0.01;
marker.scale.x = 0.1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;


vis_pub.publish( marker );


marker_id++;
marker.header.stamp = ros::Time();
marker.id = marker_id;
marker_id++;
marker.type = visualization_msgs::Marker::SPHERE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = v2.x;
marker.pose.position.y = v2.y;
marker.pose.position.z = 0.01;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1.0;
vis_pub.publish(marker);

//publish line
geometry_msgs::Point p1;
geometry_msgs::Point p2;
p1.x=v1.x;
p1.y=v1.y;
p1.z=0.01;
p2.x=v2.x;
p2.y=v2.y;
p2.z=0.01;
marker_line.header.frame_id = "map";
marker_id++;
marker_line.header.stamp = ros::Time();
marker_line.id = marker_id;
marker_id++;
marker_line.type = 4;
marker_line.action = visualization_msgs::Marker::ADD;

marker_line.pose.orientation.x = 0.0;
marker_line.pose.orientation.y = 0.0;
marker_line.pose.orientation.z = 0.0;
marker_line.pose.orientation.w = 1.0;
/*
marker_line.pose.position.x = (v1.x+v2.x)/2;
marker_line.pose.position.y = (v1.y+v2.y)/2;
marker_line.pose.position.z = 0.5;
*/

marker_line.points.push_back(p1);
marker_line.points.push_back(p2);
marker_line.color.a = 1.0;

marker_line.scale.x = 0.05;

vis_pub.publish(marker_line);

return true;
}


bool open_find(const std::vector<std::shared_ptr<Vertex>> OPEN,std::shared_ptr<Vertex> v){
bool found=false;
int f=0;
while((!found)&&(f<OPEN.size())){
	if(*OPEN[f]==*v) found=true;
	f++;
	}
return found;
}

void RRT::A_star_routine(){

std::vector<std::shared_ptr<Vertex>> open;

int sizeoflinks=nodes_[0]->links_.size();
std::shared_ptr<Vertex> Qbest;
Qbest=nodes_[0];
Qbest->set_visited(true);
Qbest->total_cost=10000;
Qbest->calc_heur(goal);
double cost=nodes_[0]->calc_heur(goal)*10;
double buff_cost;
bool open_empty=false;
open.push_back(Qbest);
int open_size=open.size();
std::cout<<"\n list size "<<open_size;
std::cout<<"\n nodes size  "<<nodes_.size();

//std::cout<<"links size "<<Qbest->links_.size()<<std::endl);
//!open.empty()
while((*Qbest!=goal)&&(open_size!=0)){
	//std::cout<<"\n ciao"<<std::endl;
	double f_func=0;
	int newbestpos=0;
	cost=open[0]->total_cost + open[0]->get_heur();
	for (int i=0;i<open.size();i++){
		f_func=open[i]->total_cost + open[i]->get_heur();
		if(cost>f_func){ cost=f_func; newbestpos=i;}
	}
	
	
	//std::cout<<"\n ciao2"<<*open.begin();
	Qbest=open[newbestpos];
	open[newbestpos]=open[open.size()-1];
	open.pop_back();
	
	
	//std::cout<<"ciao3"<<std::endl;
	for(int i=0;i<Qbest->links_.size();i++){
	//cerca il costo migliore
		if(!Qbest->links_[i]->is_visited()){
			Qbest->links_[i]->set_visited(true);
			Qbest->links_[i]->parent_=Qbest;
			Qbest->links_[i]->calc_heur(goal);
			Qbest->links_[i]->total_cost=Qbest->total_cost + Qbest->cost_[i];
			open.push_back(Qbest->links_[i]);		
			}
		else if((Qbest->links_[i]->total_cost)>(Qbest->total_cost + Qbest->cost_[i])){
			Qbest->links_[i]->parent_=Qbest;
			
			if(!open_find(open,Qbest->links_[i])){
				open.push_back(Qbest->links_[i]);
				Qbest->links_[i]->calc_heur(goal);
				Qbest->links_[i]->total_cost=Qbest->total_cost + Qbest->cost_[i];
				}
			else {
				Qbest->links_[i]->calc_heur(goal);
				Qbest->links_[i]->total_cost=Qbest->total_cost + Qbest->cost_[i];
				}
			}
		}
	open_size=open.size();
	
	//std::cout<<"\n size of open "<<open_size;

	}
	//ROS_INFO("ci sono");
	if(*Qbest!=goal){ROS_INFO("ho fallito");}
	else{
	//std::shared_ptr<Vertex> printer;
	std::vector<Vertex> path;
	//ROS_INFO("ci sono");
	//printer=Qbest;
	while(*Qbest!=source){
		ROS_INFO("[%.2f,%.2f]",Qbest->x,Qbest->y);
		//map.get_value(Qbest->x,Qbest->y);
		Vertex buff(Qbest->x,Qbest->y);
		path.push_back(buff);
		Qbest=Qbest->parent_;
	}
	Vertex buff(Qbest->x,Qbest->y);
		path.push_back(buff);
	ROS_INFO("path finito");
	ROS_INFO("\n vafancul");
	for(int i=0;i<nodes_.size();i++){
	nodes_[i].reset();
	}/*
	for(int i=0;i<Source_nodes.size();i++){
	Source_nodes[i].reset();
	}
	for(int i=0;i<Goal_nodes.size();i++){
	Goal_nodes[i].reset();
	}
	*/
	
		
		sleep(1);
		navigator.IO_FBL(path);
		//navigator.Post_Reg(path,0.21,1.2,0.8);
	}
	
}
void RRT::iter(int SorG){
	std::cout<<"start";
	bool free=false;
	double x_rand=0;
	double y_rand=0;
	double dx=0;
	double dy=0;
	double xstep=0;
	double ystep=0;
	int nearpos=0;
	double dist=1000;
	int count=1;
	Vertex qnew;
	Vertex qnear;
	Vertex qrand;
	bool accomplished=false;
	std::random_device devicex;
	std::random_device devicey;
	std::mt19937 engx(devicex());
	std::mt19937 engy(devicey());
	std::uniform_int_distribution<int> distX(-900, 900);
	std::uniform_int_distribution<int> distY(-900, 900);
	std::vector<std::shared_ptr<Vertex>> nodes;
	if (SorG==1) ADD_NODE(source,nodes);
	if (SorG==2) ADD_NODE(goal,nodes);
	while(!map.its_on()){
	//do nothing
	};
	int s=0;
	ros::Rate r(100);
	
	while(s < MAX_ITER){
		
		x_rand= distX(engx)/100;
		y_rand= distY(engy)/100;
		qrand.x=x_rand;
		qrand.y=y_rand;
		if(FIND(qrand,nodes)>=0){ 
		}
		else{
			nearpos=CLOSEST(qrand,nodes);
			qnear.x=nodes[nearpos]->x;
			qnear.y=nodes[nearpos]->y;
			dist=qrand-qnear;
			double cat_x=qrand.x-qnear.x;
			double cat_y=qrand.y-qnear.y;
			double ang = atan2(cat_y, cat_x);

			double cosTeta = cos(ang);
			double sinTeta = sin(ang);
			
			if(dist!=MAX_LENGTH){
				qnew.x=qnear.x + cosTeta*MAX_LENGTH;
				qnew.y=qnear.y + sinTeta*MAX_LENGTH;
				dist=qnew-qnear;
				}
			free=map.no_collision(qnew.x,qnew.y);
			
			if(free){
				double distance=0;
 				count=1;
				xstep=step*cosTeta;
				ystep=step*sinTeta;
				
				while(free&&(distance<dist)){
					distance=step*count;
					qnew.x=qnear.x+xstep*count;
					qnew.y=qnear.y+ystep*count;
					accomplished=(qnew-qnear)>=dist;
					free=map.no_collision(qnew.x,qnew.y);
					if(free&&(distance<dist)){
						count++;
						}
					else if(free&&((step*count)>=dist)){
						//add_node
						

						if(ADD_NODE(qnew,nodes)){ 
							nodes[nodes.size()-1]->add_link(nodes[nearpos]);
							nodes[nearpos]->add_link( nodes[nodes.size()-1]);
							//if(SorG==1) createMarker(*nodes[nearpos],*nodes[nodes.size()-1],"source");
							//if(SorG==2) createMarker(*nodes[nearpos],*nodes[nodes.size()-1],"goal");
							s++;
							//std::cout<<"\n s:"<<s;
						 }
						count=1;
						}
				
					
				}
			}
		}
	r.sleep();
	}
	if (SorG==1){
		for(int o=0;o<nodes.size();o++){
			Source_nodes.push_back(nodes[o]);
			//std::cout<<"\n ["<<nodes[o]->x<<","<<nodes[o]->y<<"]";
		}
		
	fromSource=true;
	connect();
	}
	else{
		for(int o=0;o<nodes.size();o++){
			Goal_nodes.push_back(nodes[o]);
			//std::cout<<"\n ["<<nodes[o]->x<<","<<nodes[o]->y<<"]";
		}
		
	fromGoal=true;
	while(ros::ok()){sleep(10);}
	}

}


void RRT::run(Vertex s,Vertex g) {
	fromSource=false;
	fromGoal=false;
	source=s;
	goal=g;
	
	boost::thread get_fromSource_t( &RRT::iter, this,1);
	boost::thread get_fromGoal_t( &RRT::iter, this,2);
	//boost::thread connect_t( &RRT::connect, this);
	
	ros::spin();
	/*Source_nodes.clear();
	Source_nodes.resize(0);
	Goal_nodes.clear();
	Goal_nodes.resize(0);
	nodes_.clear();
	nodes_.resize(0);*/

}

void RRT::connect(){
	
	while(!(fromSource&&fromGoal)){}
	std::cout<<" \n size of sources: "<<Source_nodes.size();
	std::cout<<" \n size of goal: "<<Goal_nodes.size();
	bool done=false;
	std::random_device devicex;
	std::random_device devicey;
	std::mt19937 engx(devicex());
	std::mt19937 engy(devicey());
	std::uniform_int_distribution<int> distX(-750,700);
	std::uniform_int_distribution<int> distY(-700,700);
	bool from_source=true;
	bool connected=false;
	int connection=0;
	double x_rand=0;
	double y_rand=0;
	double dx=0;
	double dy=0;
	double xstep=0;
	double ystep=0;
	int nearpos=0;
	double dist=1000;
	int count=1;
	Vertex qnew;
	Vertex qnear;
	Vertex qrand;
	int ConIndex;
	bool addnode=false;
	int SourceSize;
	int GoalSize;
	std::cout<<" \n begin while !connected";
while(!connected){
	x_rand= distX(engx)/100;
	y_rand= distY(engy)/100;
	qrand.x=x_rand;
	qrand.y=y_rand;
	if(from_source){
		
		nearpos=CLOSEST(qrand,Source_nodes);
		qnear.x=Source_nodes[nearpos]->x;
		qnear.y=Source_nodes[nearpos]->y;
	}
	else {	
		nearpos=CLOSEST(qrand,Goal_nodes);
		qnear.x=Goal_nodes[nearpos]->x;
		qnear.y=Goal_nodes[nearpos]->y;
		
	}
	
	dist=qrand-qnear;
	
	double cat_x=qrand.x-qnear.x;
	double cat_y=qrand.y-qnear.y;
	double ang = atan2(cat_y, cat_x);
	double cosTeta = cos(ang);
	double sinTeta = sin(ang);
	if(dist>MAX_LENGTH){
				qnew.x=qnear.x+cosTeta*MAX_LENGTH;
				qnew.y=qnear.y+sinTeta*MAX_LENGTH;
				dist=qnew-qnear;
				}
	else{   qnew.x=qrand.x;
		qnew.y=qrand.y;
				}
	bool free=map.no_collision(qnew.x,qnew.y);
	
	//EXPANSION
	xstep=step*cosTeta;
	ystep=step*sinTeta;
	addnode=false;
	
	while(free&&((step*count)<dist)){
		
		qnew.x=qnear.x+xstep*count;
		qnew.y=qnear.y+ystep*count;
		free=(map.no_collision(qnew.x,qnew.y));
		if(free){
			count++;
			}
		else if((!free)&&(count>2)){

				qnew.x-=xstep;
				qnew.y-=ystep;
				
				addnode=true;
				
			}
	}
	if((step*count)>=dist) addnode=true;
	count=1;
	if(addnode){ addnode=false;

		if(from_source){
			ConIndex=FIND(qnew,Goal_nodes);
			if(ConIndex>=0){connection++;
				Source_nodes[nearpos]->add_link(Goal_nodes[ConIndex]);
				Goal_nodes[ConIndex]->add_link(Source_nodes[nearpos]);
				
				//createMarker(*Source_nodes[nearpos],*Goal_nodes[ConIndex],"connect");
			}// *********************END EXPANSION 
			else{// *********************BEGIN CONNECTION FROM GOAL TO SOURCE
			
				if(ADD_NODE(qnew,Source_nodes)){ SourceSize=Source_nodes.size()-1;
					Source_nodes[SourceSize]->add_link(Source_nodes[nearpos]);
					Source_nodes[nearpos]->add_link( Source_nodes[SourceSize]);
				//createMarker(qnear,qnew,"source");
				}
		
			
			count=1;
			nearpos=CLOSEST(qnew,Goal_nodes);
			qnear.x=Goal_nodes[nearpos]->x;
			qnear.y=Goal_nodes[nearpos]->y;
			dist=qnew-qnear;
			cat_x=qnew.x-qnear.x;
			cat_y=qnew.y-qnear.y;
			ang = atan2(cat_y, cat_x);
			cosTeta = cos(ang);
			sinTeta = sin(ang);
			
			
			xstep=step*cosTeta;
			ystep=step*sinTeta;
			addnode=false;
			free=map.no_collision(qnew.x,qnew.y);
			
			//std::cout<<" \n begin whiel from goal to source";
			while(free&&((step*count)<dist)){

				qnew.x=qnear.x+xstep*count;
				qnew.y=qnear.y+ystep*count;
				//accomplished=(qnew-qnear)>=dist;
				free=(map.no_collision(qnew.x,qnew.y));
				if(free){
					count++;
				}
				else if((!free)&&((step*count)>=dist)){
					qnew.x-=xstep;
					qnew.y-=ystep;
					addnode=true;
				}
				else{
				if(count==1){addnode=false;}
				else{
					qnew.x-=xstep;
					qnew.y-=ystep;
					////std::cout<<" \n qnew coordinate:"<<qnew.x<<"  "<<qnew.y;
					addnode=true;
				}
					
				}
			}
			count=1;
			if(addnode){ addnode=false;
				
			//std::cout<<" \n just add a new node to goal";
				ConIndex=FIND(qnew,Source_nodes);
				if(ConIndex>=0){
				//std::cout<<" \nfound a connection";
				connection++;
				//createMarker(*Goal_nodes[nearpos],*Source_nodes[ConIndex],"connect");
				Goal_nodes[nearpos]->add_link(Source_nodes[ConIndex]);
				Source_nodes[ConIndex]->add_link(Goal_nodes[nearpos]);
				}
				else{
				if(ADD_NODE(qnew,Goal_nodes)){ GoalSize=Goal_nodes.size()-1;
				Goal_nodes[GoalSize]->add_link(Goal_nodes[nearpos]);
				Goal_nodes[nearpos]->add_link( Goal_nodes[GoalSize]);
				//createMarker(qnew,qnear,"goal");
						}
				}
	
				}
			}
			from_source=false;
		}
		else{ //viceversa
			ConIndex=FIND(qnew,Source_nodes);
			if(ConIndex>=0){
			//createMarker(*Source_nodes[ConIndex],*Goal_nodes[nearpos],"connection");
			connection++;
			
				//createMarker(*Goal_nodes[nearpos],*Source_nodes[ConIndex],"connect");
			Goal_nodes[nearpos]->add_link(Source_nodes[ConIndex]);
			Source_nodes[ConIndex]->add_link(Goal_nodes[nearpos]);
			}// *********************END EXPANSION 
			else{// *********************BEGIN CONNECTION FROM GOAL TO SOURCE
			
				if(ADD_NODE(qnew,Goal_nodes)){GoalSize=Goal_nodes.size()-1;
					Goal_nodes[GoalSize]->add_link(Goal_nodes[nearpos]);
					Goal_nodes[nearpos]->add_link( Goal_nodes[GoalSize]);
					//createMarker(qnew,qnear,"goal");
				}
			count=1;
			nearpos=CLOSEST(qnew,Source_nodes);
			qnear.x=Source_nodes[nearpos]->x;
			qnear.y=Source_nodes[nearpos]->y;
			dist=qnew-qnear;
			
			cat_x=qnew.x-qnear.x;
			cat_y=qnew.y-qnear.y;
			ang = atan2(cat_y, cat_x);
			cosTeta = cos(ang);
			sinTeta = sin(ang);
			xstep=step*cosTeta;
			ystep=step*sinTeta;
			
			addnode=false;
			free=map.no_collision(qnew.x,qnew.y);
			
			while(free&&((step*count)<dist)){
			qnew.x=qnear.x+xstep*count;
			qnew.y=qnear.y+ystep*count;
			free=(map.no_collision(qnew.x,qnew.y));
				if(free){
					count++;
				}
				else if((!free)&&((step*count)>=dist)){
					qnew.x-=xstep;
					qnew.y-=ystep;
					addnode=true;
				}
				else{
				if(count==1){addnode=false;}
				else{
					qnew.x-=xstep;
					qnew.y-=ystep;
					addnode=true;
					}
				}
			}
			if(addnode){
				
				addnode=false;
				ConIndex=FIND(qnew,Goal_nodes);
				if(ConIndex>=0){
					
				//createMarker(*Source_nodes[nearpos],*Goal_nodes[ConIndex],"connect");
					connection++;
					Source_nodes[nearpos]->add_link(Goal_nodes[ConIndex]);
					Goal_nodes[ConIndex]->add_link(Source_nodes[nearpos]);
					
					}
				else{	
				if(ADD_NODE(qnew,Source_nodes)){
				
				
				//createMarker(qnear,qnew,"source");
				SourceSize=Source_nodes.size()-1;
				Source_nodes[SourceSize]->add_link(Source_nodes[nearpos]);
				Source_nodes[nearpos]->add_link(Source_nodes[SourceSize]);
							}
				}
			}	
			count=1;	
			}
			from_source=true;
		}
}	
std::cout<<"\n connection:"<<connection;
if(connection>=20) connected=true;	
}
std::cout<<"\n size of sources: "<<Source_nodes.size()<<" "<<SourceSize;
std::cout<<"\n size of goal: "<<Goal_nodes.size()<<" "<<GoalSize;
for(int o=0;o<Source_nodes.size();o++){
nodes_.push_back(Source_nodes[o]);
}

for(int o=Goal_nodes.size()-1;o>=0;o--){
if(!find(*Goal_nodes[o])){ nodes_.push_back(Goal_nodes[o]);
				}
}

size_=nodes_.size();
std::cout<<"\n size of nodes: "<<nodes_.size();
A_star_routine();
}


/*
int main (int argc,char** argv) {
ros::init(argc,argv,"rrt" ) ;
Vertex qdest(0,0);
Vertex qinit(-6,6.5);
RRT rrt;

rrt.run(qinit,qdest);

return 0;
}
/*
if(ConIndex>=0){
		Source_nodes.push_back(Goal_nodes[ConIndex]);
		}
		else{*/

