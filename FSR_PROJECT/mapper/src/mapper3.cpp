#include "mapper.h"
// *******************GRID FUNCTIONS************************
Grid::Grid(){
_topic_sub = _nh.subscribe("/costmap/costmap/costmap", 10, &Grid::topic_callback,this);
 ison=false;
}



void Grid::topic_callback(nav_msgs::OccupancyGrid data){

for (int y = 0; y < Height; y++){
for (int x = 0; x < Width; x++){
cell[383-y][x]=data.data[y * Width + x];
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

//std::cout<<"\n il valore è :"<<cell[pos_y][pos_x]<<std::endl;
if((cell[pos_y][pos_x]>30)||(cell[pos_y][pos_x]<0)){ 
							//std::cout<<"found obstacle"; 
							return false ;}
else return true ;
}




//*********************************END GRID FUNCTIONS


//**************** RRT FUNCTIONS

RRT::RRT(){
	nodes_.resize(0);
	size_=0;
	MAX_ITER=50;
	MAX_LENGTH=0.4;
}

RRT::RRT(const Vertex v){
	
	nodes_.resize(0);
	nodes_.push_back(std::allocate_shared<Vertex> (alloc,v));
	size_=1;

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

int RRT::find(const Vertex v, std::string where){
bool found=false;
int f=0;
if(where=="Source"){
while((!found)&&(f<Source_nodes.size())){
	if(*Source_nodes[f]==v) found=true;
	f++;
	}
}
else (where=="Goal"){
while((!found)&&(f<Goal_nodes.size())){
	if(*Goal_nodes[f]==v) found=true;
	f++;
	}
}
	
return f;
}


bool RRT::add_node(const Vertex v){
if(find(v)) {//std::cout<<"\n already exist";
 return false;
}
else{	
	nodes_.push_back(std::allocate_shared<Vertex> (alloc,v));
	size_++;
	//std::cout<<"\nNODE ADDED";
	return true;
	}

}


bool add_node(const Vertex v, std::string where){
if(find(v,where)>=0) {
//std::cout<<"\n already exist";
 return false;
}
else{	
  	std::allocator<int> allocat;
	nodes.push_back(std::allocate_shared<Vertex> (allocat,v));
	//std::cout<<"\nNODE ADDED";
	return true;
	}

}












































void RRT::connect(){

while(!(fromSource&&fromGoal)){}
std::cout<<" \n size of sources: "<<Source_nodes.size();
std::cout<<" \n size of goal: "<<Goal_nodes.size();
std::random_device devicex;
std::random_device devicey;
std::mt19937 engx(devicex());
std::mt19937 engy(devicey());
std::uniform_int_distribution<int> distX(-750,700);
std::uniform_int_distribution<int> distY(-700,700);
bool from_source=true;
bool connected=false;
bool free=false;
int connection=0;
double x_rand=0;
double y_rand=0;
double dx=0;
double dy=0;
double xstep=0;
double ystep=0;
int nearpos=0;
double dist=1000;
int count=0;
int connIndex;
Vertex qnew;
Vertex qnear;
Vertex qrand;
bool S_exp=false;
while(!connected){
	S_exp=!S_exp;;
	x_rand= distX(engx)/100;
	y_rand= distY(engy)/100;
	qrand.x=x_rand;
	qrand.y=y_rand;
	free=map.no_collision(qrand.x,qrand.y);
	if(free){
		
		if(S_exp){
			
			nearpos=CLOSEST(qrand,Source_nodes);
			qnear.x=Source_node[nearpos]->x;
			qnear.y=Source_node[nearpos]->y;
			}
		else{
			nearpos=CLOSEST(qrand,Goal_nodes);
			qnear.x=Goal_node[nearpos]->x;
			qnear.y=Goal_node[nearpos]->y;
			}
		
		qnew.x=qrand.x;
		qnew.y=qrand.y;
		
		dx=qnew.x-qnear.x;
		dy=qnew.y-qnear.y;
		if((dx==0)&&(dy==0)){free=false;}
		else
		{
		double angle=atan2(dy,dx);
		count=0:
		xstep=step*cos(angle);
		ystep=step*sin(angle);
		dist=qnew-qnear;
		Vertex qbuff(qnear.x,qnear.y);
		double distance=0;
		while(free&&(distance<dist)){
			count++;
			free=map.no_collision(qbuff.x+xstep,qbuff.y+ystep);
			distance+=step;
			if(free&&(distance<dist)){
				qbuff.x+=xstep;
				qbuff.y+=ystep;				
				}
			if(free&&(distance>=dist){
				qnew.x=qbuff.x+xstep;
				qnew.y=qbuff.y+ystep;
				}
			else(!free){
				qnew.x=qbuff.x;
				qnew.y=qbuff.y;
				}
			}
		if(S_exp){
			
			connIndex=FIND(qnew,Goal_nodes);
			if(connIndex>=0){
				connection++;
				Source_nodes[nearpos]->add_link(Goal_nodes[ConIndex]);
				Goal_nodes[ConIndex]->add_link(Source_nodes[nearpos]);
				}
			}
		else{
			nearpos=CLOSEST(qrand,Goal_nodes);
			qnear.x=Goal_node[nearpos]->x;
			qnear.y=Goal_node[nearpos]->y;
			}
		
		
		
		
		
		}
	
	



}





}
































