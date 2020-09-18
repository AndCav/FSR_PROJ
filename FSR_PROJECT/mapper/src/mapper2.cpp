#include "mapper.h"
#define NOT_FOUND

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
	else return false;}


RRT::RRT(){


	nodes_.resize(0);
	size_=0;

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

bool RRT::freepath(Grid map, Vertex v1, Vertex v2){
  // The angle of the line
  double dx=v1.x-v2.x;
  double dy=v1.x-v2.y;
  double ang = atan2(dy, dx);
  double len=v1-v2;
 // cout<<"Found a node near the new one, is the path free?"<<endl;
  // The direction cosines
  double kx = cos(ang);
  double ky = sin(ang);
  bool isfree=false;
  bool done = false;
  double x=0;
  double y=0;
  double pos = step; 
  while (!done) {
          
     x = v2.x + pos * kx;
     y = v2.y + pos * ky;
    // cout<<"Point on a path (is free?): "<<x <<" "<<y<<endl;
     pos += step;
    if(!map.no_collision(x,y)){
      isfree=false;
      done=true;
    }
    else if (pos > len) {
         
         isfree=true;
         done = true; 
    }
  }

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


void A_star_routine(RRT roadmap, const Vertex goal){
//goal belongs to roadmap?
//se si tutt appo
//else aggiungi un link, trova il più vicino e collega, rifai l'algoritmo con qrand=goal;
std::vector<std::shared_ptr<Vertex>> open;
//tree[0].node=roadmap.nodes_[0];
int sizeoflinks=roadmap.nodes_[0]->links_.size();


std::shared_ptr<Vertex> Qbest;
Qbest=roadmap.nodes_[0];
Qbest->set_visited(true);
Qbest->total_cost=0;
Qbest->calc_heur(goal);
double cost=roadmap.nodes_[0]->calc_heur(goal)*10;
double buff_cost;
bool open_empty=false;
open.push_back(Qbest);
int open_size=open.size();
std::cout<<"list size "<<open_size<<std::endl;
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
	
	
	//std::cout<<"ciao2"<<std::endl;
	Qbest=open[newbestpos];
	open.erase(open.begin() + newbestpos);
	
	
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
	std::shared_ptr<Vertex> printer;
	//ROS_INFO("ci sono");
	printer=Qbest;
	while(*printer!=*roadmap.nodes_[0]){
		std::cout<<"\n ["<<printer->x<<","<<printer->y<<"]";
		printer=printer->parent_;
	}
	std::cout<<"\n ["<<printer->x<<","<<printer->y<<"]";
	}
	
}

void RRT::iter(){
	std::cout<<"start";
	Grid map;
	bool free=false;
	double x_rand=0;
	double y_rand=0;
	double dx=0;
	double dy=0;
	double xstep=0;
	double ystep=0;
	int nearpos=0;
	double dist=1000;
	int count=0;
	Vertex qnew;
	Vertex qnear;
	Vertex qrand;
	bool accomplished=false;
	std::random_device devicex;
	std::random_device devicey;
	std::mt19937 engx(devicex());
	std::mt19937 engy(devicey());
	std::uniform_int_distribution<int> distX(-900,900);
	std::uniform_int_distribution<int> distY(-900,900);

	while(!map.its_on()){
	//do nothing
	};
	int s=0;
	while(s < MAX_ITER){
		//std::cout<<"actual number of nodes : %d", get_size());
		x_rand= distX(engx)/100;
		y_rand= distY(engy)/100;
		qrand.x=x_rand;
		qrand.y=y_rand;
		if(find(qrand)){ //std::cout<<"already in memory";
		}
		else{
			nearpos=closest(qrand);
			qnear.x=nodes_[nearpos]->x;
			qnear.y=nodes_[nearpos]->y;
			dist=qrand-qnear;
			double cat_x=qrand.x-qnear.x;
			double cat_y=qrand.y-qnear.y;
			double ang = atan2(cat_y, cat_x);
			 std::cout<<"Found a node near the new one, is the path free?"<<std::endl;
			  // The direction cosines
			double cosTeta = cos(ang);
			double sinTeta = sin(ang);
			
			std::cout<<"\n check qrand"<<qrand.x<<"  "<<qrand.y;
			if(dist>0.2){
				qnew.x=qnear.x+cosTeta*0.2;
				qnew.y=qnear.y+sinTeta*0.2;
				dist=qnew-qnear;
				}
			else{   qnew.x=qrand.x;
				qnew.y=qrand.y;
				}
			
			std::cout<<"\n check dist"<<dist;
			std::cout<<"\n check qnew"<<qnew.x<<"  "<<qnew.y;
			free=map.no_collision(qnew.x,qnew.y);
			
			if(free){
				std::cout<<" \n calcolo step";

				
				xstep=step*cosTeta;
				ystep=step*sinTeta;
				double buffx=xstep;
				double buffy=ystep;
				accomplished=false;
				while(free&&(!accomplished)){
					
					accomplished=((step*count)>=dist);
					free=(map.no_collision(qnew.x + xstep,qnew.y+ystep));
					if(free&&(!accomplished)){
						count++; 
						xstep=buffx*count;
						ystep=buffy*count;
						std::cout<<"\n free"<<count;
						}
					else if(free&&accomplished){
						//aggiungi il nodo
						std::cout<<" \n 1qnew coordinate: "<<qnew.x<<"  "<<qnew.y;
						qnew.x+=(buffx*count);
						qnew.y+=(buffy*count);
						if(add_node(qnew)){ 
						//std::cout<<" \n  accomplished number of nodes "<<get_size();
						
					nodes_[nodes_.size()-1]->add_link(nodes_[nearpos]);
					nodes_[nearpos]->add_link( nodes_[nodes_.size()-1]);
						 }
						count=0;
						}
					else if(!free){
					if(count!=0){count--;}
						qnew.x+=xstep-buffx;
						qnew.y+=ystep-buffy;
						//std::cout<<" \n qnew coordinate:"<<qnew.x<<"  "<<qnew.y;
						if(add_node(qnew)){
					nodes_[nodes_.size()-1]->add_link(nodes_[nearpos]);
					nodes_[nearpos]->add_link( nodes_[nodes_.size()-1]);
						} 
						count=0;
					}
					
				}
			}
		}
	
	s=get_size(); 
	std::cout<<" \n number of s "<<s;
	}
	/*if(s>(MAX_ITER-10)){
		std::cout<<" \n number of s "<<s;
		
		
		for(int i=0;i<20;i++){
		
			std::cout<<"\n SIZE OF LINKS OF i-esimo "<<nodes_[i]->links_.size();}
		}*/
	A_star_routine(*this,*nodes_[MAX_ITER-1]);
	ROS_INFO("HO FINITO");
}


void RRT::run() {
	boost::thread get_dirkin_t( &RRT::iter, this);
	ros::spin();	

}


int main (int argc,char** argv) {
ros::init(argc,argv,"mappin" ) ;
Grid rs;
Vertex qinit(0,0);
RRT rrt(qinit);
rrt.run();

return 0;
}


