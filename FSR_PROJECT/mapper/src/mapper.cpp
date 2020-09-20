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


//****************
int FIND(const Vertex v,std::vector<std::shared_ptr<Vertex>> &nodes){
bool found=false;
int f=0;
int index=-1000;
while((!found)&&(f<nodes.size())){
	if(*nodes[f]==v) {found=true;index=f;
	//std::cout<<"\n HO TROVATO [:"<<nodes[f]->x<<","<<nodes[f]->y<<"] uguale a: ["<<v.x<<","<<v.y<<"] \n ";
	
	}
	
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
  	std::allocator<int> allocat;
	nodes.push_back(std::allocate_shared<Vertex> (allocat,v));
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
/*
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
*/
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
//goal belongs to roadmap?
//se si tutt appo
//else aggiungi un link, trova il più vicino e collega, rifai l'algoritmo con qrand=goal;
std::vector<std::shared_ptr<Vertex>> open;
//tree[0].node=nodes_[0];
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
std::cout<<"list size "<<open_size<<std::endl;
std::cout<<"nodes size "<<nodes_.size()<<std::endl;

std::cout<<"links size "<<Qbest->links_.size()<<std::endl;
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
	while(*printer!=*nodes_[0]){
		std::cout<<"\n ["<<printer->x<<","<<printer->y<<"]";
		printer=printer->parent_;
	}
	std::cout<<"\n ["<<printer->x<<","<<printer->y<<"]";
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
	int count=0;
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
	while(s < MAX_ITER){
		
//		std::cout<<" \n from begin \n ";
//		std::cout<<"actual number of nodes : "<<nodes.size();
		x_rand= distX(engx)/100;
		y_rand= distY(engy)/100;
		qrand.x=x_rand;
		qrand.y=y_rand;
		if(FIND(qrand,nodes)>=0){ //std::cout<<"already in memory";
		}
		else{
			nearpos=CLOSEST(qrand,nodes);
			qnear.x=nodes[nearpos]->x;
			qnear.y=nodes[nearpos]->y;
			dist=qrand-qnear;
			double cat_x=qrand.x-qnear.x;
			double cat_y=qrand.y-qnear.y;
			double ang = atan2(cat_y, cat_x);
			// std::cout<<"Found a node near the new one, is the path free?"<<std::endl;
			  // The direction cosines
			double cosTeta = cos(ang);
			double sinTeta = sin(ang);
			
//			std::cout<<"\n check qrand"<<qrand.x<<"  "<<qrand.y;
			if(dist>0.4){
				qnew.x=qnear.x+cosTeta*0.4;
				qnew.y=qnear.y+sinTeta*0.4;
				dist=qnew-qnear;
				}
			else{   qnew.x=qrand.x;
				qnew.y=qrand.y;
				}
			
//			std::cout<<"\n check dist"<<dist;
//			std::cout<<"\n check qnew"<<qnew.x<<"  "<<qnew.y;
			free=map.no_collision(qnew.x,qnew.y);
			
			if(free){
//				std::cout<<" \n calcolo step";

				
				xstep=step*cosTeta;
				ystep=step*sinTeta;
				double buffx=xstep;
				double buffy=ystep;
				accomplished=false;
				while(free&&(!accomplished)){
					
					accomplished=((step*count)>=dist);
					free=(map.no_collision(qnear.x + xstep,qnear.y+ystep));
					if(free&&(!accomplished)){
						count++; 
						xstep=buffx*count;
						ystep=buffy*count;
//						std::cout<<"\n free"<<count;
						}
					else if(free&&accomplished){
						//aggiungi il nodo
//						std::cout<<" \n 1qnew coordinate: "<<qnew.x<<"  "<<qnew.y;
						qnew.x=qnear.x+(buffx*count);
						qnew.y=qnear.y+(buffy*count);
						if(ADD_NODE(qnew,nodes)){ 
						//std::cout<<" \n  accomplished number of nodes "<<get_size();
						
					nodes[nodes.size()-1]->add_link(nodes[nearpos]);
					nodes[nearpos]->add_link( nodes[nodes.size()-1]);
						 }
						count=0;
						}
					else if(!free){
					if(count!=0){count--;}
						qnew.x=qnear.x+xstep-buffx;
						qnew.y=qnear.y+ystep-buffy;
						//std::cout<<" \n qnew coordinate:"<<qnew.x<<"  "<<qnew.y;
						if(ADD_NODE(qnew,nodes)){
					nodes[nodes.size()-1]->add_link(nodes[nearpos]);
					nodes[nearpos]->add_link( nodes[nodes.size()-1]);
						} 
						count=0;
					}
					
				}
			}
		}
	
	s=nodes.size();
	}
	if (SorG==1){
		Source_nodes.resize(0);
		for(int o=0;o<nodes.size();o++){
			Source_nodes.push_back(nodes[o]);
		}
		
	fromSource=true;
	}
	else{
		Goal_nodes.resize(0);
		for(int o=0;o<nodes.size();o++){
			Goal_nodes.push_back(nodes[o]);
		}
		
	fromGoal=true;
	}
	/*if(s>(MAX_ITER-10)){
		std::cout<<" \n number of s "<<s;
		
		
		for(int i=0;i<20;i++){
		
			std::cout<<"\n SIZE OF LINKS OF i-esimo "<<nodes_[i]->links_.size();}
		}*/
	//A_star_routine(*this,*nodes_[MAX_ITER-1]);

}


void RRT::run(Vertex s,Vertex g) {
	fromSource=false;
	fromGoal=false;
	source=s;
	goal=g;
	boost::thread get_fromSource_t( &RRT::iter, this,1);
	boost::thread get_fromGoal_t( &RRT::iter, this,2);
	boost::thread connect_t( &RRT::connect, this);
	ros::spin();
		

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
	std::uniform_int_distribution<int> distX(-900,900);
	std::uniform_int_distribution<int> distY(-900,900);
	bool from_source=true;
	bool connected=false;
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
	int ConIndex;
while(!connected){
	x_rand= distX(engx)/100;
	y_rand= distY(engy)/100;
	qrand.x=x_rand;
	qrand.y=y_rand;
	if(from_source){	
		nearpos=CLOSEST(qrand,Source_nodes);
		qnear.x=Source_nodes[nearpos]->x;
		qnear.y=Source_nodes[nearpos]->y;
		std::cout<<"\n da source il qnear è"<<qnear.x<<" "<<qnear.y;
	}
	else {	
		nearpos=CLOSEST(qrand,Goal_nodes);
		qnear.x=Goal_nodes[nearpos]->x;
		qnear.y=Goal_nodes[nearpos]->y;
		
		std::cout<<"\n da goal il qnear è"<<qnear.x<<" "<<qnear.y;
	}
	dist=qrand-qnear;
	double cat_x=qrand.x-qnear.x;
	double cat_y=qrand.y-qnear.y;
	double ang = atan2(cat_y, cat_x);
	double cosTeta = cos(ang);
	double sinTeta = sin(ang);
	qnew.x=qrand.x;
	qnew.y=qrand.y;
	bool free=map.no_collision(qnew.x,qnew.y);
	//				std::cout<<" \n calcolo step";

	
	xstep=step*cosTeta;
	ystep=step*sinTeta;
	double buffx=xstep;
	double buffy=ystep;
	bool accomplished=false;
	while(free&&(!accomplished)){
		
		accomplished=((step*count)>=dist);
		free=(map.no_collision(qnear.x + xstep,qnear.y+ystep));
		if(free&&(!accomplished)){
			count++; 
			xstep=buffx*count;
			ystep=buffy*count;
//			std::cout<<"\n free"<<count;
			}
		else if(free&&accomplished){
			//aggiungi il nodo
			qnew.x=qnear.x+(buffx*count);
			qnew.y=qnear.y+(buffy*count);
			
		 }
				
		else if(!free){
			if(count!=0){count--;}
			qnew.x=qnear.x+xstep-buffx;
			qnew.y=qnear.y+ystep-buffy;
			
			}
		}
		
		std::cout<<"\n il nuovo qnew è"<<qnew.x<<" "<<qnew.y;
		//std::cout<<" \n qnew coordinate:"<<qnew.x<<"  "<<qnew.y;
		if(from_source){
		
			ConIndex=FIND(qnew,Goal_nodes);
			if(ConIndex>=0){
			connected=true;
				//Source_nodes.push_back(Goal_nodes[ConIndex]);
				Source_nodes[Source_nodes.size()-1]->add_link(Goal_nodes[ConIndex]);
				Goal_nodes[ConIndex]->add_link(Source_nodes[Source_nodes.size()-1]);
			}
			else{
				if(ADD_NODE(qnew,Source_nodes)){
				Source_nodes[Source_nodes.size()-1]->add_link(Source_nodes[nearpos]);
				Source_nodes[nearpos]->add_link( Source_nodes[Source_nodes.size()-1]);
					} 
			
			count=0;
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
			buffx=xstep;
			buffy=ystep;
			accomplished=false;
			while(free&&(!accomplished)){
				
				accomplished=((step*count)>=dist);
				free=(map.no_collision(qnear.x + xstep,qnear.y+ystep));
				if(free&&(!accomplished)){
				count++; 
				xstep=buffx*count;
				ystep=buffy*count;
	//			std::cout<<"\n free"<<count;
				}
				else if(free&&accomplished){
					qnew.x=qnear.x+(buffx*count);
					qnew.y=qnear.y+(buffy*count);
			
				 }
					
				else if(!free){
					if(count!=0){count--;}
					qnew.x=qnear.x+xstep-buffx;
					qnew.y=qnear.y+ystep-buffy;
					}
				
				}
			ConIndex=FIND(qnew,Source_nodes);
			if(ConIndex>=0){
				//Goal_nodes.push_back(Source_nodes[ConIndex]);
				connected=true;
				Goal_nodes[Goal_nodes.size()-1]->add_link(Source_nodes[ConIndex]);
				Source_nodes[ConIndex]->add_link(Goal_nodes[Goal_nodes.size()-1]);
			}
			else{
			if(ADD_NODE(qnew,Goal_nodes)){
			Goal_nodes[Goal_nodes.size()-1]->add_link(Goal_nodes[nearpos]);
			Goal_nodes[nearpos]->add_link( Goal_nodes[Goal_nodes.size()-1]);
				}
			}
			}
			from_source=false;
				}
		else{ //viceversa
			ConIndex=FIND(qnew,Source_nodes);
			if(ConIndex>=0){
				connected=true;
				//Goal_nodes.push_back(Source_nodes[ConIndex]);
				Goal_nodes[Goal_nodes.size()-1]->add_link(Source_nodes[ConIndex]);
				Source_nodes[ConIndex]->add_link(Goal_nodes[Goal_nodes.size()-1]);
				
			}
			else{
				if(ADD_NODE(qnew,Goal_nodes)){
				Goal_nodes[Goal_nodes.size()-1]->add_link(Goal_nodes[nearpos]);
				Goal_nodes[nearpos]->add_link( Goal_nodes[Goal_nodes.size()-1]);
					}
			
			count=0;
			
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
			buffx=xstep;
			buffy=ystep;
			accomplished=false;
			while(free&&(!accomplished)){
				
				accomplished=((step*count)>=dist);
				free=(map.no_collision(qnear.x + xstep,qnear.y+ystep));
				if(free&&(!accomplished)){
				count++; 
				xstep=buffx*count;
				ystep=buffy*count;
	//			std::cout<<"\n free"<<count;
				}
				else if(free&&accomplished){
					qnew.x=qnear.x+(buffx*count);
					qnew.y=qnear.y+(buffy*count);
				 }
					
				else if(!free){
					if(count!=0){count--;}
					qnew.x=qnear.x+xstep-buffx;
					qnew.y=qnear.y+ystep-buffy;
					}
			}
			ConIndex=FIND(qnew,Goal_nodes);
			if(ConIndex>=0){
				connected=true;
				//Source_nodes.push_back(Goal_nodes[ConIndex]);
				Source_nodes[Source_nodes.size()-1]->add_link(Goal_nodes[ConIndex]);
				Goal_nodes[ConIndex]->add_link(Source_nodes[Source_nodes.size()-1]);
			}
			else{
			if(ADD_NODE(qnew,Source_nodes)){
			Source_nodes[Source_nodes.size()-1]->add_link(Source_nodes[nearpos]);
			Source_nodes[nearpos]->add_link(Source_nodes[Source_nodes.size()-1]);
				}
			}
			}
	from_source=true;
	}	


}
std::cout<<" \n size of sources: "<<Source_nodes.size();
std::cout<<" \n size of goal: "<<Goal_nodes.size();
for(int o=0;o<Source_nodes.size();o++){
nodes_.push_back(Source_nodes[o]);
}
for(int o=0;o<Goal_nodes.size();o++){
nodes_.push_back(Goal_nodes[o]);
if(o==0){ std::cout<<"\n il goal è:"<<nodes_[nodes_.size()-1]->x<<" "<<nodes_[nodes_.size()-1]->y<<"\n";
std::cout<<"\nil numero dei links è:"<<nodes_[nodes_.size()-1]->links_.size();}
}

for(int o=0;o<nodes_.size();o++){
if(nodes_[o]->links_.size()==0) ROS_INFO("c'è un punto isolato");
}
size_=nodes_.size();

std::cout<<" \n size of nodes: "<<nodes_.size();
if(find(goal)){
std::cout<<"\n goal belongs to nodes_";}
std::cout<<"\n mission accomplished";
A_star_routine();
}



int main (int argc,char** argv) {
ros::init(argc,argv,"mappin" ) ;
Vertex qinit(0,0);
Vertex qdest(-5.25,4.21);
RRT rrt;

rrt.run(qinit,qdest);

return 0;
}

/*if(ConIndex>=0){
		Source_nodes.push_back(Goal_nodes[ConIndex]);
		}
		else{*/

