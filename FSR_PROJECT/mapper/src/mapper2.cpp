#include "mapper.h"


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
	double step=0.05;
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
		if(true){
		//std::cout<<" \n qrand  "<<qrand.x<<" "<<qrand.y;
		//std::cout<<"1";
			nearpos=closest(qrand);
		//std::cout<<"\n nearpos "<<nearpos;
			qnear.x=nodes_[nearpos]->x;
			qnear.y=nodes_[nearpos]->y;
		//std::cout<<" \nqnear coordinates "<<qnear.x<<"  "<<qnear.y;
			dist=qrand-qnear;
			
			if(dist>0.2){
				//std::cout<<" \nFAR";
		//std::cout<<"got some point";
				dx=((qrand.x - qnear.x)*0.2)/dist;
				dy=((qrand.y - qnear.y)*0.2)/dist;
				}
			else{   //std::cout<<"\n near ";
				dx=0;
				dy=0;
				}
			qnew.x=qnear.x+dx;
			qnew.y=qnear.y+dy;
			//std::std::cout<<"\n check qnew collision";
			free=map.no_collision(qnew.x,qnew.y);
			
			if(free){
			//	std::std::cout<<" \n calcolo step";

				
				xstep=((qnew.x - qnear.x)*step)/(dist);
				ystep=((qnew.y - qnear.y)*step)/(dist);
				double buffx=xstep;
				double buffy=ystep;
				accomplished=false;
				while(free&&(!accomplished)){
					
					accomplished=((step*count)<=dist);
					free=(map.no_collision(qnew.x + xstep,qnew.y+ystep));
					if(free&&(!accomplished)){
						count++; 
						xstep=buffx*count;
						ystep=buffy*count;
						////std::cout<<"\n free"<<count;
						}
					else if(free&&accomplished){
						//aggiungi il nodo
						//std::cout<<" \n 1qnew coordinate: "<<qnew.x<<"  "<<qnew.y;
						qnew.x+=(buffx*count);
						qnew.y+=(buffy*count);
						if(add_node(qnew)){ 
						//std::cout<<" \n  accomplished number of nodes "<<get_size();
						//nodes_[nodes_.size()-1].add_link(nearpos,nodes_[nearpos]);
						//nodes_[nearpos].add_link(nodes_.size()-1,nodes_[nodes_.size()-1]);
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
		else {//std::cout<<"not a valid point";
			}
	
	}
	s=get_size(); 
	if(s>(MAX_ITER-10)) std::cout<<" \n number of s "<<s;
	}
}


void RRT::run() {
	boost::thread get_dirkin_t( &RRT::iter, this);
	ros::spin();	

}
void A_star_routine(RRT roadmap, const Vertex goal){
//goal belongs to roadmap?
//se si tutt appo
//else aggiungi un link, trova il più vicino e collega, rifai l'algoritmo con qrand=goal;
std::vector<std::share_ptr<Vertex>> open;
int sizeoflinks=roadmap.nodes_[0]->links_.size();
for(int i=0;i<sizeoflinks;i++){
open.push_back(Qbest->links_[i]);
}

std::shared_ptr<Vertex> Qbest;
Qbest=roadmap.nodes[0];
Qbest->setVisited(true);
double cost=roadmap.nodes[0]->calc_heur(goal)*10;
double buff_cost;
while((*Qbest!=goal)&&(!open.empty)){

	for(int i=0;i<Qbest->links_.size();i++){
	//cerca il costo migliore
	}
	

}





}

int main (int argc,char** argv) {
ros::init(argc,argv,"mappin" ) ;
Grid rs;
Vertex qinit(0,0);
RRT rrt(qinit);
rrt.run();

return 0;
}


