
Vertex RRT::pathfree(Vertex qnear,Vertex qnew, double MAX_LENGTH){
Vertex qbuff;
qbuff=qnew;
double dist=qnew-qnear;
double cat_x=qnew.x-qnear.x;
double cat_y=qnew.y-qnear.y;
double ang = atan2(cat_y, cat_x);
double cosTeta = cos(ang);
double sinTeta = sin(ang);
bool free=false;
if(dist>MAX_LENGTH){
	qbuff.x=qnear.x + cosTeta*MAX_LENGTH;
	qbuff.y=qnear.y + sinTeta*MAX_LENGTH;
	dist=qbuff-qnear;
	}
else{   qbuff.x=qrand.x;
	qbuff.y=qrand.y;
	}
free=map.no_collision(qbuff.x,qbuff.y);

if(free){
	double distance=0;
	count=0;
	xstep=step*cosTeta;
	ystep=step*sinTeta;
	while(free&&(distance<dist)){
		distance=step*count;
		qbuff.x=qnear.x+xstep*count;
		qbuff.y=qnear.y+ystep*count;
		free=map.no_collision(qbuff.x,qbuff.y);
		if(free&&(distance<dist)){
			count++;
			}
		else if(free&&((step*count)>=dist)){
			 }
			count=0;
			}
		else if(!free){
			qbuff.x-=xstep;
			qbuff.y-=ystep;
			count=0;
			
		}
		
	}
}


}




















































