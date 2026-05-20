//
//  RoadMap.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/25/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "RoadMap.h"

#include <algorithm>


RoadMap::RoadMap(const char *graph, const char *coordinates, bool timeGraph)
{
	double xMin = DBL_MAX;
	double xMax = -DBL_MAX;
	double yMin = DBL_MAX;
	double yMax = -DBL_MAX;
	
//	double scale = 0;
	
	std::vector<double> speed;
	g = new Graph();
	node *n = new node("");
	g->AddNode(n);
	FILE *f = fopen(coordinates, "r");
	//FILE *f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.co", "r");
	std::vector<double> xloc, yloc;
	double minx = DBL_MAX, maxx=-DBL_MAX, miny=DBL_MAX, maxy=-DBL_MAX;
	int count=0;
	while (!feof(f))
	{
		char line[255];
		fgets(line, 255, f);
		if (line[0] == 'v')
		{
			double x1, y1;
			int id;
			if (3 != sscanf(line, "v %d %lf %lf", &id, &x1, &y1))
				continue;
			if(count==0)
			{
//				std::cout<<"x1 and y1 after scanning\n"<<x1<<" "<<y1;
				count++;
			}
			//assert(id == xloc.size()+1);
			xloc.push_back(x1);
			yloc.push_back(y1);
			//printf("%d: (%f, %f) [%f, %f]\n", xloc.size(), x1, y1, minx, maxx);
			if (x1 > maxx) maxx = x1;
			if (x1 < minx) minx = x1;
			if (y1 > maxy) maxy = y1;
			if (y1 < miny) miny = y1;
			//if (maxx > -1)
		}
	}
	std::cout<<"Xloc and yloc before scaling\n"<<xloc[1]<<" "<<yloc[1];
	fclose(f);
	printf("x between (%f, %f), y between (%f, %f)\n",
		   minx, maxx, miny, maxy);
	scale = std::max(maxx-minx,maxy-miny);
	printf("Scale is %f\n", scale);
	double xoff = (maxx-minx)-scale;
	double yoff = (maxy-miny)-scale;
	
	for (unsigned int x = 0; x < xloc.size(); x++)
	{
		//printf("(%f, %f) -> ", xloc[x], yloc[x]);
		xloc[x] -= (minx);
		xloc[x] /= scale;
		xloc[x] = xloc[x]*2-1+xoff/scale;
		
		yloc[x] -= (miny);
		yloc[x] /= scale;
		yloc[x] = yloc[x]*2-1;
		yloc[x] = -yloc[x]+yoff/scale;
		
		if(xloc[x] < xMin)
			xMin = xloc[x];
		else if(xloc[x] > xMax)
			xMax = xloc[x];
		
		if(yloc[x] < yMin)
			yMin = yloc[x];
		else if(yloc[x] > yMax)
			yMax = yloc[x];
		
		
		node *n = new node("");
		int nodeNum = g->AddNode(n);
		n->SetLabelF(GraphSearchConstants::kXCoordinate, xloc[x]);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, yloc[x]);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		//std::cout<<" grid y size is "<<grid.size();
		//std::cout<<" grid x size is "<<grid[0].size();
		
	}
	//a 1 2 1988
	std::cout<<"xMin is "<<xMin<<" yMin is "<<yMin<< " xMax is "<<xMax<<" yMax is "<<yMax<<std::endl;
	std::cout<<" min x is "<<minx/scale + xoff;
	std::cout<<" max x is "<<maxx/scale + xoff;
	std::cout<<" min y is "<<miny/scale + yoff;
	std::cout<<" max y is "<<maxy/scale + yoff;
	printf("\n");
	
	
	f = fopen(graph, "r");
	//f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.gr", "r");
	int dups = 0;
	long double e=0;
	double minE=DBL_MAX;
	while (!feof(f))
	{
		//		std::cout<<"reading file\n";
		char line[255];
		fgets(line, 255, f);
		if (line[0] == 'a')
		{
			int x1, y1;
			sscanf(line, "a %d %d %Lf", &x1, &y1,&e);
			if (g->findDirectedEdge(x1, y1) == 0)
			{
				node* n1=g->GetNode(x1);
				node* n2=g->GetNode(y1);
				long double xCoord1 = n1->GetLabelF(GraphSearchConstants::kXCoordinate);
				long double yCoord1 =n1->GetLabelF(GraphSearchConstants::kYCoordinate);
				
				long double xCoord2 = n2->GetLabelF(GraphSearchConstants::kXCoordinate);
				long double yCoord2 =n2->GetLabelF(GraphSearchConstants::kYCoordinate);
				
				long double dist = scale*sqrtf((xCoord1-xCoord2)*(xCoord1-xCoord2) + (yCoord1-yCoord2)*(yCoord1-yCoord2))/2;

				if(e<minE)
					minE=e;
				if (fequal(e, 0))
					printf("**FOUND ZERO-COST EDGE\n");
				if (x1 == y1)
					printf("**FOUND SELF EDGE\n");
				
				
				g->AddEdge(new edge(x1, y1, e));
				
				speed.push_back(dist/e);
			}
			
			else if(g->findDirectedEdge(x1,y1)->GetWeight() < e)
				g->findDirectedEdge(x1,y1)->setWeight(e);
			else{
				dups++;
				//printf("Not adding duplicate directed edge between %d and %d\n", x1, y1);
			}
		}
	}
	
	std::cout<<"minE "<<minE << "\n";
	//printf("%d dups ignored\n", dups);
	fclose(f);
	std::cout<<"closed the file\n";
	ge = new GraphEnvironment(g);
	ge->SetDirected(true);
	std::cout<<"created graph environment\n";
	
//	double maxSpeed = 0;
	
	if(!timeGraph)
		maxSpeed = *(std::max_element(speed.begin(),speed.end()));
	else if(timeGraph)
		maxSpeed = *(std::max_element(speed.begin(),speed.end()));
	else
	{
		std::cout<<"\n Invalid graph type";
		exit(1);
	}
	
	printf(" max element %lf \n",maxSpeed);

	// create heuristic & pass max speed
//	
//	h1 = new myHeuristic(g,maxSpeed, scale);
//	astar.SetHeuristic(h1);
//	
//	std::cout<<" Set heuristic for A* \n";
//
//	
	
}


RoadMap::~RoadMap()
{
	delete ge;
	ge = 0;
}

void RoadMap::GetSuccessors(const intersection &nodeID, std::vector<intersection> &neighbors) const
{
	ge->GetSuccessors(nodeID, neighbors);
}

void RoadMap::GetActions(const intersection &nodeID, std::vector<neighbor> &actions) const
{
	ge->GetActions(nodeID, actions);
}


neighbor RoadMap::GetAction(const intersection &s1, const intersection &s2) const
{
	return ge->GetAction(s1, s2);
}

void RoadMap::ApplyAction(intersection &s, neighbor a) const
{
	ge->ApplyAction(s, a);
}


bool RoadMap::InvertAction(neighbor &a) const
{
	return ge->InvertAction(a);
}


/** Heuristic value between two arbitrary nodes. **/
double RoadMap::HCost(const intersection &from, const intersection &to) const
{
	node* a = g->GetNode(from);
	node* b = g->GetNode(to);
	
	double aX=a->GetLabelF(GraphSearchConstants::kXCoordinate);
	double aY=a->GetLabelF(GraphSearchConstants::kYCoordinate);
	double bX=b->GetLabelF(GraphSearchConstants::kXCoordinate);
	double bY=b->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	return scale*sqrt((aX-bX)*(aX-bX)+(aY-bY)*(aY-bY))/(2*maxSpeed);
}

double RoadMap::GCost(const intersection &node1, const intersection &node2) const
{
	return ge->GCost(node1, node2);
}

double RoadMap::GCost(const intersection &node, const neighbor &act) const
{
	return ge->GCost(node, act);
}

bool RoadMap::GoalTest(const intersection &node, const intersection &goal) const
{
	return node == goal;
}


uint64_t RoadMap::GetMaxHash() const
{
	return g->GetNumNodes();
}

uint64_t RoadMap::GetStateHash(const intersection &node) const
{
	return node;
}

void RoadMap::GetStateFromHash(uint64_t parent, intersection &s) const
{
	s = parent;
}

uint64_t RoadMap::GetActionHash(neighbor act) const
{
	return ge->GetActionHash(act);
}

//
//void RoadMap::OpenGLDraw() const
//{
//	ge->OpenGLDraw();
//}
//
//void RoadMap::OpenGLDraw(const intersection&i) const
//{
//	ge->OpenGLDraw(i);
//}
//
//void RoadMap::OpenGLDraw(const intersection&i, const neighbor&n) const
//{
//	ge->OpenGLDraw(i, n);
//}
//
//void RoadMap::GLDrawPath(const std::vector<intersection> &x) const
//{
//	ge->GLDrawPath(x);
//}

void RoadMap::SetColor(float rr, float g, float b, float t) const
{
	ge->SetColor(rr, g, b, t);
}

void RoadMap::GetColor(float& rr, float& g, float& b, float &t) const
{
	ge->GetColor(rr, g, b, t);
}


//}
//
//double HCost(const unsigned long &from, const unsigned long &to) const{
//	
//	node* a = g->GetNode(from);
//	node* b = g->GetNode(to);
//	
//	double aX=a->GetLabelF(GraphSearchConstants::kXCoordinate);
//	double aY=a->GetLabelF(GraphSearchConstants::kYCoordinate);
//	double bX=b->GetLabelF(GraphSearchConstants::kXCoordinate);
//	double bY=b->GetLabelF(GraphSearchConstants::kYCoordinate);
//	
//	return scale*sqrt((aX-bX)*(aX-bX)+(aY-bY)*(aY-bY))/(2*maxSpeed);
