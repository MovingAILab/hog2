//
//  Map2DHeading.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 11/12/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#include "Map2DHeading.h"

//const double P = 1.0;
using std::cout;
using std::endl;

Map2DHeading::Map2DHeading(Map *m)
{
	map = m;
	BuildAngleTables();
	drawWeights = false;
	DIAGONAL_COST = 1.5;
}

Map2DHeading::~Map2DHeading()
{
	
}

void Map2DHeading::GetSuccessors(const xyhLoc &nodeID, std::vector<xyhLoc> &neighbors) const
{
	neighbors.resize(0);
	xyhLoc tmp;
	xyhAct act;
	act.oldHeading = nodeID.h;
	for (int x = 0; x < 8; x++)
	{
		tmp = nodeID;
		act.newHeading = x;
		ApplyAction(tmp, act);
		if (x != nodeID.h || map->CanStep(nodeID.x, nodeID.y, tmp.x, tmp.y))
			neighbors.push_back(tmp);
	}
}

void Map2DHeading::GetActions(const xyhLoc &nodeID, std::vector<xyhAct> &actions) const
{
	actions.resize(0);
	xyhAct act;
	act.oldHeading = nodeID.h;
	for (int x = 0; x < 8; x++)
	{
		act.newHeading = x;
		if (x != nodeID.h)
			actions.push_back(act);
		else {
			xyhLoc tmp;
			GetNextState(nodeID, act, tmp);
			if (map->CanStep(nodeID.x, nodeID.y, tmp.x, tmp.y))
				actions.push_back(act);
		}
	}
}

xyhAct Map2DHeading::GetAction(const xyhLoc &s1, const xyhLoc &s2) const
{
	xyhAct a;
	a.oldHeading = s1.h;
	a.newHeading = s2.h;
	return a;
}

void Map2DHeading::ApplyAction(xyhLoc &s, xyhAct dir) const
{
	if (dir.oldHeading != dir.newHeading)
	{
		s.h = dir.newHeading;
		return;
	}

	switch (dir.newHeading)
	{
		case 6: s.y -= 1; break;
		case 0: s.x += 1; break;
		case 2: s.y += 1; break;
		case 4: s.x -= 1; break;
		case 7: s.y -= 1; s.x += 1; break;
		case 1: s.y += 1; s.x += 1; break;
		case 3: s.y += 1; s.x -= 1; break;
		case 5: s.y -= 1; s.x -= 1; break;
	}
}

bool Map2DHeading::InvertAction(xyhAct &a) const
{
	return false;
}

double Map2DHeading::HCost(const xyhLoc &l1, const xyhLoc &l2) const
{
	double h1;
	double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
	double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
	//return sqrt(a*a+b*b);
	h1 = (a>b)?(b*DIAGONAL_COST+a-b):(a*DIAGONAL_COST+b-a);
	if (l1.h != l2.h)
		h1+=1;
	return h1;
}

double Map2DHeading::GetCost(const xyhLoc &a, const xyhLoc &b, double P, double D) const
{
	if (a.x == b.x && a.y == b.y)
	{
		//cout << "gcost of " << a << " relative to " << b << " is " << 100.0 << endl;
		return 100;
	}
	float heading = atan2(a.x-b.x, a.y-b.y);
	int conv = (360.0*heading/(2*3.1415)+180+(b.h+2)*45); // -90 for face left
	conv = conv%360;
	if (conv > 180)
		conv = 360-conv;
	//printf("Relative heading: %d\n", conv);
	
	
	//	for (float P = -1; P <= 1; P += 0.5)
	//	{
	//		float cost = (P<0)?((1.0-(float)conv/180.0)*(-P)):(((float)conv/180.0)*(P));
	//		printf("P : %1.2f   C : %1.2f\n", P, cost);
	//	}
	float cost1 = max(-P*cos(1.0*2*3.1415*conv/360.0), 0);
	//float cost = (P<0)?((1.0-(float)conv/90.0)*(-P)):(((float)conv/90.0-1.0)*(P));
	//float cost = (P<0)?((1.0-(float)conv/180.0)*(-P)):(((float)conv/180.0)*(P));
	float dist = sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
	// target distance is 2.0
	float cost2 = min(fabs(D-dist)/D, 1.0);
//	dist *= dist;
	//cout << "gcost of " << a << " relative to " << b << " is " << 10*cost1/dist+cost2 << endl;
	return 10*cost1/dist+cost2;//+((dist<16)?fabs(8.0-dist):0);
}

double Map2DHeading::GCost(const xyhLoc &node1, const xyhLoc &node2) const
{
	if (node1.h != node2.h) // turn
		return 1.0;
	
	double costModifier = 1.0;

	//for (CostTable::iterator it = costs.begin(); it != costs.end(); it++)
	for (const auto &it : costs)
	{
		xyhLoc tmp;
		//GetStateFromHash(it->first, tmp);
		GetStateFromHash(it.first, tmp);
		costModifier += GetCost(node1, tmp, it.second.seen, it.second.dist);
	}
//	CostTable::iterator iter = costs.find(GetStateHash(node1));
//	if (iter != costs.end())
//	{
//		std::cout << "Found cost " << iter->second << " at state " << node1 << std::endl;
//		costModifier = iter->second;
//	}

	if (0 == (node1.h%2))
		return costModifier;
	return DIAGONAL_COST*costModifier;
}

double Map2DHeading::GCost(const xyhLoc &node1, const xyhAct &act) const
{
	if (act.oldHeading != act.newHeading)
		return 1.0;

	double costModifier = 1.0;
	
	for (const auto &it : costs)
		//for (CostTable::iterator it = costs.begin(); it != costs.end(); it++)
	{
		xyhLoc tmp;
		GetStateFromHash(it.first, tmp);
		costModifier += GetCost(node1, tmp, it.second.seen, it.second.dist);
	}
//	CostTable::iterator iter = costs.find(GetStateHash(node1));
//	if (iter != costs.end())
//	{
//		std::cout << "Found cost* " << iter->second << " at state " << node1 << std::endl;
//		costModifier = iter->second;
//	}
	
	if (0 == act.newHeading%2)
		return costModifier;
	return DIAGONAL_COST*costModifier;
}

bool Map2DHeading::LegalState(const xyhLoc &s)
{
	if ((map->GetTerrainType(s.x, s.y)&kGround) != 0)
		return true;
	return false;
}

bool Map2DHeading::GoalTest(const xyhLoc &node, const xyhLoc &goal) const
{
	return (node == goal);
}

uint64_t Map2DHeading::GetStateHash(const xyhLoc &node) const
{
	uint64_t res = node.x;
	res = (res<<16)|node.y;
	res = (res<<16)|node.h;

	return res;
}

void Map2DHeading::GetStateFromHash(uint64_t hash, xyhLoc &node) const
{
	node.h = hash&0xFFFF;
	hash>>=16;
	node.y = hash&0xFFFF;
	hash>>=16;
	node.x = hash&0xFFFF;
}

uint64_t Map2DHeading::GetActionHash(xyhAct act) const
{
	return (act.newHeading<<8)+act.oldHeading;
}

//void Map2DHeading::OpenGLDraw() const
//{
//	map->OpenGLDraw();
//
//	SetColor(0.0, 1.0, 0.0);
//	if (!drawWeights)
//		return;
//	xyhLoc l;
//	for (CostTable::const_iterator it = costs.begin(); it != costs.end(); it++)
//	{
//		GetStateFromHash(it->first, l);
//		OpenGLDraw(l);
//	}
//
//}
//
//void Map2DHeading::OpenGLDraw(const xyhLoc &l) const
//{
//	GLdouble xx, yy, zz, rad;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	map->GetCoord(l.x, l.y, xx, yy, zz, rad);
//	
//	GLdouble yoffset = mySin(l.h)*rad;//sin(TWOPI*rot/16)*rad;
//	GLdouble xoffset = myCos(l.h)*rad;//cos(TWOPI*rot/16)*rad;
//		
//	glBegin(GL_TRIANGLES);
//	Graphics::point surfaceNormal;
//	surfaceNormal.x = (((-0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
//	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((0.5*yoffset) - (rad)));
//	surfaceNormal.z = (((0.5*yoffset) * (-2*yoffset)) - ((-0.5*xoffset) - (-2*xoffset)));
//	surfaceNormal.normalise();
//	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
//	
//	surfaceNormal.x = (((+0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
//	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((-0.5*yoffset) - (rad)));
//	surfaceNormal.z = (((-0.5*yoffset) * (-2*yoffset)) - ((+0.5*xoffset) - (-2*xoffset)));
//	surfaceNormal.normalise();
//	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
//	glEnd();
//
//}
//
//void Map2DHeading::OpenGLDraw(const xyhLoc &oldState, const xyhLoc &newState, float perc) const
//{
//	int DEG = 8;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	//printf("Drawing %f percent\n", perc);
//	//	std::cout << oldState << std::endl;
//	//	std::cout << newState << std::endl;
//	
//	GLdouble xx, yy, zz, rad;
//	
//	map->GetCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
//	
//	float rot = (1-perc)*oldState.h+perc*newState.h;
//
//	if ((oldState.h >= DEG-2) && (newState.h <= 2))
//	{
//		rot = (1-perc)*oldState.h+perc*(newState.h+DEG);
//		if (rot >= DEG)
//			rot -= DEG;
//	}
//	else if ((newState.h >= DEG-2) && (oldState.h <= 2))
//	{
//		rot = (1-perc)*(oldState.h+DEG)+perc*(newState.h);
//		if (rot >= DEG)
//			rot -= DEG;
//	}
//
//	GLdouble yoffset = sin(TWOPI*rot/8.0)*rad;
//	GLdouble xoffset = cos(TWOPI*rot/8.0)*rad;
//		
//	glBegin(GL_TRIANGLES);
//	Graphics::point surfaceNormal;
//	surfaceNormal.x = (((-0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
//	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((0.5*yoffset) - (rad)));
//	surfaceNormal.z = (((0.5*yoffset) * (-2*yoffset)) - ((-0.5*xoffset) - (-2*xoffset)));
//	surfaceNormal.normalise();
//	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
//	
//	surfaceNormal.x = (((+0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
//	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((-0.5*yoffset) - (rad)));
//	surfaceNormal.z = (((-0.5*yoffset) * (-2*yoffset)) - ((+0.5*xoffset) - (-2*xoffset)));
//	surfaceNormal.normalise();
//	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
//	glEnd();
//}
//
//void Map2DHeading::OpenGLDraw(const xyhLoc &, const xyhAct &) const
//{
//	
//}
//
//void Map2DHeading::GLLabelState(const xyhLoc &, const char *) const
//{
//	
//}
//
//void Map2DHeading::GLDrawLine(const xyhLoc &x, const xyhLoc &y) const
//{
//	
//}


void Map2DHeading::GetNextState(const xyhLoc &currents, xyhAct dir, xyhLoc &news) const
{
	news = currents;
	ApplyAction(news, dir);
}

void Map2DHeading::BuildAngleTables()
{
	float DEG = 8;
	for (float x = 0; x < DEG; x++)
	{
		sinTable.push_back(sin(TWOPI*(x)/DEG));
		//printf("sin(%d) = %f\n", x, sinTable.back());
		cosTable.push_back(cos(TWOPI*(x)/DEG));
		//printf("cos(%d) = %f\n", x, cosTable.back());
	}
}

float Map2DHeading::mySin(int dir) const
{
	return sinTable[dir];
}

float Map2DHeading::myCos(int dir) const
{
	return cosTable[dir];
}

void Map2DHeading::SetCost(const xyhLoc &l, double seen, double dist)
{
	hdData d = {seen, dist};
	costs[GetStateHash(l)] = d;
}

void Map2DHeading::ClearCost(const xyhLoc &l)
{
	costs.erase(GetStateHash(l));
}

void Map2DHeading::ClearAllCosts()
{
	costs.clear();
}
