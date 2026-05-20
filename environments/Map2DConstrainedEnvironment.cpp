//
//  Map2DConstrainedEnvironment.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/3/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#include "Map2DConstrainedEnvironment.h"

//std::vector<constraint> constraints;
//MapEnvironment *mapEnv;

bool operator==(const xytLoc &l1, const xytLoc &l2)
{
	return (l1.t == l2.t) && (l1.l == l2.l);
}

Map2DConstrainedEnvironment::Map2DConstrainedEnvironment(Map *m)
{
	mapEnv = new MapEnvironment(m);
	mapEnv->SetFourConnected();
}

void Map2DConstrainedEnvironment::AddConstraint(constraint c)
{
	constraints.push_back(c);
}

void Map2DConstrainedEnvironment::AddConstraint(xytLoc loc)
{
	AddConstraint(loc, kTeleport);
}

void Map2DConstrainedEnvironment::AddConstraint(xytLoc loc, tDirection dir)
{
	constraint c;
	c.loc = loc;
	c.dir = dir;
	constraints.push_back(c);
}

void Map2DConstrainedEnvironment::ClearConstraints()
{
	constraints.resize(0);
}

void Map2DConstrainedEnvironment::GetSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const
{
	std::vector<xyLoc> n;
	mapEnv->GetSuccessors(nodeID.l, n);

	// TODO: remove illegal successors
	for (unsigned int x = 0; x < n.size(); x++)
	{
		if (!ViolatesConstraint(nodeID.l, n[x], nodeID.t))
		{
			xytLoc newLoc;
			newLoc.l = n[x];
			newLoc.t = nodeID.t+1;
			neighbors.push_back(newLoc);
		}
	}
	// TODO: Add kStay
	if (!ViolatesConstraint(nodeID.l, nodeID.l, nodeID.t))
	{
		xytLoc newLoc;
		newLoc.l = nodeID.l;
		newLoc.t = nodeID.t+1;
		neighbors.push_back(newLoc);
	}
//	std::cout << "Successors of " << nodeID << " are:" << std::endl;
//	for (unsigned int x = 0; x < neighbors.size(); x++)
//		std::cout << neighbors[x] << " ";
//	std::cout << std::endl;
	
}

bool Map2DConstrainedEnvironment::ViolatesConstraint(const xyLoc &from, const xyLoc &to, int time) const
{
	xyLoc tmp;
	for (unsigned int x = 0; x < constraints.size(); x++)
	{
		if ((constraints[x].dir == kTeleport) && time+1 == constraints[x].loc.t && (to == constraints[x].loc.l))
			return true;
		if ((constraints[x].dir != kTeleport) && time+1 == constraints[x].loc.t && (constraints[x].loc.l == to))
		{
			tmp = to;
			mapEnv->UndoAction(tmp, constraints[x].dir);
			if (tmp == from)
				return true;
		}
	}
	return false;
}

void Map2DConstrainedEnvironment::GetActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const
{
	mapEnv->GetActions(nodeID.l, actions);

	// TODO: remove illegal actions
}

tDirection Map2DConstrainedEnvironment::GetAction(const xytLoc &s1, const xytLoc &s2) const
{
	return mapEnv->GetAction(s1.l, s2.l);
}

void Map2DConstrainedEnvironment::ApplyAction(xytLoc &s, tDirection a) const
{
	mapEnv->ApplyAction(s.l, a);
	s.t+=1;
}

void Map2DConstrainedEnvironment::UndoAction(xytLoc &s, tDirection a) const
{
	mapEnv->UndoAction(s.l, a);
	s.t-=1;
}


bool Map2DConstrainedEnvironment::InvertAction(tDirection &a) const
{
	return mapEnv->InvertAction(a);
}


/** Heuristic value between two arbitrary nodes. **/
double Map2DConstrainedEnvironment::HCost(const xytLoc &node1, const xytLoc &node2) const
{
	double res1 = mapEnv->HCost(node1.l, node2.l);
	double res2 = (node2.t>node1.t)?(node2.t-node1.t):0;
	//std::cout << "h(" << node1 << ", " << node2 << ") = " << res1 << " " << res2 << std::endl;
	return max(res1, res2);
}

bool Map2DConstrainedEnvironment::GoalTest(const xytLoc &node, const xytLoc &goal) const
{
	return (node.l == goal.l && node.t >= goal.t);
}


uint64_t Map2DConstrainedEnvironment::GetStateHash(const xytLoc &node) const
{
	uint64_t hash;
	hash = node.l.x;
	hash <<= 16;
	hash |= node.l.y;
	hash <<= 16;
	hash |= node.t;
	return hash;
}

uint64_t Map2DConstrainedEnvironment::GetActionHash(tDirection act) const
{
	return act;
}


//void Map2DConstrainedEnvironment::OpenGLDraw() const
//{
//	mapEnv->OpenGLDraw();
//	// draw constrains
//	Map *map = mapEnv->GetMap();
//	for (unsigned int x = 0; x < constraints.size(); x++)
//	{
//		GLdouble xx, yy, zz, rad;
//		map->GetCoord(constraints[x].loc.l.x, constraints[x].loc.l.y, xx, yy, zz, rad);
//		glColor4f(1.0, 0.0, 0.0, 0.5);
//		//glColor3f(0.5, 0.5, 0.5);
//		DrawSphere(xx, yy, zz-constraints[x].loc.t*rad, rad);
//	}
//}
//
//void Map2DConstrainedEnvironment::OpenGLDraw(const xytLoc& l) const
//{
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	Map *map = mapEnv->GetMap();
//	GLdouble xx, yy, zz, rad;
//	map->GetCoord(l.l.x, l.l.y, xx, yy, zz, rad);
//	glColor4f(r, g, b, t);
//	DrawSphere(xx, yy, zz-l.t*rad, rad); // zz-l.t*2*rad
//}
//
//void Map2DConstrainedEnvironment::OpenGLDraw(const xytLoc&, const tDirection&) const
//{
//	
//}
//
//void Map2DConstrainedEnvironment::GLDrawLine(const xytLoc &x, const xytLoc &y) const
//{
//	GLdouble xx, yy, zz, rad;
//	Map *map = mapEnv->GetMap();
//	map->GetCoord(x.l.x, x.l.y, xx, yy, zz, rad);
//	
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	glColor4f(r, g, b, t);
//	glBegin(GL_LINES);
//	glVertex3f(xx, yy, zz-x.t*rad);
//	map->GetCoord(y.l.x, y.l.y, xx, yy, zz, rad);
//	glVertex3f(xx, yy, zz-y.t*rad);
//	glEnd();
//}
//
