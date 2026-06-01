//
//  MotionCaptureMovement.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 11/9/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "MotionCaptureMovement.h"

const double worldSize = 20;
const double goalTolerance = 0.5;

struct mcData {
	float turnSpeed;
	float xOffset, yOffset, zOffset, headingOffset;
};

const int numActions = 27;

mcData d[] = {
//	{-110.0, -0.911747098, -0.00472318381, 0.563538074, -123.847564697},
//	{-100.0, -0.869839549, -0.00453251554, 0.668048501, -112.786506653},
	{-90.0, -0.820025563, -0.00566683663, 0.761402547, -102.513450623},
	{-80.0, -0.754989743, -0.00520792603, 0.846449375, -90.6535682678},
	{-70.0, -0.68668443, -0.00474862708, 0.92180711, -79.623046875},
	{-60.0, -0.621659517, -0.00342348195, 0.979949296, -70.1457858086},
	{-50.0, -0.556169391, 0.0011247769, 1.02253902, -62.9730377197},
	{-40.0, -0.492958367, 0.00525747985, 1.06271756, -55.8160095215},
	{-30.0, -0.373254806, 0.00502402289, 1.10341728, -42.5149187148},
	{-20.0, -0.233692259, 0.00385937118, 1.13184297, -27.7257080078},
	{-10.0, -0.106234029, 0.0014725402, 1.12895668, -13.5424880981},
	{-8.0,  -0.0791881531, 0.00153348595, 1.12579584, -10.5965538025},
	{-6.0,  -0.0550608225, 0.0005375818, 1.12165511, -7.847448349},
	{-4.0,  -0.0291669685, 0.000460080802, 1.11724615, -4.96613055468},
	{-2.0,  -0.00520789856, -8.18446206e-005, 1.11239338, -2.18479919434},
	{0.0 , 0.0199751724, -0.000237718239, 1.10685933, 0.656613349915},
	{2.0 , 0.0474604815, -0.000476859539, 1.10631061, 3.42151546478},
	{4.0 , 0.0753834769, -0.000586710812, 1.10548747, 6.21252536774},
	{6.0 , 0.103622861, -0.000562481582, 1.10425735, 9.0348777771},
	{8.0 , 0.132121757, -0.000393480062, 1.10258472, 11.8929595947},
	{10.0, 0.161176234, -0.000701442303, 1.10047519, 14.4690093994},
	{20.0, 0.305605233, -0.000723838806, 1.07536352, 28.0411071777},
	{30.0, 0.435892373, 0.00164459657, 1.01915348, 41.6651306152},
	{40.0, 0.560899138, 0.00394713879, 0.949841857, 53.7326016426},
	{50.0, 0.64627862, 0.00594675541, 0.905627429, 60.9953536987},
	{60.0, 0.730919778, 0.00894816965, 0.86073643, 67.0545349121},
	{70.0, 0.835809886, 0.00966545194, 0.751601994, 78.9754219055},
	{80.0, 0.922447503, 0.00939706713, 0.622487068, 91.9488887787},
	{90.0, 0.992749035, 0.00875499006, 0.477440089, 105.611335754},
//	{100.0, 1.0382669, 0.0079389289, 0.327931404, 118.453689575},
//	{110.0, 1.04703259, 0.00703916652, 0.195105791, 130.582626343},
};

void MCEnvironment::GetSuccessors(const mcMovementState &nodeID, std::vector<mcMovementState> &neighbors) const
{
	mcMovementState s;
	neighbors.resize(0);
	for (int x = 0; x < numActions; x++)
	{
		s = nodeID;
		ApplyAction(s, x);
//		s.x = nodeID.x + d[x].zOffset*cos(TWOPI*s.heading/360.0)+d[x].xOffset*sin(TWOPI*s.heading/360.0);
//		s.y = nodeID.y + d[x].xOffset*cos(TWOPI*s.heading/360.0)+d[x].zOffset*sin(TWOPI*s.heading/360.0);
//		s.heading = nodeID.heading + d[x].headingOffset;
		if (s.x > worldSize || s.x < -worldSize)
			continue;
		if (s.y > worldSize || s.y < -worldSize)
			continue;
		//s.heading += nodeID.heading + d[x].headingOffset;
//		if (s.heading > 360)
//			s.heading -= 360;
//		if (s.heading < 0)
//			s.heading += 360;
		neighbors.push_back(s);
	}
}

void MCEnvironment::GetActions(const mcMovementState &nodeID, std::vector<mcMovementAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < numActions; x++)
	{
		if (nodeID.x + d[x].zOffset*cos(TWOPI*nodeID.heading/360.0)+d[x].xOffset*sin(TWOPI*nodeID.heading/360.0) > worldSize)
			continue;
		if (nodeID.x + d[x].zOffset*cos(TWOPI*nodeID.heading/360.0)+d[x].xOffset*sin(TWOPI*nodeID.heading/360.0) < -worldSize)
			continue;
		if (nodeID.y + d[x].xOffset*cos(TWOPI*nodeID.heading/360.0)+d[x].zOffset*sin(TWOPI*nodeID.heading/360.0) > worldSize)
			continue;
		if (nodeID.y + d[x].xOffset*cos(TWOPI*nodeID.heading/360.0)+d[x].zOffset*sin(TWOPI*nodeID.heading/360.0) < -worldSize)
			continue;

//		if ((nodeID.x + d[x].xOffset > worldSize) || (nodeID.x + d[x].xOffset < -worldSize))
//			continue;
//		if ((nodeID.y + d[x].yOffset > worldSize) || (nodeID.y + d[x].yOffset < -worldSize))
//			continue;
		actions.push_back(x);
	}
}

//int GetNumSuccessors(const mcMovementState &stateID) const;
mcMovementAction MCEnvironment::GetAction(const mcMovementState &s1, const mcMovementState &s2) const
{
	mcMovementState tmp;
	for (int x = 0; x < numActions; x++)
	{
		GetNextState(s1, x, tmp);
		if (tmp == s2)
			return x;
	}
	std::vector<mcMovementState> moves;
	GetSuccessors(s1, moves);
	assert(false);
	return 0;
//	mcMovementAction a;
//	return a;
}

void MCEnvironment::ApplyAction(mcMovementState &s, mcMovementAction a) const
{
	//	{-160.0, -0.815222442, -0.0184309911, 0.244376302, -152.432628632},
	s.x += d[a].zOffset*cos(TWOPI*s.heading/360.0)+d[a].xOffset*sin(TWOPI*s.heading/360.0);
	s.y += d[a].xOffset*cos(TWOPI*s.heading/360.0)+d[a].zOffset*sin(TWOPI*s.heading/360.0);
	s.heading += d[a].headingOffset;
	if (s.heading > 360)
		s.heading -= 360;
	if (s.heading < 0)
		s.heading += 360;
}

void MCEnvironment::GetNextState(const mcMovementState &s1, mcMovementAction a, mcMovementState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool MCEnvironment::InvertAction(mcMovementAction &a) const
{
	return false;
}

/** Heuristic value between two arbitrary nodes. **/
double MCEnvironment::HCost(const mcMovementState &node1, const mcMovementState &node2) const
{
	return distance(node1, node2);
}

double MCEnvironment::GCost(const mcMovementState &node1, const mcMovementState &node2) const
{
	return distance(node1, node2);
}

double MCEnvironment::GCost(const mcMovementState &node, const mcMovementAction &act) const
{
	mcMovementState node2;
	GetNextState(node, act, node2);
	return distance(node, node2);
}

bool MCEnvironment::GoalTest(const mcMovementState &node, const mcMovementState &goal) const
{
	return (distance(node, goal) < goalTolerance && abs(node.heading-goal.heading) < 5.0);
}

uint64_t MCEnvironment::GetStateHash(const mcMovementState &node) const
{
	uint64_t x = (node.x+worldSize)*100;
	uint64_t y = (node.y+worldSize)*100;
	uint64_t ang = node.heading;
	return (x<<40)|(y<<20)|(ang);
}

uint64_t MCEnvironment::GetActionHash(mcMovementAction act) const
{
	return act;
}

double MCEnvironment::distance(const mcMovementState &n1, const mcMovementState &n2) const
{
	return sqrt((n1.x - n2.x)*(n1.x - n2.x) + (n1.y-n2.y)*(n1.y-n2.y));
}

//void MCEnvironment::OpenGLDraw() const
//{
//	
//}
//
//void MCEnvironment::OpenGLDraw(const mcMovementState&) const
//{
//	
//}
//
///** Draw the transition at some percentage 0...1 between two states */
//void MCEnvironment::OpenGLDraw(const mcMovementState&, const mcMovementState&, float) const
//{
//	
//}
//
//void MCEnvironment::OpenGLDraw(const mcMovementState&, const mcMovementAction&) const
//{
//	
//}

void MCEnvironment::OpenGLDraw(const mcMovementState& oldState, const mcMovementState &newState, float perc) const
{
	int DEG = 360;
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	
	GLdouble xx, yy, zz, rad;
	GetOpenGLCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
	
	float rot = (1-perc)*oldState.heading+perc*newState.heading;
	if ((oldState.heading >= 3*DEG/4) && (newState.heading <= DEG/4))
	{
		rot = (1-perc)*oldState.heading+perc*(newState.heading+DEG);
		if (rot >= DEG)
			rot -= DEG;
	}
	else if ((newState.heading >= 3*DEG/4) && (oldState.heading <= DEG/4))
	{
		rot = (1-perc)*(oldState.heading+DEG)+perc*(newState.heading);
		if (rot >= DEG)
			rot -= DEG;
	}
	GLdouble yoffset = sin(TWOPI*rot/DEG)*rad;
	GLdouble xoffset = cos(TWOPI*rot/DEG)*rad;
	
	glBegin(GL_TRIANGLES);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
	
	glColor4f(r, g/2, b, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
	glEnd();
}


void MCEnvironment::OpenGLDraw(const mcMovementState &l) const
{
	GLdouble xx, yy, zz, rad;
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	
	GLdouble yoffset = sin(TWOPI*l.heading/360.0)*rad;//sin(TWOPI*rot/16)*rad;
	GLdouble xoffset = cos(TWOPI*l.heading/360.0)*rad;//cos(TWOPI*rot/16)*rad;
	
	glBegin(GL_TRIANGLES);
	recVec surfaceNormal;
	surfaceNormal.x = (((-0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((0.5*yoffset) - (rad)));
	surfaceNormal.z = (((0.5*yoffset) * (-2*yoffset)) - ((-0.5*xoffset) - (-2*xoffset)));
	surfaceNormal.normalise();
	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
	
	surfaceNormal.x = (((+0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((-0.5*yoffset) - (rad)));
	surfaceNormal.z = (((-0.5*yoffset) * (-2*yoffset)) - ((+0.5*xoffset) - (-2*xoffset)));
	surfaceNormal.normalise();
	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx+xoffset, yy+yoffset, zz);
	glColor4f(r, g, b/2, t);
	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
	glColor4f(r, g/2, b, t);
	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
	glEnd();
}


void MCEnvironment::OpenGLDraw(const mcMovementState& l, const mcMovementAction &) const
{
	GLdouble xx, yy, zz, rad;
	GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	glColor3f(0.5f, 0.5f, 0.5);
	DrawSphere(xx-rad+l.x, yy-rad+l.y, zz, rad);
}


void MCEnvironment::GLDrawLine(const mcMovementState &a, const mcMovementState &b) const
{
	GLdouble xx, yy, zz, rad;
	GetOpenGLCoord(a.x, a.y, xx, yy, zz, rad);
	
	GLfloat rr, gg, bb, t;
	GetColor(rr, gg, bb, t);
	glColor4f(rr, gg, bb, t);
	
	glBegin(GL_LINES);
	glVertex3f(xx, yy, zz-rad/2);
	GetOpenGLCoord(b.x, b.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-rad/2);
	glEnd();
}

bool MCEnvironment::GetOpenGLCoord(float x_, float y_, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const
{
	x = x_/worldSize;
	y = y_/worldSize;
	z = 0;
	radius = 1.0/worldSize;
	return true;
}

