/*
 *  SteeringEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/12/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#include "SteeringEnvironment.h"

float maxSpeed = 0.5f;
float worldRadius = 200.0f;

void SteeringEnvironment::GetSuccessors(const steeringState &nodeID, std::vector<steeringState> &neighbors) const
{
	std::vector<steeringAction> act;
	GetActions(nodeID, act);
	for (unsigned int x = 0; x < act.size(); x++)
	{
		steeringState s = nodeID;
		ApplyAction(s, act[x]);
		neighbors.push_back(s);
	}
}

void SteeringEnvironment::GetActions(const steeringState &nodeID, std::vector<steeringAction> &actions) const
{
	actions.resize(0);
	actions.push_back(steeringAction(-1, -1));
	actions.push_back(steeringAction(-1,  0));
	actions.push_back(steeringAction(-1,  1));
	actions.push_back(steeringAction( 0, -1));
	actions.push_back(steeringAction( 0,  0));
	actions.push_back(steeringAction( 0,  1));
	actions.push_back(steeringAction( 1, -1));
	actions.push_back(steeringAction( 1,  0));
	actions.push_back(steeringAction( 1, -1));
}

steeringAction SteeringEnvironment::GetAction(const steeringState &, const steeringState &) const
{
	assert(false);
	return 0;
}

void SteeringEnvironment::ApplyAction(steeringState &s, steeringAction a) const
{
	// scale movement vector
	if (a.dv > maxSpeed)
	{
		a.dv = maxSpeed;
	}
	else if (a.dv < -maxSpeed)
		a.dv = -maxSpeed;
	
	float yoffset = sin(TWOPI*s.heading/360.0)*s.v;//sin(TWOPI*rot/16)*rad;
	float xoffset = cos(TWOPI*s.heading/360.0)*s.v;//cos(TWOPI*rot/16)*rad;

	s.x += xoffset;//a.dx;
	s.y += yoffset;//a.dy;
	s.v += a.dv;
	if (s.v > maxSpeed)
		s.v = maxSpeed;
	if (s.v < 0)
		s.v = 0;

	s.heading += a.dh;
	if (s.heading > 360)
		s.heading-=360;
	if (s.heading < 0)
		s.heading+=360;
	
	if (s.x > worldRadius)
		s.x -= worldRadius*2;
	if (s.x < -worldRadius)
		s.x += worldRadius*2;
	if (s.y > worldRadius)
		s.y -= worldRadius*2;
	if (s.y < -worldRadius)
		s.y += worldRadius*2;

}

void SteeringEnvironment::GetNextState(const steeringState &s1, steeringAction a, steeringState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool SteeringEnvironment::InvertAction(steeringAction &a) const
{ return false; }

/** Heuristic value between two arbitrary nodes. **/
double SteeringEnvironment::HCost(const steeringState &node1, const steeringState &node2) const
{ if (node1 == node2) return 0; return 1; }

double SteeringEnvironment::GCost(const steeringState &, const steeringState &) const
{ return 1; }
double SteeringEnvironment::GCost(const steeringState &, const steeringAction &) const
{ return 1; }
bool SteeringEnvironment::GoalTest(const steeringState &node, const steeringState &goal) const
{ return node == goal; }


uint64_t SteeringEnvironment::GetStateHash(const steeringState &node) const
{ assert(false); return 3; }

uint64_t SteeringEnvironment::GetActionHash(steeringAction act) const
{ assert(false); return 3; }


//void SteeringEnvironment::OpenGLDraw() const
//{
//	glLineWidth(10);
//	glColor4f(0.0, 0.0, 1.0, 1.0);
//	glBegin(GL_LINE_LOOP);
//	glVertex3f(-1.0, -1.0, 0);
//	glVertex3f(+1.0, -1.0, 0);
//	glVertex3f(+1.0, +1.0, 0);
//	glVertex3f(-1.0, +1.0, 0);
//	glEnd();
//	glLineWidth(1);
//}
//
//void SteeringEnvironment::OpenGLDraw(const steeringState&s) const
//{
//	GLdouble xx, yy, zz, rad;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	xx = s.x/(worldRadius);
//	yy = s.y/(worldRadius);
//	zz = 0;
//	rad = 2.0f/worldRadius;
//	//map->GetCoord(l.x, l.y, xx, yy, zz, rad);
//	
//	GLdouble yoffset = sin(TWOPI*s.heading/360.0)*rad;//sin(TWOPI*rot/16)*rad;
//	GLdouble xoffset = cos(TWOPI*s.heading/360.0)*rad;//cos(TWOPI*rot/16)*rad;
//	
//	//	glColor3f(0, 0, 1.0);
//	//	glBegin(GL_LINE_STRIP);
//	//	glVertex3f(xx-rad, yy-rad, zz-rad);
//	//	glVertex3f(xx-rad, yy-rad, zz);
//	//	glEnd();
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
//void SteeringEnvironment::OpenGLDraw(const steeringState&s1, const steeringState&s2, float mix) const
//{
//	GLdouble xx, yy, zz, rad;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	xx = (s1.x+s2.x)/(2*worldRadius);
//	yy = (s1.y+s2.y)/(2*worldRadius);
//	zz = 0;
//	rad = 2.0f/worldRadius;
//	//map->GetCoord(l.x, l.y, xx, yy, zz, rad);
//	
//	GLdouble yoffset = sin(TWOPI*(s1.heading+s2.heading)/(2*360.0))*rad;//sin(TWOPI*rot/16)*rad;
//	GLdouble xoffset = cos(TWOPI*(s1.heading+s2.heading)/(2*360.0))*rad;//cos(TWOPI*rot/16)*rad;
//	
//	//	glColor3f(0, 0, 1.0);
//	//	glBegin(GL_LINE_STRIP);
//	//	glVertex3f(xx-rad, yy-rad, zz-rad);
//	//	glVertex3f(xx-rad, yy-rad, zz);
//	//	glEnd();
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
//void SteeringEnvironment::OpenGLDraw(const steeringState&, const steeringAction&) const { }
