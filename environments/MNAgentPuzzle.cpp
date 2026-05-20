/*
 *  MNAgentPuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/25/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#include "MNAgentPuzzle.h"
#include <assert.h>

void MNAgentEnvironment::GetSuccessors(const MNAgentPuzzleState &nodeID, std::vector<MNAgentPuzzleState> &neighbors) const
{
	neighbors.resize(0);
	std::vector<tAgentAction> actions;
	GetActions(nodeID, actions);
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		MNAgentPuzzleState s = nodeID;
		ApplyAction(s, actions[x]);
		neighbors.push_back(s);
	}
}

void MNAgentEnvironment::GetActions(const MNAgentPuzzleState &s, std::vector<tAgentAction> &actions) const
{
	int where = -1;
	for (unsigned int w = 0; w < s.locations.size(); w++)
		if (((s.locations[w]>>s.currentAgent)&0x1) != 0)
		{
			where = w;
			break;
		}
	assert(where != -1);
	
	uint64_t mask = (1<<s.currentAgent)-1;
	if ((where-s.width > 0) && (s.locations[where-s.width]&mask) == 0)
	{
		actions.push_back(kAgentUp);
	}
	if ((where+s.width < s.locations.size()) && (s.locations[where+s.width]&mask) == 0)
	{
		actions.push_back(kAgentDown);
	}
	if ((where%s.width > 0) && (s.locations[where-1]&mask) == 0)
	{
		actions.push_back(kAgentLeft);
	}
	if ((where%s.width != s.width-1) && (s.locations[where+1]&mask) == 0)
	{
		actions.push_back(kAgentRight);
	}
	if ((s.locations[where]&mask) == 0)
	{
		actions.push_back(kAgentStay);
	}
}

tAgentAction MNAgentEnvironment::GetAction(const MNAgentPuzzleState &s1, const MNAgentPuzzleState &s2) const
{
	assert(false);
}

void MNAgentEnvironment::ApplyAction(MNAgentPuzzleState &s, tAgentAction a) const
{
	uint64_t mask1 = (uint64_t)1<<s.currentAgent;
	uint64_t mask2 = ~mask1;

	int where = -1;
	for (unsigned int w = 0; w < s.locations.size(); w++)
		if (((s.locations[w]>>s.currentAgent)&0x1) != 0)
		{
			where = w;
			break;
		}
	assert(where != -1);

	s.locations[where]&=mask2;
	// move 
	switch (a)
	{
		case kAgentStay: s.locations[where]|=mask1; break;
		case kAgentLeft: s.locations[where-1]|=mask1; break;
		case kAgentRight:s.locations[where+1]|=mask1; break;
		case kAgentDown: s.locations[where+s.width]|=mask1; break;
		case kAgentUp:   s.locations[where-s.width]|=mask1; break;
	}
	s.currentAgent = (s.currentAgent+1)%s.numAgents;
}

void MNAgentEnvironment::GetNextState(const MNAgentPuzzleState &s1, tAgentAction a, MNAgentPuzzleState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool MNAgentEnvironment::InvertAction(tAgentAction &a) const
{
	assert(false);
}

/** Heuristic value between two arbitrary nodes. **/
double MNAgentEnvironment::HCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2) const
{
	// for now we don't need this
	return 1;
}

double MNAgentEnvironment::GCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2) const
{ return 1; }
double MNAgentEnvironment::GCost(const MNAgentPuzzleState &node, const tAgentAction &act) const
{ return 1; }

bool MNAgentEnvironment::GoalTest(const MNAgentPuzzleState &node, const MNAgentPuzzleState &goal) const
{
	return (node == goal);
}

uint64_t MNAgentEnvironment::GetStateHash(const MNAgentPuzzleState &node) const
{
	int base = node.numAgents;
	uint64_t hash = 0;
	for (unsigned int x = 0; x < node.locations.size(); x++)
	{
		int which = -1;
		for (unsigned int w = 0; w < node.numAgents; w++)
		{
			if (((node.locations[x]>>w)&0x1) != 0)
			{
				which = w;
				break;
			}
		}
		which += 1;
		if ((node.locations[x] != 0) && (node.locations[x] != filled))
		{
			hash = hash*base+which;
		}
	}
	return hash;
}

uint64_t MNAgentEnvironment::GetActionHash(tAgentAction act) const
{
	return act;
}

//void MNAgentEnvironment::OpenGLDraw() const
//{
//}
//
//void MNAgentEnvironment::OpenGLDraw(const MNAgentPuzzleState&s) const
//{
//	glBegin(GL_QUADS);
//	glColor3f(0.5, 0.5, 0.5);
//	glVertex3f(-1, -1, 0.0);
//	glVertex3f( 1, -1, 0.0);
//	glVertex3f( 1,  1, 0.0);
//	glVertex3f(-1,  1, 0.0);
//	glEnd();
//
//	double radius = max(s.height, s.width);
//	radius = 1/radius;
//	for (unsigned int x = 0; x < s.locations.size(); x++)
//	{
//		if (s.locations[x] == -1)
//		{
//			glColor3f(0.0, 0.0, 0.0);
//			DrawSphere(-1.0+radius+(x%s.width)*2.0*radius,
//					   -1.0+radius+((double)((int)x/s.width))*2.0*radius,
//					   0, radius);
//		}
//		else if (s.locations[x] > 0)
//		{
//			glColor3f(0.0, (double)s.locations[x]/(double)(1<<s.numAgents), 1-(double)s.locations[x]/(double)(1<<s.numAgents));
//			DrawSphere(-1.0+radius+(x%s.width)*2.0*radius,
//					   -1.0+radius+((double)((int)x/s.width))*2.0*radius,
//					   0, radius);
////			glPushMatrix();
////			
////			GLdouble xx, yy, zz, rad;
////			GLfloat r, g, b, t;
////			GetColor(r, g, b, t);
////			glColor4f(r, g, b, t);
////			
////			glTranslatef(xx-rad, yy+rad/2, zz-rad);
////			glScalef(rad/(300.0), rad/300.0, 1);
////			glRotatef(180, 0.0, 0.0, 1.0);
////			glRotatef(180, 0.0, 1.0, 0.0);
////			//glTranslatef((float)x/width-0.5, (float)y/height-0.5, 0);
////			glDisable(GL_LIGHTING);
////			glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+);
////			glEnable(GL_LIGHTING);
////			//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
////			glPopMatrix();
////
//		}
//	}
//}
//
///** Draw the transition at some percentage 0...1 between two MNAgentPuzzleStates */
//void MNAgentEnvironment::OpenGLDraw(const MNAgentPuzzleState&, const MNAgentPuzzleState&, float) const
//{
//}
//
//void MNAgentEnvironment::OpenGLDraw(const MNAgentPuzzleState&, const tAgentAction&) const
//{
//}
