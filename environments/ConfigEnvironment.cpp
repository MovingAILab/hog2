/*
 *  ConfigEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/9/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include "ConfigEnvironment.h"
#include "FPUtil.h"
#include <iostream>
#include <math.h>

ConfigEnvironment::ConfigEnvironment()
{
	goal_stored = false;
}

ConfigEnvironment::~ConfigEnvironment()
{
}

void ConfigEnvironment::GetSuccessors(const Graphics::point &nodeID, std::vector<Graphics::point> &neighbors) const
{
	//[(-0.300000, -0.320000) to (0.000000, 0.000000)] does not cross [(-0.320000, -0.300000) to (0.320000, -0.300000)]
//	line2d a(Graphics::point(-0.300000, -0.320000, 0), Graphics::point(0, -0.4, 0));
//	line2d b(Graphics::point(-0.320000, -0.300000, 0), Graphics::point(0.320000, -0.300000, 0));
//	line2d c(Graphics::point(-0.300000, -0.320000, 0), Graphics::point(-0.300000, 0.320000, 0));
//	if (a.crosses(b))
//		printf("1 They Cross!\n");
//	else
//		printf("1 No crossing here!\n");
//	if (b.crosses(a))
//		printf("2 They Cross!\n");
//	else
//		printf("2 No crossing here!\n");
//	if (a.crosses(c))
//		printf("3 They Cross!\n");
//	else
//		printf("3 No crossing here!\n");
//	if (c.crosses(a))
//		printf("4 They Cross!\n");
//	else
//		printf("4 No crossing here!\n");

	neighbors.resize(0);
	neighbors.push_back(goal);
	for (unsigned int x = 0; x < obstacles.size(); x++)
	{
		neighbors.push_back(obstacles[x].start);
		neighbors.push_back(obstacles[x].end);
	}
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (!Legal(nodeID, neighbors[x]))
		{
			neighbors[x] = neighbors.back();
			neighbors.pop_back();
			x--;
		}
	}
//	printf("Returning:\n");
//	for (unsigned int x = 0; x < neighbors.size(); x++)
//		std::cout << neighbors[x] << " ";
//	std::cout << std::endl;
}

void ConfigEnvironment::GetActions(const Graphics::point &nodeID, std::vector<line2d> &actions) const
{
	actions.resize(0);
	actions.push_back(line2d(nodeID, goal));
	for (unsigned int x = 0; x < obstacles.size(); x++)
	{
		actions.push_back(line2d(nodeID, obstacles[x].start));
		actions.push_back(line2d(nodeID, obstacles[x].end));
	}
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		if (!Legal(actions[x].start, actions[x].end))
		{
			actions[x] = actions.back();
			actions.pop_back();
			x--;
		}
	}
}

bool ConfigEnvironment::Legal(const Graphics::point &a, const Graphics::point &b) const
{
	line2d l(a, b);
	for (unsigned int x = 0; x < obstacles.size(); x++)
	{
		if (obstacles[x].crosses(l))
		{
//			printf("[(%f, %f) to (%f, %f)] CROSSES [(%f, %f) to (%f, %f)]\n",
//				   a.x, a.y, b.x, b.y,
//				   obstacles[x].start.x, obstacles[x].start.y,
//				   obstacles[x].end.x, obstacles[x].end.y);
			return false;
		}
//		printf("[(%f, %f) to (%f, %f)] does not cross [(%f, %f) to (%f, %f)]\n",
//			   a.x, a.y, b.x, b.y,
//			   obstacles[x].start.x, obstacles[x].start.y,
//			   obstacles[x].end.x, obstacles[x].end.y);
	}
	return true;
}

line2d ConfigEnvironment::GetAction(const Graphics::point &s1, const Graphics::point &s2) const
{
	line2d d;
	d.start = s1;
	d.end = s2;
	return d;
}

void ConfigEnvironment::ApplyAction(Graphics::point &s, line2d dir) const
{
	s = dir.end;
}


bool ConfigEnvironment::InvertAction(line2d &a) const
{
	line2d b = a;
	a.start = b.end;
	a.end = b.start;
	return true;
}


double ConfigEnvironment::HCost(const Graphics::point &node1, const Graphics::point &node2) const
{
	return sqrt((node1.x-node2.x)*(node1.x-node2.x) +
				(node1.y-node2.y)*(node1.y-node2.y) +
				(node1.z-node2.z)*(node1.z-node2.z));
}

double ConfigEnvironment::GCost(const Graphics::point &node1, const  Graphics::point &node2) const
{
	return sqrt((node1.x-node2.x)*(node1.x-node2.x) +
				(node1.y-node2.y)*(node1.y-node2.y) +
				(node1.z-node2.z)*(node1.z-node2.z));
}

double ConfigEnvironment::GCost(const Graphics::point &node1, const line2d &act) const
{
	return sqrt((node1.x-act.end.x)*(node1.x-act.end.x) +
				(node1.y-act.end.y)*(node1.y-act.end.y) +
				(node1.z-act.end.z)*(node1.z-act.end.z));
}

bool ConfigEnvironment::GoalTest(const Graphics::point &node, const Graphics::point &theGoal) const
{
	return (fequal(node.x, theGoal.x) && fequal(node.y, theGoal.y) && fequal(node.z, theGoal.z));
}

uint64_t ConfigEnvironment::GetStateHash(const Graphics::point &node) const
{
	int x, y;
	bool nx = node.x<0, ny = node.y<0;
	x = (int)((double)fabs(node.x)*10000.0);
	y = (int)((double)fabs(node.y)*10000.0);
	uint64_t val = 0;
	val = x;
	val = (val<<14)|y;
	val = (val<<1)|(nx?1:0);
	val = (val<<1)|(ny?1:0);
	return val;
}

uint64_t ConfigEnvironment::GetActionHash(line2d act) const
{
	return (GetStateHash(act.start)^GetStateHash(act.end));
}


//void ConfigEnvironment::OpenGLDraw() const
//{
//	glColor3f(0, 0, 1.0);
//	for (unsigned int x = 0; x < obstacles.size(); x++)
//		DrawLine(obstacles[x]);
//}
//
//void ConfigEnvironment::OpenGLDraw(const Graphics::point &l) const
//{
//	glBegin(GL_POINT);
//	glVertex3f(l.x, l.y, l.z);
//	glEnd();
//}
//
//void ConfigEnvironment::OpenGLDraw(const Graphics::point &, const line2d &) const
//{
//}

//void ConfigEnvironment::OpenGLDraw(const Graphics::point &, const line2d &, GLfloat r, GLfloat g, GLfloat b) const
//{
//}
//
//void ConfigEnvironment::OpenGLDraw(const Graphics::point &l, GLfloat r, GLfloat g, GLfloat b) const
//{
//}


void ConfigEnvironment::GetNextState(const Graphics::point &, line2d dir, Graphics::point &news) const
{
	news = dir.end;
}

//void ConfigEnvironment::DrawLine(line2d l) const
//{
//	glLineWidth(3);
//	glBegin(GL_LINES);
//	glVertex3f(l.start.x, l.start.y, 0);
//	glVertex3f(l.end.x, l.end.y, 0);
//	glEnd();
//	glLineWidth(1);
//}
