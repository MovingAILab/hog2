/*
 *  NaryTree.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/20/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#include "NaryTree.h"
#include <iostream>
#include <cinttypes>

NaryTree::NaryTree(int branchingFactor, int depth) :b(branchingFactor), d(depth)
{
	scaleWidth = 1.0;
	uint64_t tot = 1;
	uint64_t sum = 1;
	for (int x = 0; x <= d; x++)
	{
		nodesAtDepth.push_back(tot);
		totalNodesAtDepth.push_back(sum);
		tot*=b;
		sum += tot;
	}
//	for (size_t x = 0; x < nodesAtDepth.size(); x++)
//	{
//		printf("%d %" PRId64 " %" PRId64 "\n", x, nodesAtDepth[x], totalNodesAtDepth[x]);
//	}
}

void NaryTree::GetSuccessors(const NaryState &nodeID, std::vector<NaryState> &neighbors) const
{
//	std::cout << nodeID << " has depth " << GetDepth(nodeID) << std::endl;
	neighbors.resize(0);
	if (GetDepth(nodeID) >= d)
		return;
	for (int x = 0; x < b; x++)
		neighbors.push_back(nodeID*b+x+1);
}

void NaryTree::GetActions(const NaryState &nodeID, std::vector<NaryAction> &actions) const
{
	assert(false); // code not tested -
	actions.resize(0);
	if (GetDepth(nodeID) >= d)
		return;
	actions.resize(b);
	for (unsigned x = 0; x < actions.size(); x++)
		actions[x] = x+1;
}

NaryAction NaryTree::GetAction(const NaryState &, const NaryState &) const
{
	assert(false);
	return 0;
}

void NaryTree::ApplyAction(NaryState &s, NaryAction a) const
{
	// code not tested - should probably not be adding 1
	assert(false);
	if (a > 0)
		s = s*b+a+1;
	else {
		s = (s-1)/b;
	}
}

void NaryTree::GetNextState(const NaryState &s1, NaryAction a, NaryState &s2) const
{
	if (a > 0)
		s2 = s1*b+a;
	else {
		s2 = (s1-1)/b;
	}

}

bool NaryTree::InvertAction(NaryAction &a) const
{ a = -a; return true; }

/** Heuristic value between two arbitrary nodes. **/
double NaryTree::HCost(const NaryState &node1, const NaryState &node2) const
{ if (node1 == node2) return 0; return 1; }

double NaryTree::GCost(const NaryState &, const NaryState &) const
{ return 1; }
double NaryTree::GCost(const NaryState &, const NaryAction &) const
{ return 1; }
bool NaryTree::GoalTest(const NaryState &node, const NaryState &goal) const
{ return node == goal; }


uint64_t NaryTree::GetStateHash(const NaryState &node) const
{ return node; }

uint64_t NaryTree::GetActionHash(NaryAction act) const
{ return act+b; }

float sqdist(float x1, float y1, float x2, float y2)
{
	return (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
}

NaryState NaryTree::GetClosestNode(float x, float y)
{
	float bestDist = 100;
	float x1, y1;
	uint64_t best = 0;
	for (uint64_t t = 0; t < totalNodesAtDepth.back(); t++)
	{
		GetLocation(t, x1, y1);
		if (sqdist(x, y, x1, y1) < bestDist)
		{
			bestDist = sqdist(x, y, x1, y1);
			best = t;
		}
	}
	return best;
}


void NaryTree::GetLocation(const NaryState &s, float &x, float &y) const
{
	int depth = GetDepth(s);
	x = -1.0+(2.0*GetOffset(s))/nodesAtDepth[depth]+1.0/nodesAtDepth[depth];
	x *= scaleWidth;
	y = 2.0*float(depth)/float(d)-1;
	x *= 0.95;
	y *= 0.95;
	//printf("%" PRId64 " depth %d offset %" PRId64 "\n", s, depth, GetOffset(s));
}

int NaryTree::GetDepth(const NaryState s) const
{
	if (s == 0)
		return 0;
	if (s <= b)
		return 1;
	return 1+GetDepth((int)(s-1)/b);
}

uint64_t NaryTree::GetOffset(const NaryState s) const
{
	if (s == 0)
		return 0;
	for (int x = 0; x < totalNodesAtDepth.size(); x++)
	{
		if (s >= totalNodesAtDepth[x]-1 && s < totalNodesAtDepth[x+1])
			return s - totalNodesAtDepth[x];
	}
	return 0;
}

NaryState NaryTree::GetParent(NaryState s) const
{
	if (s == 0)
		return s;
	return (s-1)/b;
}

//void NaryTree::OpenGLDraw() const
//{
//	std::vector<NaryState> succ;
//	float x1, y1, x2, y2;
//	glBegin(GL_LINES);
//	glColor3f(1.0, 1.0, 1.0);
//	for (uint64_t t = 0; t < totalNodesAtDepth.back(); t++)
//	{
//		GetLocation(t, x1, y1);
//		GetSuccessors(t, succ);
//		for (uint64_t s : succ)
//		{
//			GetLocation(s, x2, y2);
//			glVertex3f(x1, y1, 0);
//			glVertex3f(x2, y2, 0);
//		}
//	}
//	glEnd();
//}
//
//void NaryTree::OpenGLDraw(const NaryState &s) const
//{
//	float x1, y1;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	glColor4f(r, g, b, t);
//	GetLocation(s, x1, y1);
//	double r1 = 2.0/nodesAtDepth[GetDepth(s)];
//	double r2 = 0.1/d;
//	DrawSphere(x1, y1, 0, std::min(r1, r2));
//}
//
//void NaryTree::GLDrawLine(const NaryState &s1, const NaryState &s2) const
//{
//	float x1, y1, x2, y2;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	glLineWidth(6.0);
//	glBegin(GL_LINES);
//	glColor4f(r, g, b, t);
//	GetLocation(s1, x1, y1);
//	GetLocation(s2, x2, y2);
//	glVertex3f(x1, y1, 0);
//	glVertex3f(x2, y2, 0);
//	glEnd();
//	glLineWidth(1.0);
//}

void NaryTree::Draw(Graphics::Display &display) const
{
	rgbColor color = GetColor();
	std::vector<NaryState> succ;
	float x1, y1, x2, y2;
	float r1, r2;
	// loop through the total number of nodes in the whole tree
	for (uint64_t t = 0; t < totalNodesAtDepth.back(); t++)
	{
		GetLocation(t, x1, y1);
		GetSuccessors(t, succ);
		int depth = GetDepth(t);
		r1 = GetDepthRadius(depth);
		for (uint64_t s : succ)
		{
			GetLocation(s, x2, y2);
			r2 = GetDepthRadius(depth+1);

			float theta = atan2(x1-x2, y1-y2);
			float stheta = -sin(theta);
			float ctheta = cos(theta);
//			display.DrawLine({x1, y1}, {x2, y2}, 2.0-1.8*(((y1+y2)/2.0+1.0)/2.0), color);
//			float r1 = 1.0/nodesAtDepth[GetDepth(s)];
//			float r2 = 0.1/d;
//			float r = std::min(r1, r2);
//			display.DrawLine({x1, y1}, {x2, y2}, r, color);
			
			display.FillTriangle({x1-r1*ctheta, y1-r1*stheta}, {x2-r2*ctheta, y2-r2*stheta}, {x1+r1*ctheta, y1+r1*stheta}, color);
			display.FillTriangle({x1+r1*ctheta, y1+r1*stheta}, {x2-r2*ctheta, y2-r2*stheta}, {x2+r2*ctheta, y2+r2*stheta}, color);
		}
		display.FillCircle({x1-r1, y1-r1, x1+r1, y1+r1}, Colors::gray);
	}
}

float NaryTree::GetDepthRadius(int depth) const
{
	float r1 = 0.33333/nodesAtDepth[depth];
	float r2 = 0.008;
	float r = std::min(r1, r2);
	return r;
}

void NaryTree::Draw(Graphics::Display &display, const NaryState &s) const
{
	float x1, y1;
	GetLocation(s, x1, y1);
//	float r1 = 2.0/nodesAtDepth[GetDepth(s)];
//	float r2 = 0.1/d;
	float r = std::max(0.01f, 3.0f*GetDepthRadius(GetDepth(s)));//std::min(r1, r2));
	display.FillCircle({x1-r, y1-r, x1+r, y1+r}, color);
}

void NaryTree::DrawLine(Graphics::Display &display, const NaryState &f, const NaryState &t, float width) const
{
	float x1, y1, x2, y2;
	GetLocation(f, x1, y1);
	int depth = GetDepth(f);
	float r1 = GetDepthRadius(depth)*width;

	GetLocation(t, x2, y2);
	float r2 = GetDepthRadius(depth+1)*width;

	float theta = atan2(x1-x2, y1-y2);
	float stheta = -sin(theta);
	float ctheta = cos(theta);

	display.FillCircle({x1, y1}, r1, color);
	display.FillCircle({x2, y2}, r2, color);
	display.FillTriangle({x1-r1*ctheta, y1-r1*stheta}, {x2-r2*ctheta, y2-r2*stheta}, {x1+r1*ctheta, y1+r1*stheta}, color);
	display.FillTriangle({x1+r1*ctheta, y1+r1*stheta}, {x2-r2*ctheta, y2-r2*stheta}, {x2+r2*ctheta, y2+r2*stheta}, color);
}


//void NaryTree::OpenGLDraw(const NaryState&, const NaryState&, float) const { }
//void NaryTree::OpenGLDraw(const NaryState&, const NaryAction&) const { }
