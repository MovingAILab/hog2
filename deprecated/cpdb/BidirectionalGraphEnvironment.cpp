/*
 *  BidirectionalGraphEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/13/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include "BidirectionalGraphEnvironment.h"

BidirectionalGraphEnvironment::BidirectionalGraphEnvironment(GraphCanonicalHeuristic *h)
:gh(h)
{
	m = h->GetMap();
	g = h->GetGraph();
 	directed = true;
	swap = true;
	h = 0;
}

BidirectionalGraphEnvironment::BidirectionalGraphEnvironment(Graph *graph, GraphHeuristic *_h)
{
	h = _h;
	gh = 0;
	m = 0;
	g = graph;
 	directed = true;
	swap = true;
}

BidirectionalGraphEnvironment::~BidirectionalGraphEnvironment()
{
	// delete g; ??
	//	delete g;
	//	delete h;
}

void BidirectionalGraphEnvironment::GetSuccessors(const graphStatePair &stateID, std::vector<graphStatePair> &neighbors) const
{
	neighbors.resize(0);
	node *n = g->GetNode(stateID.start);
	
	if (n == 0)
	{
		return;
	}
	
	if (directed)
	{
//		std::cout << "Expanding " << stateID << std::endl;
		graphState goal = stateID.goal;
		edge_iterator ei = n->getOutgoingEdgeIter();
		for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
		{
			graphState here = e->getTo();
			graphState parent = e->getFrom();
			if (here == stateID.previousStart)
				continue;
			//if (swap && (gh->GetDistToCanonicalState(goal) < gh->GetDistToCanonicalState(here))) // swap if goal closer than current state
			//if (swap && (gh->GetDistToCanonicalState(goal) > gh->GetDistToCanonicalState(here))) // swap if goal farther than current state (suggested rule)
			//if (swap) // always swap
			//if (swap && (gh->GetDistToCanonicalState(here) == 0)) // swap only at canonical heuristic centers
			//if (swap && (gh->GetCanonicalStateID(parent) != gh->GetCanonicalStateID(here)))
			//if (swap && (n->getNumOutgoingEdges() == 2) && (gh->GetDistToCanonicalState(here) == 0))
			if (swap && (g->GetNode(here)->getNumOutgoingEdges() > 2+g->GetNode(parent)->getNumOutgoingEdges()))
			{
				//printf("x");
				neighbors.push_back(graphStatePair(goal, here, stateID.previousGoal, stateID.start));
			}
			else {
				neighbors.push_back(graphStatePair(here, goal, stateID.start, stateID.previousGoal));
			}
//			std::cout << neighbors.back();
		}
//		std::cout << std::endl;
	}
	else {
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			if (stateID.start != e->getTo())
				neighbors.push_back(graphStatePair(e->getTo(), stateID.goal));
			else
				neighbors.push_back(graphStatePair(e->getFrom(), stateID.goal));
		}
	}
}

void BidirectionalGraphEnvironment::GetActions(const graphStatePair &stateID, std::vector<graphMovePair> &actions) const
{
	actions.resize(0);
	node *n = g->GetNode(stateID.start);
	
	if (n == 0)
	{
		return;
	}
	
	if (directed)
	{
		edge_iterator ei = n->getOutgoingEdgeIter();
		for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
		{
			actions.push_back(graphMovePair(e->getFrom(),e->getTo()));
		}
	}
	else {
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			if(stateID.start != e->getTo())
				actions.push_back(graphMovePair(e->getFrom(),e->getTo()));
			else
				actions.push_back(graphMovePair(e->getTo(),e->getFrom()));
		}
	}
	
}

graphMovePair BidirectionalGraphEnvironment::GetAction(const graphStatePair &s1, const graphStatePair &s2) const
{
	return graphMovePair(s1.start, s2.start);
}

void BidirectionalGraphEnvironment::ApplyAction(graphStatePair &s, graphMovePair a) const
{
	assert(s.start == a.from);
	s.start = a.to;
}

bool BidirectionalGraphEnvironment::InvertAction(graphMovePair &a) const
{
	uint16_t tmp = a.from;
	a.from = a.to;
	a.to = tmp;
	if (g->findDirectedEdge(a.from, a.to))
		return true;
	return false;
}

double BidirectionalGraphEnvironment::HCost(const graphStatePair &state1, const graphStatePair &) const
{
	if (state1.start == state1.goal)
		return 0;
	if (gh)
		return std::max(1.0, gh->HCost(state1.start, state1.goal));
	if (h)
		return std::max(1.0, h->HCost(state1.start, state1.goal));
	return 1;
}

double BidirectionalGraphEnvironment::GCost(const graphStatePair &, const graphMovePair &move) const
{
	edge *e = g->FindEdge(move.from, move.to);
	assert(e);
	return e->GetWeight();
}

double BidirectionalGraphEnvironment::GCost(const graphStatePair &state1, const graphStatePair &state2) const
{
	edge *e = 0;
	if (state1.goal == state2.goal)
		e = g->FindEdge(state1.start, state2.start);
	else if (state1.start == state2.goal)
		e = g->FindEdge(state1.goal, state2.start);
	else if (state2.start == state1.goal)
		e = g->FindEdge(state2.goal, state1.start);
	//	if (!e)
	//		return -1000.0;
	assert(e);
	return e->GetWeight();
}

bool BidirectionalGraphEnvironment::GoalTest(const graphStatePair &state, const graphStatePair &) const
{
	return (state.start == state.goal);	
}

uint64_t BidirectionalGraphEnvironment::GetStateHash(const graphStatePair &state) const
{
	if (state.start > state.goal)
		return (((uint64_t)g->GetNode(state.start)->getUniqueID()&0xFFFFFFFF)<<32)|
		((uint64_t)g->GetNode(state.goal)->getUniqueID()&0xFFFFFFFF);
	return (((uint64_t)g->GetNode(state.goal)->getUniqueID()&0xFFFFFFFF)<<32)|
	((uint64_t)g->GetNode(state.start)->getUniqueID()&0xFFFFFFFF);
}

uint64_t BidirectionalGraphEnvironment::GetActionHash(graphMovePair act) const
{
	return (g->GetNode(act.from)->getUniqueID()<<16)|
	(g->GetNode(act.to)->getUniqueID());
}

void BidirectionalGraphEnvironment::OpenGLDraw() const
{
}

void BidirectionalGraphEnvironment::OpenGLDraw(const graphStatePair &s) const
{
	if (m)
	{
		GLdouble xx, yy, zz, rad;
		int x1, y1;
		node *n;
		n = g->GetNode(s.start);
		x1 = n->GetLabelL(GraphSearchConstants::kMapX);
		y1 = n->GetLabelL(GraphSearchConstants::kMapY);
		m->GetOpenGLCoord(x1, y1, xx, yy, zz, rad);
		GLfloat red, gre, blue, t;
		GetColor(red, gre, blue, t);
		glColor4f(red, gre, blue, t);
		glLineWidth(2.0);
		glBegin(GL_LINES);

		n = g->GetNode(s.start);
		x1 = n->GetLabelL(GraphSearchConstants::kMapX);
		y1 = n->GetLabelL(GraphSearchConstants::kMapY);
		m->GetOpenGLCoord(x1, y1, xx, yy, zz, rad);
		glVertex3d(xx, yy, zz);

		n = g->GetNode(s.goal);
		x1 = n->GetLabelL(GraphSearchConstants::kMapX);
		y1 = n->GetLabelL(GraphSearchConstants::kMapY);
		m->GetOpenGLCoord(x1, y1, xx, yy, zz, rad);
		glVertex3d(xx, yy, zz-20*rad);
		
		glEnd();
		glLineWidth(1.0);
	}
	else {
//		node *n = g->GetNode(s.start);
//		DrawSphere((GLdouble)n->GetLabelF(GraphSearchConstants::kXCoordinate),
//				   (GLdouble)n->GetLabelF(GraphSearchConstants::kYCoordinate),
//				   (GLdouble)n->GetLabelF(GraphSearchConstants::kZCoordinate),
//				   (GLdouble)2.0/(g->GetNumNodes()*g->GetNumNodes()));
	}
}

void BidirectionalGraphEnvironment::OpenGLDraw(const graphStatePair &, const graphMovePair &) const
{
	// if we want to draw a set of moves we use this to do so
}

