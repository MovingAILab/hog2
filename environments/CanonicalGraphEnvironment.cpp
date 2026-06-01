//
//  CanonicalGraphEnvironment.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/11/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include "CanonicalGraphEnvironment.h"


bool operator==(const canGraphState &s1, const canGraphState &s2)
{
	//return (s1.parent == s2.parent) && (s1.s == s2.s);
	return (s1.s == s2.s);
}

std::ostream &operator<<(std::ostream &out, const canGraphState &s1)
{
	out << s1.s << "(p: " << s1.parent << ")";
	
	return out;
}


CanonicalGraphEnvironment::CanonicalGraphEnvironment(Graph *g)
:g(g)
{
	directed = false;
	drawEdgeCosts = false;
	drawNodeLabels = false;
	nodeScale = 1.0;
	ComputeOrdering();
}

CanonicalGraphEnvironment::~CanonicalGraphEnvironment()
{
	
}

void CanonicalGraphEnvironment::GetSuccessors(const canGraphState &stateID, std::vector<canGraphState> &neighbors) const
{
	neighbors.clear();
	//std::cout << "Successors of " << stateID << " are ";
	
	if (stateID.parent == stateID.s) // initial state
	{
		node *n = g->GetNode(stateID.parent);
		for (int e = 0; e < n->GetNumEdges(); e++)
		{
			//n->getEdge(e)->setMarked(true);
			if (n->getEdge(e)->getFrom() == stateID.parent)
			{
				neighbors.push_back({stateID.parent, n->getEdge(e)->getTo()});
				//std::cout << neighbors.back() << " ";
			}
			else {
				neighbors.push_back({stateID.parent, n->getEdge(e)->getFrom()});
				//std::cout << neighbors.back() << " ";
			}
		}
	}
	else {
		auto i = canonicalOrdering.find(stateID);
		if (i != canonicalOrdering.end())
		{
			for (int x = 0; x < 8; x++)
			{
				if (i->second&(1<<x))
				{
					// add {i->first.second, x} to edges
					node *n = g->GetNode(i->first.s);
					edge *e = n->getEdge(x);
					
					if (e->getFrom() == n->GetNum())
					{
						neighbors.push_back({i->first.s, e->getTo()});
						//std::cout << neighbors.back() << " ";
					}
					else {
						neighbors.push_back({i->first.s, e->getFrom()});
						//std::cout << neighbors.back() << " ";
					}
				}
			}
		}
		else {
			//printf(" NOT FOUND ");
		}
	}
	//std::cout << "\n";
}

//int CanonicalGraphEnvironment::GetNumSuccessors(const canGraphState &stateID) const
//{
//
//}

void CanonicalGraphEnvironment::GetActions(const canGraphState &stateID, std::vector<graphMove> &actions) const
{
	assert(false);
}

graphMove CanonicalGraphEnvironment::GetAction(const canGraphState &s1, const canGraphState &s2) const
{
	assert(false);
	return {0,0};
}

void CanonicalGraphEnvironment::ApplyAction(canGraphState &s, graphMove a) const
{
	// cannot apply canonical action
	assert(false);
}

bool CanonicalGraphEnvironment::InvertAction(graphMove &a) const
{
	uint32_t tmp = a.from;
	a.from = a.to;
	a.to = tmp;
	if (g->findDirectedEdge(a.from, a.to))
		return true;
	return false;
}


double CanonicalGraphEnvironment::HCost(const canGraphState &state1, const canGraphState &state2) const
{
	return 0;
}

double CanonicalGraphEnvironment::GCost(const canGraphState &state1, const canGraphState &state2) const
{
	edge *e = g->FindEdge(state1.s, state2.s);
	assert(e);
	return e->GetWeight();
}

double CanonicalGraphEnvironment::GCost(const canGraphState &state1, const graphMove &state2) const
{
	edge *e = g->FindEdge(state2.from, state2.to);
	assert(e);
	return e->GetWeight();
}

bool CanonicalGraphEnvironment::GoalTest(const canGraphState &state, const canGraphState &goal) const
{
	return state.s == goal.s;
}

uint64_t CanonicalGraphEnvironment::GetMaxHash() const
{
	return g->GetNumNodes();
}

uint64_t CanonicalGraphEnvironment::GetStateHash(const canGraphState &state) const
{
	return g->GetNode(state.s)->GetNum();
}

uint64_t CanonicalGraphEnvironment::GetActionHash(graphMove act) const
{
	return (g->GetNode(act.from)->getUniqueID()<<16)|
	(g->GetNode(act.to)->getUniqueID());
}

//void CanonicalGraphEnvironment::OpenGLDraw() const
//{
//	// Hittable background behind graph
//	glBegin(GL_TRIANGLE_FAN);
//	glColor4f(0, 0, 0, 0.01);
//	glVertex3f(-1, -1, 0.01);
//	glVertex3f(1, -1, 0.01);
//	glVertex3f(1, 1, 0.01);
//	glVertex3f(-1, 1, 0.01);
//	glEnd();
//	glBegin(GL_LINES);
//	glNormal3f(0, 1, 0);
//	
//	GLdouble off = 0;
//	edge_iterator ei = g->getEdgeIter();
//	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
//	{
//		//int x, y;
//		//double offsetx, offsety;
//		node *n;
//		n = g->GetNode(e->getFrom());
//		
//		{
//			GLfloat r, g, b, t;
//			GetColor(r, g, b, t);
//			glColor4f(r, g, b, t);
//		}
//		if (e->getMarked())
//		{
//			glColor3f(1.0, 0.0, 0.0);
//			off = -0.01;
//		}
//		else {
//			off = 0;
//		}
//		
//		GLdouble x, y, z;
//		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//		glVertex3f(x, y, z+off);
//		
//		n = g->GetNode(e->getTo());
//		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//		
//		glVertex3f(x, y, z+off);
//	}
//	glEnd();
//	
//	char label[5];
//	if (drawEdgeCosts)
//	{
//		glLineWidth(3.0);
//		glColor4f(0.0, 0.0, 0.0, 1.0);
//		edge_iterator ei = g->getEdgeIter();
//		for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
//		{
//			sprintf(label, "%1.2f", e->GetWeight());
//			node *n;
//			n = g->GetNode(e->getFrom());
//			
//			GLdouble x, y, z;
//			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//			
//			n = g->GetNode(e->getTo());
//			x += n->GetLabelF(GraphSearchConstants::kXCoordinate);
//			y += n->GetLabelF(GraphSearchConstants::kYCoordinate);
//			z += n->GetLabelF(GraphSearchConstants::kZCoordinate);
//			
//			DrawText(x/2, y/2, z/2-0.003, 0.2, label);
//		}
//		glLineWidth(1.0);
//	}
//	if (drawNodeLabels)
//	{
//		glLineWidth(3.0);
//		glColor4f(0.0, 0.0, 0.0, 1.0);
//		for (int x = 0; x < g->GetNumNodes(); x++)
//		{
//			node *n = g->GetNode(x);
//			DrawTextCentered(n->GetLabelF(GraphSearchConstants::kXCoordinate),
//							 n->GetLabelF(GraphSearchConstants::kYCoordinate),
//							 n->GetLabelF(GraphSearchConstants::kZCoordinate)-0.003,
//							 0.2, n->GetName());
//		}
//		glLineWidth(1.0);
//	}
//}
//
//void CanonicalGraphEnvironment::OpenGLDraw(const canGraphState &cs) const
//{
//	graphState s = cs.s;
//	
//	GLfloat r, gr, b, t;
//	GetColor(r, gr, b, t);
//	glColor4f(r, gr, b, t);
//	
//	
//	node *n = g->GetNode(s);
//	GLdouble x, y, z, rad;
//	x = (GLdouble)n->GetLabelF(GraphSearchConstants::kXCoordinate);
//	y = (GLdouble)n->GetLabelF(GraphSearchConstants::kYCoordinate);
//	z = (GLdouble)n->GetLabelF(GraphSearchConstants::kZCoordinate);
//	//rad = 20*(GLdouble)0.4/(g->GetNumNodes());
//	rad = nodeScale*(GLdouble)0.4/(g->GetNumNodes());
//	DrawSquare(x, y, z-0.002, rad);
//	glLineWidth(2.0);
//	//			if (r+gr+b < 1.5)
//	//				glColor4f(1, 1, 1, t);
//	//			else
//	glColor4f(0, 0, 0, t);
//	OutlineRect(x-rad, y-rad, x+rad, y+rad, z-0.0021);
//	glLineWidth(1.0);
//}
//
//void CanonicalGraphEnvironment::OpenGLDraw(const canGraphState &s, const graphMove &gm) const
//{
//	
//}
//
//void CanonicalGraphEnvironment::GLDrawLine(const canGraphState &f, const canGraphState &t) const
//{
//	graphState from = f.s;
//	graphState to = f.s;
//	{
//		GLfloat r, g, b, t;
//		GetColor(r, g, b, t);
//		glColor4f(r, g, b, t);
//	}
//	
//	glBegin(GL_LINES);
//	
//	GLdouble x, y, z;
//	
//	node *n = g->GetNode(from);
//	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//	glVertex3f(x, y, z);
//	
//	n = g->GetNode(to);
//	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
//	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
//	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
//	glVertex3f(x, y, z);
//	
//	glEnd();
//}

void CanonicalGraphEnvironment::ComputeOrdering()
{
	GraphEnvironment ge(g);
	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	std::vector<graphState> thePath;
	canonicalOrdering.clear();
	double maxEdge = 0;
	edge_iterator ei = g->getEdgeIter();
	while (1)
	{
		edge *e = (edge *)g->edgeIterNext(ei);
		if (!e)
			break;
		if (e->GetWeight() > maxEdge)
			maxEdge = e->GetWeight();
	}
	
	for (graphState searchStart = 0; searchStart < g->GetNumNodes(); searchStart++)
	{
//		printf("Analyzing from node %s\n", g->GetNode(searchStart)->GetName());
		double currentBound;
		astar.SetStopAfterGoal(false);
		astar.InitializeSearch(&ge, searchStart, searchStart, thePath);
		while (astar.GetNumOpenItems() > 0)
		{
			graphState next = astar.CheckNextNode();
			astar.GetOpenListGCost(next, currentBound);
			if (fgreater(currentBound, 2*maxEdge))
			{
				break;
			}
			else {
				astar.DoSingleSearchStep(thePath);
			}
		}
		node *searchStartNode = g->GetNode(searchStart);
		for (int startEdge = 0; startEdge < searchStartNode->GetNumEdges(); startEdge++)
		{
			// for all neighbors, get the outgoing moves that are on optimal paths.
			node *canonicalNode;
			edge *e = searchStartNode->getEdge(startEdge);
			if (e->getFrom() == searchStart)
				canonicalNode = g->GetNode(e->getTo());
			else
				canonicalNode = g->GetNode(e->getFrom());
			if (astar.GetParent(canonicalNode->GetNum()) != searchStart)
				continue;
			
			
			AStarOpenClosedDataWithF<graphState> parent;
			astar.GetClosedItem(canonicalNode->GetNum(), parent);
			
//			printf("  Checking neighbor %s cost %f\n", canonicalNode->GetName(), parent.g);
			for (int z = 0; z < canonicalNode->GetNumEdges(); z++)
			{
				node *finalNode;
				edge *e = canonicalNode->getEdge(z);
				if (e->getFrom() == canonicalNode->GetNum())
					finalNode = g->GetNode(e->getTo());
				else
					finalNode = g->GetNode(e->getFrom());
				AStarOpenClosedDataWithF<graphState> child;
				astar.GetClosedItem(finalNode->GetNum(), child);
				graphState destNum = finalNode->GetNum();
//				printf("  --Parent of %s is %s ", finalNode->GetName(), g->GetNode(astar.GetParent(destNum))->GetName());
				if (astar.GetParent(destNum) == canonicalNode->GetNum())
				{
//					printf("(adding)\n");
					canGraphState p = {searchStart, canonicalNode->GetNum()};
					canonicalOrdering[p] |= 1<<z;
				}
				else {
//					printf("(not adding)\n");
				}
			}
		}
	}
	printf("%lu edges stored (of %d max)\n", canonicalOrdering.size(), g->GetNumEdges()*2);
	if (0)
	{
		for (auto &p : canonicalOrdering)
		{
			printf("Going from %s to %s, successors: ", g->GetNode(p.first.parent)->GetName(), g->GetNode(p.first.s)->GetName());
			//std::cout << p.first;
			for (int x = 0; x < 8; x++)
			{
				if (p.second&(1<<x))
				{
					edge *e = g->GetNode(p.first.s)->getEdge(x);
					node *n;
					if (e->getFrom() == p.first.s)
						n = g->GetNode(e->getTo());
					else
						n = g->GetNode(e->getFrom());
					printf("%s ", n->GetName());
				}
			}
			printf("\n");
		}
	}
}
