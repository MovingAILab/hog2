/*
 *  IRAStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/12/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "IRAStar.h"
#include <vector>

#define HEURISTIC_COLOR
#define MAX_HEURISTIC_VALUE 50.0
#define PATHMAX	// Not necessary, but improves performance for inconsistent heuristics

using namespace IRAStarConstants;

IRAStar::IRAStar( IRAStarConstants::Caching _caching )
:SearchAlgorithm(), caching(_caching)
{
	g = 0;
}

IRAStar::~IRAStar()
{
}

const char *IRAStar::GetName()
{
	return "IRAStar";
}

path *IRAStar::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *)
{
	if (!InitializeSearch(aMap, from, to))
		return 0;
	path *p = 0;
	while (p == 0)
	{
		p = DoOneSearchStep();
		//g->Print(std::cout);
	}
	return p;
}

path *IRAStar::DoOneSearchStep()
{
	if (q.size() == 0)
		return 0;
	node *gNext = q.top().n;
	q.pop();
	//printf("---\nAnalyzing %d next g:%f h:%f\n", gNext->GetNum(), GetGCost(gNext), GetHCost(gNext));
	//std::cout << *gNext << std::endl;

#ifdef PATHMAX
	// PATHMAX: Check for and corrects Inconsistencies 
	if ( caching == IRAStarConstants::OPTIMAL_PATH_CACHING )
		if ( Inconsistent(gNext) )
		{
			// Do not expand, just put back on open list with new h value
			q.Add(GNode(gNext));
			//closedList.remove(gNext);
			return 0;
		}
#endif
	
	if (gNext == gGoal) // we found the goal
	{
		if ((q.size() > 0) &&
				(fequal(GetFCost(q.top().n), GetFCost(gGoal))))
		{
			node *temp = gNext;
			gNext = q.top().n;
			q.pop();
			q.Add(GNode(temp));
			//printf("+++\nAnalyzing %d next g:%f h:%f\n", gNext->GetNum(), GetGCost(gNext), GetHCost(gNext));
		}
		else {
			//SetCachedHCost(gNext, GetGCost(gNext));
			closedList[gNext->GetNum()] = GNode(gNext);
			path *p = ExtractAndRefinePath();
			if (done)
			{
				//printf("%d refined nodes %d expanded nodes with pathlength %u \n", nodesRefined, nodesExpanded, p->length() );
				assert(p);
				return p;
			}
			if (p)
			{
				q.reset();
				closedList.clear();
			}
			return p;
		}
	}

	nodesExpanded++;
	ExpandNeighbors(gNext);
	//SetCachedHCost(gNext, GetGCost(gNext));
	closedList[gNext->GetNum()] = GNode(gNext);

	return 0;
}

bool IRAStar::InitializeSearch(GraphAbstraction *aMap, node *from, node *to)
{
	closedList.clear();
	q.reset();

	currentIteration = 0;
	gStart = 0;
	absGraph = aMap;
	aStart = from;
	aGoal = to;
	delete g;
	g = new Graph();
	nodesExpanded = nodesTouched = nodesRefined = 0;
	
	// find most abstract node in Graph
	node *top = FindTopLevelNode(from, to, aMap);
	if (top == 0)
		return false;
	node *newTop = new node("top");
	gStart = gGoal = newTop;
	g->AddNode(newTop);
	SetInitialValues(newTop, top, 0);
	q.Add(GNode(newTop));
	return true;
}

node *IRAStar::FindTopLevelNode(node *one, node *two, GraphAbstraction *aMap)
{
	if ((one == 0) || (two == 0))
		return 0;
	if (aMap->GetAbstractionLevel(one) >= (int)aMap->getNumAbstractGraphs())
		return 0;
	if (aMap->GetAbstractionLevel(one) == (int)aMap->getNumAbstractGraphs() - 1)
	{
		if (one == two)
			return one;
		return 0;
	}
	node *tmp = FindTopLevelNode(aMap->GetParent(one), aMap->GetParent(two), aMap);
	if ((tmp == 0) && (one == two))
		return one;
	return tmp;
}

/*
 * aRealNode is the node inside the absGraph
 * gNewNode is the node about to be added to Graph
 * gParent is the parent inside the Graph
 */
void IRAStar::SetInitialValues(node *gNewNode, node *aRealNode, node *gParent)
{
	gNewNode->SetLabelL(kAbstractionLevel, absGraph->GetAbstractionLevel(aRealNode));
	gNewNode->SetLabelL(kCorrespondingNode, aRealNode->GetNum());
	if (gParent)
	{
		SetGCost(gNewNode, 0);
		SetHCost(gNewNode, GetHCost(gParent) );
		//std::cout << " " << GetHCost(gNewNode) ;
		//gNewNode->SetLabelF(kCachedHCost1, gParent->GetLabelF(kCachedHCost1));
		//gNewNode->SetLabelF(kCachedHCost2, gParent->GetLabelF(kCachedHCost2));
	}
	else {
		//gNewNode->SetLabelL(kIteration, -1);
		SetGCost(gNewNode, 0);
		SetHCost(gNewNode, 0);
		//gNewNode->SetLabelF(kCachedHCost1, 0);
		//gNewNode->SetLabelF(kCachedHCost2, 0);
	}
//	gNewNode->SetLabelL(kOptimalFlag, 0);
//	gNewNode->SetLabelL(kInOpenList, 1);

	if (absGraph->IsParentOf(aRealNode, aStart) || (aRealNode == aStart))
	{
		//std::cout << "Assigning start to " << *gNewNode << std::endl;
		gStart = gNewNode;
	}
	if (absGraph->IsParentOf(aRealNode, aGoal) || (aRealNode == aGoal))
	{
		//std::cout << "Assigning goal to " << *gNewNode << std::endl;
		gGoal = gNewNode;
	}
}

/*
 * return true if inconsistency caused gNode to change heuristic value
 */
bool IRAStar::Inconsistent(node *gNode)
{
	neighbor_iterator ni ;
	node *gNeighbor ;
	edge *e ;
	bool changedNodeH = false;

	ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		gNeighbor = g->GetNode(next);
		e = g->FindEdge(gNode->GetNum(), gNeighbor->GetNum());
		assert( e != 0 );
		// Correct the heuristic value of neighboring nodes
		if ( fless(GetHCost(gNeighbor), GetHCost(gNode)-e->GetWeight()) )
		{
			SetHCost( gNeighbor, GetHCost(gNode)-e->GetWeight());
			// remove from queue and add in again (with new priority)
			if ( q.IsIn(GNode(gNeighbor)) )
				q.DecreaseKey(GNode(gNeighbor));
		}
		// Correct the heuristic value of node
		if ( fless(GetHCost(gNode), GetHCost(gNeighbor)-e->GetWeight()) )
		{

			//printf("%f < %f - %f \n", GetHCost(gNode), GetHCost(gNeighbor), e->GetWeight());
			SetHCost( gNode, GetHCost(gNeighbor)-e->GetWeight());
			// Put back on the open list instead of closed list (in main loop)
			changedNodeH = true;
		}
	}
	return changedNodeH;
}

/* 
 * return true if node was expanded.
 * return false if an inconsistency was found and we had to update gNode's heuristic value.
 */
void IRAStar::ExpandNeighbors(node *gNode)
{
	neighbor_iterator ni ;
	node *gNeighbor ;
	edge *e;

//	printf("Expanding %5d f-cost is %1.2f, g-cost is %1.2f\n", gNode->GetNum(), GetFCost(gNode), GetGCost(gNode));

	ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		nodesTouched++;
		gNeighbor = g->GetNode(next);
		e = g->FindEdge(gNode->GetNum(), gNeighbor->GetNum());
		
		assert( e != 0 );
		// If already in open list
		if (q.IsIn(GNode(gNeighbor))) // check for lower cost
		{
			if (fless(GetGCost(gNode)+e->GetWeight() /*1.0*/,
								GetGCost(gNeighbor)))
			{
				//printf("Updating neighbor %d to gCost %f\n", gNeighbor->GetNum(), GetGCost(gNode)+1);
				SetGCost(gNeighbor, GetGCost(gNode)+e->GetWeight()/*1.0*/);
				//gNeighbor->SetLabelL(kIteration, -1);
				q.DecreaseKey(GNode(gNeighbor));
			}
		}
		// if already in closed list (do nothing if consistent heuristic)
		else if (closedList.find(gNeighbor->GetNum()) != closedList.end())
		{
			// when using an INCONSISTENT HEURISTIC, we might have closed a node with the wrong g-value.
			// if this is the case, re-open the node
			if ( caching == IRAStarConstants::OPTIMAL_PATH_CACHING )
				if (fless(GetGCost(gNode)+e->GetWeight(), GetGCost(gNeighbor) ))
				{
					//printf("Re-opening neighbor with g=%f to g=%f\n", GetGCost(gNeighbor), GetGCost(gNode)+e->GetWeight());
					SetGCost(gNeighbor, GetGCost(gNode)+e->GetWeight()/*1.0*/);
					q.Add(GNode(gNeighbor));
				}
		}
		else { // add to open list
			//SetHCost(gNeighbor, GetCachedHCost(gNeighbor));
			SetGCost(gNeighbor, GetGCost(gNode)+e->GetWeight()/*1.0*/);
			q.Add(GNode(gNeighbor));
//			printf("Adding neighbor %d with gCost %f, hCost %f\n",
//						 gNeighbor->GetNum(), GetGCost(gNode)+1, GetHCost(gNeighbor));
		}
	}
}

path *IRAStar::ExtractAndRefinePath()
{
	done = true;

	path *p = GetSolution(gGoal);

	if ( caching == IRAStarConstants::P_G_CACHING )
		SetHValues( p->length() - 1 );		// set heuristic values for all nodes on closed list
							// path p contains all nodes (not the edges), so the number of edges is slightly less.

	iterationLimits.push_back(GetGCost(gGoal));
	for (path *i = p; i; i = i->next)
	{
		if (i->n->GetLabelL(kAbstractionLevel) != 0)
		{
			done = false;
		}
		
		if ( caching == IRAStarConstants::OPTIMAL_PATH_CACHING )
			if ( i->n != gGoal )
				SetHCost( i->n, p->length()-1 - GetGCost(i->n) );
	}
	if (done)
		return p;

	std::vector<node*> nodes;
	GetAllSolutionNodes(gGoal, nodes);
	for (unsigned int x = 0; x < nodes.size(); x++)
	{
		//printf("## Refining %d\n", nodes[x]->GetNum());
		RefineNode(nodes[x]);
	}
	//printf("%d refined nodes %d expanded nodes on iteration %d with pathlength %u \n", nodesRefined, nodesExpanded, currentIteration, p->length() );
	closedList.clear();
	q.reset();

	currentIteration++;
//	// never switch search directions
//	//if ( gStart->GetLabelL(kAbstractionLevel)%2 == 0 )  // Always switch diretions on base level
//	{
//		node *tmp = gStart;
//		gStart = gGoal;
//		gGoal = tmp;
//	}
	SetGCost(gStart, 0);
	//SetHCost(gStart, GetCachedHCost(gStart));
	q.Add(GNode(gStart));
		
	if (done)
		return p;
	delete p;
	return 0;
}

void IRAStar::GetAllSolutionNodes(node *goal, std::vector<node*> &nodes)
{
	nodes.push_back(goal);
	closedList.erase(goal->GetNum());
	for (unsigned int x = 0; x < nodes.size(); x++)
	{
		node *gNode = nodes[x];

		neighbor_iterator ni = gNode->getNeighborIter();
		for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
		{
			node *gNeighbor = g->GetNode(next);
			if (closedList.find(gNeighbor->GetNum()) != closedList.end())
			{
//				printf("Neighbor (%d) has g-cost %1.2f, solution path through (%d) has cost %1.2f\n",
//							 gNeighbor->GetNum(), gNeighbor->GetLabelF(kGCost),
//							 gNode->GetNum(), gNode->GetLabelF(kGCost));
				if (!fgreater(GetGCost(gNeighbor)+1.0, GetGCost(gNode)))
				{
//					printf("Adding to list!\n");
					closedList.erase(gNeighbor->GetNum());
					nodes.push_back(gNeighbor);
				}
			}
		}
	}
}

path *IRAStar::GetSolution(node *gNode)
{
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
		edge *e = g->FindEdge(gNode->GetNum(), gNeighbor->GetNum());
		assert( e != 0 );
		if (closedList.find(gNeighbor->GetNum()) != closedList.end())
		{
			node* n = closedList[gNeighbor->GetNum()].n; // silly!
			if (fequal(GetGCost(n) + e->GetWeight(), GetGCost(gNode)))
			{
				return new path(gNode, GetSolution(n));
			}
		}
	}	
	return new path(gNode, 0);
}

void IRAStar::RefineNode(node *gNode)
{
	if (absGraph->GetAbstractionLevel(gNode) == 0)
	{
		if (GetRealNode(gNode) == aStart)
			gStart = gNode;
		if (GetRealNode(gNode) == aGoal)
			gStart = gNode;
		return;
	}
	nodesRefined++;
	std::vector<node *> aChildren;
	std::vector<node *> gChildren;
	node *aNode = GetRealNode(gNode);
	for (int x = 0; x < absGraph->GetNumChildren(aNode); x++)
	{
		aChildren.push_back(absGraph->GetNthChild(aNode, x));
	}

	// first, add children to graph g
	for (unsigned int x = 0; x < aChildren.size(); x++)
	{
		gChildren.push_back(new node("child"));
		g->AddNode(gChildren[x]);
		SetInitialValues(gChildren[x], aChildren[x], gNode);
		//q.Add(GNode(gChildren[x]));
//		std::cout << "Adding child:" << std::endl;
//		std::cout << *gChildren[x] << std::endl;
	}

	// first, connect children to each other
	Graph *aGraph = absGraph->GetAbstractGraph(absGraph->GetAbstractionLevel(aChildren[0]));
	for (unsigned int x = 0; x < gChildren.size(); x++)
	{
		for (unsigned int y = 0; y < gChildren.size(); y++)
		{
			if (x != y)
			{
				edge *e;
				if ((e = aGraph->findDirectedEdge(aChildren[x]->GetNum(), aChildren[y]->GetNum())) != 0)
				{
					g->AddEdge(new edge(gChildren[x]->GetNum(), gChildren[y]->GetNum(), 1.0));
					//(absGraph->GetAbstractionLevel(aChildren[0])==0)?e->GetWeight():1.5));
				}
			}
		}
	}

	// check neighbors of original node
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
		if (gNeighbor->GetLabelL(kAbstractionLevel) == gNode->GetLabelL(kAbstractionLevel)-1)
		{
			// same level
			for (unsigned int x = 0; x < gChildren.size(); x++)
			{
				edge *e;
				if ((e = aGraph->FindEdge(aChildren[x]->GetNum(), GetRealNode(gNeighbor)->GetNum())) != 0)
				{
					g->AddEdge(new edge(gChildren[x]->GetNum(), gNeighbor->GetNum(), 1.0));
															//(absGraph->GetAbstractionLevel(aChildren[0])==0)?e->GetWeight():1.5));
				}
			}
		}
		else if (gNeighbor->GetLabelL(kAbstractionLevel) < gNode->GetLabelL(kAbstractionLevel)-1)
		{ // neighbor is lower
			for (unsigned int x = 0; x < aChildren.size(); x++)
			{
				if (ShouldAddEdge(GetRealNode(gNeighbor), aChildren[x]))
					g->AddEdge(new edge(gChildren[x]->GetNum(), gNeighbor->GetNum(), 1.0));
			}
		}
		else { // neighbor is higher
			for (unsigned int x = 0; x < aChildren.size(); x++)
			{
				if (ShouldAddEdge(aChildren[x], GetRealNode(gNeighbor)))
					g->AddEdge(new edge(gChildren[x]->GetNum(), gNeighbor->GetNum(), 1.0));
			}
		}
	}
	
	// last thing we do!
	g->RemoveNode(gNode);
}

node *IRAStar::GetRealNode(node *gNode) const
{
	return absGraph->GetAbstractGraph(gNode->GetLabelL(kAbstractionLevel))->GetNode(gNode->GetLabelL(kCorrespondingNode));
}

bool IRAStar::ShouldAddEdge(node *aLowerNode, node *aHigherNode)
{
	neighbor_iterator ni = aLowerNode->getNeighborIter();
	for (int next = aLowerNode->nodeNeighborNext(ni); next != -1; next = aLowerNode->nodeNeighborNext(ni))
	{
		node *aNeighbor = absGraph->GetAbstractGraph(absGraph->GetAbstractionLevel(aLowerNode))->GetNode(next);
		if (absGraph->IsParentOf(aHigherNode, aNeighbor))
			return true;
	}
	return false;
}

void IRAStar::OpenGLDraw() const
{
	if ((g == 0) || (g->GetNumNodes() == 0))
	{
		return;
	}
	
	glLineWidth(3.0);
	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		node *n;
		n = g->GetNode(e->getFrom());
#ifdef HEURISTIC_COLOR
		if (q.peek().n == n)
			glColor3f(	GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4, 
					0.0, 
					GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4);
		else if (n == gStart)
			glColor3f(	0, 
					0, 
					GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4);
		else if (n == gGoal)
			glColor3f(	0, 
					GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4,
					0);
		else if (closedList.find(n->GetNum()) != closedList.end()) // on closed list
			glColor3f(	GetHCost(n), 
					GetHCost(n), 
					GetHCost(n));
		else if (q.IsIn(GNode(n)))
			glColor3f(	GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE);
		else
			glColor3f(	GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE);
		
		recVec rv = absGraph->GetNodeLoc(GetRealNode(n));
		glVertex3f(rv.x, rv.y, rv.z);
		
		n = g->GetNode(e->getTo());
		if (q.peek().n == n)
			glColor3f(	GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4, 
					0.0, 
					GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4);
		else if (n == gStart)
			glColor3f(	0, 
					0, 
					GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4);
		else if (n == gGoal)
			glColor3f(	0, 
					GetHCost(n)*0.6/MAX_HEURISTIC_VALUE+0.4,
					0);
		else if (closedList.find(n->GetNum()) != closedList.end()) // on closed list
			glColor3f(	GetHCost(n), 
					GetHCost(n), 
					GetHCost(n));
		else if (q.IsIn(GNode(n)))
			glColor3f(	GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE);
		else
			glColor3f(	GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE, 
					GetHCost(n)*1.0/MAX_HEURISTIC_VALUE);
#else
		if (q.top().n == n)
			glColor3f(1.0, 0.0, 1.0);
		else if (n == gStart)
			glColor3f(0, 0, 1);
		else if (n == gGoal)
			glColor3f(0, 1, 0);
		else if (closedList.find(n->GetNum()) != closedList.end()) // on closed list
			glColor3f(1.0, 1.0, 0);
		else if (q.IsIn(GNode(n)))
			glColor3f(0.5, 0.5, 0.5);
		else
			glColor3f(1, 1, 1);
		
		recVec rv = absGraph->GetNodeLoc(GetRealNode(n));
		glVertex3f(rv.x, rv.y, rv.z);
		
		n = g->GetNode(e->getTo());
		if (q.top().n == n)
			glColor3f(1.0, 0.0, 1.0);
		else if (n == gStart)
			glColor3f(0, 0, 1);
		else if (n == gGoal)
			glColor3f(0, 1, 0);
		else if (closedList.find(n->GetNum()) != closedList.end()) // on closed list
			glColor3f(1.0, 1.0, 0);
		else if (q.IsIn(GNode(n)))
			glColor3f(0.5, 0.5, 0.5);
		else
			glColor3f(1, 1, 1);
#endif

		rv = absGraph->GetNodeLoc(GetRealNode(n));
		
		glVertex3f(rv.x, rv.y, rv.z);
	}

	glEnd();
	glLineWidth(1.0);
	
}

// Set heuristic values of all nodes on closed list according to P-g caching (h = f-g)
void IRAStar::SetHValues( int f )
{
	NodeLookupTable::iterator ni ;
	//typedef std::unordered_map<uint32_t, GNode> NodeLookupTable;
	
	for ( ni = closedList.begin(); ni != closedList.end(); ni++ )
	{
		node * n = ni->second.n;
		//node* n = closedList[gNeighbor->GetNum()].n; // silly!

		// h = f - g
		//n->SetLabelF(kHCost, val);
		//std::cout << " f " << f
		//	<< " g " << GetGCost(n) 
		//	<< " h " << f - GetGCost(n) << std::endl;
		//SetCachedHCost( n, std::max( 0.0, (double) f - GetGCost(n) ) );
		SetHCost( n, (double)f - GetGCost(n) );
	}
}

double IRAStar::GetHCost(node *n) const
{
//	if (n->GetLabelL(kIteration) < currentIteration-1)
//		return std::max(iterationLimits.back(), val);
	//return max( n->GetLabelF(kHCost), absGraph->h(n,aGoal) );
	//return absGraph->h(n,aGoal);
	return n->GetLabelF(kHCost);
	//return std::max( n->GetLabelF(kHCost), GetCachedHCost(n) );
}

void IRAStar::SetHCost(node *n, double val)
{
	n->SetLabelF(kHCost, val);
}

//double IRAStar::GetCachedHCost(node *n)
//{
//	double val = 0;
//	val = n->GetLabelF(kCachedHCost1);	// one search direction
///*
//	if ( gStart->GetLabelL(kAbstractionLevel)%2 == 0 )  // Always switch diretions on base level
//		val = n->GetLabelF(kCachedHCost2);
//	else
//		val = n->GetLabelF(kCachedHCost1);
//		*/
//}
//
//void IRAStar::SetCachedHCost(node *n, double val)
//{
//	n->SetLabelF(kCachedHCost1, val);	// one search direction
//
///*	if ( gStart->GetLabelL(kAbstractionLevel)%2 == 0 )  // Always switch diretions on base level
//		n->SetLabelF(kCachedHCost1, val);
//	else
//		n->SetLabelF(kCachedHCost2, val);
//		*/
//}

double IRAStar::GetGCost(node *n) const
{
	return n->GetLabelF(kGCost);
}

void IRAStar::SetGCost(node *n, double val)
{
	n->SetLabelF(kGCost, val);
}

double IRAStar::GetFCost(node *n) const
{
	return n->GetLabelF(kGCost)+n->GetLabelF(kHCost);
}
