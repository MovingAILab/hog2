/*
 *  IRDijkstra.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "IRDijkstra.h"
#include <vector>

using namespace IRDijkstraConstants;

IRDijkstra::IRDijkstra()
:SearchAlgorithm()
{
	g = 0;
}

IRDijkstra::~IRDijkstra()
{
}

const char *IRDijkstra::GetName()
{
	return "IRDijkstra";
}

path *IRDijkstra::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *)
{
	if (!InitializeSearch(aMap, from, to))
		return 0;
	path *p = 0;
	while (p == 0)
	{
		p = DoOneSearchStep();
		//std::cout << p << endl;
	}
	return p;
}

path *IRDijkstra::DoOneSearchStep()
{
	if (q.size() == 0)
		return 0;
	node *gNext = q.top().n;
	q.pop();
	//printf("Analyzing %d next\n", gNext->GetNum());
	//std::cout << *gNext << std::endl;

	
	if (gNext == gGoal) // we found the goal
	{
		if ((q.size() > 0) &&
				(fequal(q.top().n->GetLabelF(kGCost), gGoal->GetLabelF(kGCost))))
		{
			node *temp = gNext;
			gNext = q.top().n;
			q.pop();
			q.Add(GNode(temp));
		}
		else {
			closedList[gNext->GetNum()] = GNode(gNext);
			path *p = ExtractAndRefinePath();
			if (done)
			{
				//std::cout<<"finished" << std::endl;
				//printf("%d refined nodes %d expanded nodes with pathlength %u \n", nodesRefined, nodesExpanded, p->length() );
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
	
	closedList[gNext->GetNum()] = GNode(gNext);
	return 0;
}

bool IRDijkstra::InitializeSearch(GraphAbstraction *aMap, node *from, node *to)
{
	closedList.clear();
	q.reset();

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

node *IRDijkstra::FindTopLevelNode(node *one, node *two, GraphAbstraction *aMap)
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
void IRDijkstra::SetInitialValues(node *gNewNode, node *aRealNode, node *gParent)
{
	gNewNode->SetLabelL(kAbstractionLevel, absGraph->GetAbstractionLevel(aRealNode));
	gNewNode->SetLabelL(kCorrespondingNode, aRealNode->GetNum());
	if (gParent)
	{
		gNewNode->SetLabelF(kGCost, gParent->GetLabelF(kGCost));
		//gNewNode->SetLabelF(kHCost, gParent->GetLabelF(kHCost));
	}
	else {
		gNewNode->SetLabelF(kGCost, 0.0);
		//gNewNode->SetLabelF(kHCost, 0.0);
	}
//	gNewNode->SetLabelL(kOptimalFlag, 0);
//	gNewNode->SetLabelL(kInOpenList, 1);

	if ((gParent == gStart) && (absGraph->IsParentOf(aRealNode, aStart)))
	{
		//std::cout << "Assigning start to " << *gNewNode << std::endl;
		gStart = gNewNode;
	}
	if ((gParent == gGoal) && (absGraph->IsParentOf(aRealNode, aGoal)))
	{
		//std::cout << "Assigning goal to " << *gNewNode << std::endl;
		gGoal = gNewNode;
	}
}

void IRDijkstra::ExpandNeighbors(node *gNode)
{
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		nodesTouched++;
		node *gNeighbor = g->GetNode(next);
		if (q.IsIn(GNode(gNeighbor))) // check for lower cost
		{
			if (fless(gNode->GetLabelF(kGCost)+1.0,
								gNeighbor->GetLabelF(kGCost)))
			{
				gNeighbor->SetLabelF(kGCost, gNode->GetLabelF(kGCost)+1.0);
				q.DecreaseKey(GNode(gNeighbor));
			}
		}
		else if (closedList.find(gNeighbor->GetNum()) != closedList.end())
		{
		}
		else { // add to open list
			gNeighbor->SetLabelF(kGCost, gNode->GetLabelF(kGCost)+1.0);
			q.Add(GNode(gNeighbor));
		}
	}
}

path *IRDijkstra::ExtractAndRefinePath()
{
	done = true;

	path *p = GetSolution(gGoal);
	for (path *i = p; i; i = i->next)
	{
		if (i->n->GetLabelL(kAbstractionLevel) != 0)
		{
			done = false;
		}
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
	//printf("%d refined nodes %d expanded nodes with pathlength %u \n", nodesRefined, nodesExpanded, p->length() );
	closedList.clear();
	q.reset();
	q.Add(GNode(gStart));
		
	if (done)
		return p;
	delete p;
	return 0;
}

void IRDijkstra::GetAllSolutionNodes(node *goal, std::vector<node*> &nodes)
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
				if (!fgreater(gNeighbor->GetLabelF(kGCost)+1, gNode->GetLabelF(kGCost)))
				{
//					printf("Adding to list!\n");
					closedList.erase(gNeighbor->GetNum());
					nodes.push_back(gNeighbor);
				}
			}
		}
	}
}

path *IRDijkstra::GetSolution(node *gNode)
{
	neighbor_iterator ni = gNode->getNeighborIter();
	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
	{
		node *gNeighbor = g->GetNode(next);
		if (closedList.find(gNeighbor->GetNum()) != closedList.end())
		{
			node* n = closedList[gNeighbor->GetNum()].n; // silly!
			if (fequal(n->GetLabelF(kGCost) + 1.0, gNode->GetLabelF(kGCost)))
			{
				return new path(gNode, GetSolution(n));
			}
		}
	}	
	return new path(gNode, 0);
}

//void IRDijkstra::UpdateNode(node *gNode)
//{
//	UpdateH(gNode);
//	std::cout << "After UpdateH " << *gNode << std::endl;
//	UpdateG(gNode);
//	std::cout << "After UpdateG " << *gNode << std::endl;
//	UpdateOptH(gNode);
//	std::cout << "After UpdateOptH " << *gNode << std::endl;
//	gNode->SetLabelL(kInOpenList, 0);
//	q.Add(GNode(gNode));
//}

//void IRDijkstra::UpdateH(node *gNode)
//{
//	double minH = DBL_MAX;
//
//	// update h
//	neighbor_iterator ni = gNode->getNeighborIter();
//	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
//	{
//		node *gNeighbor = g->GetNode(next);
//		double tmpH = gNeighbor->GetLabelF(kHCost) + g->FindEdge(next, gNode->GetNum())->GetWeight();
//		if (fless(tmpH, minH))
//			minH = tmpH;
//	}
//	if (gNode->GetLabelL(kAbstractionLevel) == 0)
//		minH = std::max(minH, absGraph->h(gNode, aGoal));
//	if (fgreater(minH, gNode->GetLabelF(kHCost)) &&
//			(gNode != gGoal))
//	{
//		gNode->SetLabelF(kHCost, minH);
//		MakeNeighborsOpen(gNode);
//	}
//}

//void IRDijkstra::UpdateG(node *gNode)
//{
//	double minG = DBL_MAX;
//
//	// update g
//	neighbor_iterator ni = gNode->getNeighborIter();
//	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
//	{
//		node *gNeighbor = g->GetNode(next);
//		double tmpG = gNeighbor->GetLabelF(kGCost) + g->FindEdge(next, gNode->GetNum())->GetWeight();
//		if (fless(tmpG, minG))
//			minG = tmpG;
//	}
//	if (gNode->GetLabelL(kAbstractionLevel) == 0)
//		minG = std::max(minG, absGraph->h(gNode, aStart));
//	if (fgreater(minG, gNode->GetLabelF(kGCost)) &&
//			(gNode != gStart))
//	{
//		gNode->SetLabelF(kGCost, minG);
//		MakeNeighborsOpen(gNode);
//	}
//}

//void IRDijkstra::UpdateOptH(node *gNode)
//{
//	bool optH = false;
//	if ((gNode->GetLabelL(kAbstractionLevel) == 0) &&
//			(gNode->GetLabelL(kOptimalFlag) == 0))
//	{
//		if (gNode == gGoal)
//		{
//			optH = true;
//		}
//		else {
//			neighbor_iterator ni = gNode->getNeighborIter();
//			for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
//			{
//				node *gNeighbor = g->GetNode(next);
//				if ((gNeighbor->GetLabelL(kOptimalFlag) == 1) &&
//						(fequal(gNeighbor->GetLabelF(kHCost) + g->FindEdge(next, gNode->GetNum())->GetWeight(),
//										gNode->GetLabelF(kHCost))))
//				{
//					optH = true;
//				}
//			}
//		}
//		
//		if ((gNode->GetLabelL(kOptimalFlag) == 0) && (optH))
//		{
//			gNode->SetLabelL(kOptimalFlag, 1);
//			MakeNeighborsOpen(gNode);
//		}
//	}
//}

//void IRDijkstra::MakeNeighborsOpen(node *gNode)
//{
//	neighbor_iterator ni = gNode->getNeighborIter();
//	for (int next = gNode->nodeNeighborNext(ni); next != -1; next = gNode->nodeNeighborNext(ni))
//	{
//		node *gNeighbor = g->GetNode(next);
//		if (gNeighbor->GetLabelL(kInOpenList) == 0)
//		{
//			gNeighbor->SetLabelL(kInOpenList, 1);
//			q.DecreaseKey(GNode(gNeighbor));
//		}
//	}
//}

void IRDijkstra::RefineNode(node *gNode)
{
	if (absGraph->GetAbstractionLevel(gNode) == 0)
		return;
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
		q.Add(GNode(gChildren[x]));
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

node *IRDijkstra::GetRealNode(node *gNode) const
{
	return absGraph->GetAbstractGraph(gNode->GetLabelL(kAbstractionLevel))->GetNode(gNode->GetLabelL(kCorrespondingNode));
}

bool IRDijkstra::ShouldAddEdge(node *aLowerNode, node *aHigherNode)
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

void IRDijkstra::OpenGLDraw() const
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
		if (q.peek().n == n)
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
		if (q.peek().n == n)
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

		rv = absGraph->GetNodeLoc(GetRealNode(n));
		
		glVertex3f(rv.x, rv.y, rv.z);
	}

	glEnd();
	glLineWidth(1.0);
	
}

