/*
 *  $Id: aStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/22/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "AStar.h"
#include "float.h"
#include "FPUtil.h"

using namespace GraphAbstractionConstants;
using namespace AStar3Util;
static const bool verbose = false;
//const int gMaxAbstraction = 0;

const char *aStar::GetName()
{
	static char name[32];
	sprintf(name, "aStar[]");
	return name;
}


//aStar::aStar(GraphAbstraction *_abstr, node *_start, node *_goal,
//															 path *corridor, int corridorWidth, int _absLevel)
path *aStar::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp)
{
	//assert(openList.size() == 0);
	assert(openQueue.size() == 0);
	assert(closedList.size() == 0);
	nodesTouched = nodesExpanded = 0;
	abstr = aMap;
	start = from;
	goal = to;
	if ((from == 0) || (to == 0) || (from == to) || (!aMap->Pathable(from, to)))
		return 0;
	g = abstr->GetAbstractGraph(start);
	
	SearchNode first(internalHeuristic(goal, start), 0, 0, start, start);
	openQueue.Add(first);
	//openList[start] = first;
		
	return getPathToNode(goal, (from->GetLabelL(kAbstractionLevel) == 0)?rp:0);
	//printf("##Initial nodes expanded: %d, touched: %d\n", nodesExpanded, nodesTouched);
}

void aStar::setCorridor(path *corridor, int width)
{
	eligibleNodes.resize(0);
	buildCorridor(corridor, width);
}

path *aStar::getPathToNode(node *target, reservationProvider *rp)
{
	node *currentOpenNode = 0;
	while ((openQueue.size() > 0) && (currentOpenNode != target))
	{
		// get top of queue
		currentOpenNode = getNextNode();
		if (currentOpenNode == target)
			break;
		//assert(openList.size() == openQueue.size());
		if (currentOpenNode == 0)
			printf("Oh no! The current open node is NULL\n");
		edge_iterator ei = currentOpenNode->getEdgeIter();
		
		// iterate over all the children
		for (edge *e = currentOpenNode->edgeIterNext(ei); e; e = currentOpenNode->edgeIterNext(ei))
		{
			nodesTouched++;
			unsigned int which;
			if ((which = e->getFrom()) == currentOpenNode->GetNum()) which = e->getTo();
			node *neighbor = g->GetNode(which);
			assert(neighbor != 0);
			
			if (closedList.find(neighbor) != closedList.end())
			{
				if (verbose) { printf("skipping node %d\n", neighbor->GetNum()); }
				continue;
			}
			else if (rp && (neighbor != target) && rp->nodeOccupied(neighbor))
			{
				if (verbose) { printf("skipping occupied node %d\n", neighbor->GetNum()); }
				continue;
			}
			else if (!nodeInCorridor(neighbor))
			{
				if (verbose) { printf("node %d not in corridor\n", neighbor->GetNum()); }
			}
			else if (openQueue.IsIn(SearchNode(neighbor)))
			//else if (openList.find(neighbor) != openList.end())
			{
				if (verbose) { printf("updating node %d\n", neighbor->GetNum()); }
				updateWeight(currentOpenNode, neighbor, e);
				//assert(openList.size() == openQueue.size());
			}
			else {
				if (verbose) { printf("addinging node %d\n", neighbor->GetNum()); }
				addToOpenList(currentOpenNode, neighbor, e);
				//assert(openList.size() == openQueue.size());
			}
		}
	}
	if (currentOpenNode == target)
	{
		//		if (abstr->GetAbstractionLevel(whence) == 0)
		path *p = extractPathToStart(g, currentOpenNode);
		closedList.clear();
		//openList.clear();
		openQueue.reset();
		return p;
	}
	closedList.clear();
	//openList.clear();
	openQueue.reset();
	return 0; // no path found!
}

node *aStar::getNextNode()
{
	nodesExpanded++;
	node *next;
	SearchNode it = openQueue.Remove();
	next = it.currNode;
//	if (openList.find(next) == openList.end())
//	{
//		printf("Error; next node in openQueue not in openList!\n");
//	}
	//openList.erase(openList.find(next));
	closedList[next] = it;
//	if (openList.size() != openQueue.size())
//	{
//		printf("openList size %u, openQueue size %u\n", (unsigned int)openList.size(),
//					 (unsigned int)openQueue.size());
//		printf("Opening node %d (f = %1.2f, g = %1.2f. [h=%1.2f])\n",
//					 next->GetNum(), it.fCost, it.gCost, abstr->h(next, goal));
//		assert(openList.size() == openQueue.size());
//	}
	return next;
}

void aStar::updateWeight(node *currOpenNode, node *neighbor, edge *e)
{
	assert(openQueue.IsIn(SearchNode(neighbor)));
	SearchNode prev = openQueue.find(SearchNode(neighbor));
	//SearchNode prev = openList[neighbor];
	SearchNode alt = closedList[currOpenNode];
	double altCost = alt.gCost+e->GetWeight()+(prev.fCost-prev.gCost);
	if (fgreater(prev.fCost, altCost))
	{
		//prev.steps = alt.steps+1;
		prev.fCost = altCost;
		prev.gCost = alt.gCost+e->GetWeight();
		prev.prevNode = currOpenNode;
		prev.e = e;
		// reset neighbor in queue
		//openQueue.erase(openQueue.find(neighbor));
		//openQueue.push(prev);
		openQueue.DecreaseKey(prev);
//		openList[neighbor] = prev;
	}
}

void aStar::addToOpenList(node *currOpenNode, node *neighbor, edge *e)
{
	SearchNode n(closedList[currOpenNode].gCost+e->GetWeight()+internalHeuristic(neighbor, goal),
							 closedList[currOpenNode].gCost+e->GetWeight(),
							 e,
							 neighbor, currOpenNode/*, closedList[currOpenNode].steps+1*/);
	if (verbose) 
	{ printf("Adding %u to openQueue, old size %u, ", neighbor->GetNum(), openQueue.size()); }
	openQueue.Add(n);

	if (verbose)
	{ 
		printf("New size %u\n", openQueue.size());
//		printf("Adding %u to openList, old size %u, ", (unsigned int)neighbor->GetNum(),
//					 (unsigned int)openList.size());
	}
//	openList[neighbor] = n;
	if (verbose) 
	{
//		printf("New size %u\n", (unsigned int)openList.size());
//		if (openList.find(neighbor) == openList.end())
//			printf("!!Hey; can't lookup the item I just inserted!\n");
//		assert(openList.find(neighbor) != openList.end());
	}
}

path *aStar::extractPathToStart(Graph *_g, node *goalNode)
{
	path *p = 0;
	SearchNode n = closedList[goalNode];
	do {
		if (n.currNode && n.prevNode)
			_g->FindEdge(n.currNode->GetNum(), n.prevNode->GetNum())->setMarked(true);
		p = new path(n.currNode, p);
		n = closedList[n.prevNode];
	} while (n.currNode != n.prevNode);
	p = new path(n.currNode, p);
	return p;
}

void aStar::printStats()
{
	printf("%u items in closed list\n", (unsigned int)closedList.size());
//	printf("%u items in open list\n", (unsigned int)openList.size());
	printf("%u items in open queue\n", (unsigned int)openQueue.size());
}

int aStar::getMemoryUsage()
{
	return closedList.size()+openQueue.size();
}

void aStar::buildCorridor(path *p, int windowSize)
{
	//	Corridor eligibleNodes;
	for (path *t = p; t; t = t->next)
		addNeighborsToCorridor(abstr->GetAbstractGraph(t->n), t->n, windowSize);
}

void aStar::addNeighborsToCorridor(Graph *_g, node *n, int windowSize)
{
	// if node is already in corridor, stop
	//	if (eligibleNodes.find(n) != eligibleNodes.end())
	//		return;
	eligibleNodes[n] = true;
	edge_iterator ei = n->getEdgeIter();
	
	// iterate over all the neighbors
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		unsigned int which;
		if ((which = e->getFrom()) == n->GetNum()) which = e->getTo();
		
		node *neighbor = _g->GetNode(which);
		if (windowSize)
			addNeighborsToCorridor(_g, neighbor, windowSize-1);
		else
			eligibleNodes[neighbor] = true;
	}
}

bool aStar::nodeInCorridor(node *n)
{
	return ((eligibleNodes.size() == 0) ||
					(eligibleNodes.find(abstr->GetParent(n)) != eligibleNodes.end()));
}

double aStar::internalHeuristic(node *from, node *to)
{
	//	if ((abstr->GetAbstractionLevel(from) >= gMaxAbstraction) || (abstraction == 0))
		return abstr->h(from, to);
	//	return abstraction->getHVal(abstr->GetParent(from));
}
