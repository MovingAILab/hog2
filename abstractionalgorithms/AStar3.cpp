/*
 *  $Id: aStar3.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/29/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "FPUtil.h"
#include "AStar3.h"
#include "Heap.h"
#include <string.h>

using namespace GraphAbstractionConstants;
static const int verbose = 0;

// The constructor
aStarOld::aStarOld(double _weight, bool _doPathDraw)
:SearchAlgorithm()
{
	wh = _weight;
	doPathDraw = _doPathDraw;
	// Generate algorithm's name
	if (fequal(wh,1.0))
		strcpy(aStarName,"aStarOld");
	else
		sprintf(aStarName,"aStarOld(%1.1f)",wh);
}


// The same A*, but counts the number of states expanded
path *aStarOld::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp)
{
	// Reset the number of states expanded
	nodesExpanded = 0;
	nodesTouched = 0;
	
	if ((from == 0) || (to == 0) || (!aMap->Pathable(from, to)) || (from == to))
		return 0;
	map = aMap;
	Graph *g = map->GetAbstractGraph(from->GetLabelL(kAbstractionLevel));
	Heap *openList = new Heap(30);
	std::vector<node *> closedList;
	node *n;
	
	// label start node cost 0
	n = from;
	n->SetLabelF(kTemporaryLabel, wh*map->h(n, to));
	n->markEdge(0);
	
	while (1)
	{
		nodesExpanded++;
		edge_iterator ei;
		
		// move current node onto closed list
		// mark node with its location in the closed list
		closedList.push_back(n);
		n->key = closedList.size()-1;
		
		if (verbose)
			printf("Working on %d with cost %1.2f\n", n->GetNum(), n->GetLabelF(kTemporaryLabel));
		
		ei = n->getEdgeIter();
		
		// iterate over all the children
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			nodesTouched++;
			unsigned int which;
			if ((which = e->getFrom()) == n->GetNum()) which = e->getTo();
			
			node *nextChild = g->GetNode(which);
			
			// if it's on the open list, we can still update the weight
			if (openList->IsIn(nextChild))
			{
				//nodesExpanded++;
				relaxEdge(openList, g, e, n->GetNum(), which, to);
			}
			else if (rp && (from->GetLabelL(kAbstractionLevel)==0) && (nextChild != to) &&
							 rp->nodeOccupied(nextChild))
			{
				//printf("Can't path to %d, %d\n", (unsigned int)nextChild->GetLabelL(kFirstData), (unsigned int)nextChild->GetLabelL(kFirstData+1));
				closedList.push_back(nextChild);
				nextChild->key = closedList.size()-1;
				// ignore this tile if occupied.
			}
			// if it's not on the open list, then add it to the open list
			else if ((nextChild->key >= closedList.size()) ||
							 (closedList[nextChild->key] != nextChild))
			{
				nextChild->SetLabelF(kTemporaryLabel, MAXINT);
				nextChild->SetKeyLabel(kTemporaryLabel);
				nextChild->markEdge(0);
				openList->Add(nextChild);
				if (verbose)
					printf("Adding neighbor/child %d\n", which);
				//nodesExpanded++;
				relaxEdge(openList, g, e, n->GetNum(), which, to);
			}
		}
		
		// get the next (the best) node off the open list
		n = (node*)openList->Remove();
		
		// this means we have expanded all reachable nodes and there is no path
		if (n == 0) { delete openList; return 0; }
		if (verbose) printf("Expanding %d\n", n->GetNum());
		if (n == to) break; // we found the goal
	}
	delete openList;
	return extractBestPath(g, n->GetNum());
}

// this is the standard definition of relaxation as in Introduction to Algorithms (cormen, leiserson and rivest)
void aStarOld::relaxEdge(Heap *nodeHeap, Graph *g, edge *e, int source, int nextNode, node *d)
{
	double weight;
	node *from = g->GetNode(source);
	node *to = g->GetNode(nextNode);
	weight = from->GetLabelF(kTemporaryLabel)-wh*map->h(from, d)+wh*map->h(to, d)+e->GetWeight();
	if (fless(weight, to->GetLabelF(kTemporaryLabel)))
	{
		if (verbose)
			printf("Updating %d to %1.2f from %1.2f\n", nextNode, weight, to->GetLabelF(kTemporaryLabel));
		to->SetLabelF(kTemporaryLabel, weight);
		nodeHeap->DecreaseKey(to);
		// this is the edge used to get to this node in the min. path tree
		to->markEdge(e);
	}
}

path *aStarOld::extractBestPath(Graph *g, unsigned int current)
{
	path *p = 0;
	edge *e;
	// extract best path from Graph -- each node has a single parent in the Graph which is the marked edge
	// for visuallization purposes, an edge can be marked meaning it will be drawn in white
	while ((e = g->GetNode(current)->getMarkedEdge()))
	{
		if (verbose) printf("%d <- ", current);
		
		p = new path(g->GetNode(current), p);
		
		if (doPathDraw)
			e->setMarked(true);
		
		if (e->getFrom() == current)
			current = e->getTo();
		else
			current = e->getFrom();
	}
	p = new path(g->GetNode(current), p);
	if (verbose) printf("%d\n", current);
	return p;	
}
