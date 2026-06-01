/*
 *  $Id: corridorAStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/22/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "CorridorAStar.h"
#include "FPUtil.h"

using namespace GraphAbstractionConstants;
const static int verbose = 0;

corridorAStar::corridorAStar()
{
	corridor = &emptyCorridor;
}

void corridorAStar::setCorridor(const std::vector<node *> *c)
{
	corridor = c;
}

path *corridorAStar::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp)
{
	return getBestPath(aMap, from, to, to, rp);
}

path *corridorAStar::getBestPath(GraphAbstraction *aMap, node *from, node *to, node *hGoal, reservationProvider *rp)
{
	nodesExpanded = 0;
	nodesTouched = 0;

  node *n=0;
  Heap *nodeHeap = new Heap(corridor->size());
  std::vector<node *> expandedNodes(100);
	neighbor_iterator ni;
	bool exactGoal = true;
	
	if (to->GetLabelL(kAbstractionLevel) != from->GetLabelL(kAbstractionLevel))
		exactGoal = false;
	
	// get current layer & abstract corridor layer
  int absLayer = from->GetLabelL(kAbstractionLevel);
	int corridorLayer = absLayer;
	if (corridor->size() > 0)
		corridorLayer = (*corridor)[0]->GetLabelL(kAbstractionLevel);
	Graph *absGraph = aMap->GetAbstractGraph(absLayer);
	
  // mark location of eligible nodes
  for (unsigned int x = 0; x < corridor->size(); x++)
    ((*corridor)[x])->key = x;
	
  // label start node cost 0
  n = from;
  n->SetLabelF(kTemporaryLabel, aMap->h(n, hGoal));
  n->markEdge(0);
  if (verbose) printf("Starting on %d with cost %1.2f\n", from->GetNum(), n->GetLabelF(kTemporaryLabel));
	nodesExpanded++;
  while (1) {
    expandedNodes.push_back(n);
    n->key = expandedNodes.size()-1;
		
    if (verbose)
      printf("really working on %d with cost %1.2f\n", n->GetNum(), n->GetLabelF(kTemporaryLabel));
		
		ni = n->getNeighborIter();
		
		for (node *currNode = absGraph->GetNode(n->nodeNeighborNext(ni));
				 currNode; currNode = absGraph->GetNode(n->nodeNeighborNext(ni)))
		{
			nodesTouched++;
      if (verbose) printf("considering neighbor %d\n", currNode->GetNum());
      if (nodeHeap->IsIn(currNode))
			{
				nodesExpanded++;
				relaxEdge(nodeHeap, absGraph, aMap,
									absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
									n, currNode, hGoal);
      }
      else if (rp && (absLayer==0) && (currNode != to) && rp->nodeOccupied(currNode))
			{
				if (verbose) printf("Can't path to %d, %d (occupied)\n", (unsigned int)currNode->GetLabelL(kFirstData), (unsigned int)currNode->GetLabelL(kFirstData+1));
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
				// ignore this tile if occupied.
      }
      // this node is unexpanded if (1) it isn't on the expanded list
      else if ((currNode->key >= expandedNodes.size()) ||
							 (expandedNodes[currNode->key] != currNode))
			{
				node *par = aMap->GetNthParent(currNode, corridorLayer);
				unsigned int parentKey = par->key;
				
				// this node is unexpanded if (2) it's parent is in the eligible parent list
				// (3) or having no eligible parents means we can search anywhere!
				if ((corridor->size() == 0) ||
						((parentKey < corridor->size()) && ((*corridor)[parentKey] == par)))
				{
					currNode->SetLabelF(kTemporaryLabel, MAXINT);
					currNode->SetKeyLabel(kTemporaryLabel);
					currNode->markEdge(0);
					nodeHeap->Add(currNode);
					if (verbose) printf("Adding neighbor %d\n", currNode->GetNum());
					nodesExpanded++;
					relaxEdge(nodeHeap, absGraph, aMap,
										absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
										n, currNode, hGoal);
				}
				else { if (verbose) printf("%d not eligible\n", currNode->GetNum()); }
      }
      else { if (verbose) printf("%d already expanded\n", currNode->GetNum()); }
    }
		
    n = (node*)nodeHeap->Remove();
    if (n == 0)
		{
			if (verbose) printf("Error: We expanded every possible node!\n");
			break;
		}

		// found a node who's nth parent is inside our target
		if ((!exactGoal) &&
				(aMap->GetNthParent(n, to->GetLabelL(kAbstractionLevel)) == to))
			break;
		
    if (n == to) { /*printf("Found goal %d\n", dest);*/ break; }
		
    if (verbose) printf("working on %d with cost %1.2f\n", n->GetNum(), n->GetLabelF(kTemporaryLabel));
  }
	
	if ((n != to) && (exactGoal))
	{
		if (verbose)
			printf("Corridor A* ran and didn't find goal in corridor!\n");
	}
  delete nodeHeap;
 	corridor = &emptyCorridor;
	if (n != 0) // we found a goal
		return extractBestPath(absGraph, n->GetNum());
	return 0;
}

path *corridorAStar::getBestPath(GraphAbstraction *aMap, node *afrom, node *ato, node *from, node *, reservationProvider *rp)
{
	nodesExpanded = 0;
	nodesTouched = 0;
	
  node *n=0;
  Heap *nodeHeap = new Heap(corridor->size());
  std::vector<node *> expandedNodes(100);
	neighbor_iterator ni;
	bool exactGoal = true;
	
	if (ato->GetLabelL(kAbstractionLevel) != afrom->GetLabelL(kAbstractionLevel))
		exactGoal = false;
	
	// get current layer & abstract corridor layer
  int absLayer = afrom->GetLabelL(kAbstractionLevel);
	int corridorLayer = absLayer;
	if (corridor->size() > 0)
		corridorLayer = (*corridor)[0]->GetLabelL(kAbstractionLevel);
	Graph *absGraph = aMap->GetAbstractGraph(absLayer);
	
  // mark location of eligible nodes
  for (unsigned int x = 0; x < corridor->size(); x++)
    ((*corridor)[x])->key = x;
	
  // label start node cost 0
  n = afrom;
  n->SetLabelF(kTemporaryLabel, aMap->h(n, ato));
  n->markEdge(0);
  if (verbose) printf("Starting on %d with cost %1.2f\n", afrom->GetNum(), n->GetLabelF(kTemporaryLabel));
	nodesExpanded++;
  while (1)
	{
    expandedNodes.push_back(n);
    n->key = expandedNodes.size()-1;
		
    if (verbose)
      printf("really working on %d with cost %1.2f\n", n->GetNum(), n->GetLabelF(kTemporaryLabel));
		
		ni = n->getNeighborIter();
		
		for (node *currNode = absGraph->GetNode(n->nodeNeighborNext(ni));
				 currNode; currNode = absGraph->GetNode(n->nodeNeighborNext(ni)))
		{
			nodesTouched++;
      if (verbose) printf("considering neighbor %d\n", currNode->GetNum());
      if (nodeHeap->IsIn(currNode))
			{
				nodesExpanded++;
				if (n == afrom)
					relaxFirstEdge(nodeHeap, absGraph, aMap,
										absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
										from, n, currNode, ato);
				if (currNode == ato)
					relaxFinalEdge(nodeHeap, absGraph, aMap,
												 absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
												 n, currNode, ato);
				else				
					relaxEdge(nodeHeap, absGraph, aMap,
									absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
									n, currNode, ato);
      }
      else if (rp && (absLayer==0) && (currNode != ato) && rp->nodeOccupied(currNode))
			{
				if (verbose) printf("Can't path to %d, %d (occupied)\n", (unsigned int)currNode->GetLabelL(kFirstData), (unsigned int)currNode->GetLabelL(kFirstData+1));
				expandedNodes.push_back(currNode);
				currNode->key = expandedNodes.size()-1;
				// ignore this tile if occupied.
      }
      // this node is unexpanded if (1) it isn't on the expanded list
      else if ((currNode->key >= expandedNodes.size()) ||
							 (expandedNodes[currNode->key] != currNode))
			{
				node *par = aMap->GetNthParent(currNode, corridorLayer);
				unsigned int parentKey = par->key;
				
				// this node is unexpanded if (2) it's parent is in the eligible parent list
				// (3) or having no eligible parents means we can search anywhere!
				if ((corridor->size() == 0) ||
						((parentKey < corridor->size()) && ((*corridor)[parentKey] == par)))
				{
					currNode->SetLabelF(kTemporaryLabel, MAXINT);
					currNode->SetKeyLabel(kTemporaryLabel);
					currNode->markEdge(0);
					nodeHeap->Add(currNode);
					if (verbose) printf("Adding neighbor %d\n", currNode->GetNum());
					nodesExpanded++;

					if (n == afrom)
						relaxFirstEdge(nodeHeap, absGraph, aMap,
													 absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
													 from, n, currNode, ato);
					if (currNode == ato)
						relaxFinalEdge(nodeHeap, absGraph, aMap,
													 absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
													 n, currNode, ato);
					else				
						relaxEdge(nodeHeap, absGraph, aMap,
											absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
											n, currNode, ato);
//					relaxEdge(nodeHeap, absGraph, aMap,
//										absGraph->FindEdge(n->GetNum(), currNode->GetNum()),
//										n, currNode, hGoal);
				}
				else { if (verbose) printf("%d not eligible\n", currNode->GetNum()); }
      }
      else { if (verbose) printf("%d already expanded\n", currNode->GetNum()); }
    }
		
    n = (node*)nodeHeap->Remove();
    if (n == 0)
		{
			if (verbose) printf("Error: We expanded every possible node!\n");
			break;
		}
		
		// found a node who's nth parent is inside our target
		if ((!exactGoal) &&
				(aMap->GetNthParent(n, ato->GetLabelL(kAbstractionLevel)) == ato))
			break;
		
    if (n == ato) { /*printf("Found goal %d\n", dest);*/ break; }
		
    if (verbose) printf("working on %d with cost %1.2f\n", n->GetNum(), n->GetLabelF(kTemporaryLabel));
  }
	
	if ((n != ato) && (exactGoal))
	{
		if (verbose)
			printf("Corridor A* ran and didn't find goal in corridor!\n");
	}
  delete nodeHeap;
 	corridor = &emptyCorridor;
	if (n != 0) // we found a goal
		return extractBestPath(absGraph, n->GetNum());
	return 0;
}

void corridorAStar::relaxEdge(Heap *nodeHeap, Graph *, GraphAbstraction *aMap,
															edge *e, node *from, node *to, node *dest)
{
  double weight;
  weight = from->GetLabelF(kTemporaryLabel)-aMap->h(from, dest)+aMap->h(to, dest)+e->GetWeight();
  if (fless(weight, to->GetLabelF(kTemporaryLabel)))
	{
    if (verbose)
      printf("Updating %d to %1.2f from %1.2f\n", to->GetNum(), weight, to->GetLabelF(kTemporaryLabel));
		//weight -= 0.001*(weight-map->h(to, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
    to->SetLabelF(kTemporaryLabel, weight);
    nodeHeap->DecreaseKey(to);
    // this is the edge used to get to this node in the min. path tree
    to->markEdge(e);
  }
}

void corridorAStar::relaxFirstEdge(Heap *nodeHeap, Graph *, GraphAbstraction *aMap,
																	 edge *e, node *from, node *afrom, node *ato, node *dest)
{
  double weight;
  weight = afrom->GetLabelF(kTemporaryLabel)-aMap->h(afrom, dest)+aMap->h(ato, dest)+aMap->h(from, ato);
  if (fless(weight, ato->GetLabelF(kTemporaryLabel)))
	{
    if (verbose)
      printf("Updating %d to %1.2f from %1.2f\n", ato->GetNum(), weight, ato->GetLabelF(kTemporaryLabel));
		//weight -= 0.001*(weight-map->h(ato, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
    ato->SetLabelF(kTemporaryLabel, weight);
    nodeHeap->DecreaseKey(ato);
    // this is the edge used to get to this node in the min. path tree
    ato->markEdge(e);
  }
}

void corridorAStar::relaxFinalEdge(Heap *nodeHeap, Graph *, GraphAbstraction *aMap,
																	 edge *e, node *from, node *to, node *realDest)
{
  double weight;
  weight = from->GetLabelF(kTemporaryLabel)-aMap->h(from, to)+aMap->h(from, realDest);
  if (fless(weight, to->GetLabelF(kTemporaryLabel)))
	{
    if (verbose)
      printf("Updating %d to %1.2f from %1.2f\n", to->GetNum(), weight, to->GetLabelF(kTemporaryLabel));
		//weight -= 0.001*(weight-map->h(to, d)); // always lower g-cost slightly so that we tie break in favor of higher g cost
    to->SetLabelF(kTemporaryLabel, weight);
    nodeHeap->DecreaseKey(to);
    // this is the edge used to get to this node in the min. path tree
    to->markEdge(e);
  }
}

path *corridorAStar::extractBestPath(Graph *g, unsigned int current)
{
  path *p = 0;
  edge *e;
  // extract best path from Graph -- each node has a single parent in the Graph which is the marked edge
  // for visuallization purposes, an edge can be marked meaning it will be drawn in white
  while ((e = g->GetNode(current)->getMarkedEdge()))
	{
    if (verbose) printf("%d <- ", current);
		
    p = new path(g->GetNode(current), p);
		
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
