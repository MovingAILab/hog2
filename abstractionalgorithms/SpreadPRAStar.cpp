/*
 *  $Id: spreadPRAStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/27/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "SpreadPRAStar.h"
#include "FPUtil.h"

using namespace GraphAbstractionConstants;
static const int verbose = 0;

spreadPRAStar::spreadPRAStar()
{
	setPartialPathLimit(-1);
	lastPath = 0;
	expandSearchRadius = true;
}

path *spreadPRAStar::GetPath(GraphAbstraction *_aMap, node *from, node *to, reservationProvider *_rp)
{
  std::vector<node *> fromChain;
  std::vector<node *> toChain;
	path *lastPth = 0;
	
  if (_aMap->GetAbstractGraph(from->GetLabelL(kAbstractionLevel))->FindEdge(from->GetNum(), to->GetNum()))
    return new path(from, new path(to));

	setupSearch(_aMap, fromChain, from, toChain, to);
	if (fromChain.size() == 0)
		return 0;
  do {
		lastPth = buildNextAbstractPath(_aMap, lastPth, fromChain, toChain, _rp);
  } while (lastPth && (lastPth->n->GetLabelL(kAbstractionLevel) > 0));
	return lastPth;
}

void spreadPRAStar::setTargets(GraphAbstraction *_aMap, node *s, node *e, reservationProvider *_rp)
{
	rp = _rp; start = s; end = e; aMap = _aMap;
	startChain.resize(0);
	endChain.resize(0);
	setupSearch(aMap, startChain, start, endChain, end);
	delete lastPath;
	lastPath = 0;
}

/** how many times do we have to "think" to find the solution, return -1 if unknown */
int spreadPRAStar::getNumThinkSteps()
{
	return -1;
}

/** do next processing for path, returns avaliability of path moves */
path *spreadPRAStar::think()
{
	nodesExpanded = 0;
	nodesTouched = 0;
	if (startChain.size() == 0)
		return 0;
	lastPath = buildNextAbstractPath(aMap, lastPath, startChain, endChain, rp);
	if (lastPath && (lastPath->n->GetLabelL(kAbstractionLevel) == 0))
	{
		path *p = lastPath;
		lastPath = 0;
		return p;
	}
	return 0;
}

void spreadPRAStar::setupSearch(GraphAbstraction *_aMap,
																std::vector<node *> &fromChain, node *from,
																std::vector<node *> &toChain, node *to)
{
	nodesExpanded = 0;
	nodesTouched = 0;
	
  if ((from == 0) || (to == 0) || (!_aMap->Pathable(from, to)) || (from == to))
	{
    if (verbose)
		{
      if (!from)
				printf("spreadPRAStar: from == 0\n");
      if (!to)
				printf("spreadPRAStar: to == 0\n");
      if (from == to)
				printf("spreadPRAStar: from == to\n");
      if (from && to && (!_aMap->Pathable(from, to)))
				printf("spreadPRAStar: no path from %p to %p\n", (void*)from, (void*)to);
			//cout << "praStar: no path from " << *from << " to " << *to << endl;
    }
		return;
  }
	
  if (verbose)
    printf("At nodes #%d and %d\n", from->GetNum(), to->GetNum());
//  if (aMap->GetAbstractGraph(0)->FindEdge(from->GetNum(), to->GetNum()))
//	{ // we are 1 step away
//    return new path(from, new path(to));
//  }
	
  _aMap->GetNumAbstractGraphs(from, to, fromChain, toChain);
	//	assert(aMap->GetAbstractGraph(fromChain.back()->GetLabelL(kAbstractionLevel))->
	//					FindEdge(fromChain.back()->GetNum(), toChain.back()->GetNum()));
	
	unsigned int previousSize = fromChain.size();
	int minNode = (int)(2*sqrt(_aMap->GetAbstractGraph(0)->GetNumNodes()));
	while ((fromChain.size() > 2) && ((fromChain.size() > (previousSize)/2) ||
																		(_aMap->GetAbstractGraph(fromChain.size())->GetNumNodes() < minNode)))
	{
		toChain.pop_back();
		fromChain.pop_back();
	}
}

path *spreadPRAStar::buildNextAbstractPath(GraphAbstraction *_aMap, path *lastPth,
																			std::vector<node *> &fromChain,
																			std::vector<node *> &toChain,
																			reservationProvider *_rp)
{
	node *to, *from, *hTarget = 0;
	to = toChain.back();
	toChain.pop_back();
	from = fromChain.back();
	fromChain.pop_back();
	
	if (verbose)
		printf("Expanded %d nodes before doing level %d\n", nodesExpanded, (int)from->GetLabelL(kAbstractionLevel));		
	
	if (verbose)
		printf("Building path from %d to %d (%ld/%ld)\n",
					 from->GetNum(), to->GetNum(), from->GetLabelL(kParent), to->GetLabelL(kParent));
	
	std::vector<node *> eligibleNodeParents;
	
	if (lastPth)
	{
		// cut path down to size of partial path limit
		if (partialLimit > 0)
		{
			path *trav = lastPth;
			
			for (int x = 0; x < partialLimit; x++)
				if (trav->next) 
					trav = trav->next;
			// we don't need to reset the target if we have a complete path
			// but we do if our complete path doesn't end in our target node
			if ((trav->next != 0) || ((trav->next == 0) && ((long)trav->n->GetNum() != to->GetLabelL(kParent))))
			{
				to = trav->n;
				if (trav->next)
					hTarget = trav->next->n;
				else
					hTarget = to;
				delete trav->next;
				trav->next = 0;
			}
		}
		
		Graph *g = _aMap->GetAbstractGraph(lastPth->n->GetLabelL(kAbstractionLevel));
		// find eligible nodes for lower level expansions
		for (path *trav = lastPth; trav; trav = trav->next)
		{
			if (expandSearchRadius)
			{
				edge_iterator ei = trav->n->getEdgeIter();
				for (edge *e = trav->n->edgeIterNext(ei); e; e = trav->n->edgeIterNext(ei)) {
					if (e->getFrom() == trav->n->GetNum())
						eligibleNodeParents.push_back(g->GetNode(e->getTo()));
					else
						eligibleNodeParents.push_back(g->GetNode(e->getFrom()));
				}
			}
			eligibleNodeParents.push_back(trav->n);
		}
	}
	cAStar.setCorridor(&eligibleNodeParents);
	path *result;
	if (hTarget != 0)
	{
		result = cAStar.getBestPath(_aMap, from, to, hTarget, _rp);
		if (result == 0)
		{
			if (verbose) printf("NULL result from getBestPath in spreadPRAStar\n");
		}
	}
	else {
		result = cAStar.GetPath(_aMap, from, to, _rp);
		if (result == 0)
		{
			if (verbose) printf("NULL result from GetPath in spreadPRAStar\n");
		}
	}
	nodesExpanded += cAStar.GetNodesExpanded();
	nodesTouched += cAStar.GetNodesTouched();
	return result;
}

path *spreadPRAStar::trimPath(path *lastPth, node *origDest)
{
	if (partialLimit != -1)
	{
		int parent = -1;
		path *change = 0, *last = 0;
		for (path *trav = lastPth; trav; trav = trav->next)
		{
			if (trav->n == origDest)
				return lastPth;
			if (trav->n->GetLabelL(kParent) != parent)
			{
				parent = trav->n->GetLabelL(kParent);
				change = last;
			}
			last = trav;
		}
		if (change)
		{
			delete change->next;
			change->next = 0;
		}
	}
	return lastPth;
}
