/*
 *  $Id: praStar2.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/23/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "PRAStar2.h"
#include "FPUtil.h"

using namespace GraphAbstractionConstants;
static const int verbose = 0;

praStar2::praStar2()
:SearchAlgorithm()
{
	partialLimit = -1;
	expandSearchRadius = true;
	enhancedAbstractPathing = false;
	sprintf(algName,"PRA2*(%d)", partialLimit);
	skip = 0; //don't skip any levels 
	fixedPlanLevel = -1; //don't use a fixed plan level 
	planFromMiddle = true; //dynamically find starting level
	numLevels = 0;
	topLevel = -1;
}

path *praStar2::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp)
{
	std::vector<node *> fromChain;
	std::vector<node *> toChain;
	path *lastPath = 0;
	
	if (aMap->GetAbstractGraph(from->GetLabelL(kAbstractionLevel))->FindEdge(from->GetNum(), to->GetNum()))
		return new path(from, new path(to));
	
	setupSearch(aMap, fromChain, from, toChain, to);
	
	if (fromChain.size() == 0)
		return 0;
	
	do {
		lastPath = buildNextAbstractPath(aMap, lastPath, fromChain, toChain, rp);
	} while (lastPath==0 || lastPath->n->GetLabelL(kAbstractionLevel) > 0);
	
	return lastPath;
}

void praStar2::setupSearch(GraphAbstraction *aMap,
													 std::vector<node *> &fromChain, node *from,
													 std::vector<node *> &toChain, node *to)
{
	numLevels=0;
	
	nodesExpanded = 0;
	nodesTouched = 0;
	
	if ((from == 0) || (to == 0) || (!aMap->Pathable(from, to)) || (from == to))
	{
		if (verbose)
		{
			if (!from)
				printf("praStar2: from == 0\n");
			if (!to)
				printf("praStar2: to == 0\n");
			if (from == to)
				printf("praStar2: from == to\n");
			if (from && to && (!aMap->Pathable(from, to)))
				printf("praStar2: no path from %p to %p\n", (void*)from, (void*)to);
			//cout << "praStar: no path from " << *from << " to " << *to << endl;
		}
		return;
	}
	
	if (verbose)
		printf("At nodes #%d and %d\n", from->GetNum(), to->GetNum());
	
	aMap->GetNumAbstractGraphs(from, to, fromChain, toChain);
	
	//Check for fixed plan level and dynamic plan level
	selectTopAbstractionLevel(aMap, fromChain,toChain);
	
}

void praStar2::selectTopAbstractionLevel(GraphAbstraction *aMap,
																				 std::vector<node *> &fromChain,
																				 std::vector<node *> & toChain)
{
	if (fixedPlanLevel != -1)
	{
		if (verbose) std::cout<<"fixed set\n";
		// We've manually chosen some top level to use
		while (((int)fromChain.size() > 1) && ((int)toChain.size() > fixedPlanLevel + 1))
		{
			toChain.pop_back();
			fromChain.pop_back();
		}
	}
	else if (planFromMiddle)
	{
		if (verbose) std::cout<<"plan from middle\n";
		// Dynamically find middle of hierarchy to start planning
		
		unsigned int previousSize = fromChain.size();
		int minNode = (int)(2*sqrt(aMap->GetAbstractGraph(aMap->GetAbstractionLevel(fromChain[0]))->GetNumNodes()));
		
		while ((fromChain.size() > 2) && 
					 ((fromChain.size() > (previousSize)/2) ||
						(aMap->GetAbstractGraph(fromChain.size())->GetNumNodes() < minNode)))
		{
			toChain.pop_back();
			fromChain.pop_back();
		}
	}
	topLevel = toChain.size() -1;
	
	if (verbose)
		std::cout<<"Top praStar level: " << (toChain.size() -1) <<std::endl;
}


path *praStar2::buildNextAbstractPath(GraphAbstraction *aMap, path *lastPath,
																			std::vector<node *> &fromChain,
																			std::vector<node *> &toChain,
																			reservationProvider *rp)
{
	numLevels++;
	if (verbose)
		std::cout<<"Now finding a path on level " <<(int)(toChain.size() -1) <<std::endl;
	
	node *to, *from, *hTarget = 0;
	to = toChain.back();
	from = fromChain.back();
	toChain.pop_back();
	fromChain.pop_back();	
	
	if (skip==0 || skip<-1)
	{
		
		//Go down one level each time - don't need to do anything else
	}
	else if (skip==-1)
	{
		
		//Top level + bottom level only 
		while(toChain.size()>1)
		{
			
			toChain.pop_back();
			fromChain.pop_back();
		}	    
	}
	else{
		// Skip some levels
		// If we get to the bottom early, we skip less than skip
		int skipme = skip;
		while(toChain.size()>1 && skipme > 0)
		{
			
			toChain.pop_back();
			fromChain.pop_back();
			skipme--;
		}
	}
	
	
	if (verbose)
		printf("Expanded %d nodes before doing level %d\n", nodesExpanded, (int)from->GetLabelL(kAbstractionLevel));		
	
	if (verbose)
		printf("Building path from %d to %d (%ld/%ld)\n",
					 from->GetNum(), to->GetNum(), from->GetLabelL(kParent), to->GetLabelL(kParent));
	
	std::vector<node *> eligibleNodeParents;
	
	if (lastPath)
	{
		// cut path down to size of partial path limit
		if (partialLimit > 0)
		{
			path *trav = lastPath;
			
			for (int x = 0; x < partialLimit; x++)
				if (trav->next) 
					trav = trav->next;
			// we don't need to reset the target if we have a complete path
			// but we do if our complete path doesn't end in our target node
			if ((trav->next != 0) || ((trav->next == 0) && ((int)trav->n->GetNum() != to->GetLabelL(kParent))))
			{
				to = trav->n;
				if (trav->next)
					hTarget = trav->next->n;
				else
					hTarget = to;
				delete trav->next;
				trav->next = 0;
				if (verbose) printf("Setting target parent to %d\n", to->GetNum());
			}
		}
		
		Graph *g = aMap->GetAbstractGraph(lastPath->n->GetLabelL(kAbstractionLevel));
		
		// find eligible nodes for lower level expansions
		for (path *trav = lastPath; trav; trav = trav->next)
		{
			if (expandSearchRadius)
			{
				edge_iterator ei = trav->n->getEdgeIter();
				for (edge *e = trav->n->edgeIterNext(ei); e; e = trav->n->edgeIterNext(ei))
				{
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
	delete lastPath;
	path *result;
	
	if (hTarget != 0)
	{
		if (enhancedAbstractPathing)
			result = cAStar.getBestPath(aMap, fromChain[0], toChain[0], from, to, rp);
		else
			result = cAStar.getBestPath(aMap, from, to, hTarget, rp);
	}
	else {
		result = cAStar.GetPath(aMap, from, to, rp);
	}
	nodesExpanded += cAStar.GetNodesExpanded();
	nodesTouched += cAStar.GetNodesTouched();
	return result;
}

path *praStar2::trimPath(path *lastPath, node *origDest)
{
	if (partialLimit != -1)
	{
		int parent = -1;
		path *change = 0, *last = 0;
		for (path *trav = lastPath; trav; trav = trav->next)
		{
			if (trav->n == origDest)
				return lastPath;
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
	return lastPath;
}
