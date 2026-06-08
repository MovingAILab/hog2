/*
 *  GenericIDAStar.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 1/3/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "GenericIDAStar.h"
#include <math.h>
#include "FPUtil.h"

using namespace OldSearchCode;


void GenericIDAStar::GetPath(SearchEnvironment *env, uint32_t from, uint32_t to,
														 std::vector<uint32_t> &thePath)
{
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	//updateNextBound(0, env->heuristic(from, to));
	UpdateNextBound(0, floor(env->heuristic(from, to)));
	while (thePath.size() == 0)
	{
		nodeTable.clear();
		printf("Starting iteration with bound %f\n", nextBound);
		DoIteration(env, from, from, to, thePath, nextBound, 0, 0);
	}
}

double GenericIDAStar::DoIteration(SearchEnvironment *env,
																	 uint32_t parent, uint32_t currState, uint32_t goal,
																	 std::vector<uint32_t> &thePath, double bound, double g,
																	 double maxH)
{
	nodesExpanded++;
	double h = floor(env->heuristic(currState, goal));
	//double h = env->heuristic(currState, goal);

	// path max
	if (usePathMax && fless(h, maxH))
		h = maxH;
	if (fgreater(g+h, bound))
	{
		UpdateNextBound(bound, g+h);
		//printf("Stopping at (%d, %d). g=%f h=%f\n", currState>>16, currState&0xFFFF, g, h);
		return h;
	}
	if (nodeTable.find(currState) != nodeTable.end()) // already seen
	{
		if (fless(g/*+h*/, nodeTable[currState])) // with lower g-cost
		{
		}
		else {
			return h;
		}
	}
	nodeTable[currState] = g;//+h;
	
	std::vector<uint32_t> neighbors;
	env->getNeighbors(currState, neighbors);
	nodesTouched += neighbors.size();

	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (neighbors[x] == parent)
			continue;
		thePath.push_back(neighbors[x]);
		double edgeCost = env->gcost(currState, neighbors[x]);
		double childH = DoIteration(env, currState, neighbors[x], goal, thePath, bound,
																g+edgeCost, maxH - edgeCost);
		if (thePath.back() == goal)
			return 0;
		thePath.pop_back();
		// pathmax
		if (usePathMax && fgreater(childH-edgeCost, h))
		{
			nodeTable[currState] = g;//+h
			h = childH-edgeCost;
			if (fgreater(g+h, bound))
			{
				UpdateNextBound(bound, g+h);
				return h;
			}
		}
	}
	return h;
}

void GenericIDAStar::UpdateNextBound(double currBound, double fCost)
{
	fCost = floor(fCost);
	if (!fgreater(nextBound, currBound))
		nextBound = fCost;
	else if (fgreater(fCost, currBound) && fless(fCost, nextBound))
		nextBound = fCost;
}
