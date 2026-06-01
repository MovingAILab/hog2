//
//  CanonicalRTAStar.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/21/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include "CanonicalRTAStar.h"
#include <limits>

/** The core routine of LRTAStar -- computes at most one-move path */
void CanonicalRTAStar::GetPath(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath)
{
	// Look at neighbors. If any haven't been visited, go there. Otherwise backtrack.
	
	if (localEnv == 0)
	{
		localEnv = new CanonicalGrid::CanonicalGrid(env->GetMap());
	}
	externalEnv = env;
	goal = to;
	thePath.resize(0);
	if (from==to)
		return;

	
	// go to the
	
//
//	nodesExpanded = 0;
//	nodesTouched = 0;
//	nodesTouched++;
//	
//	nodesExpanded++;
//
	AddToTable(from, kAll, CanonicalGrid::kAll);
//
//	if (heur.find(from) == heur.end())
//	{
//		heur[from].theHeuristic = env->HCost(from, to);
//		heur[from].parent = CanonicalGrid::kAll;
//		CanonicalGrid::xyLoc l;
//		l.x = from.x;
//		l.y = from.y;
//		l.parent = CanonicalGrid::kAll;
//		heur[from].theState = l;
//	}
//	
	std::vector<CanonicalGrid::xyLoc> neighbors;
	localEnv->GetSuccessors(heur[from].theState, neighbors);
//	std::vector<double> fcost(neighbors.size());
//	
	if (neighbors.size() == 0)
	{
		xyLoc next;
		env->GetNextState(from, heur[from].parent, next);
		thePath.push_back(next);
		thePath.push_back(from);
		heur[from] =  std::numeric_limits<double>::infinity();
		return;
	}
//
//	int best = 0;
//	int secondBest = 0;
//	for (unsigned int x = 0; x < neighbors.size(); x++)
//	{
//		// Generate a child
//		nodesTouched++;
//		
//		// Compute the g values
//		double g = localEnv->GCost(heur[from].theState, neighbors[x]);// e->getWeight();
//		
//		// See if the min on this level needs to be updated
//		double h = HCost(neighbors[x]);
//		double f = g + h;
//		fcost[x] = f;
//		
//		if (fless(f, fcost[best]))
//		{
//			secondBest = best;
//			best = x;
//		}
//		else if (fequal(f, fcost[best]))
//		{
//			secondBest = x;
//		}
//		else if (fless(f, fcost[secondBest]))
//		{
//			secondBest = x;
//		}
//	}
//
//	if (neighbors.size() <= 1)
//	{
//		heur[from] =  std::numeric_limits<double>::infinity();
//	}
//	else {
//		heur[from].theHeuristic = fcost[secondBest];
//	}
//	
//	// Move -------------------------------------------------------------------------
//	thePath.push_back(neighbors[best]);
//	thePath.push_back(from);
	return;
}

void CanonicalRTAStar::OpenGLDraw(const MapEnvironment *e) const
{
//	// TODO: handle INFINITY!
//	double learned = 0;
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double thisState = (*it).second.theHeuristic;
//		if (learned < thisState)
//			learned = thisState;
//	}
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double r = (*it).second.theHeuristic;
//		if (r > 0)
//		{
//			e->SetColor(0.5+0.5*r/learned, 0, 0, 0.1+0.8*r/learned);
//			e->OpenGLDraw((*it).second.theState);
//		}
//	}
}

