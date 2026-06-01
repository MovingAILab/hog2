/*
 *  DFID.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef DFID_H
#define DFID_H

#include <iostream>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"

typedef std::unordered_map<uint64_t, double> NodeHashTable;

template <class state, class action>
class DFID {
public:
	DFID() { useHashTable = usePathMax = false; }
	virtual ~DFID() {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
							 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);

	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void SetUseBDPathMax(bool val) { usePathMax = val; }
private:
	unsigned long nodesExpanded, nodesTouched;
	
	bool DoIteration(SearchEnvironment<state, action> *env,
					   state parent, state currState,
					 std::vector<state> &thePath, double bound, double g);
	bool DoIteration(SearchEnvironment<state, action> *env,
					   action forbiddenAction, state &currState,
					 std::vector<action> &thePath, double bound, double g);
	
	void UpdateNextBound(double currBound, double gCost);
	state goal;
	double nextBound;
	NodeHashTable nodeTable;
	bool usePathMax;
	bool useHashTable;
};	

template <class state, class action>
void DFID<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<state> &thePath)
{
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	UpdateNextBound(0, 0);
	goal = to;
	double lastBound;
	while (thePath.size() == 0)
	{
		nodeTable.clear();
//		printf("Starting iteration with bound %f\n", nextBound);
		lastBound = nextBound;
		DoIteration(env, from, from, thePath, nextBound, 0);
		std::cout << nodesExpanded << " after next iteration with cost limit " << nextBound << std::endl;
		if (fequal(lastBound, nextBound))
			break;
	}
}

template <class state, class action>
void DFID<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<action> &thePath)
{
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	UpdateNextBound(0, 0);
	goal = to;
	std::vector<action> act;
	env->GetActions(from, act);
	double lastBound;

	while (thePath.size() == 0)
	{
		nodeTable.clear();
//		printf("Starting iteration with bound %f\n", nextBound);
		lastBound = nextBound;
		DoIteration(env, act[0], from, thePath, nextBound, 0);
		std::cout << nodesExpanded << " after iteration with cost limit " << nextBound << std::endl;
		if (fequal(lastBound, nextBound))
			break;
	}
}

template <class state, class action>
bool DFID<state, action>::DoIteration(SearchEnvironment<state, action> *env,
										   state parent, state currState,
										   std::vector<state> &thePath, double bound, double g)
{
	if (fgreater(g, bound))
	{
		UpdateNextBound(bound, g);
		//printf("Stopping at (%d, %d). g=%f h=%f\n", currState>>16, currState&0xFFFF, g, h);
		return false;
	}
//	if (env->GoalTest(currState, goal))
//		return true;
		
	nodesExpanded++;

	std::vector<state> neighbors;
	env->GetSuccessors(currState, neighbors);
	nodesTouched += neighbors.size();

	//std::cout << "Expanding " << currState << std::endl;
	
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (neighbors[x] == parent)
			continue;
		thePath.push_back(neighbors[x]);
		double edgeCost = env->GCost(currState, neighbors[x]);
		if (DoIteration(env, currState, neighbors[x], thePath, bound, g+edgeCost))
			return true;
		thePath.pop_back();
	}
	return false;
}

template <class state, class action>
bool DFID<state, action>::DoIteration(SearchEnvironment<state, action> *env,
										   action forbiddenAction, state &currState,
										   std::vector<action> &thePath, double bound, double g)
{
	if (fgreater(g, bound))
	{
		UpdateNextBound(bound, g);
		//printf("Stopping at (%d, %d). g=%f h=%f\n", currState>>16, currState&0xFFFF, g, h);
		return false;
	}
//	if (env->GoalTest(currState, goal))
//		return true; // found goal
	
	nodesExpanded++;

	std::vector<action> actions;
	env->GetActions(currState, actions);
	nodesTouched += actions.size();
	int depth = thePath.size();
	
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		if ((depth != 0) && (actions[x] == forbiddenAction))
			continue;

		thePath.push_back(actions[x]);

		double edgeCost = env->GCost(currState, actions[x]);
		env->ApplyAction(currState, actions[x]);
		env->InvertAction(actions[x]);
		if (DoIteration(env, actions[x], currState, thePath, bound, g+edgeCost))
		{
			env->ApplyAction(currState, actions[x]);
			return true;
		}
		env->ApplyAction(currState, actions[x]);

		thePath.pop_back();
	}
	return false;
}


template <class state, class action>
void DFID<state, action>::UpdateNextBound(double currBound, double gCost)
{
	if (!fgreater(nextBound, currBound))
	{
		nextBound = gCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
	else if (fgreater(gCost, currBound) && fless(gCost, nextBound))
	{
		nextBound = gCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
}


#endif
