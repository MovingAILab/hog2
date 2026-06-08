/*
 *  DFS.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/15/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#ifndef DFS_H
#define DFS_H

#include <iostream>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"

template <class state, class action>
class DFS {
public:
	DFS() { }
	virtual ~DFS() {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
private:
	typedef std::unordered_map<uint64_t, bool, Hash64> DFSClosedList;
	
	void DoIteration(SearchEnvironment<state, action> *env,
					 state parent, state currState,
					 std::vector<state> &thePath, double bound, double g);
	void DoIteration(SearchEnvironment<state, action> *env,
					 action forbiddenAction, state &currState,
					 std::vector<action> &thePath, double bound, double g);
	
	unsigned long nodesExpanded, nodesTouched;
	double nextBound;
	std::deque<state> mOpen;
	std::deque<int> depth;
//	DFSClosedList mClosed; // store parent id!
};	

template <class state, class action>
void DFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<state> &thePath)
{
	nextBound = DBL_MAX;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
//	goal = to;
	DoIteration(env, from, from, thePath, nextBound, 0);
}

template <class state, class action>
void DFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<action> &thePath)
{
	nextBound = DBL_MAX;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
//	goal = to;
	std::vector<action> act;
	env->GetActions(from, act);
	DoIteration(env, act[0], from, thePath, nextBound, 0);
}

template <class state, class action>
void DFS<state, action>::DoIteration(SearchEnvironment<state, action> *env,
										   state parent, state currState,
										   std::vector<state> &thePath, double bound, double g)
{
	nodesExpanded++;
	
	std::vector<state> neighbors;
	env->GetSuccessors(currState, neighbors);
	nodesTouched += neighbors.size();
	
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (neighbors[x] == parent)
			continue;
//		if (mClosed.find(env->GetStateHash(neighbors[x])) != mClosed.end()) // check for duplicates on path
//			continue;

		thePath.push_back(neighbors[x]);
		double edgeCost = env->GCost(currState, neighbors[x]);
//		mClosed[env->GetStateHash(neighbors[x])] = true;
		DoIteration(env, currState, neighbors[x], thePath, bound, g+edgeCost);
//		mClosed.erase(mClosed.find(env->GetStateHash(neighbors[x])));
	//		if (env->GoalTest(thePath.back(), goal))
//			return 0;
		thePath.pop_back();
	}
}

template <class state, class action>
void DFS<state, action>::DoIteration(SearchEnvironment<state, action> *env,
										   action forbiddenAction, state &currState,
										   std::vector<action> &thePath, double bound, double g)
{
	nodesExpanded++;
	
//	if (env->GoalTest(currState, goal))
//		return -1; // found goal
	
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

//		if (mClosed.find(env->GetStateHash(currState)) == mClosed.end()) // check for duplicates on path
//		{
//			mClosed[env->GetStateHash(currState)] = true;
		DoIteration(env, actions[x], currState, thePath, bound, g+edgeCost);
//			mClosed.erase(mClosed.find(env->GetStateHash(currState)));
//		}

		env->ApplyAction(currState, actions[x]);
//		if (fequal(childH, -1)) // found goal
//			return -1;
		
		thePath.pop_back();
		
	}
}


#endif
