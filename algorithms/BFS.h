/*
 *  BFS.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/15/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#ifndef BFS_H
#define BFS_H

#include "SearchEnvironment.h"
#include "FPUtil.h"

#include <algorithm>
#include <iostream>
#include <unordered_map>

template <class state, class action, class environment>
class BFS {
public:
	BFS():stopAtGoal(true), nodeLimit(~(0ull)), verbose(true) {}
	virtual ~BFS() {}
	void DoBFS(environment *env, state from);
	void GetPath(environment *env, state from, state to,
				 std::vector<state> &thePath);
	
	void SetNodeLimit(uint64_t value) {nodeLimit = value; }
	void ClearNodeLimit() {nodeLimit = ~(0ull);}
	void SetVerbose(bool v) {verbose = v;}
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	bool stopAtGoal;
private:
	bool verbose;
	uint64_t nodeLimit;
	uint64_t nodesExpanded, nodesTouched;
	
};

// pure BFS, just marking which states have been visited
// no path is saved
template <class state, class action, class environment>
void BFS<state, action, environment>::DoBFS(environment *env, state from)
{
//	typedef std::unordered_map<uint64_t, bool, Hash64> BFSClosedList;
//	BFSClosedList mClosed; // store parent id!
	std::deque<state> mOpen;
	std::deque<int> depth;
	std::unordered_map<state, bool> mClosed;
	
	std::vector<state> successors;
	nodesExpanded = nodesTouched = 0;
	
	mOpen.clear();
	mClosed.clear();
	depth.clear();
	
	depth.push_back(0);
	mOpen.push_back(from);
	
	int currDepth = 0;
	uint64_t lastNodes = 0, lastIter = 0;
	while (mOpen.size() > 0)
	{
		assert(mOpen.size() == depth.size());
		state s = mOpen.front();
		mOpen.pop_front();
		if (depth.front() != currDepth)
		{
//			printf("%ld tot %d inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
			lastIter = nodesExpanded-lastNodes;
			lastNodes = nodesExpanded;
		}
		currDepth = depth.front();
		depth.pop_front();
		
		if (mClosed.find(s) != mClosed.end())
		{
//			if (mOpen.size() == 0)
//			{
//				std::cout << "Final state:\n" << s << std::endl;
//			}
			continue;
		}
		mClosed[s] = true;
		
		nodesExpanded++;
		if (nodesExpanded > nodeLimit)
		{
			printf("Node limit hit. Aborting search\n");
			return;
		}
		env->GetSuccessors(s, successors);
		for (unsigned int x = 0; x < successors.size(); x++)
		{
			if (mClosed.find(successors[x]) == mClosed.end())
			{
				mOpen.push_back(successors[x]);
				depth.push_back(currDepth+1);
			}
		}
//		if (mOpen.size() == 0)
//		{
//			std::cout << "Final state:\n" << s << std::endl;
//		}
	}
//	printf("Final depth: %d, Nodes Expanded %lu, Exponential BF: %f\n", currDepth, nodesExpanded, pow(nodesExpanded, (double)1.0/currDepth));
}

// Richer BFS which saves information to allow the best path to be reconstructed.
template <class state, class action, class environment>
void BFS<state, action, environment>::GetPath(environment *env,
								 state from, state to,
								 std::vector<state> &thePath)
{
//	typedef std::unordered_map<uint64_t, uint64_t, Hash64> BFSClosedList;
	std::deque<std::pair<state, uint16_t>> mOpen;
	//std::deque<int> depth;
	std::unordered_map<state, state> mClosed; // store parent with each state

	thePath.resize(0);
	bool goalFound = false;
	int numGoals = 0;
	nodesExpanded = nodesTouched = 0;
	
	mOpen.clear();
	mClosed.clear();
	//depth.clear();
	
	//depth.push_back(0);
	mOpen.push_back({from, 0});
	mClosed[from] = from; // root has itself as parent

	int currDepth = 0;
	uint64_t lastNodes = 0, lastIter = 0;
	state s;
	state goal;
	while (mOpen.size() > 0)
	{
		s = mOpen.front().first;
		if (mOpen.front().second != currDepth)
		{
			if (verbose)
				printf("%ld tot %d inc %lld b %.2f [%d]\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter, numGoals);
			lastIter = nodesExpanded-lastNodes;
			lastNodes = nodesExpanded;
		}			
		currDepth = mOpen.front().second;//depth.front();
		mOpen.pop_front();
//		depth.pop_front();
		if (env->GoalTest(s, to))
		{
			if (numGoals == 0)
				goal = s;
			goalFound = true;
			numGoals++;
			if (stopAtGoal)
				break;
		}
		else { // don't expand goal nodes
			nodesExpanded++;
			if (nodesExpanded > nodeLimit)
			{
				printf("Node limit hit. Aborting search\n");
				thePath.clear();
				return;
			}
			
			env->GetSuccessors(s, thePath);
			for (unsigned int x = 0; x < thePath.size(); x++)
			{
				if (mClosed.find(thePath[x]) == mClosed.end())
				{
					mOpen.push_back({thePath[x], currDepth+1});
					//depth.push_back(currDepth+1);
					//				printf("Setting parent of %" PRId64 " to be %" PRId64 "\n", env->GetStateHash(thePath[x]),
					//					   env->GetStateHash(s));
					mClosed[thePath[x]] = s;
				}
			}
		}
	}
//	printf("%d tot %d inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
//	std::cout << "Final state:\n" << s << std::endl;

	thePath.resize(0);
	if (goalFound)
	{
		state parent;
		s = goal;
		//	std::cout << s << std::endl;
		do {
			thePath.push_back(s);
			parent = s;
			s = mClosed[s];
		} while (!(s == parent));
	}
	std::reverse(thePath.begin(), thePath.end());
	//	printf("Final depth: %d, Nodes Expanded %" PRId64 ", Exponential BF: %f\n", currDepth, nodesExpanded, pow(nodesExpanded, (double)1.0/currDepth));
}

//template <class state, class action, class environment>
//void BFS<state, action>::GetPath(environment *env,
//								 state from, state to,
//								 std::vector<action> &thePath)
//{
//	assert(!"not defined");
//}


#endif
