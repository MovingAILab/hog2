/*
 *  IDAStarCR.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/6/19
 *
 */

#ifndef IDAStarCR_H
#define IDAStarCR_H

#include <iostream>
#include <functional>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"
#include "vectorCache.h"


typedef std::unordered_map<uint64_t, double> NodeHashTable;

template <class state, class action, int buckets = 50, bool verbose = true>
class IDAStarCR {
public:
	IDAStarCR() { storedHeuristic = false; }
	virtual ~IDAStarCR() {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void SetHeuristic(Heuristic<state> *heur) { heuristic = heur; if (heur != 0) storedHeuristic = true; else storedHeuristic = false; }
private:
	unsigned long long nodesExpanded, nodesTouched;
	
	//	void DoIteration(SearchEnvironment<state, action> *env,
	//					   state parent, state currState,
	//					   std::vector<state> &thePath, double bound, double g);
	void DoIteration(SearchEnvironment<state, action> *env,
					 action forbiddenAction, state &currState,
					 std::vector<action> &thePath, double bound, double g);
	state start, goal;
	double growth;
	double solutionCost;
	bool storedHeuristic;
	vectorCache<action> actCache;
	Heuristic<state> *heuristic;
	std::vector<action> solution;
	std::array<int, buckets> bucketCount;
};

template <class state, class action, int buckets, bool verbose>
void IDAStarCR<state, action, buckets, verbose>::GetPath(SearchEnvironment<state, action> *env,
												state from, state to,
												std::vector<action> &thePath)
{
	if (!storedHeuristic)
		heuristic = env;
	solutionCost = DBL_MAX;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	solution.resize(0);
	
	if (env->GoalTest(from, to))
		return;
	
	goal = to;
	start = from;
	std::vector<action> act;
	env->GetActions(from, act);
	uint64_t k = 1;
	
	double currBound = heuristic->HCost(from, to);
	while (solution.size() == 0)
	{
		bucketCount.fill(0);
		uint64_t oldNodes = nodesExpanded;
		if (verbose)
			printf("Starting iteration with bound %f; %llu expanded, %llu generated\n", currBound, nodesExpanded, nodesTouched);
		fflush(stdout);
		DoIteration(env, act[0], from, thePath, currBound, 0);
		if (solution.size() != 0)
			break;
		
		uint64_t newNodes = nodesExpanded-oldNodes;
		uint64_t total = bucketCount[0];
		double nextBound = currBound*(1.0+1.0/100.0);
		for (int i = 1; i < buckets; i++)
		{
			total+=bucketCount[i];
			if (bucketCount[i] > 0)
				nextBound = currBound*(1.0+static_cast<double>(i)/100.0);
			// This is the exponential bound for the paper. But, it would likely be better to grow based on
			// the last iteration, not the absolute size of a 2^k tree for iteration k.
			if (total >= (1<<k))
				break;
		}
		printf(" Ending  iteration next bound %f; %llu new exp  %llu predicted\n", nextBound, newNodes, total);
		currBound = nextBound;
		k++;
	}
	thePath = solution;
}

template <class state, class action, int buckets, bool verbose>
void IDAStarCR<state, action, buckets, verbose>::DoIteration(SearchEnvironment<state, action> *env,
													action forbiddenAction, state &currState,
													std::vector<action> &thePath, double bound, double g)
{
	double h = heuristic->HCost(currState, goal);
	if (fgreater(g+h, bound) || fgreater(g+h, solutionCost))
	{
		unsigned int index = ((g+h)/bound-1)*100;
		if (index < buckets)
			bucketCount[index]++;
		return;
	}
	// must do this after we check the f-cost bound
	if (env->GoalTest(currState, goal))
	{
		if (fless(env->GetPathLength(start, thePath), solutionCost))
		{
			solution = thePath;
			solutionCost = env->GetPathLength(start, thePath);
			printf("Improved solution to %1.5f\n", solutionCost);
		}
		return;
	}
	
	std::vector<action> &actions = *actCache.getItem();
	env->GetActions(currState, actions);
	nodesTouched += actions.size();
	nodesExpanded++;
	int depth = (int)thePath.size();
	
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		if ((depth != 0) && (actions[x] == forbiddenAction))
			continue;
		
		thePath.push_back(actions[x]);
		double edgeCost = env->GCost(currState, actions[x]);
		env->ApplyAction(currState, actions[x]);
		action a = actions[x];
		env->InvertAction(a);
		DoIteration(env, a, currState, thePath, bound, g+edgeCost);
		env->UndoAction(currState, actions[x]);
		thePath.pop_back();
	}
	actCache.returnItem(&actions);
}

#endif
