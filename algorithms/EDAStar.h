/*
 *  EDAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/6/19
 *
 */

#ifndef EDASTAR_H
#define EDASTAR_H

#include <iostream>
#include <functional>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"
#include "vectorCache.h"


typedef std::unordered_map<uint64_t, double> NodeHashTable;

template <class state, class action, bool verbose = true>
class EDAStar {
public:
	EDAStar(double growthFactor):growth(growthFactor) { storedHeuristic = false; }
	virtual ~EDAStar() {}
	//	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
	//							 std::vector<state> &thePath);
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
};

//template <class state, class action, bool verbose>
//void EDAStar<state, action, verbose>::GetPath(SearchEnvironment<state, action> *env,
//									 state from, state to,
//									 std::vector<state> &thePath)
//{
//	if (!storedHeuristic)
//		heuristic = env;
//	solutionCost = DBL_MAX;
//	nodesExpanded = nodesTouched = 0;
//	thePath.resize(0);
//	goal = to;
//	thePath.push_back(from);
//	double currBound = heuristic->HCost(from, to);
//
//	while (true) //thePath.size() == 0)
//	{
//		if (verbose)
//			printf("Starting iteration with bound %f\n", currBound);
//		if (DoIteration(env, from, from, thePath, currBound, 0) == 0)
//			break;
//		currBound *= growth;
//	}
//}

template <class state, class action, bool verbose>
void EDAStar<state, action, verbose>::GetPath(SearchEnvironment<state, action> *env,
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

	double currBound = heuristic->HCost(from, to);
	while (solution.size() == 0)
	{
		if (verbose)
			printf("Starting iteration with bound %f; %llu expanded, %llu generated\n", currBound, nodesExpanded, nodesTouched);
		fflush(stdout);
		DoIteration(env, act[0], from, thePath, currBound, 0);
		currBound *= growth;
	}
	thePath = solution;
}

template <class state, class action, bool verbose>
void EDAStar<state, action, verbose>::DoIteration(SearchEnvironment<state, action> *env,
													action forbiddenAction, state &currState,
													std::vector<action> &thePath, double bound, double g)
{
	double h = heuristic->HCost(currState, goal);
	if (fgreater(g+h, bound) || fgreater(g+h, solutionCost))
	{
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
