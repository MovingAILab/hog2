/*
 *  SFIDAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/16/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#ifndef SFIDASTAR_H
#define SFIDASTAR_H

#include <iostream>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"

typedef std::unordered_map<uint64_t, double> NodeHashTable;

template <class state, class action>
class SFIDAStar {
public:
	SFIDAStar() { }
	virtual ~SFIDAStar() {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
private:
	unsigned long nodesExpanded, nodesTouched;
	
	double DoIteration(SearchEnvironment<state, action> *env,
					   state &currState, state &goalState,
					   action forbiddenForwardAction, bool validForward,
					   action forbiddenBackAction, bool validBack,
					   std::vector<action> &thePath, double bound, double g);
	bool ShouldSearchForward1(SearchEnvironment<state, action> *env, state &currState, state &goalState,
							  double gCost, double bound);
	bool ShouldSearchForward2(SearchEnvironment<state, action> *env, state &currState, state &goalState,
							  double gCost, double bound);
	
	void UpdateNextBound(double currBound, double fCost);
	double nextBound;
};	

template <class state, class action>
void SFIDAStar<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<state> &thePath)
{
	assert(!"Not implemented -- too expensive for large state reps.");
}

template <class state, class action>
void SFIDAStar<state, action>::GetPath(SearchEnvironment<state, action> *env,
									 state from, state to,
									 std::vector<action> &thePath)
{
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	UpdateNextBound(0, env->HCost(from, to));
	std::vector<action> act;
	env->GetActions(from, act);
	while (thePath.size() == 0)
	{
//		printf("Starting iteration with bound %f\n", nextBound);
		DoIteration(env, from, to, act[0], false, act[0], false, thePath, nextBound, 0);
	}
}

template <class state, class action>
double SFIDAStar<state, action>::DoIteration(SearchEnvironment<state, action> *env,
											 state &currState, state &goalState,
											 action forbiddenForwardAction, bool validForward,
											 action forbiddenBackAction, bool validBack,
											 std::vector<action> &thePath, double bound, double g)
{
	nodesExpanded++;
	double h = env->HCost(currState, goalState);
	
	if (fgreater(g+h, bound))
	{
		UpdateNextBound(bound, g+h);
		//printf("Stopping at (%d, %d). g=%f h=%f\n", currState>>16, currState&0xFFFF, g, h);
		return h;
	}
	if (env->GoalTest(currState, goalState))
		return -1; // found goal
	
	if (ShouldSearchForward1(env, currState, goalState, g, bound))
	{
		std::vector<action> actions;
		env->GetActions(currState, actions);
		nodesTouched += actions.size();
		
		for (unsigned int x = 0; x < actions.size(); x++)
		{
			if (validForward && (actions[x] == forbiddenForwardAction))
				continue;
			
			thePath.push_back(actions[x]);
			
			double edgeCost = env->GCost(currState, actions[x]);
			env->ApplyAction(currState, actions[x]);
			env->InvertAction(actions[x]);
			double childH = DoIteration(env, currState, goalState, 
										actions[x], true,
										forbiddenBackAction, validBack,
										thePath,
										bound, g+edgeCost);
			env->ApplyAction(currState, actions[x]);
			if (fequal(childH, -1)) // found goal
				return -1;
			
			thePath.pop_back();
		}
	}
	else {
		std::vector<action> actions;
		env->GetActions(goalState, actions);
		nodesTouched += actions.size();
		
		for (unsigned int x = 0; x < actions.size(); x++)
		{
			if (validBack && (actions[x] == forbiddenBackAction))
				continue;
			
			thePath.push_back(actions[x]);
			
			double edgeCost = env->GCost(goalState, actions[x]);
			env->ApplyAction(goalState, actions[x]);
			env->InvertAction(actions[x]);
			double childH = DoIteration(env, currState, goalState, 
										forbiddenForwardAction, validForward,
										actions[x], true,
										thePath,
										bound, g+edgeCost);
			env->ApplyAction(goalState, actions[x]);
			if (fequal(childH, -1)) // found goal
				return -1;
			
			thePath.pop_back();
		}
	}
	return h;
}

template <class state, class action>
bool SFIDAStar<state, action>::ShouldSearchForward1(SearchEnvironment<state, action> *env, state &currState, state &goalState,
													double gCost, double bound)
{
	std::vector<action> actions;
//	std::vector<action> actions2;
//	double base = env->HCost(currState, goalState);
	//double limit = bound-(gCost+base);
	int forward = 0;
	int backward = 0;

	forward = env->GetNumSuccessors(currState);
	//	env->GetActions(currState, actions);
//	forward+=actions.size();
//	for (unsigned int x = 0; x < actions.size(); x++)
//	{
//		env->ApplyAction(currState, actions[x]);
//		env->InvertAction(actions[x]);
//
//		if (gCost + env->GCost(currState, actions[x]) + env->HCost(currState, goalState) > bound)
//			forward+=2;
//		if (gCost + env->GCost(currState, actions[x]) + env->HCost(currState, goalState) == bound)
//			forward++;
//		env->ApplyAction(currState, actions[x]);
//	}		

//	env->GetActions(goalState, actions);
//	backward += actions.size();
	backward = env->GetNumSuccessors(goalState);
//	for (unsigned int x = 0; x < actions.size(); x++)
//	{
//		env->ApplyAction(goalState, actions[x]);
//		env->InvertAction(actions[x]);
//
//		if (gCost + env->GCost(goalState, actions[x]) + env->HCost(goalState, currState) > bound)
//			backward+=2;
//		if (gCost + env->GCost(goalState, actions[x]) + env->HCost(goalState, currState) == bound)
//			backward+=1;
//		env->ApplyAction(goalState, actions[x]);
//	}
	if (forward < backward)
		return true;
	return false;
}

template <class state, class action>
bool SFIDAStar<state, action>::ShouldSearchForward2(SearchEnvironment<state, action> *env, state &currState, state &goalState,
													double gCost, double bound)
{
	std::vector<action> actions;
	std::vector<action> actions2;
//	double base = env->HCost(currState, goalState);
	//double limit = bound-(gCost+base);
	int forward = 0;
	int backward = 0;
	
	env->GetActions(currState, actions);
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		env->ApplyAction(currState, actions[x]);
		env->InvertAction(actions[x]);
		
		double firstCost = env->GCost(currState, actions[x]);
		env->GetActions(currState, actions2);
		for (unsigned int y = 0; y < actions2.size(); y++)
		{
			if (actions[x] == actions2[y]) continue;
			
			env->ApplyAction(currState, actions2[y]);
			if (firstCost + gCost + env->GCost(currState, actions[y]) + env->HCost(currState, goalState) > bound)
				forward+=2;
			if (firstCost + gCost + env->GCost(currState, actions[y]) + env->HCost(currState, goalState) == bound)
				forward++;
			env->InvertAction(actions2[y]);
			env->ApplyAction(currState, actions2[y]);
		}		
		env->ApplyAction(currState, actions[x]);
	}		
	
	env->GetActions(goalState, actions);
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		env->ApplyAction(goalState, actions[x]);
		env->InvertAction(actions[x]);
		
		//		env->GetActions(goalState, actions2);
		//		for (unsigned int y = 0; y < actions2.size(); y++)
		//		{
		//			if (actions[x] == actions2[y]) continue;
		//
		//			env->ApplyAction(goalState, actions2[y]);
		if (gCost + env->GCost(goalState, actions[x]) + env->HCost(goalState, currState) > bound)
			backward+=2;
		if (gCost + env->GCost(goalState, actions[x]) + env->HCost(goalState, currState) == bound)
			backward+=1;
		//			env->InvertAction(actions2[y]);
		//			env->ApplyAction(goalState, actions2[y]);
		//		}
		env->ApplyAction(goalState, actions[x]);
	}
	if (forward > backward)
		return true;
	return false;
}


template <class state, class action>
void SFIDAStar<state, action>::UpdateNextBound(double currBound, double fCost)
{
	if (!fgreater(nextBound, currBound))
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
	else if (fgreater(fCost, currBound) && fless(fCost, nextBound))
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
}


#endif

//template <class state, class action>
//class SearchEnvironment {
//public:
//	virtual ~SearchEnvironment() {}
//	virtual void GetSuccessors(const state &nodeID, std::vector<state> &neighbors) = 0;
//	virtual void GetActions(const state &nodeID, std::vector<action> &actions) = 0;
//	virtual action GetAction(state &s1, state &s2) = 0;
//	virtual void ApplyAction(state &s, action a) = 0;
//	
//	virtual double HCost(state &node1, state &node2) = 0;
//	virtual double GCost(state &node1, state &node2) = 0;
//	virtual bool GoalTest(state &node, state &goal) = 0;
//	
//	virtual uint64_t GetStateHash(const state &node) = 0;
//	virtual uint64_t GetActionHash(action act) = 0;
//};
