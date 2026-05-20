//
//  IncrementalDFID.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef IncrementalDFID_h
#define IncrementalDFID_h

#include "Heuristic.h"

#include <algorithm>

template <class state, class action>
class IncrementalIDA {
public:
	IncrementalIDA(double initialBound = 0) :bound(initialBound), initialBound(initialBound), nextBound(initialBound)
	{ ResetNodeCount(); previousBound = 0; }
	bool InitializeSearch(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
						  std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount()
	{
		nodesExpanded = nodesTouched = 0;
		newNodeCount = newNodesLastIteration = 0;
	}
	void Reset() { bound = initialBound; nextBound = initialBound; path.clear();
		history.clear(); search.clear(); ResetNodeCount(); previousBound = 0; }
//	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	state GetCurrentState() const { if (search.size() > 0) return search.back().currState; return start; }
	void GetCurrentPath(std::vector<state> &p) const
	{ p.clear(); for (auto &i : search) p.push_back(i.currState); }
	double GetCurrentFLimit() { return bound; }
	double GetNextFLimit() { return nextBound; }
	uint64_t GetNewNodesLastIteration() { return newNodesLastIteration; }
private:
	enum kSearchStatus {
		kGoingDown,
		kGoingAcross
	};
	struct currSearchState {
		state currState;
		kSearchStatus status;
		double pathCost;
		std::vector<state> succ;
	};
	std::vector<currSearchState> search;
	bool IterationComplete() { return search.size() == 0; }
	unsigned long nodesExpanded, nodesTouched;
	
	void SetupIteration(double cost);
	bool StepIteration();
	
	std::vector<std::pair<state, double>> history;
	std::vector<state> path;
	state start, goal;
	double previousBound;
	double bound;
	double initialBound;
	double nextBound;
	SearchEnvironment<state, action> *env;
	Heuristic<state> *h;
	std::vector<state> succ;
	uint64_t newNodeCount, newNodesLastIteration;
};

template <class state, class action>
void IncrementalIDA<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											Heuristic<state> *h, std::vector<state> &thePath)
{
	if (InitializeSearch(e, from, to, h, thePath))
		return;
	while (!DoSingleSearchStep(thePath))
	{}
}

template <class state, class action>
void IncrementalIDA<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
			 std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool IncrementalIDA<state, action>::InitializeSearch(SearchEnvironment<state, action> *e, state from, state to, Heuristic<state> *h,
													 std::vector<state> &thePath)
{
	Reset();
	if (from==to)
	{
		thePath.clear();
		thePath.push_back(from);
		return true;
	}
	start = from;
	goal = to;
	this->env = e;
	start = from;
	this->h = h;
	SetupIteration(h->HCost(start, goal));
	return false;
}

template <class state, class action>
void IncrementalIDA<state, action>::SetupIteration(double cost)
{
	search.resize(1);
	search.back().currState = start;
	search.back().status = kGoingDown;
	search.back().pathCost = 0;
	previousBound = bound;
	bound = cost;//nextBound;
	nextBound = -1;
	path.clear();
//	printf("Starting iteration bound %1.1f\n", bound);
	newNodesLastIteration = newNodeCount;
	newNodeCount = 0;
}

template <class state, class action>
bool IncrementalIDA<state, action>::StepIteration()
{
	if (env->GoalTest(search.back().currState, goal) && flesseq(search.back().pathCost, bound))
	{
		printf("Done!");
		path.resize(0);
		for (size_t x = 0; x < search.size(); x++)
			path.push_back(search[x].currState);
		return true;
	}
	
	if (search.back().status == kGoingDown)
	{
		double f = search.back().pathCost+h->HCost(search.back().currState, goal);
		// exceeded path cost bound
		if (fgreater(f, bound))
		{
//			printf("Above bound: %f/%f\n", bound, f);
			if (nextBound == -1)
				nextBound = f;
			else if (fless(f, nextBound))
				nextBound = f;
			
			search.pop_back();
			return false;
		}
		
		if (fgreater(f, previousBound) && flesseq(f, bound))
			newNodeCount++;
		
		// continue search
//		printf("Generating next set of successors\n");
		search.back().status = kGoingAcross;
		env->GetSuccessors(search.back().currState, search.back().succ);
		nodesExpanded++;
		for (size_t x = 0; x < search.back().succ.size(); x++)
		{
			if (search.size() > 1 && search.back().succ[x] == search[search.size()-2].currState)
			{
				search.back().succ.erase(search.back().succ.begin()+x);
				x--;
			}
		}
		
		// reverse and then handle them from back to front
		std::reverse(search.back().succ.begin(), search.back().succ.end());
		//return false;
	}
	
	if (search.back().status == kGoingAcross)
	{
		// no more succ to go down - go up
		if (search.back().succ.size() == 0)
		{
//			printf("Out of successors\n");
			search.pop_back();
			return false;
		}
		
//		printf("Taking next successors\n");
		// going down - generate next successor
		search.resize(search.size()+1);
		auto &s = search[search.size()-2];
		search.back().currState = s.succ.back();
		search.back().status = kGoingDown;
		search.back().pathCost = s.pathCost+env->GCost(s.currState, s.succ.back());
		s.succ.pop_back();
		return false;
	}
	assert(false);
	return false;

}


template <class state, class action>
bool IncrementalIDA<state, action>::DoSingleSearchStep(std::vector<state> &thePath)
{
	// starting new iteration
	if (IterationComplete())
	{
		SetupIteration(nextBound);
		// pause between iterations
		return false;
	}

	return StepIteration();
}

template <class state, class action>
void IncrementalIDA<state, action>::Draw(Graphics::Display &display) const
{
	for (int x = 1; x < search.size(); x++)
	{
		env->DrawLine(display, search[x-1].currState, search[x].currState, 10);
	}
}

//template <class state, class action>
//void IncrementalIDA<state, action>::OpenGLDraw()
//{
////	for (auto x : history)
////		env->OpenGLDraw(x.first);
//	for (int x = 1; x < search.size(); x++)
//		env->GLDrawLine(search[x-1].currState, search[x].currState, 10);
////	for (int x = 1; x < path.size(); x++)
////		env->GLDrawLine(path[x-1], path[x]);
//}


#endif /* IncrementalDFID_h */
