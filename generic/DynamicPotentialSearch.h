//
//  DynamicPotentialSearch.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/24/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef DynamicPotentialSearch_h
#define DynamicPotentialSearch_h


#include <iostream>
#include "FPUtil.h"
#include <unordered_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
#include "TemplateAStar.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"
#include <algorithm> // for vector reverse
#include "GenericSearchAlgorithm.h"
#include <unordered_map>
#include <map>

template<typename state>
class DPSData {
public:
	DPSData() {}
	DPSData(const state &theData, double gCost, double hCost, const state &par)
	:data(theData), g(gCost), h(hCost), parent(par), reopened(false) {}
	state data;
	double g;
	double h;
	state parent;
//	bool open;
	bool reopened;
};

/*
 * Dynamic Potential Search
 *
 * SoCS 2016
 *
 * This algorithm prioritizes states according to the minimum f-cost on open.
 * When the min f-cost is changed open must be re-sorted. This implementation
 * rebuilds the openQ which contains all states on open. This could be bucketed
 * for better performance.
 */
template <class state, class action, class environment>
class DynamicPotentialSearch {
public:
	DynamicPotentialSearch() { ResetNodeCount(); env = 0; weight=3; bound = 1.5; theHeuristic = 0; }
	virtual ~DynamicPotentialSearch() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
//	AStarOpenClosed<state, AStarCompare<state>> open;
	std::unordered_map<uint64_t, DPSData<state>> open;
	std::multimap<double, uint64_t, std::greater<double>> openQ;
	std::unordered_map<uint64_t, DPSData<state>> closed;
	typename std::unordered_map<uint64_t, DPSData<state>>::const_iterator iter;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
//	void AddAdditionalStartState(state& newState);
//	void AddAdditionalStartState(state& newState, double cost);
	
//	state CheckNextNode();
	void ExtractPathToStart(const state &node, std::vector<state> &thePath)
	{
		thePath.push_back(node);
		auto it = open.find(env->GetStateHash(node));
		if (it != open.end()) // found on open
		{
			const auto &i = it->second;
			if (i.parent == node)
				return;
			ExtractPathToStart(i.parent, thePath);
			return;
		}
		it = closed.find(env->GetStateHash(node));
		if (it != closed.end()) // found on closed
		{
			const auto &i = it->second;
			if (i.parent == node)
				return;
			ExtractPathToStart(i.parent, thePath);
			return;
		}
		assert(!"State not found on open or closed");
	}
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
//	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();

	double GetBestFMin();

	/* Internal iterator to allow getting the g- and h-cost of all items on open. Resets this to start over */
	void ResetIterator();
	/* Gets the g- and h-cost of the next item on open. \returns true if successful */
	bool GetNext(double &g, double &h);

	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
//	void LogFinalStats(StatCollection *) {}
	
//	void OpenGLDraw() const {};
	void Draw(Graphics::Display &d) const;
	
//	void SetWeight(double w) {weight = w;}
	void SetOptimalityBound(double w) {bound = w;}
	double GetOptimalityBound() {return bound;}
private:
	DPSData<state> *GetBestOnOpen();
	void RebuildOpenQ();

	uint64_t nodesTouched, nodesExpanded;
	
	std::vector<state> neighbors;
	environment *env;
	double bestSolution;
	double weight;
	double bound;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;
	//double fMin;
	std::map<double, int> fCostDistribution;
};

//static const bool verbose = false;

/**
 * Return the name of the algorithm.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment>
const char *DynamicPotentialSearch<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "DynamicPotentialSearch[%1.2f, %1.2f]", bound, weight);
	return name;
}

/**
 * Perform an A* search between two states.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @param thePath A vector of states which will contain an optimal path
 * between from and to when the function returns, if one exists.
 */
template <class state, class action, class environment>
void DynamicPotentialSearch<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	//discardcount=0;
	if (!InitializeSearch(_env, from, to, thePath))
    {
        return;
    }
    while (!DoSingleSearchStep(thePath))
    {
        if (10000000 <= nodesExpanded){
            //Terminate the search after 10 million node expansions.
            printf("%" PRId64 " nodes expanded, %" PRId64 " generated. ", nodesExpanded, nodesTouched);
            std::cout<<"DPS => Terminated.\n";
            break;
        }
    }
}

template <class state, class action, class environment>
void DynamicPotentialSearch<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
{
	std::vector<state> thePath;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	path.resize(0);
	while (!DoSingleSearchStep(thePath))
	{	}
	for (int x = 0; x < thePath.size()-1; x++)
	{
		path.push_back(_env->GetAction(thePath[x], thePath[x+1]));
	}
}


/**
 * Initialize the A* search
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @return TRUE if initialization was successful, FALSE otherwise
 */
template <class state, class action, class environment>
bool DynamicPotentialSearch<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	bestSolution = DBL_MAX;
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	open.clear();
	closed.clear();
	openQ.clear();
	ResetNodeCount();
	start = from;
	goal = to;
	fCostDistribution.clear();
	
	if (env->GoalTest(from, to)) //assumes that from and to are valid states
	{
		return false;
	}
	DPSData<state> next = {start, 0, theHeuristic->HCost(start, goal), start};
	open[env->GetStateHash(start)] = next;
	fCostDistribution[next.g+next.h] = 1;
	openQ.insert({(bound * GetBestFMin() - next.g)/next.h, env->GetStateHash(start)});
	return true;
}

template <class state, class action, class environment>
DPSData<state> *DynamicPotentialSearch<state,action,environment>::GetBestOnOpen()
{
	// old code for checking:
//	double fmin = GetBestFMin();
//	DPSData<state> *next = 0;
//	{
//		// get best priority
//		double bestP = 0;
//		//DPSData<state> *next = 0;
//		for (auto &item : open)
//		{
//			auto &i = item.second;
//			// (B × fmin − g(n))/h(n)
//			{
//				double pr = DBL_MAX;
//				if (i.h != 0)
//					pr = (bound * fmin - i.g)/i.h;
//				if (fgreater(pr, bestP))
//				{
//					bestP = pr;
//					next = &i;
//				}
//			}
//		}
//	}
	
//	// check if data structure is consistent with current BestFMin
//	for (auto o : openQ)
//	{
//		auto i = open.find(o.second);
//		if (i == open.end())
//			continue;
//		assert(o.first == (bound * fmin - i->second.g)/i->second.h);
//	}
	
	double fmin = GetBestFMin();
	assert(openQ.size() > 0);
	while (true)
	{
		uint64_t hash = (openQ.begin())->second;
		auto i = open.find(hash);
		while (true) // will break when valid entry is found
		{
			// Needs to be on open - loop until it is
			while (i == open.end())
			{
				openQ.erase(openQ.begin()); // no longer on open
				if (openQ.size() == 0)
				{
					assert(open.size() == 0);
					return 0;
				}
				hash = (openQ.begin())->second;
				i = open.find(hash);
			}
			// Needs to have correct priority - wrong priority means it's an old entry
			// since we aren't deleting states from openQ when they get updated cost
			if (!fequal((bound * fmin - i->second.g)/i->second.h, openQ.begin()->first))
			{
				openQ.erase(openQ.begin()); // no longer on open
				hash = (openQ.begin())->second;
				i = open.find(hash);
				continue;
			}
			break;
		}
		
//		// check if data structure is consistent with current BestFMin
//		for (auto o : openQ)
//		{
//			auto i = open.find(o.second);
//			if (i == open.end())
//				continue;
//			assert(o.first == (bound * fmin - i->second.g)/i->second.h);
//		}

//		assert(fequal((bound * fmin - next->g)/next->h,
//					  (bound * fmin - i->second.g)/i->second.h));
		return &(i->second);
	}
	return 0;
//	// get best priority
//	double bestP = 0;
//	DPSData<state> *next = 0;
//	for (auto &item : open)
//	{
//		auto &i = item.second;
//		// (B × fmin − g(n))/h(n)
//		{
//			double pr = DBL_MAX;
//			if (i.h != 0)
//				pr = (bound * fmin - i.g)/i.h;
//			if (fgreater(pr, bestP))
//			{
//				bestP = pr;
//				next = &i;
//			}
//		}
//	}
//	return next;
}

/**
 * Expand a single node.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param thePath will contain an optimal path from start to goal if the
 * function returns TRUE
 * @return TRUE if there is no path or if we have found the goal, FALSE
 * otherwise
 */
template <class state, class action, class environment>
bool DynamicPotentialSearch<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	double fmin = GetBestFMin();
	DPSData<state> *next = GetBestOnOpen();

	if (next == 0)
	{
		// no path found
		return true;
	}
	// Note: will move to closed at the end
	
	nodesExpanded++;
	if (env->GoalTest(next->data, goal))
	{
		ExtractPathToStart(next->data, thePath);
		return true;
	}
	
	env->GetSuccessors(next->data, neighbors);

	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		uint64_t hash = env->GetStateHash(neighbors[x]);
		double edgeCost = env->GCost(next->data, neighbors[x]);
		nodesTouched++;
		
		auto itemOpen = open.find(hash);
		auto itemClosed = closed.find(hash);

		if (itemOpen == open.end() && itemClosed == closed.end()) // not found
		{
			DPSData<state> n = {neighbors[x], next->g+edgeCost, theHeuristic->HCost(neighbors[x], goal), next->data};
			open[hash] = n;
			openQ.insert({(bound * fmin - n.g)/n.h, hash});
			fCostDistribution[n.g+n.h]++;
			continue;
		}
		if (itemOpen != open.end()) // found on open
		{
			auto &i = itemOpen->second;
			if (fless(next->g+edgeCost+i.h, i.g+i.h)) // found shorter path
			{
				fCostDistribution[i.g+i.h]--;
				i.parent = next->data;
				i.g = next->g+edgeCost;
				fCostDistribution[i.g+i.h]++;
				openQ.insert({(bound * fmin - i.g)/i.h, hash});
			}
		}
		else if (itemClosed != closed.end()) // found on closed
		{
			auto &i = itemClosed->second;
			if (fless(next->g+edgeCost+i.h, i.g+i.h)) // found shorter path
			{
				i.parent = next->data;
				i.g = next->g+edgeCost;
				i.reopened = true;
				open[hash] = i;
				openQ.insert({(bound * fmin - i.g)/i.h, hash});
				fCostDistribution[i.g+i.h]++;
				closed.erase(itemClosed);
			}
		}
	}
	// Expanded state moves to closed
	closed[env->GetStateHash(next->data)] = *next;
	fCostDistribution[next->g+next->h]--;
	open.erase(open.find(env->GetStateHash(next->data)));

	if (!fequal(fmin, GetBestFMin()))
		RebuildOpenQ();
	return false;
}
template <class state, class action, class environment>
void DynamicPotentialSearch<state, action,environment>::RebuildOpenQ()
{
	openQ.clear();
	double fmin = GetBestFMin();
	for (const auto &i : open)
		openQ.insert({(bound * fmin - i.second.g)/i.second.h,
			env->GetStateHash(i.second.data)});
}


template <class state, class action, class environment>
double DynamicPotentialSearch<state, action,environment>::GetBestFMin()
{
	if (fCostDistribution.size() == 0)
		return DBL_MAX;
	while (fCostDistribution.begin()->second <= 0)
	{
		assert(fCostDistribution.begin()->second >= 0);
		fCostDistribution.erase(fCostDistribution.begin());
		if (fCostDistribution.size() == 0)
			return DBL_MAX;
		//printf("up->\n");
	}
	return fCostDistribution.begin()->first; // lowest fcost
//	// get best f on open
//	for (const auto &item : open)
//	{
//		auto &i = item.second;
//		if (fless(i.g+i.h, fmin))
//			fmin = i.g+i.h;
//	}
//	//std::cout << "-->Best fmnin " << fmin << "\n";
//	return fmin;
}

template <class state, class action, class environment>
void DynamicPotentialSearch<state, action,environment>::ResetIterator()
{
	iter = open.begin();
}

template <class state, class action, class environment>
bool DynamicPotentialSearch<state, action,environment>::GetNext(double &g, double &h)
{
	if (iter == open.end())
		return false;
	g = iter->second.g;
	h = iter->second.h;
	iter++;
	return true;
}

template <class state, class action, class environment>
void DynamicPotentialSearch<state, action,environment>::Draw(Graphics::Display &d) const
{
	double transparency = 1.0;
	
	for (const auto &item : open)
	{
		const auto &i = item.second;
		if (i.reopened)
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(d, i.data);
		}
		else {
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(d, i.data);
		}
	}
	for (const auto &item : closed)
	{
		const auto &i = item.second;
		if (i.reopened)
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(d, i.data);
		}
		else {
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->Draw(d, i.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(d, goal);
}

#endif /* DynamicPotentialSearch_h */
