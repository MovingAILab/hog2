/**
 * @file PEAStar.h
 * @package hog2
 * @brief A templated version of the original HOG's genericAstar.h
 * @author Nathan Sturtevant
 * SearchEnvironment
 *
 * This file is part of HOG2.
 * HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
 * HOG2: http://code.google.com/p/hog2/
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG2; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef PEAStar_H
#define PEAStar_H

#define __STDC_CONSTANT_MACROS
#include <stdint.h>
// this is defined in stdint.h, but it doesn't always get defined correctly
// even when __STDC_CONSTANT_MACROS is defined before including stdint.h
// because stdint might be included elsewhere first...
#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#include "FPUtil.h"
#include <unordered_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"

#include <algorithm> // for vector reverse

#include "GenericSearchAlgorithm.h"

template <class state>
struct PEAStarCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		if (fequal(i1.g+i1.h, i2.g+i2.h))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(i1.g+i1.h, i2.g+i2.h));
	}
};

/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment>
class PEAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	PEAStar() { ResetNodeCount(); env = 0; stopAfterGoal = true; weight=1; reopenNodes = false; }
	virtual ~PEAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	
	AStarOpenClosed<state, PEAStarCompare<state> > openClosedList;
	//BucketOpenClosed<state, PEAStarCompare<state> > openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; openClosedList.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	//closedList_iterator GetClosedListIter() const;
	//	void GetClosedListIter(closedList_iterator);
	//	bool ClosedListIterNext(closedList_iterator& it, state& next) const;
	//bool GetClosedListGCost(state &val, double &gCost) const;
	bool GetClosedListGCost(const state &val, double &gCost) const;
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
		
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
//	void OpenGLDraw() const;
	
	void SetWeight(double w) {weight = w;}
private:
	std::vector<state> succ;
	uint64_t nodesTouched, nodesExpanded;
	environment *env;
	bool stopAfterGoal;
	
	double weight; 
	
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;
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
const char *PEAStar<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "PEAStar[]");
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
void PEAStar<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	//discardcount=0;
  	if (!InitializeSearch(_env, from, to, thePath))
  	{	
  		return;
  	}
  	while (!DoSingleSearchStep(thePath)) {}
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
bool PEAStar<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	theHeuristic = _env;
	thePath.resize(0);
	//if (useRadius)
	//std::cout<<"Using radius\n";
	env = _env;
	//	closedList.clear();
	//	openQueue.reset();
	//	assert(openQueue.size() == 0);
	//	assert(closedList.size() == 0);
	openClosedList.Reset();
	//openClosedList.Print();
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	
	openClosedList.AddOpenNode(start, env->GetStateHash(start), 0, weight*theHeuristic->HCost(start, goal));
	//openClosedList.Print();

	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment>
void PEAStar<state,action,environment>::AddAdditionalStartState(state& newState)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment>
void PEAStar<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
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
bool PEAStar<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (openClosedList.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}
	uint64_t nodeid = openClosedList.Peek();
	const state &currOpenNode = openClosedList.Lookup(nodeid).data;

	if (!openClosedList.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;

	if ((stopAfterGoal) && (env->GoalTest(currOpenNode, goal)))
	{
		ExtractPathToStartFromID(nodeid, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end()); 
		return true;
	}
	
	env->GetSuccessors(currOpenNode, succ);
	double fCost = openClosedList.Lookup(nodeid).h+openClosedList.Lookup(nodeid).g;
	bool keep = false;
	
	nodesTouched++;
	//double edgeCost;
	double nextBest = 0;
	uint64_t theID;
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		double edgeCost = env->GCost(currOpenNode, succ[x]);
		double newFCost = openClosedList.Lookup(nodeid).g+edgeCost+weight*theHeuristic->HCost(succ[x], goal);	
		dataLocation loc = openClosedList.Lookup(env->GetStateHash(succ[x]), theID);
		if ((loc == kClosedList) ||
			(loc == kOpenList && !fless(newFCost, openClosedList.Lookup(theID).g+openClosedList.Lookup(theID).h)))
		{
			// ignore!
		}
		else if (fgreater(newFCost, fCost))
		{
			keep = true;
			if (nextBest == 0)
				nextBest = newFCost;
			else if (newFCost < nextBest)
				nextBest = newFCost;
		}
	}
	if (!keep)
	{
		openClosedList.Close();
	}
	else {
		//printf("Increasing cost by %f\n", nextBest-fCost);
		openClosedList.Lookup(nodeid).h += nextBest-fCost;
		openClosedList.KeyChanged(nodeid);
	}

	
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		double edgeCost = env->GCost(currOpenNode, succ[x]);
		double newFCost = openClosedList.Lookup(nodeid).g+edgeCost+weight*theHeuristic->HCost(succ[x], goal);	
		if (!fequal(fCost, newFCost))
			continue;

		switch (openClosedList.Lookup(env->GetStateHash(succ[x]), theID))
		{
			case kClosedList:
				if (reopenNodes)
				{
					// do something here...
				}
				break;
			case kOpenList:
				//edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
				if (fless(openClosedList.Lookup(nodeid).g+edgeCost, openClosedList.Lookup(theID).g))
				{
					openClosedList.Lookup(theID).parentID = nodeid;
					openClosedList.Lookup(theID).g = openClosedList.Lookup(nodeid).g+edgeCost;
					openClosedList.KeyChanged(theID);
					//openClosedList.Print();
				}
				break;
			case kNotFound:
				
				if (fequal(newFCost, fCost))
				{
					openClosedList.AddOpenNode(succ[x],
											   env->GetStateHash(succ[x]),
											   openClosedList.Lookup(nodeid).g+edgeCost,
											   weight*theHeuristic->HCost(succ[x], goal),
											   nodeid);
				}
				//openClosedList.Print();
		}
	}
	return false;
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue). 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment>
state PEAStar<state, action,environment>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}


/**
 * Get the path from a goal state to the start state 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment>
void PEAStar<state, action,environment>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

/**
 * A function that prints the number of states in the closed list and open
 * queue. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment>
void PEAStar<state, action,environment>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by PEAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment>
int PEAStar<state, action,environment>::GetMemoryUsage()
{
	return openClosedList.size();
}

/**
 * Get state from the closed list
 * @author Nathan Sturtevant
 * @date 10/09/07
 * 
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * more states
 */
template <class state, class action, class environment>
bool PEAStar<state, action,environment>::GetClosedListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kClosedList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 * 
 */
//template <class state, class action, class environment>
//void PEAStar<state, action,environment>::OpenGLDraw() const
//{
//	double transparency = 1.0;
//	if (openClosedList.size() == 0)
//		return;
//	uint64_t top = -1;
//	if (openClosedList.OpenSize() > 0)
//		top = openClosedList.Peek();
//	for (unsigned int x = 0; x < openClosedList.size(); x++)
//	{
//		const AStarOpenClosedData<state> &data = openClosedList.Lookat(x);
//		if (x == top)
//		{
//			env->SetColor(1.0, 1.0, 0.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
//		if ((data.where == kOpenList) && (data.reopened))
//		{
//			env->SetColor(0.0, 0.5, 0.5, transparency);
//			env->OpenGLDraw(data.data);
//		}
//		else if (data.where == kOpenList) 
//		{
//			env->SetColor(0.0, 1.0, 0.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
//		else if ((data.where == kClosedList) && (data.reopened))
//		{
//			env->SetColor(0.5, 0.0, 0.5, transparency);
//			env->OpenGLDraw(data.data);
//		}
//		else if (data.where == kClosedList)
//		{
//			env->SetColor(1.0, 0.0, 0.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
//	}
//}

#endif
