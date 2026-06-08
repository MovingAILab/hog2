/**
 * @file OldTemplateAStar.h
 * @package hog2
 * @brief A templated version of the original HOG's genericAstar.h
 * @author Nathan Sturtevant
 * @author Modified by Renee Jansen to work with templates for HOG2's 
 * SearchEnvironment
 * @date 3/22/06, modified 06/13/2007
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

#ifndef OldTemplateAStar_H
#define OldTemplateAStar_H

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
#include "OpenClosedList.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"

#include <algorithm> // for vector reverse

#include "GenericSearchAlgorithm.h"

//int discardcount;
namespace OldOldTemplateAStarUtil
{
	/**
	 * A search node class to use with hash maps
	 */
	template <class state>
	class SearchNode {
	public:
		SearchNode(state &curr, state &prev, double _fCost, double _gCost, uint64_t key)
		:fCost(_fCost), gCost(_gCost), currNode(curr), prevNode(prev), hashKey(key){}
		
		SearchNode(state &curr, uint64_t key)
		:fCost(0), gCost(0), currNode(curr), prevNode(curr), hashKey(key) {}
		
		// This probably needs to be fixed. Problem is you can't set the states to 
		// 0 - so how to initialize?
		SearchNode()
		:fCost(0), gCost(0), hashKey(0) {}
		
		double fCost;
		double gCost;
		state currNode;
		state prevNode;
		uint64_t hashKey;
	};
	
	template <class state>
	struct SearchNodeEqual {
		bool operator()(const SearchNode<state> &i1, const SearchNode<state> &i2) const
	{ return (i1.currNode == i2.currNode); } };
	
	template <class state>
	struct SearchNodeCompare {
		bool operator()(const SearchNode<state> &i1, const SearchNode<state> &i2) const
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} };
	
	template <class state>
	struct SearchNodeHash {
		size_t operator()(const SearchNode<state> &x) const
		{ return (size_t)(x.hashKey); }
	};
}


/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment>
class OldTemplateAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	OldTemplateAStar() { env = 0; useBPMX = false; radius = 4.0; stopAfterGoal = true; weight=1; useRadius=false; useOccupancyInfo=false; radEnv = 0;}
	virtual ~OldTemplateAStar() {}
	void GetPath(environment *env, state& from, state& to, std::vector<state> &thePath);
	
	void GetPath(environment *, state& , state& , std::vector<action> & ) { assert(false); };
	
	typedef OpenClosedList<OldOldTemplateAStarUtil::SearchNode<state>, OldOldTemplateAStarUtil::SearchNodeHash<state>,
		OldOldTemplateAStarUtil::SearchNodeEqual<state>, OldOldTemplateAStarUtil::SearchNodeCompare<state> > PQueue;
	
	typedef std::unordered_map<uint64_t, OldOldTemplateAStarUtil::SearchNode<state>, Hash64 > NodeLookupTable;
	
	PQueue openQueue;
	NodeLookupTable closedList; //openList
	state goal, start;
	
	typedef typename NodeLookupTable::const_iterator closedList_iterator;
	
	bool InitializeSearch(environment *env, state& from, state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	
	state CheckNextNode();
	void ExtractPathToStart(state& n, std::vector<state> &thePath);
	void DoAbstractSearch(){useOccupancyInfo = false; useRadius = false;}
	virtual const char *GetName();
	
	void PrintStats();
	//long GetNodesExpanded() { return nodesExpanded; }
	//long GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	int GetMemoryUsage();
	
	//closedList_iterator GetClosedListIter() const;
	void GetClosedListIter(closedList_iterator);
	bool ClosedListIterNext(closedList_iterator& it, state& next) const;
	bool GetClosedListGCost(state &val, double &gCost) const;
	
	void SetUseBPMX(bool use) { useBPMX = use; }
	bool GetUsingBPMX() { return useBPMX; }
	
	//state ClosedListIterNext(closedList_iterator&) const;
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void SetRadius(double rad) {radius = rad;}
	double GetRadius() { return radius; }
	
	void SetRadiusEnvironment(environment *e) {radEnv = e;}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
//	void OpenGLDraw() const;
	
	/** Use weighted A* and set the weight
	 *
	 * Use f = g + weight * h during search
	 *
	 * @author Renee Jansen
	 * @date 10/23/2007
	 */
	void SetWeight (double w){weight = w;}
private:
	uint64_t nodesTouched, nodesExpanded;
	bool GetNextNode(state &next);
	//state Node();
	void UpdateClosedNode(environment *env, state& currOpenNode, state& neighbor);
	void UpdateWeight(environment *env, state& currOpenNode, state& neighbor);
	void AddToOpenList(environment *env, state& currOpenNode, state& neighbor);
	
	std::vector<state> neighbors;
	environment *env;
	bool stopAfterGoal;
	
	double radius; // how far around do we consider other agents?
	double weight; 
	
	bool useOccupancyInfo;// = false;
	bool useRadius;// = false;
	bool firstRound;
	bool useBPMX;
	environment *radEnv;
};

using namespace OldOldTemplateAStarUtil;
static const bool verbose = false;
/**
 * Return the name of the algorithm. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment>
const char *OldTemplateAStar<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "OldTemplateAStar[]");
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
void OldTemplateAStar<state,action,environment>::GetPath(environment *_env, state& from, state& to, std::vector<state> &thePath)
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
bool OldTemplateAStar<state,action,environment>::InitializeSearch(environment *_env, state& from, state& to, std::vector<state> &thePath)
{
	thePath.resize(0);
	//if (useRadius)
	//std::cout<<"Using radius\n";
	firstRound = true;
	env = _env;
	if (!radEnv)
		radEnv = _env;
	closedList.clear();
	openQueue.reset();
	assert(openQueue.size() == 0);
	assert(closedList.size() == 0);
	nodesTouched = nodesExpanded = 0;
	start = from;
	goal = to;
	
	if ((from == to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	
	//SearchNode<state> first(env->heuristic(goal, start), 0, start, start);
	SearchNode<state> first(start, start, weight*env->HCost(start, goal), 0, env->GetStateHash(start));
	openQueue.Add(first);
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment>
void OldTemplateAStar<state,action,environment>::AddAdditionalStartState(state& newState)
{
	SearchNode<state> first(newState, newState, weight*env->HCost(goal, newState), 0,env->GetStateHash(newState));
	openQueue.Add(first);
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
bool OldTemplateAStar<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	state currentOpenNode; 
	
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}
	
	// get top of queue
	if (!GetNextNode(currentOpenNode))
	{
		printf("Oh no! No more open nodes!\n");
	}
	//env->OutputXY(currentOpenNode);
	//std::cout<<std::endl;
	if ((stopAfterGoal) && (env->GoalTest(currentOpenNode, goal)))
	{
		ExtractPathToStart(currentOpenNode, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end()); 
		//		closedList.clear();
		//		openQueue.reset();
		//		env = 0;
		return true;
	}
	
 	neighbors.resize(0);
 	env->GetSuccessors(currentOpenNode, neighbors);
	
	if (useBPMX) // propagate best child to parent
	{
		SearchNode<state> currNode = closedList[env->GetStateHash(currentOpenNode)];
		double oldf = currNode.fCost;
		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			double newh = env->HCost(neighbors[x], goal)-env->GCost(currentOpenNode, neighbors[x]);
			if (fgreater(newh, currNode.fCost-currNode.gCost))
				currNode.fCost = currNode.gCost + newh;
		}
		//		std::cout << "Updating parent (" << currNode.currNode << ") f-cost from " << oldf << " to " << currNode.fCost << std::endl;
		closedList[env->GetStateHash(currentOpenNode)].fCost = currNode.fCost;
	}
	//printf("Expanding %d\n", currentOpenNode);
	
	// iterate over all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		state neighbor = neighbors[x];
		
		if (closedList.find(env->GetStateHash(neighbor)) != closedList.end())
		{
			//UpdateClosedNode(env, currentOpenNode, neighbor);
		}
		else if (openQueue.IsIn(SearchNode<state>(neighbor, env->GetStateHash(neighbor))))
		{
			UpdateWeight(env, currentOpenNode, neighbor);
		}
		else if (useRadius && useOccupancyInfo && env->GetOccupancyInfo() && radEnv && (radEnv->HCost(start, neighbor) < radius) &&(env->GetOccupancyInfo()->GetStateOccupied(neighbor)) && ((!(radEnv->GoalTest(neighbor, goal)))))// || (currentOpenNode == start )) )
		{
			SearchNode<state> sn(neighbor, env->GetStateHash(neighbor));
			closedList[env->GetStateHash(neighbor)] = sn;
		}
		else {
			AddToOpenList(env, currentOpenNode, neighbor);
		}
		
		firstRound = false; 
	}
	
	if ((openQueue.size() == 0) && (stopAfterGoal == false))
	{
		ExtractPathToStart(currentOpenNode, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end()); 
		return true;
	}
	
	//	std::cout<<std::endl;
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
state OldTemplateAStar<state, action,environment>::CheckNextNode()
{
	return openQueue.top().currNode;
}

/**
 * Removes the top node from the open list  
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param state will contain the next state in the open list
 * @return TRUE if next contains a valid state, FALSE if there is no  more states in the
 * open queue 
 */
template <class state, class action, class environment>
bool OldTemplateAStar<state,action,environment>::GetNextNode(state &next)
{
	nodesExpanded++;
	if (openQueue.Empty())
		return false;
	SearchNode<state> it = openQueue.Remove();
	//if (it == openQueue.end())
	//	return false;
	next = it.currNode;
	//printf("h-cost\t%f\n", it.fCost-it.gCost);
	
	//	std::cout << "Current open node " << next << " f-cost: " << it.fCost << " g-cost: " << it.gCost <<
	//	" h-value: " << env->HCost(next, goal) << "/" << it.fCost-it.gCost << std::endl;
	
	closedList[env->GetStateHash(next)] = it;
	//	std::cout<<"Getting "<<it.gCost<<" ";
	return true;
}

/**
 * Check and update the weight of a closed node. 
 * @author Nathan Sturtevant
 * @date 11/10/08
 * 
 * @param currOpenNode The node that's currently being expanded
 * @param neighbor The node whose weight will be updated
 */
template <class state, class action, class environment>
void OldTemplateAStar<state,action,environment>::UpdateClosedNode(environment *e, state &currOpenNode, state &neighbor)
{
	//printf("Found in closed list!\n");
	SearchNode<state> prev = closedList[e->GetStateHash(neighbor)];
	//openQueue.find(SearchNode<state>(neighbor, e->GetStateHash(neighbor)));
	SearchNode<state> alt = closedList[e->GetStateHash(currOpenNode)];
	double edgeWeight = e->GCost(currOpenNode, neighbor);
	double altCost = alt.gCost+edgeWeight+(prev.fCost-prev.gCost);
	if (fgreater(prev.fCost, altCost))
	{
		//std::cout << "Reopening node " << neighbor << " setting parent to " << currOpenNode << std::endl;
		//printf("Reopening node %d setting parent to %d - %f vs. %f\n", neighbor, currOpenNode, prev.fCost, altCost);
		prev.fCost = altCost;
		prev.gCost = alt.gCost+edgeWeight;
	 	prev.prevNode = currOpenNode;
		// this is generally unneeded. But, in a search space where two
		// nodes may be "equal" but not identical, it is important.
		prev.currNode = neighbor;
		closedList.erase(e->GetStateHash(neighbor));
		assert(closedList.find(e->GetStateHash(neighbor)) == closedList.end());
		openQueue.Add(prev);
	}
}

/**
 * Update the weight of a node. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @param currOpenNode The node that's currently being expanded
 * @param neighbor The node whose weight will be updated
 */
template <class state, class action, class environment>
void OldTemplateAStar<state,action,environment>::UpdateWeight(environment *e, state &currOpenNode, state &neighbor)
{
	//printf("Found in open list!\n");
	SearchNode<state> prev = openQueue.find(SearchNode<state>(neighbor, e->GetStateHash(neighbor)));
	SearchNode<state> alt = closedList[e->GetStateHash(currOpenNode)];
	double edgeWeight = e->GCost(currOpenNode, neighbor);
	double altCost = alt.gCost+edgeWeight+(prev.fCost-prev.gCost);
	if (fgreater(prev.fCost, altCost))
	{
		//		std::cout << "Updating node " << neighbor << " setting parent to " << currOpenNode << std::endl;
		//printf("Resetting node %d setting parent to %d\n", neighbor, currOpenNode);
		prev.fCost = altCost;
		prev.gCost = alt.gCost+edgeWeight;
		prev.prevNode = currOpenNode;
		// this is generally unneeded. But, in a search space where two
		// nodes may be "equal" but not identical, it is important.
		prev.currNode = neighbor;
		openQueue.DecreaseKey(prev);
	}
}

/**
 * Add a node to the open list
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @param currOpenNode the state that's currently being expanded
 * @param neighbor the state to be added to the open list
 */
template <class state, class action,class environment>
void OldTemplateAStar<state, action,environment>::AddToOpenList(environment *e, state &currOpenNode, state &neighbor)
{
	//printf("Adding node %d setting parent to %d\n", neighbor, currOpenNode);
	//	std::cout << "Adding node " << neighbor << " setting parent to " << currOpenNode << std::endl;
	double edgeWeight = e->GCost(currOpenNode, neighbor);
	
	double oldfCost = closedList[e->GetStateHash(currOpenNode)].fCost;
	double oldgCost = closedList[e->GetStateHash(currOpenNode)].gCost;
	double gCost = oldgCost+edgeWeight;
	double fCost = gCost+weight*e->HCost(neighbor, goal);
	if (/*(useBPMX) && */(fgreater(oldfCost, fCost))) // pathmax rule
	{
		//		std::cout << "Adding node: (" << neighbor << ") f-cost from " << fCost << " to " << oldfCost
		//		<< " h-cost: " << fCost-gCost << " g-cost: " << gCost << " hash: " << e->GetStateHash(neighbor) << std::endl;
		fCost = oldfCost;
	}
	else {
		//		std::cout << "Adding node: (" << neighbor << ") f-cost " << fCost 
		//		<< " h-cost: " << fCost-gCost << " g-cost: " << gCost << " hash: " << e->GetStateHash(neighbor) << std::endl;
	}
	SearchNode<state> n(neighbor, currOpenNode, fCost, gCost, e->GetStateHash(neighbor));
	
	openQueue.Add(n);	
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
void OldTemplateAStar<state, action,environment>::ExtractPathToStart(state &goalNode,
																  std::vector<state> &thePath)
{
	SearchNode<state> n;
	if (closedList.find(env->GetStateHash(goalNode)) != closedList.end())
	{
		n = closedList[env->GetStateHash(goalNode)];
	}
	else n = openQueue.find(SearchNode<state>(goalNode, env->GetStateHash(goalNode)));
	
	do {
		//std::cout << "Extracting " << n.currNode << " with parent " << n.prevNode << std::endl;
		//printf("Extracting %d with parent %d\n", n.currNode, n.prevNode);
		thePath.push_back(n.currNode);
		if (closedList.find(env->GetStateHash(n.prevNode)) != closedList.end())
		{
			n = closedList[env->GetStateHash(n.prevNode)];
		}
		else {
			printf("No backward path found!\n");
			break;
		}
	} while (!(n.currNode == n.prevNode));
	thePath.push_back(n.currNode);
}

/**
 * A function that prints the number of states in the closed list and open
 * queue. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment>
void OldTemplateAStar<state, action,environment>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)closedList.size());
	printf("%u items in open queue\n", (unsigned int)openQueue.size());
}

/**
 * Return the amount of memory used by OldTemplateAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment>
int OldTemplateAStar<state, action,environment>::GetMemoryUsage()
{
	return closedList.size()+openQueue.size();
}

/**
 * Get an iterator for the closed list
 * @author Nathan Sturtevant
 * @date 06/13/07
 * 
 * @return An iterator pointing to the first node in the closed list
 */
template <class state, class action,class environment>
//std::unordered_map<state, OldOldTemplateAStarUtil::SearchNode<state> >::const_iterator
void OldTemplateAStar<state, action,environment>::GetClosedListIter(closedList_iterator) //const
{
	return closedList.begin();
}

/**
 * Get the next state in the closed list
 * @author Nathan Sturtevant
 * @date 06/13/07
 * 
 * @param it A closedList_iterator pointing at the current state in the closed 
 * list
 * @return The next state in the closed list. Returns UINT_MAX if there's no 
 * more states
 */
template <class state, class action, class environment>
bool OldTemplateAStar<state, action,environment>::ClosedListIterNext(closedList_iterator& it, state& next) const
{
	if (it == closedList.end())
		return false;
	next = (*it).first;
	it++;
	return true;
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
bool OldTemplateAStar<state, action,environment>::GetClosedListGCost(state &val, double &gCost) const
{
	if (closedList.find(env->GetStateHash(val)) != closedList.end())
	{
		gCost = closedList.find(env->GetStateHash(val))->second.gCost;
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
//void OldTemplateAStar<state, action,environment>::OpenGLDraw() const
//{
//	int x = 0;
//	if (env == 0)
//		return;
//	//env->SetColor(0.0, 0.0, 0.0, 0.15);
//	for (closedList_iterator it = closedList.begin(); it != closedList.end(); it++)
//	{
//		env->OpenGLDraw((*it).second.currNode);
//	}
//}

#endif
