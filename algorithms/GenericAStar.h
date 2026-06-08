/**
 * @file GenericAStar.h
 * @package hog2
 * @brief A* designed to work on any environment. Can have duplicate hash values. Requires hash for state in std namespace.
 * @author Nathan Sturtevant
 * SearchEnvironment
 * @date 07/10/23
 *
 */

#ifndef GenericAStar_H
#define GenericAStar_H

#include <iostream>
#include "Constraint.h"
#include "FPUtil.h"
#include <unordered_map>
#include "Graphics.h"
//#include "float.h"
#include <algorithm> // for vector reverse
//#include "GenericSearchAlgorithm.h"
//#include "TemplateAStar.h" // For comparison functions

template <class state>
struct GenericAStarCompare {
	// returns true if i2 is preferred over i1
	bool operator()(const GenericAStarData<state> &i1, const GenericAStarData<state> &i2) const
	{
		if (fequal(i1.f, i2.f))
		{
			return (fless(i1.g, i2.g));
		}
		return fgreater(i1.f, i2.f);
	}
};

/**
 * A* that works on any class, even if the hash function has collisions. (TemplateAStar requires unique hash values.)
 */
template <class state, class action, class environment, class openList = GenericAStarOpenClosed<state, GenericAStarCompare<state>, GenericAStarData<state>> >
class GenericAStar {
public:
	GenericAStar() {
		ResetNodeCount(); env = 0; useBPMX = 0; stopAfterGoal = true; weight=1; reopenNodes = false; theHeuristic = 0; directed = false;
		theConstraint = 0;
		phi = [](double h, double g){ return g+h; };
	}
	virtual ~GenericAStar() {}
	void GetPath(environment *env, Heuristic<state> *, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, Heuristic<state> *, const state&, const state&, std::vector<action> & );
	
	openList openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, Heuristic<state> *, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(const state& newState);
	void AddAdditionalStartState(const state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{
		thePath.clear();
		uint64_t theID;
		if (openClosedList.Lookup(env->GetStateHash(node), theID) != kNotFound)
			ExtractPathToStartFromID(theID, thePath);
	}
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	
	
	bool GetClosedListGCost(const state &val, double &gCost) const;
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetHCost(const state &val, double &hCost) const;
	bool GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &);
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedDataWithF<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedDataWithF<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key); }
	
	void SetUseBPMX(int depth) { useBPMX = depth; if (depth) reopenNodes = true; }
	int GetUsingBPMX() { return useBPMX; }

	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }

	// Only necessary for BPMX computation
	void SetDirected(bool d) { directed = d; }
	void SetConstraint(Constraint<state> *c) { theConstraint = c; }

	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const;
	void LogFinalStats(StatCollection *) {}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
	void FullBPMX(uint64_t nodeID, int distance);
	
	void Draw(Graphics::Display &disp) const;
	
	/** Setting this function */
	void SetPhi(std::function<double(double, double)> p)
	{
		phi = p;
	}
	double Phi(double h, double g)
	{
		return phi(h, g);
	}
	void SetWeight(double w)
	{
		weight = w;
		phi = [=](double h, double g){ return g+weight*h; };
	}
	double GetWeight() { return weight; }
private:
	uint64_t nodesTouched, nodesExpanded;
	
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	environment *env;
	bool stopAfterGoal;
	
	double goalFCost;
	double weight;
	std::function<double(double, double)> phi;
	bool directed;
	int useBPMX;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
	Heuristic<state> *theHeuristic;
	Constraint<state> *theConstraint;
};

/**
 * Return the name of the algorithm. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment, class openList>
const char *GenericAStar<state,action,environment,openList>::GetName()
{
	static char name[32];
	sprintf(name, "GenericAStar[]");
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
template <class state, class action, class environment, class openList>
void GenericAStar<state,action,environment,openList>::GetPath(environment *_env, Heuristic<state> *h, const state& from, const state& to, std::vector<state> &thePath)
{
	if (!InitializeSearch(_env, h, from, to, thePath))
  	{	
  		return;
  	}
  	while (!DoSingleSearchStep(thePath))
	{
//		if (0 == nodesExpanded%100000)
//			printf("%" PRId64 " nodes expanded, %" PRId64 " generated\n", nodesExpanded, nodesTouched);
	}
}

template <class state, class action, class environment, class openList>
void GenericAStar<state,action,environment,openList>::GetPath(environment *_env, Heuristic<state> *h, const state& from, const state& to, std::vector<action> &path)
{
	std::vector<state> thePath;
	if (!InitializeSearch(_env, h, from, to, thePath))
	{
		return;
	}
	path.resize(0);
	while (!DoSingleSearchStep(thePath))
	{
	}
	for (size_t x = 0; x < thePath.size()-1; x++)
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
template <class state, class action, class environment, class openList>
bool GenericAStar<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	openClosedList.Reset(env->GetMaxHash());
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	
	double h = theHeuristic->HCost(start, goal);
	openClosedList.AddOpenNode(start, env->GetStateHash(start), phi(h, 0), 0, h);
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment, class openList>
void GenericAStar<state,action,environment,openList>::AddAdditionalStartState(const state& newState)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), phi(h, 0), 0, h);
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment, class openList>
void GenericAStar<state,action,environment,openList>::AddAdditionalStartState(const state& newState, double cost)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), phi(h, cost), cost, h);
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
template <class state, class action, class environment, class openList>
bool GenericAStar<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath)
{
	// Simple code:
	// Check if open empty
	// Get next
	// Check if goal
	// Add each successor to open/closed
	
	// BPMX is more complicated:
	// (1) Lookup child h-costs
	// (2) Parent is max of children-edge
	// (3) Children get max of parent-edge
	
	// q: can we do this more automatically while keeping efficiency? (not really)
	
	// 1. turn state into reference
	
	if (openClosedList.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}
	uint64_t nodeid = openClosedList.Close();
//	if (openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h > lastF)
//	{ lastF = openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h;
//		//printf("Updated limit to %f\n", lastF);
//	}
	if (!openClosedList.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;

	if ((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
	{
		ExtractPathToStartFromID(nodeid, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end()); 
		goalFCost = openClosedList.Lookup(nodeid).f;// + openClosedList.Lookup(nodeid).h;
		return true;
	}
	
 	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	//std::cout << "Expanding: " << env->GetStateHash(openClosedList.Lookup(nodeid).data) << " with f:";
	//std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
 	env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
	double bestH = openClosedList.Lookup(nodeid).h;
	double lowHC = DBL_MAX;
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
		if (useBPMX)
		{
			if (neighborLoc.back() != kNotFound)
			{
				if (!directed)
					bestH = std::max(bestH, openClosedList.Lookup(theID).h-edgeCosts.back());
				lowHC = std::min(lowHC, openClosedList.Lookup(theID).h+edgeCosts.back());
			}
			else {
				double tmpH = theHeuristic->HCost(neighbors[x], goal);
				if (!directed)
					bestH = std::max(bestH, tmpH-edgeCosts.back());
				lowHC = std::min(lowHC, tmpH+edgeCosts.back());
			}
		}
	}
	
	if (useBPMX) // propagate best child to parent
	{
		if (!directed)
			openClosedList.Lookup(nodeid).h = std::max(openClosedList.Lookup(nodeid).h, bestH);
		openClosedList.Lookup(nodeid).h = std::max(openClosedList.Lookup(nodeid).h, lowHC);
		openClosedList.Lookup(nodeid).f = phi(openClosedList.Lookup(nodeid).h, openClosedList.Lookup(nodeid).g);
	}
	
	// iterate again updating costs and writing out to memory
	for (size_t x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		//std::cout << "looking at child with hash : " << env->GetStateHash(neighbors[x]) << "and g-cost"<<openClosedList.Lookup(nodeid).g+edgeCosts[x]<<std::endl;
		if (theConstraint &&
			theConstraint->ShouldNotGenerate(start, openClosedList.Lookup(nodeid).data, neighbors[x],
											 openClosedList.Lookup(nodeid).g+edgeCosts[x], goal))
			continue;

		switch (neighborLoc[x])
		{
			case kClosedList:
				if (useBPMX) // propagate parent to child - do this before potentially re-opening
				{
					if (fless(openClosedList.Lookup(neighborID[x]).h, bestH-edgeCosts[x]))
					{
						auto &i = openClosedList.Lookup(neighborID[x]);
						i.h = bestH-edgeCosts[x];
						i.f = phi(i.h, i.g);
						if (useBPMX > 1) FullBPMX(neighborID[x], useBPMX-1);
					}
				}
				if (reopenNodes)
				{
					if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
					{
						auto &i = openClosedList.Lookup(neighborID[x]);
						i.parentID = nodeid;
						i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
						i.f = phi(i.h, i.g);
						openClosedList.Reopen(neighborID[x]);
						// This line isn't normally needed, but in some state spaces we might have
						// equality but different meta information, so we need to make sure that the
						// meta information is also copied, since this is the most generic A* implementation
						i.data = neighbors[x];
					}
				}
				break;
			case kOpenList:
				//edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
				if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
				{
					auto &i = openClosedList.Lookup(neighborID[x]);
					i.parentID = nodeid;
					i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
					i.f = phi(i.h, i.g);
					// This line isn't normally needed, but in some state spaces we might have
					// equality but different meta information, so we need to make sure that the
					// meta information is also copied, since this is the most generic A* implementation
					i.data = neighbors[x];
					openClosedList.KeyChanged(neighborID[x]);
//					std::cout << " Reducing cost to " << openClosedList.Lookup(nodeid).g+edgeCosts[x] << "\n";
					// TODO: unify the KeyChanged calls.
				}
				else {
//					std::cout << " no cheaper \n";
				}
				if (useBPMX) // propagate best child to parent
				{
					if (fgreater(bestH-edgeCosts[x], openClosedList.Lookup(neighborID[x]).h))
					{
						auto &i = openClosedList.Lookup(neighborID[x]);
						i.h = std::max(i.h, bestH-edgeCosts[x]);
						i.f = phi(i.h, i.g);
						openClosedList.KeyChanged(neighborID[x]);
					}
				}
				break;
			case kNotFound:
				{ // add node to open list
					//double edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
//					std::cout << " adding to open ";
//					std::cout << double(theHeuristic->HCost(neighbors[x], goal)+openClosedList.Lookup(nodeid).g+edgeCosts[x]);
//					std::cout << " \n";
					double h = theHeuristic->HCost(neighbors[x], goal);
					if (useBPMX)
					{
						h = std::max(h, openClosedList.Lookup(nodeid).h-edgeCosts[x]);
						openClosedList.AddOpenNode(neighbors[x],
												   env->GetStateHash(neighbors[x]),
												   phi(std::max(h, openClosedList.Lookup(nodeid).h-edgeCosts[x]), openClosedList.Lookup(nodeid).g+edgeCosts[x]),
												   openClosedList.Lookup(nodeid).g+edgeCosts[x],
												   h,
												   nodeid);
					}
					else {
						openClosedList.AddOpenNode(neighbors[x],
												   env->GetStateHash(neighbors[x]),
												   phi(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
												   openClosedList.Lookup(nodeid).g+edgeCosts[x],
												   h,
												   nodeid);
					}
//					if (loc == -1)
//					{ // duplicate edges
//						neighborLoc[x] = kOpenList;
//						x--;
//					}
				}
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
template <class state, class action, class environment, class openList>
state GenericAStar<state, action,environment,openList>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}

/**
 * Perform a full bpmx propagation
 * @author Nathan Sturtevant
 * @date 6/9/9
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment, class openList>
void GenericAStar<state, action,environment,openList>::FullBPMX(uint64_t nodeID, int distance)
{
	if (distance <= 0)
		return;
	
	nodesExpanded++;
	std::vector<state> succ;
 	env->GetSuccessors(openClosedList.Lookup(nodeID).data, succ);
	double parentH = openClosedList.Lookup(nodeID).h;
	
	// load all the children and push parent heuristic value to children
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		uint64_t theID;
		dataLocation loc = openClosedList.Lookup(env->GetStateHash(succ[x]), theID);
		double edgeCost = env->GCost(openClosedList.Lookup(nodeID).data, succ[x]);
		double newHCost = parentH-edgeCost;
		
		switch (loc)
		{
			case kClosedList:
			{
				if (fgreater(newHCost, openClosedList.Lookup(theID).h))
				{
					openClosedList.Lookup(theID).h = newHCost;
					FullBPMX(theID, distance-1);
				}
			}
			case kOpenList:
			{
				if (fgreater(newHCost, openClosedList.Lookup(theID).h))
				{
					openClosedList.Lookup(theID).h = newHCost;
					openClosedList.KeyChanged(theID);
				}
			}
			case kNotFound: break;
		}
	}
}


/**
 * Get the path from a goal state to the start state 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment,class openList>
void GenericAStar<state, action,environment,openList>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

template <class state, class action,class environment,class openList>
const state &GenericAStar<state, action,environment,openList>::GetParent(const state &s)
{
	uint64_t theID;
	openClosedList.Lookup(env->GetStateHash(s), theID);
	theID = openClosedList.Lookup(theID).parentID;
	return openClosedList.Lookup(theID).data;
}

template <class state, class action, class environment, class openList>
uint64_t GenericAStar<state, action,environment,openList>::GetNecessaryExpansions() const
{
	uint64_t n = 0;
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (fless(data.g + data.h, goalFCost))
			n++;
	}
	return n;
}


/**
 * A function that prints the number of states in the closed list and open
 * queue. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment, class openList>
void GenericAStar<state, action,environment,openList>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Get state from the closed list
 * @author Nathan Sturtevant
 * @date 10/09/07
 * 
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * the states
 */
template <class state, class action, class environment, class openList>
bool GenericAStar<state, action,environment,openList>::GetClosedListGCost(const state &val, double &gCost) const
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

template <class state, class action, class environment, class openList>
bool GenericAStar<state, action,environment,openList>::GetOpenListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool GenericAStar<state, action,environment,openList>::GetHCost(const state &val, double &hCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc != kNotFound)
	{
		hCost = openClosedList.Lookat(theID).h;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool GenericAStar<state, action,environment,openList>::GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &result)
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(s), theID);
	if (loc == kClosedList)
	{
		result = openClosedList.Lookat(theID);
		return true;
	}
	return false;

}


/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 7/12/16
 *
 */
template <class state, class action, class environment, class openList>
void GenericAStar<state, action,environment,openList>::Draw(Graphics::Display &disp) const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kClosedList)
		{
			//			if (top != -1)
			//			{
			//				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
			//			}
			//			else {
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			//			}
			env->Draw(disp, data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(disp, goal);
}



#endif
