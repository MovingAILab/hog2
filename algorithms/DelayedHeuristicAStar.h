/**
 * @file DelayedHeuristicAStar.h
 * @package hog2
 * @brief This is the A* used in Doran Nadav's MSc Thesis (BGU 2020-2021) The original version was in Python
 * @author Nathan Sturtevant
 * SearchEnvironment
 * @date Jaunary 7, 2022
 *
 */

#ifndef DelayedHeuristicAStar_H
#define DelayedHeuristicAStar_H

// Borrow most data structures from A*
#include "TemplateAStar.h"


template <class state, class Environment>
class HeuristicLookupBuffer {
public:
	HeuristicLookupBuffer()
	{
		e = 0;
	}
	void Reset(Environment *env, const state &goal, unsigned int nodeLimit)
	{
		e = env;
		this->goal = goal;
		this->nodeLimit = nodeLimit;
	}
	
	bool HitNodeLimit()
	{
		return states.size() >= nodeLimit;
	}
	
	void Add(state &s)
	{
		states.push_back(s);
	}
	
	const std::vector<int> &Evaluate()
	{
		results.resize(states.size());
		for (int x = 0; x < states.size(); x++)
			results[x] = e->HCost(states[x], goal);
		states.resize(0);
		return results;
	}
private:
	Environment *e;
	state goal;
	std::vector<state> states;
	std::vector<int> results;
	int nodeLimit;
};

 
/**
 * A templated version of A*, based on TemplateAStar, which delays heuristic lookups as long as possible and batches them.
 */
template <class state, class action, class environment, class batchHeuristic = HeuristicLookupBuffer<state, environment>, class openList = AStarOpenClosed<state, AStarCompareWithF<state>, AStarOpenClosedDataWithF<state>> >
class DelayedHeuristicAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	DelayedHeuristicAStar(int batchLookupSize)
	:batch(), batchLookupSize(batchLookupSize)
	{
		ResetNodeCount(); env = 0; stopAfterGoal = true; weight=1; reopenNodes = false; theHeuristic = 0; directed = false;
		theConstraint = 0;
		phi = [](double h, double g){ return g+h; };
	}
	virtual ~DelayedHeuristicAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state&, const state&, std::vector<action> & );
	
	openList openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
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
	int GetMemoryUsage();
	
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
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }

	// Only necessary for BPMX computation
	void SetDirected(bool d) { directed = d; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	void SetConstraint(Constraint<state> *c) { theConstraint = c; }

	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const;
	void LogFinalStats(StatCollection *) {}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
	void FullBPMX(uint64_t nodeID, int distance);
	
//	void OpenGLDraw() const;
	void Draw(Graphics::Display &disp) const;
	std::string SVGDraw() const;
	std::string SVGDrawDetailed() const;
	
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
	void HandleBatchedStates();
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
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
	Heuristic<state> *theHeuristic;
	Constraint<state> *theConstraint;
	struct tempData {
//		tempData(state s, uint64_t hash, double g, uint64_t parent)
//		:s(s), hash(hash), g(g), parent(parent) {}
		state s;
		uint64_t hash;
		double g;
		uint64_t parent;
	};
	std::vector<tempData> delayedStates;
	double currentCostLimit;
	batchHeuristic batch;
	int batchLookupSize;
};

/**
 * Return the name of the algorithm. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment, class batchHeuristic, class openList>
const char *DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetName()
{
	static char name[32];
	sprintf(name, "DelayedHeuristicAStar[]");
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
template <class state, class action, class environment, class batchHeuristic, class openList>
void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (!InitializeSearch(_env, from, to, thePath))
  	{	
  		return;
  	}
  	while (!DoSingleSearchStep(thePath))
	{
//		if (0 == nodesExpanded%100000)
//			printf("%" PRId64 " nodes expanded, %" PRId64 " generated\n", nodesExpanded, nodesTouched);
	}
}

template <class state, class action, class environment, class batchHeuristic, class openList>
void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
{
	std::vector<state> thePath;
	if (!InitializeSearch(_env, from, to, thePath))
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
template <class state, class action, class environment, class batchHeuristic, class openList>
bool DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	openClosedList.Reset(env->GetMaxHash());
	ResetNodeCount();
	start = from;
	goal = to;
	
	currentCostLimit = 0;
	delayedStates.resize(0);
	batch.Reset(_env, to, batchLookupSize);

	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}

	// No purpose in looking up the heuristic of the start state.
	// It is never used.
	double h = 0;//theHeuristic->HCost(start, goal);
	openClosedList.AddOpenNode(start, env->GetStateHash(start), phi(h, 0), 0, h);
	
	return true;
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
template <class state, class action, class environment, class batchHeuristic, class openList>
bool DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (openClosedList.OpenSize() == 0)
	{
		HandleBatchedStates();
	}
	if (openClosedList.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}
	if (fgreater(openClosedList.Lookup(openClosedList.Peek()).f, currentCostLimit))
		HandleBatchedStates();

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
	
//	std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data << " with f:";
//	std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
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
	}
	
	// iterate again updating costs and writing out to memory
	for (size_t x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;

		if (theConstraint &&
			theConstraint->ShouldNotGenerate(start, openClosedList.Lookup(nodeid).data, neighbors[x],
											 openClosedList.Lookup(nodeid).g+edgeCosts[x], goal))
			continue;

		switch (neighborLoc[x])
		{
			case kClosedList:
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
			case kNotFound:
				{ // add node to open list
					//double edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
//					std::cout << " adding to open ";
//					std::cout << double(theHeuristic->HCost(neighbors[x], goal)+openClosedList.Lookup(nodeid).g+edgeCosts[x]);
//					std::cout << " \n";
					delayedStates.push_back({neighbors[x],
						env->GetStateHash(neighbors[x]),
						openClosedList.Lookup(nodeid).g+edgeCosts[x],
						nodeid});
					batch.Add(neighbors[x]);
					if (batch.HitNodeLimit())
						HandleBatchedStates();
//					double h = theHeuristic->HCost(neighbors[x], goal);
//					openClosedList.AddOpenNode(neighbors[x],
//											   env->GetStateHash(neighbors[x]),
//											   phi(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
//											   openClosedList.Lookup(nodeid).g+edgeCosts[x],
//											   h,
//											   nodeid);
				}
		}
	}
		
	return false;
}

template <class state, class action, class environment, class batchHeuristic, class openList>
void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::HandleBatchedStates()
{
	if (delayedStates.size() == 0)
		return;
	auto vec = batch.Evaluate();
	for (int x = 0; x < delayedStates.size(); x++)
	{
		double h = vec.at(x);
		openClosedList.AddOpenNode(delayedStates[x].s,
								   delayedStates[x].hash,
								   phi(h, delayedStates[x].g),
								   delayedStates[x].g,
								   h,
								   delayedStates[x].parent);
	}
	delayedStates.resize(0);
	currentCostLimit = openClosedList.Lookup(openClosedList.Peek()).f;
//	batch.Reset(_env, to, batchLookupSize);
//	batch.
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue). 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment, class batchHeuristic, class openList>
state DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::CheckNextNode()
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
template <class state, class action, class environment, class batchHeuristic, class openList>
void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::FullBPMX(uint64_t nodeID, int distance)
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
template <class state, class action, class environment, class batchHeuristic, class openList>
void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

template <class state, class action, class environment, class batchHeuristic, class openList>
const state &DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetParent(const state &s)
{
	uint64_t theID;
	openClosedList.Lookup(env->GetStateHash(s), theID);
	theID = openClosedList.Lookup(theID).parentID;
	return openClosedList.Lookup(theID).data;
}

template <class state, class action, class environment, class batchHeuristic, class openList>
uint64_t DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetNecessaryExpansions() const
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
template <class state, class action, class environment, class batchHeuristic, class openList>
void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by DelayedHeuristicAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment, class batchHeuristic, class openList>
int DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetMemoryUsage()
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
 * the states
 */
template <class state, class action, class environment, class batchHeuristic, class openList>
bool DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetClosedListGCost(const state &val, double &gCost) const
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

template <class state, class action, class environment, class batchHeuristic, class openList>
bool DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetOpenListGCost(const state &val, double &gCost) const
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

template <class state, class action, class environment, class batchHeuristic, class openList>
bool DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetHCost(const state &val, double &hCost) const
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

template <class state, class action, class environment, class batchHeuristic, class openList>
bool DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &result)
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
 * @date 03/12/09
 * 
 */
//template <class state, class action, class environment, class batchHeuristic, class openList>
//void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::OpenGLDraw() const
//{
//	double transparency = 1.0;
//	if (openClosedList.size() == 0)
//		return;
//	uint64_t top = -1;
////	double minf = 1e9, maxf = 0;
//	if (openClosedList.OpenSize() > 0)
//	{
//		top = openClosedList.Peek();
//	}
////	for (unsigned int x = 0; x < openClosedList.size(); x++)
////	{
////		const AStarOpenClosedData<state> &data = openClosedList.Lookat(x);
////		double f = data.g+data.h;
////		if (f > maxf)
////			maxf = f;
////		if (f < minf)
////			minf = f;
////	}
//	for (unsigned int x = 0; x < openClosedList.size(); x++)
//	{
//		const auto &data = openClosedList.Lookat(x);
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
////			if (top != -1)
////			{
////				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
////			}
////			else {
//			if (data.parentID == x)
//				env->SetColor(1.0, 0.5, 0.5, transparency);
//			else
//				env->SetColor(1.0, 0.0, 0.0, transparency);
////			}
//			env->OpenGLDraw(data.data);
//		}
//	}
//	env->SetColor(1.0, 0.5, 1.0, 0.5);
//	env->OpenGLDraw(goal);
//}

/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 7/12/16
 *
 */
template <class state, class action, class environment, class batchHeuristic, class openList>
void DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::Draw(Graphics::Display &disp) const
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
	for (unsigned int x = 0; x < delayedStates.size(); x++)
	{
		env->SetColor(Colors::blue);
		env->Draw(disp, delayedStates[x].s);
	}
}

template <class state, class action, class environment, class batchHeuristic, class openList>
std::string DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::SVGDraw() const
{
	std::string s;
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return s;
	uint64_t top = -1;
	
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
			s+=env->SVGDraw(data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			s+=env->SVGDraw(data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			s+=env->SVGDraw(data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			s+=env->SVGDraw(data.data);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			s+=env->SVGDraw(data.data);
		}
	}
	return s;
}

template <class state, class action, class environment, class batchHeuristic, class openList>
std::string DelayedHeuristicAStar<state,action,environment,batchHeuristic,openList>::SVGDrawDetailed() const
{
	std::string s;
	//double transparency = 1.0;
	if (openClosedList.size() == 0)
		return s;
	uint64_t top = -1;
	
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		
//		if (x == top)
//		{
//			env->SetColor(1.0, 1.0, 0.0, transparency);
//			s+=env->SVGDraw(data.data);
//		}
//		else if ((data.where == kOpenList) && (data.reopened))
//		{
//			env->SetColor(0.0, 0.5, 0.5, transparency);
//			s+=env->SVGDraw(data.data);
//		}
//		else if (data.where == kOpenList)
//		{
//			env->SetColor(0.0, 1.0, 0.0, transparency);
//			s+=env->SVGDraw(data.data);
//		}
//		else if ((data.where == kClosedList) && (data.reopened))
//		{
//			env->SetColor(0.5, 0.0, 0.5, transparency);
//			s+=env->SVGDraw(data.data);
//		}
//		else if (data.where == kClosedList)
//		{
//			env->SetColor(1.0, 0.0, 0.0, transparency);
//			s+=env->SVGDraw(data.data);
//		}
		env->SetColor(0.0, 0.0, 0.0);
		char d[32];
		sprintf(d, "%1.1f", data.g+data.h);
		s+=env->SVGLabelState(data.data, d, 0.35, -0.6, -1.1);
		sprintf(d, "g:%1.1f", data.g);
		s+=env->SVGLabelState(data.data, d, 0.25, -0.6,  -0.75);
		sprintf(d, "h:%1.1f", data.h);
		s+=env->SVGLabelState(data.data, d, 0.25, -0.6,  -0.48);
	}
	return s;
}


#endif
