//
//  IOS.h
//  hog2 mac native demos
//
//  Improved Optimistic Search
//  This is my re-implementation of the algorithm.
//
//
//  Created by Nathan Sturtevant on 7/11/19.
//  Copyright © 2019 NS Software. All rights reserved.
//

#ifndef IOS_h
#define IOS_h

template<typename state>
class IOSOpenClosedData {
public:
	IOSOpenClosedData() { }
	IOSOpenClosedData(const state &theData, double fCost, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), f(fCost), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location)
	{
		reopened = false; onOptimalPath = false;
	}
	state data;
	double f;
	double g;
	double h;
	bool onOptimalPath;
	uint64_t parentID;
	uint64_t openLocation;
	bool reopened;
	dataLocation where;
};

template <class state>
struct IOSCompare {
	// returns true if i2 is preferred over i1
	bool operator()(const IOSOpenClosedData<state> &i1, const IOSOpenClosedData<state> &i2) const
	{
		if (fequal(i1.f, i2.f))
		{
			return (fless(i1.g, i2.g));
		}
		return fgreater(i1.f, i2.f);
	}
};

/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment>
class ImprovedOptimisticSearch : public GenericSearchAlgorithm<state,action,environment> {
public:
	ImprovedOptimisticSearch()
	{
		ResetNodeCount(); env = 0; weight=3; bound = 1.5; theHeuristic = 0;
		greedyPhi = phi = [](double x, double y){return x+y;};
	}
	virtual ~ImprovedOptimisticSearch() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
	typedef AStarOpenClosed<state, IOSCompare<state>, IOSOpenClosedData<state>> openList;
	// uses admissible heuristic (regular A* search)
	AStarOpenClosed<state, IOSCompare<state>, IOSOpenClosedData<state>> openClosedList;
	
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
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
	bool GetFocalListGCost(const state &val, double &gCost) const;
	bool GetClosedItem(const state &s, IOSOpenClosedData<state> &);
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const IOSOpenClosedData<state> &GetOpenItem(unsigned int which)
	{ return openClosedList.Lookup(openClosedList.GetOpenItem(which)); }
		
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const IOSOpenClosedData<state> &GetItem(unsigned int which)
	{ return openClosedList.GetItem(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound;  }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key); }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
//	void OpenGLDraw() const;
	void Draw(Graphics::Display &d) const;

	void SetWeight(double w) {weight = w;}
	double GetWeight() { return weight; }
	void SetOptimalityBound(double w) {bound = w;}
	double GetOptimalityBound() {return bound;}
	void SetPhi(std::function<double(double, double)> p)
	{
		greedyPhi = p;
	}
	double Phi(double h, double g)
	{
		return greedyPhi(h, g);
	}

private:
	void DoGreedyStep(std::vector<state> &thePath);
	void DoOptimalStep(std::vector<state> &thePath);

	uint64_t nodesTouched, nodesExpanded;
	
	std::vector<state> internalPath;
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	
	environment *env;
	std::function<double(double, double)> phi;
	std::function<double(double, double)> greedyPhi;
	double bestSolution, solutionReduction;
	double weight;
	double bound;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;
	bool doneGreedy;
	double maxPriority;
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
const char *ImprovedOptimisticSearch<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "IOS[%1.2f, %1.2f]", bound, weight);
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
void ImprovedOptimisticSearch<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	//discardcount=0;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{ }
}

template <class state, class action, class environment>
void ImprovedOptimisticSearch<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
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
bool ImprovedOptimisticSearch<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	doneGreedy = false;
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	ResetNodeCount();
	start = from;
	goal = to;
	bestSolution = DBL_MAX;
	solutionReduction = 0;
	maxPriority = 0;
	internalPath.clear();
	openClosedList.Reset();

	double w = GetWeight(); // could use 2w-1 on bound
	phi = greedyPhi;
//	phi = ([=](double x, double y){return y/(w)+x;});

	double h = theHeuristic->HCost(from, to);
	openClosedList.AddOpenNode(from, env->GetStateHash(from), phi(h, 0), 0, h);
	if (env->GoalTest(from, to)) //assumes that from and to are valid states
	{
		return true;
	}
	return false;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment>
void ImprovedOptimisticSearch<state,action,environment>::AddAdditionalStartState(state& newState)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), phi(h, 0), 0, h);
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment>
void ImprovedOptimisticSearch<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
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
template <class state, class action, class environment>
bool ImprovedOptimisticSearch<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	// Solution proven
	if (flesseq(bestSolution-solutionReduction, GetOptimalityBound()*maxPriority))
	{
		ExtractPathToStart(goal, thePath);
		return true;
	}
		
	// TODO: Unclear if we could return no solution if the last node expanded was the goal
	if (openClosedList.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		return true;
	}
	
	// Do greedy search
	if (bestSolution == DBL_MAX)
	{
		DoGreedyStep(internalPath);
		return false;
	}
	else {
		DoOptimalStep(internalPath);
		return false;
	}
	return false;
}

template <class state, class action, class environment>
void ImprovedOptimisticSearch<state,action,environment>::DoGreedyStep(std::vector<state> &thePath)
{
	uint64_t nodeid = openClosedList.Close();
	maxPriority = std::max(maxPriority, openClosedList.Lookup(nodeid).f);
	
	if (!openClosedList.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;
	
	if ((env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
	{
		// 1. Clear open list
		openClosedList.CloseAllOnOpen();
		// 2. Change search priority
		phi = [](double x, double y){return x+y;};
		// 3. put start back on open
		uint64_t ID;
		openClosedList.Lookup(env->GetStateHash(start), ID);
		openClosedList.Lookup(ID).f = phi(openClosedList.Lookup(ID).h, openClosedList.Lookup(ID).g);
		openClosedList.Reopen(ID);
		// 4. extract path
		ExtractPathToStartFromID(nodeid, thePath);
		// 5. record path cost
		bestSolution = env->GetPathLength(thePath);
		// 6. record states on path
		for (const auto &i : thePath)
		{
			uint64_t key;
			openClosedList.Lookup(env->GetStateHash(i), key);
			openClosedList.Lookup(key).onOptimalPath = true;
		}
		printf("IOS: Found initial solution cost %f -- C* >= %f [%f]\n", bestSolution, maxPriority, bestSolution/maxPriority);
		return;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	//	std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data << " with f:";
	//	std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
	env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				// No re-openings in greedy search
				break;
			case kOpenList:
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
				}
				break;
			case kNotFound:
			{
				double h = theHeuristic->HCost(neighbors[x], goal);
				openClosedList.AddOpenNode(neighbors[x],
										   env->GetStateHash(neighbors[x]),
										   phi(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
										   openClosedList.Lookup(nodeid).g+edgeCosts[x],
										   h,
										   nodeid);
			}
		}
	}
}

template <class state, class action, class environment>
void ImprovedOptimisticSearch<state,action,environment>::DoOptimalStep(std::vector<state> &thePath)
{
	uint64_t nodeid = openClosedList.Close();
	maxPriority = std::max(maxPriority, openClosedList.Lookup(nodeid).f);
	
	if (!openClosedList.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;
	
	if ((env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
	{
		// Solution is optimal. Return.
		// 1. extract path
		ExtractPathToStartFromID(nodeid, thePath);
		reverse(thePath.begin(), thePath.end());
		bestSolution = env->GetPathLength(thePath);
		return;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	//	std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data << " with f:";
	//	std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
	env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		
		switch (neighborLoc[x])
		{
			case kClosedList:
			{
				auto &i = openClosedList.Lookup(neighborID[x]);
				if (i.onOptimalPath) // must be closed
				{
					if ((i.g)>(openClosedList.Lookup(nodeid).g+edgeCosts[x]))
					{
						printf("IOS: Reduced solution cost by %f\n", (i.g)-(openClosedList.Lookup(nodeid).g+edgeCosts[x]));
						solutionReduction = std::max(solutionReduction,
													 (i.g)-(openClosedList.Lookup(nodeid).g+edgeCosts[x])
													 );
						printf("IOS: Current solution cost %f -- C* >= %f [%f]\n", bestSolution-solutionReduction,
							   maxPriority, (bestSolution-solutionReduction)/maxPriority);
					}
				}
				if (i.reopened == false) // Re-open at most once (from suboptimal to optimal search)
				{
					i.reopened = true;
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
				}
				break;
			case kNotFound:
			{
				double h = theHeuristic->HCost(neighbors[x], goal);
				openClosedList.AddOpenNode(neighbors[x],
										   env->GetStateHash(neighbors[x]),
										   phi(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
										   openClosedList.Lookup(nodeid).g+edgeCosts[x],
										   h,
										   nodeid);
			}
		}
	}
}


/**
 * Returns the next state on the open list (but doesn't pop it off the queue).
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The first state in the open list.
 */
template <class state, class action, class environment>
state ImprovedOptimisticSearch<state, action,environment>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
}

/**
 * Get the path from a goal state to the start state
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param node the state from which to extract the goal
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment>
void ImprovedOptimisticSearch<state, action,environment>::ExtractPathToStartFromID(uint64_t node,
																				   std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

template <class state, class action,class environment>
const state &ImprovedOptimisticSearch<state, action,environment>::GetParent(const state &s)
{
	uint64_t theID;
	openClosedList.Lookup(env->GetStateHash(s), theID);
	theID = openClosedList.Lookup(theID).parentID;
	return openClosedList.Lookup(theID).data;
}

/**
 * A function that prints the number of states in the closed list and open
 * queue.
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment>
void ImprovedOptimisticSearch<state, action,environment>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by ImprovedOptimisticSearch
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment>
int ImprovedOptimisticSearch<state, action,environment>::GetMemoryUsage()
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
template <class state, class action, class environment>
bool ImprovedOptimisticSearch<state, action,environment>::GetClosedListGCost(const state &val, double &gCost) const
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

template <class state, class action, class environment>
bool ImprovedOptimisticSearch<state, action,environment>::GetOpenListGCost(const state &val, double &gCost) const
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

/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 *
 */
//template <class state, class action, class environment>
//void ImprovedOptimisticSearch<state, action,environment>::OpenGLDraw() const
//{
//}

template <class state, class action, class environment>
void ImprovedOptimisticSearch<state, action,environment>::Draw(Graphics::Display &disp) const
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
		else if ((data.where == kClosedList) && (data.onOptimalPath))
		{
			env->SetColor(0.5, 0.5, 0.5, transparency);
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


#endif /* IOS_h */
