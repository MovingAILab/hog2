//
//  IncrementalBGS.h
//  hog2
//
//  This is an incremental version of Budgeted Graph Search - IBEX on trees with Bounded Dijkstra at the low level.
//  This code isn't meant for production - just to animate how the algorithm works as a comparison to IDA*.
//  See (Helmert et al, IJCAI 2019) for details of the algorithm
//

#ifndef IncrementalBGS_h
#define IncrementalBGS_h

#include "Heuristic.h"
#include "AStarOpenClosed.h"

template <class state, class action>
class IncrementalBGS {
public:
	IncrementalBGS(double initialBound = 0) :bound(initialBound), initialBound(initialBound), nextBound(initialBound)
	{ ResetNodeCount(); previousBound = 0; }
	bool InitializeSearch(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
						  std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetIterationNodesExpanded() { return data.nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount()
	{
		nodesExpanded = nodesTouched = 0;
		newNodeCount = newNodesLastIteration = 0;
	}
	void Reset() { bound = initialBound; nextBound = initialBound; solutionPath.clear();
		history.clear(); ResetNodeCount(); previousBound = 0; }
//	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	state GetCurrentState() const
	{
		if (q.size() > 0)
			return q.Lookup(q.Peek()).data;
//			return search.back().currState;
//		if (flesseq(solutionCost, data.solutionInterval.lowerBound))
//			return goal;
//		return start;
		
	}
	void GetCurrentPath(std::vector<state> &p) const
	{ p = solutionPath; }
	double GetCurrentFLimit() { return std::min(bound, solutionCost); }
	double GetNextFLimit() { return nextBound; }
	uint64_t GetNewNodesLastIteration() { return newNodesLastIteration; }
	const uint64_t c1 = 2;
	const uint64_t c2 = 8;
	const uint64_t gamma = 2;
	const int infiniteWorkBound = -1;
	void GetGlobalCostInterval(double &lower, double &upper)
	{ lower = data.solutionInterval.lowerBound; upper = data.solutionInterval.upperBound; }
	void GetNodeInterval(uint64_t &lower, uint64_t &upper)
	{ lower = data.nodeLB*c1; upper = data.workBound; }
	std::string stage;
	std::string fEquation;
private:
	struct costInterval {
		double lowerBound;
		double upperBound;
		costInterval &operator&=(const costInterval &i)
		{
			lowerBound = std::max(lowerBound, i.lowerBound);
			upperBound = std::min(upperBound, i.upperBound);
			return *this;
		}
	};
	struct IBEXData {
		uint64_t nodesExpanded;
		uint64_t workBound;
		uint64_t nodeLB;
		costInterval solutionInterval;
		double delta;
	};
	IBEXData data;
	struct searchData {
		double f_below;
		double f_above;
	};
	searchData sd;
//	enum kSearchStatus {
//		kGoingDown,
//		kGoingAcross
//	};
//	struct currSearchState {
//		state currState;
//		kSearchStatus status;
//		double pathCost;
//		std::vector<state> succ;
//	};
//	std::vector<currSearchState> search;
	double solutionCost;
	bool IterationComplete() { return q.OpenSize() == 0; }
	unsigned long nodesExpanded, nodesTouched;
	
	void SetupIteration(double cost);
	bool StepIteration();
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);

	std::vector<std::pair<state, double>> history;
	std::vector<state> solutionPath;
	state start, goal;
	double previousBound;
	double bound;
	double initialBound;
	double nextBound;
	SearchEnvironment<state, action> *env;
	Heuristic<state> *h;
	std::vector<state> succ;
	uint64_t newNodeCount, newNodesLastIteration;
	
	struct BFHSCompare {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g, i2.g));
		}
	};

	// Data for BFHS
	AStarOpenClosed<state, BFHSCompare> q;
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	std::vector<state> solutionStates;
};

template <class state, class action>
void IncrementalBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											Heuristic<state> *h, std::vector<state> &thePath)
{
	if (InitializeSearch(e, from, to, h, thePath))
		return;
	while (!DoSingleSearchStep(thePath))
	{}
}

template <class state, class action>
void IncrementalBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool IncrementalBGS<state, action>::InitializeSearch(SearchEnvironment<state, action> *e, state from, state to, Heuristic<state> *h,
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

	solutionPath.clear();

	// Setup BTS control data
	data.nodesExpanded = 0;
	data.workBound = infiniteWorkBound;
	data.nodeLB = 0;
	data.solutionInterval.lowerBound = h->HCost(start, goal);
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 0;
	fEquation = to_string_with_precision(h->HCost(start, goal), 0);
	solutionCost = DBL_MAX;
	SetupIteration(h->HCost(start, goal));
	stage = "NEW ITERATION";
	return false;
}

template <class state, class action>
void IncrementalBGS<state, action>::SetupIteration(double cost)
{
	q.Reset(env->GetMaxHash());
	// put start in open
	q.AddOpenNode(start, env->GetStateHash(start), 0.0, 0.0);

//	search.resize(1);
//	search.back().currState = start;
//	search.back().status = kGoingDown;
//	search.back().pathCost = 0;
	previousBound = bound;
	bound = cost;//nextBound;
	nextBound = -1;
	printf("Starting iteration bound %1.1f\n", bound);
	newNodesLastIteration = newNodeCount;
	newNodeCount = 0;
	data.nodesExpanded = 0;
	sd.f_below = 0;
	sd.f_above = DBL_MAX;
}

template <class state, class action>
void IncrementalBGS<state, action>::ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
{
	thePath.clear();
	do {
		thePath.push_back(q.Lookup(node).data);
		node = q.Lookup(node).parentID;
	} while (q.Lookup(node).parentID != node);
	thePath.push_back(q.Lookup(node).data);
}

template <class state, class action>
bool IncrementalBGS<state, action>::StepIteration()
{
	uint64_t nodeid = q.Close();
	
	// Check if we are done
	if (env->GoalTest(q.Lookup(nodeid).data, goal))
	{
		ExtractPathToStartFromID(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		solutionCost = env->GetPathLength(solutionPath);
		return false;
	}
	
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	//				std::cout << "Expanding: " << q.Lookup(nodeid).data << " with f:";
	//				std::cout << q.Lookup(nodeid).g << std::endl;
	
	env->GetSuccessors(q.Lookup(nodeid).data, neighbors);
	nodesExpanded++;
	
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(q.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(q.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		double childGCost = q.Lookup(nodeid).g+edgeCosts[x];
		double childFCost = childGCost+h->HCost(neighbors[x], goal);
		if (fgreater(childFCost, bound))
		{
			if (nextBound == -1)
				nextBound = childFCost;
			else if (fless(childFCost, nextBound))
				nextBound = childFCost;

			sd.f_above = std::min(sd.f_above, childFCost);
			continue;
		}
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				break;
			case kOpenList:
				if (fless(childGCost, q.Lookup(neighborID[x]).g))
				{
					q.Lookup(neighborID[x]).parentID = nodeid;
					q.Lookup(neighborID[x]).g = childGCost;
					q.KeyChanged(neighborID[x]);
				}
				break;
			case kNotFound:
			{
				q.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  0,
							  nodeid);
			}
		}
	}
	return false;
	
//	if (search.back().status == kGoingDown)
//	{
//		double f = search.back().pathCost+h->HCost(search.back().currState, goal);
//		// exceeded path cost bound
//		if (fgreater(f, bound))
//		{
//			//printf("Above bound: %f/%f\n", bound, f);
//			sd.f_above = std::min(sd.f_above, f);
//			if (nextBound == -1)
//				nextBound = f;
//			else if (fless(f, nextBound))
//				nextBound = f;
//
//			search.pop_back();
//			return false;
//		}
//		if (fgreater(f, solutionCost))
//		{
//			search.pop_back();
//			return false;
//		}
//		if (flesseq(f, bound))
//			sd.f_below = std::max(sd.f_below, f);
//		//			data.solutionInterval.upperBound = std::max(data.solutionInterval.upperBound, f);
//
//		if (fgreater(f, previousBound) && flesseq(f, bound))
//			newNodeCount++;
//
//		// continue search
//		//printf("Generating next set of successors\n");
//		search.back().status = kGoingAcross;
//		env->GetSuccessors(search.back().currState, search.back().succ);
//		nodesExpanded++;
//		for (int x = 0; x < search.back().succ.size(); x++)
//		{
//			if (search.size() > 1 && search.back().succ[x] == search[search.size()-2].currState)
//			{
//				search.back().succ.erase(search.back().succ.begin()+x);
//				x--;
//			}
//		}
//
//		// reverse and then handle them from back to front
//		std::reverse(search.back().succ.begin(), search.back().succ.end());
//		//return false;
//	}
//
//	if (search.back().status == kGoingAcross)
//	{
//		// no more succ to go down - go up
//		if (search.back().succ.size() == 0)
//		{
//			//printf("Out of successors\n");
//			search.pop_back();
//			return false;
//		}
//
//		//printf("Taking next successors\n");
//		// going down - generate next successor
//		search.resize(search.size()+1);
//		auto &s = search[search.size()-2];
//		search.back().currState = s.succ.back();
//		search.back().status = kGoingDown;
//		search.back().pathCost = s.pathCost+env->GCost(s.currState, s.succ.back());
//		s.succ.pop_back();
//		return false;
//	}
//	assert(false);
//	return false;
	
}


template <class state, class action>
bool IncrementalBGS<state, action>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (flesseq(solutionCost, data.solutionInterval.lowerBound))
	{
		thePath = solutionPath;
		printf("Found solution cost %1.5f\n", solutionCost);
		return true;
	}
	
	if (!IterationComplete() && data.nodesExpanded < data.workBound)
	{
		uint64_t tmp = nodesExpanded;
		StepIteration();
		data.nodesExpanded += nodesExpanded-tmp;
		return false;
	}
	
	// Invariant - iteration complete *or* exceeded work limit
	// last search completed - set the interval from this search
	costInterval v;
	if (data.nodesExpanded >= data.workBound)
	{
		v.lowerBound = 0;
		v.upperBound = sd.f_below;
	}
	else if (solutionCost != DBL_MAX && fgreatereq(sd.f_below, solutionCost))
	{
		v.lowerBound = solutionCost;
		v.upperBound = solutionCost;
	}
	else {
		v.lowerBound = sd.f_above;
		v.upperBound = DBL_MAX;
	}
	data.solutionInterval &= v;
	
	printf("--> Iteration complete - %" PRId64 " expanded; target [%" PRId64 ", %" PRId64 ")\n", data.nodesExpanded, c1*data.nodeLB, c2*data.nodeLB);
	// Move to next iteration
	if (data.nodesExpanded >= c1*data.nodeLB && data.workBound == infiniteWorkBound)
	{
		printf("Expanded %" PRId64 " - needed at least %" PRId64 "\n", data.nodesExpanded, c1*data.nodeLB);
		if (data.solutionInterval.lowerBound == DBL_MAX) // No new nodes in iteration
			printf("[HIT]--Critical f in [%1.5f, %1.5f]\n", solutionCost, solutionCost);
		else
			printf("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
		data.nodeLB = data.nodesExpanded;
		data.solutionInterval.upperBound = DBL_MAX;
		data.nodesExpanded = 0;
		fEquation = to_string_with_precision(nextBound, 0);
		SetupIteration(nextBound);
		return false;
	}
		
	// Check if bounds are equal or whether we fell within the node bounds
	if (!
		(fequal(data.solutionInterval.upperBound, data.solutionInterval.lowerBound) ||
		 (data.nodesExpanded >= c1*data.nodeLB && data.nodesExpanded < c2*data.nodeLB)
		 ))
	{
		if (data.solutionInterval.upperBound == DBL_MAX)
			printf("    ]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
		else
			printf("    ]--Critical f in [%1.5f, %1.5f]\n", data.solutionInterval.lowerBound, data.solutionInterval.upperBound);
			
		double nextCost;
		if (data.solutionInterval.upperBound == DBL_MAX)
		{
			nextCost = data.solutionInterval.lowerBound+pow(gamma, data.delta);
			fEquation = to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+to_string_with_precision(gamma, 0)+"^"+to_string_with_precision(data.delta, 0)+"="+to_string_with_precision(nextCost, 0);
			stage = "EXP";
		}
		else {
			nextCost = (data.solutionInterval.lowerBound+data.solutionInterval.upperBound)/2.0;
			fEquation = "("+to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+ to_string_with_precision(data.solutionInterval.upperBound, 0)+")/2"+"="+to_string_with_precision(nextCost, 0);
			stage = "BIN";
		}
		data.delta += 1;
		data.workBound = c2*data.nodeLB;
		SetupIteration(nextCost);
		printf("-> Starting new iteration with f: %f; node limit: %" PRId64 "\n", nextCost, data.workBound);

		return false;
	}

	if (data.solutionInterval.lowerBound == DBL_MAX)
		printf("[HIT]--Critical f in [∞, ∞]\n");
	else
		printf("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
	// finished iteration with current bound - ready to start next IBEX/BTS iteration
	data.nodeLB = std::max(data.nodesExpanded, c1*data.nodeLB);
	data.solutionInterval.upperBound = DBL_MAX;
	data.workBound = infiniteWorkBound;
	data.nodesExpanded = 0;
	data.delta = 0;
	fEquation = to_string_with_precision(nextBound, 0);
	SetupIteration(nextBound);
	stage = "NEW ITERATION";
	return false;
}

template <class state, class action>
void IncrementalBGS<state, action>::Draw(Graphics::Display &disp) const
{
	double transparency = 1.0;
	if (q.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (q.OpenSize() > 0)
	{
		top = q.Peek();
	}
	for (unsigned int x = 0; x < q.size(); x++)
	{
		const auto &data = q.Lookat(x);
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
	env->Draw(disp, goal);}

//template <class state, class action>
//void IncrementalBGS<state, action>::OpenGLDraw()
//{
//}


#endif /* IncrementalBGS_h */
