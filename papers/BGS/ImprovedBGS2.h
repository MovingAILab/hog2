//
//  ImprovedBGS2.h
//  hog2
//
// This implementation contains working of Algo2 for the efficient BGS. 
// The search can start from the current f-cost without discarding any nodes in the queue.
// There is only need to update the budget values correctly.

#ifndef ImprovedBGS2h
#define ImprovedBGS2_h

#include "Heuristic.h"
#include "AStarOpenClosed.h"

#ifndef DEBUG 
#define DEBUG 0 // set debug mode
#endif

#if DEBUG
#define debug_print(fmt, ...) printf(fmt,__VA_ARGS__)
#else
#define debug_print(fmt, ...) do {} while (0)
#endif


template <class state, class action>
class ImprovedBGS2 {
public:
	ImprovedBGS2(double initialBound = 0) :bound(initialBound), initialBound(initialBound), nextBound(initialBound)
	{ ResetNodeCount(); previousBound = 0; }
	bool InitializeSearch(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
						  std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);

	uint64_t GetNodesExpanded() { return nodesExpanded + nodesReexpanded; }
	uint64_t GetIterationNodesExpanded() { return data.nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount()
	{
		nodesExpanded = nodesTouched = 0;
		newNodeCount = newNodesLastIteration = 0;
	}
	void Reset() { bound = initialBound; nextBound = initialBound; solutionPath.clear();
		history.clear(); ResetNodeCount(); previousBound = 0; }
	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	state GetCurrentState() const
	{
		if (q_f.size() > 0)
			return q_f.Lookup(q_f.Peek()).data;
		
	}
	void GetCurrentPath(std::vector<state> &p) const
	{ p = solutionPath; }
	double GetCurrentFLimit() { return std::min(bound, solutionCost); }
	double GetNextFLimit() { return nextBound; }
	uint64_t GetNewNodesLastIteration() { return newNodesLastIteration; }
	const uint64_t c1 = 2;
	const uint64_t c2 = 8;
	const uint64_t gamma = 2;
	int K = 1;
	uint64_t infiniteWorkBound = INT64_MAX;
	void GetGlobalCostInterval(double &lower, double &upper)
	{ lower = data.solutionInterval.lowerBound; upper = data.solutionInterval.upperBound; }
	void GetNodeInterval(uint64_t &lower, uint64_t &upper)
	{ lower = data.nodeLB*c1; upper = data.workBound; }
	std::string stage;
	std::string fEquation;
	void SetUseBPMX();
	void SetK(int a);  
	double GetSolutionCost()
	{ return solutionCost; }
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
		uint64_t nodesReexpanded;
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
	double solutionCost;
	bool IterationCompleteG() { return q_g.OpenSize() == 0; }
	bool IterationCompleteF() { return q_f.OpenSize() == 0; }
	void SetupIterationF(double cost);
	bool StepIterationUsingF();
	bool StepIterationUsingG();
	// Use these functions in case of debugging
	bool StepIterationUsingF(int i);
	bool StepIterationUsingG(int i);
	void ExtractPathToStartFromIDf(uint64_t node, std::vector<state> &thePath);
	void ExtractPathToStartFromIDg(uint64_t node, std::vector<state> &thePath);
	void FindtheBoundAndNextBoundF();
	void FindtheBoundAndNextBoundG();
	void FindtheCurrentBoundF();
	bool CurrentIterationComplete();
	bool SetupExponentialBinaryIteration();
	bool DoAStar();
	bool DoFLimitedAStar();
	bool DoBFHS();
	bool ExponentialBinaryStepIteration();
	void PrintQueueStatus();
	bool isHeap(int i, int n);
	std::vector<std::pair<state, double>> history;
	std::vector<state> solutionPath;
	state start, goal;
	double previousBound;
	double bound;
	double initialBound;
	double nextBound;
	double bound_g;
	bool InitializeBFHS = 0;
	bool InitializeFLimitedAStar = 0;
	bool useBPMX = false;
	unsigned long nodesTouched;
	SearchEnvironment<state, action> *env;
	Heuristic<state> *h;
	std::vector<state> succ;
	uint64_t newNodeCount, newNodesLastIteration;
	double fc;
	double fc_next;
	unsigned long nodesExpanded;
	unsigned long nodesReexpanded;
	std::vector< std::pair<uint64_t,float> > explist;
	std::vector <float> hp;
	enum STAGE { AStar, FLimitedAStar, BFHS, EXPONENTIAL };
	STAGE s = AStar;
	bool InitializeExponentialBinaryIteration = false;

	struct BFHSCompare {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			
			return (fgreater(i1.g, i2.g));
		}
	};
    struct BFHSCompare_f {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{

			if (fequal(i1.g+i1.h, i2.g+i2.h))
			{
			return (fless(i1.g, i2.g));
			}
			return fgreater(i1.g+i1.h, i2.g+i2.h);
		}
	};


	// Data for BFHS
	AStarOpenClosed<state, BFHSCompare> q_g;
	AStarOpenClosed<state, BFHSCompare_f> q_f;

	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	std::vector<state> solutionStates;

	std::vector<state> neighbors_m;
	std::vector<uint64_t> neighborID_m;
	std::vector<double> edgeCosts_m;
	std::vector<dataLocation> neighborLoc_m;
	std::vector<state> solutionStates_m;

	std::vector<state> neighbors_c;
	std::vector<uint64_t> neighborID_c;
	std::vector<double> edgeCosts_c;
	std::vector<dataLocation> neighborLoc_c;
	std::vector<state> solutionStates_c;
};

template <class state, class action>
bool ImprovedBGS2<state, action>::isHeap(int i, int n){
		if(i >= (n-1)/2){
			return 2;
		}

		if(flesseq(hp[i],hp[2*i+1])  &&  flesseq(hp[i],hp[2*i+2]) && isHeap(2*i+1,n) && isHeap(2*i+2,n)){
			return true;
		}
		else {
			return false;
		}
}



template <class state, class action>
void ImprovedBGS2<state, action>::SetUseBPMX()
{
	useBPMX = true;
}

/**
 * Sets the value of K, parameter for re-expansion budget
 */
template <class state, class action>
void ImprovedBGS2<state, action>::SetK(int a)
{

    if (a <= 8) 
	{
		K = a;   // set re-expansion budget
	}
	else if (a == 9)
	{
		K = 10000;   // infinite re-expansion budget
	}
}


template <class state, class action>
void ImprovedBGS2<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											Heuristic<state> *h, std::vector<state> &thePath)
{
	if (InitializeSearch(e, from, to, h, thePath))
		return;
	while (!DoSingleSearchStep(thePath))
	{}
}

template <class state, class action>
void ImprovedBGS2<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											std::vector<action> &thePath)
{
	
}

template <class state, class action>
void ImprovedBGS2<state, action>::ExtractPathToStartFromIDf(uint64_t node, std::vector<state> &thePath)
{
	
	thePath.clear();
	do {
		thePath.push_back(q_f.Lookup(node).data);
		node = q_f.Lookup(node).parentID;
	} while (q_f.Lookup(node).parentID != node);
	thePath.push_back(q_f.Lookup(node).data);
	
}

template <class state, class action>
void ImprovedBGS2<state, action>::ExtractPathToStartFromIDg(uint64_t node, std::vector<state> &thePath)
{
	
	thePath.clear();
	do {
		thePath.push_back(q_g.Lookup(node).data);
		node = q_g.Lookup(node).parentID;
	} while (q_g.Lookup(node).parentID != node);
	thePath.push_back(q_g.Lookup(node).data);	
}




template <class state, class action>
bool ImprovedBGS2<state, action>::InitializeSearch(SearchEnvironment<state, action> *e, state from, state to, Heuristic<state> *h,
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
	nodesExpanded = 0;
	nodesReexpanded = 0;
	solutionPath.clear();
	solutionCost = DBL_MAX;

    // reset two queues
    q_g.Reset(env->GetMaxHash());
	q_f.Reset(env->GetMaxHash());

	// Initialize data for an iteration
	data.nodesExpanded = 0;
	data.nodesReexpanded = 0;
	data.workBound = infiniteWorkBound;
	data.nodeLB = 0;
	data.solutionInterval.lowerBound = h->HCost(start, goal); 
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 1;
	fEquation = to_string_with_precision(h->HCost(start, goal), 0);
	fc = DBL_MAX;   // f-cost completely expanded in an iteration
	fc_next = DBL_MAX;   // f-cost for the next iteration

	stage = "INITIALIZE SEARCH";
	SetupIterationF(h->HCost(start, goal));
	return false;
}


template <class state, class action>
bool ImprovedBGS2<state, action>::DoSingleSearchStep(std::vector<state> &thePath)
{

    int b = data.nodeLB;
	b = b * K;
	debug_print("this is expansion number %d\n",data.nodesExpanded);
	if (!CurrentIterationComplete())
	{   // Main Iteration is over when the number of expansions grow by a factor.
	    // check if we are done
		if (flesseq(solutionCost, data.solutionInterval.lowerBound))
				{
					thePath = solutionPath;
					debug_print("Found solution cost %1.5f\n", solutionCost);   
					return true;
				}
		if (s == AStar || s == FLimitedAStar || s == BFHS )
		{   
			// case 1 : Run A* when within the re-expansion budget
			if (data.nodesReexpanded <= b && ((data.nodesExpanded < c1*data.nodeLB && nodesExpanded > 0) || nodesExpanded == 0))
			{    
				return DoAStar();	
			} 
			 
			// case 2 : Done enough number of node expansions to complete the current iteration, finish the current f-cost
			else if(data.nodesReexpanded <= b && data.nodesExpanded >= c1*data.nodeLB)
			{ 
				s = FLimitedAStar;
				return DoFLimitedAStar();
			}

			// case 3: Hit the re-expansion budget && done enough number of node expansions for the current iteration, Run BFHS as can not afford re-expansions
			else if(data.nodesReexpanded > b && data.nodesExpanded >= c1*data.nodeLB)
			{  
				s = BFHS;
				return DoBFHS();
			}

			// case 4: Hit the re-expansion budget && Number of expansions did not grow by a factor, Run Exponential Search
			else
		    {
				return SetupExponentialBinaryIteration();
			}
		}
		else{     
			// check if we are done 
			if (flesseq(solutionCost, data.solutionInterval.lowerBound))
				{
					thePath = solutionPath;
					debug_print("Found solution cost %1.5f\n", solutionCost);
					return true;
				}
			
			// call Exponential Binary Search Step Iteration
			return ExponentialBinaryStepIteration();
		}
				
	}
	else{

		debug_print("Main iteration complete\n");
		data.solutionInterval.lowerBound = bound;
		data.solutionInterval.upperBound = DBL_MAX;
		data.nodeLB = nodesExpanded;
		data.nodesExpanded = 0;
		data.nodesReexpanded = 0;
		data.workBound = -1;
		data.delta = 1;
		s = AStar;  // set enum back to stage ASTAR
		InitializeFLimitedAStar = 0;
		InitializeBFHS = 0;
		return false;
	}
}

template <class state, class action>
bool ImprovedBGS2<state, action>::DoAStar()
{
	debug_print("DO AStar\n",1);
	int temp1 = nodesExpanded;
	int temp2 = nodesReexpanded;

	// Node expansion from low to high f-cost
	if(!IterationCompleteF())
	{
		StepIterationUsingF();
	}
	// update the node expansions and node re-expansions
	data.nodesExpanded += nodesExpanded-temp1;
	data.nodesReexpanded += nodesReexpanded-temp2;
	return false;
}


template <class state, class action>
bool ImprovedBGS2<state, action>::DoFLimitedAStar()
{
    debug_print("Do FLimitedAStar\n",1);
	// Initialize the case by finding the f-cost (previousBound) that has been expanded in the last iteration
	if(!InitializeFLimitedAStar)
	{
		FindtheCurrentBoundF();
		InitializeFLimitedAStar = 1;
		return false;
	}
	else{
		if(!q_f.empty())
		{
			uint64_t node = q_f.Peek(); // look at the node with the least f-cost that has not been expanded
			state s = q_f.Lookup(node).data;
			double g_value = q_f.Lookup(node).g;
			double h_value = q_f.Lookup(node).h;
			double f_value = g_value+h_value;

			debug_print("f-value is %1.5f anf previous bound is %1.5f\n",f_value,previousBound);
			if(flesseq(f_value,previousBound))
			{   // expand the node with the previousBound
				int temp1 = nodesExpanded;
				int temp2 = nodesReexpanded;
				StepIterationUsingF();
				data.nodesExpanded += nodesExpanded-temp1;
				debug_print("nodes expanded in limited f is %d\n",nodesExpanded);
				data.nodesReexpanded += nodesReexpanded-temp2;
				return false;
			}
			// update the iteration intervals when all the nodes with previousBound have been expanded
			data.solutionInterval.lowerBound = previousBound;   
			data.solutionInterval.upperBound = previousBound;
			FindtheBoundAndNextBoundF();   // find the bounds for the next iteration
			return false;
		}
		// update the iteration intervals when all the nodes in OPEN_f have been expanded
		data.solutionInterval.lowerBound = previousBound;
		data.solutionInterval.upperBound = previousBound;
		FindtheBoundAndNextBoundF();  // find the bounds for the next iteration
		return false;
		}

}


template <class state, class action>
bool ImprovedBGS2<state, action>::DoBFHS()
{
	debug_print("Do BFHS\n",1);
	// Initialize the case by finding the f-cost (previousBound) that has been expanded in the iteration 
	if(!InitializeBFHS)
	{
		FindtheBoundAndNextBoundF();
		bound_g = previousBound;
		InitializeBFHS = 1;
		return false;
	}
	else{
		if(!q_g.empty())
		{
			uint64_t node = q_g.Peek();     // look at the node with the g-cost that has not been expanded
			state s = q_g.Lookup(node).data;
			double g_value = q_g.Lookup(node).g;
			double h_value = q_g.Lookup(node).h;
			double f_value = g_value+h_value;

			debug_print("f-value is %1.5f anf previous bound is %1.5f\n",f_value,previousBound);
			if(flesseq(f_value,bound_g))
			{   // expand the node with the previousBound
				int temp1 = nodesExpanded;
				StepIterationUsingG();
				data.nodesExpanded += nodesExpanded-temp1;
				return false;
			}
			// update the iteration intervals when all the nodes with previousBound have been expanded
			data.solutionInterval.lowerBound = previousBound;
			data.solutionInterval.upperBound = previousBound;
			FindtheBoundAndNextBoundF();  // find the bounds for the next iteration
			bound_g = previousBound;  
			return false;
		}
		// update the iteration intervals when all the nodes in OPEN_g have been expanded
		data.solutionInterval.lowerBound = previousBound;
		data.solutionInterval.upperBound = previousBound;
		FindtheBoundAndNextBoundF();   // find the bounds for the next iteration
		bound_g = previousBound;   
		return false;
	}


}


template <class state, class action>
bool ImprovedBGS2<state, action>::SetupExponentialBinaryIteration()
{
			
			// find the bounds for the iteration
			FindtheCurrentBoundF();
			FindtheBoundAndNextBoundF();
			if(fequal(previousBound,bound))
			{
				bound_g = previousBound;   // To expand the rest of the nodes that did not get expanded in the last iteration
			}
			else{
				assert(fgreater(bound,previousBound));
				bound_g = bound;    // To expand all the nodes with the next smallest f-cost
			}
			s = EXPONENTIAL;  // set the enum to Exponential Search
			debug_print("largest closed f is %1.5f and smallest open f is %1.5f\n",bound,previousBound);
			return false;
}


template <class state, class action>
bool ImprovedBGS2<state, action>::ExponentialBinaryStepIteration()
{
		debug_print("Exponential Binary Step\n",1);
		// Expand nodes from low to high g-cost
		if(!q_g.empty() && data.nodesExpanded < data.workBound && data.nodesExpanded < c1*data.nodeLB)
		{
			uint64_t node = q_g.Peek();
			state s = q_g.Lookup(node).data;
			double g_value = q_g.Lookup(node).g;
			double h_value = q_g.Lookup(node).h;
			double f_value = g_value+h_value;

			debug_print("f-value is %1.5f anf previous bound is %1.5f\n",f_value,previousBound);
			if(flesseq(f_value,bound_g))
			{      
				int temp1 = nodesExpanded;
				StepIterationUsingG();
				data.nodesExpanded += nodesExpanded-temp1;
				return false;
			}
		}
			
		// Invariant - iteration complete *or* exceeded work limit
		// last search completed - set the interval from this search
		debug_print("workbound is %" PRIu64 "&& nodesexpanded is %d\n",data.workBound,data.nodesExpanded);
		if (data.nodesExpanded >= data.workBound)
		{
			debug_print("case 1\n",1);
			data.solutionInterval.lowerBound = 0;
			data.solutionInterval.upperBound = sd.f_below;
		}
		else if (solutionCost != DBL_MAX && fgreatereq(sd.f_below, solutionCost))
		{
			debug_print("case 2\n",1);
			data.solutionInterval.lowerBound = solutionCost;
			data.solutionInterval.upperBound = solutionCost;
		}
		else
		{
			debug_print("case 3\n",1);
			data.solutionInterval.lowerBound = bound_g;
			data.solutionInterval.upperBound = DBL_MAX;
		}

		debug_print("--> Iteration complete - %" PRId64 " expanded; target [%" PRId64 ", %" PRId64 ")\n", data.nodesExpanded, c1*data.nodeLB, c2*data.nodeLB);
		debug_print("workbound is %d && infiniteworkbound is %d\n",data.workBound,infiniteWorkBound);
		// Move to next iteration as the node expansions grew by a factor
		if (data.nodesExpanded >= c1*data.nodeLB && data.workBound == infiniteWorkBound)
		{
			debug_print("Expanded %" PRId64 " - needed at least %" PRId64 "\n", data.nodesExpanded, c1*data.nodeLB);
			if (data.solutionInterval.lowerBound == DBL_MAX) // No new nodes in iteration
			{
				debug_print("[HIT]--Critical f in [%1.5f, %1.5f]\n", solutionCost, solutionCost);
			}
			else
			{
				debug_print("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
			}
				
			data.solutionInterval.upperBound = bound_g;
			fEquation = to_string_with_precision(nextBound, 0);
			previousBound = bound_g;
			FindtheBoundAndNextBoundG();
			bound_g = bound;
			return false;
		}
			
		// Check if bounds are equal or whether we fell within the node bounds
		if (!
			(fequal(data.solutionInterval.upperBound, data.solutionInterval.lowerBound) ||
			(data.nodesExpanded >= c1*data.nodeLB && data.nodesExpanded < c2*data.nodeLB)
			))
		{
			if (data.solutionInterval.upperBound == DBL_MAX)
			{
				debug_print("    ]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
			}
				
			else
			{
				debug_print("    ]--Critical f in [%1.5f, %1.5f]\n", data.solutionInterval.lowerBound, data.solutionInterval.upperBound);	
			}
				
				
			double nextCost;
			data.delta *= gamma;
			if (data.solutionInterval.upperBound == DBL_MAX)
			{

				// nextCost = data.solutionInterval.lowerBound+pow(gamma, data.delta);
				nextCost = data.solutionInterval.lowerBound * data.delta;
				fEquation = to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+to_string_with_precision(gamma, 0)+"^"+to_string_with_precision(data.delta, 0)+"="+to_string_with_precision(nextCost, 0);
				stage = "EXP";
			}
			else {
				nextCost = (data.solutionInterval.lowerBound+data.solutionInterval.upperBound)/2.0;
				fEquation = "("+to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+ to_string_with_precision(data.solutionInterval.upperBound, 0)+")/2"+"="+to_string_with_precision(nextCost, 0);
				stage = "BIN";
			}
			// data.delta += 1;
			data.workBound = c2*data.nodeLB;
			previousBound = bound;
			bound = nextCost; 
			bound_g = bound;
			nextBound = -1;
			debug_print("-> Starting new iteration with f: %f; node limit: %" PRId64 "\n", nextCost, data.workBound);
			return false;
		}
		
		if (data.solutionInterval.lowerBound == DBL_MAX)
		{
			debug_print("[HIT]--Critical f in [∞, ∞]\n",1);
		}
			
		else
		{
			debug_print("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
		}
			
		
        
		data.solutionInterval.upperBound = bound_g;
		previousBound = bound_g;
		FindtheBoundAndNextBoundG();
		bound_g = bound;
		fEquation = to_string_with_precision(nextBound, 0);
		stage = "NEW ITERATION";
		return false;

}


/**
 * 
 * Main Iteration is over when the number of expansions grow by a factor.
 * 
 */
template <class state, class action>
bool ImprovedBGS2<state, action>::CurrentIterationComplete()
{
	if(data.solutionInterval.upperBound != DBL_MAX)
	{
		return true;
	}
	else{
		return false;
	}
}

/**
 * Finds the largest f-cost expanded in the last iteration.
 * It iterates through the entire list and finds the largest f-cost expanded in the last iteration.
 * 
 */
template <class state, class action>
void ImprovedBGS2<state, action>::FindtheCurrentBoundF()
{

	double tmp = 0.0;
	double tmp2 = DBL_MAX;
	for(uint64_t i = 0; i < q_f.size();i++)
	{
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kClosedList)
		{
			if(fgreater(f_value,tmp))
			{
				tmp = f_value;
			}
		}
	}
	previousBound = tmp;
	debug_print("nodes with f-value %1.5f have been closed\n",previousBound);
}

/**
 * Finds f-cost values in OPEN_f
 * It iterates through the entire list and finds the largest f-cost expanded in the last iteration, next smallest f-cost to be expanded and second smallest f-cost to be expanded.
 * 
 */
template <class state, class action>
void ImprovedBGS2<state, action>::FindtheBoundAndNextBoundF()
{

	double tmp = DBL_MAX;
	double tmp2 = DBL_MAX;
	double tmp3 = 0;
	for(uint64_t i = 0; i < q_f.size();i++)
	{
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kClosedList && fgreater(f_value,tmp3))
		{    // largest f-cost expanded in the last iteration
				tmp3 = f_value;
		}
	}
	previousBound = tmp3;   
	for(uint64_t i = 0; i < q_f.size();i++)
	{
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kOpenList)
		{	// next smallest f-cost to be expanded
			if(fgreatereq(f_value,tmp3) && fless(f_value,tmp))
			{
				tmp = f_value;
			}
		}
	}
	if(fequal(tmp,DBL_MAX))
	{
		tmp = tmp3;
	}
	bound = tmp;    // bound is the next f-cost that is to be expanded or the largest f-value expanded otherwise
	for(uint64_t i = 0; i < q_f.size();i++)
	{
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kOpenList)
		{
			if(fgreater(f_value,tmp))
			{
				if(fless(f_value,tmp2))
				{
				tmp2 = f_value;
			}
			}
		}
	}
	nextBound = tmp2;      // second smallest f-cost to be expanded.
	debug_print("previous bound %1.5f bound is %1.5f and nextBound is %1.5f\n",tmp3,bound,nextBound);
}

/**
 * Finds f-cost values in OPEN_g
 * It iterates through the entire list and finds the largest f-cost expanded in the last iteration, next smallest f-cost to be expanded and second smallest f-cost to be expanded.
 * 
 */

template <class state, class action>
void ImprovedBGS2<state, action>::FindtheBoundAndNextBoundG()
{

    double tmp = DBL_MAX;
	double tmp2 = DBL_MAX;
	double tmp3 = 0;
	for(uint64_t i = 0; i < q_g.size();i++)
	{
		state s = q_g.Lookup(i).data;
		double g_value = q_g.Lookup(i).g;
		double h_value = q_g.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_g.Lookup(hk,ok) == kClosedList && fgreater(f_value,tmp3))
		{       // largest f-cost expanded in the last iteration
				tmp3 = f_value;
		}
	}
	previousBound = tmp3;   
	for(uint64_t i = 0; i < q_g.size();i++)
	{
		state s = q_g.Lookup(i).data;
		double g_value = q_g.Lookup(i).g;
		double h_value = q_g.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kOpenList)
		{
			if(fgreatereq(f_value,tmp3) && fless(f_value,tmp))
			{   // next smallest f-cost to be expanded
				tmp = f_value;
			}
		}
	}
	if(fequal(tmp,DBL_MAX))
	{
		tmp = tmp3;
	}
	bound = tmp;    // bound is the next f-cost that is to be expanded or the largest f-value expanded otherwise
	for(uint64_t i = 0; i < q_g.size();i++)
	{
		state s = q_g.Lookup(i).data;
		double g_value = q_g.Lookup(i).g;
		double h_value = q_g.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_g.Lookup(hk,ok) == kOpenList)
		{    // second smallest f-cost to be expanded.
			if(fgreater(f_value,tmp))
			{
				if(fless(f_value,tmp2))
				{
				tmp2 = f_value;
			}
			}
		}
	}
	nextBound = tmp2;    
}


template <class state, class action>
void ImprovedBGS2<state, action>::SetupIterationF(double cost)
{

	q_f.Reset(env->GetMaxHash());
	q_f.AddOpenNode(start, env->GetStateHash(start), 0.0, h->HCost(start, goal));
    q_g.Reset(env->GetMaxHash());
	q_g.AddOpenNode(start, env->GetStateHash(start), 0.0, h->HCost(start, goal));

	previousBound = bound;
	bound = cost;
	nextBound = -1;
}



/**
 * Does Step Iteration bounded by f-cost.
 * 
 */

template <class state, class action>
bool ImprovedBGS2<state, action>::StepIterationUsingF()
{


    // Close the node
	uint64_t nodeid = q_f.Close();

	// To check if heap is balanced - - Use only for debugging
	/*
	hp.clear();
	for(int m = 0 ; m < q_f.OpenSize();m++){
		uint64_t temp_ok = q_f.GetOpenItem(m);
		hp.push_back(q_f.Lookup(temp_ok).g+q_f.Lookup(temp_ok).h);

	}
    assert((isHeap(0,hp.size()-1)) == true);
	*/

	// Close the node by objkey from q_g
    uint64_t ok_parent;
    uint64_t hk = env->GetStateHash(q_f.Lookup(nodeid).data);
    assert(q_g.Lookup(hk,ok_parent) == kOpenList);
    q_g.Close(ok_parent);
	
	if(q_f.Lookup(nodeid).reopened == true)
	{
		nodesReexpanded++;
	}
	else{
		nodesExpanded++;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);

	// Check if we are done
	if (env->GoalTest(q_f.Lookup(nodeid).data, goal))
	{
		ExtractPathToStartFromIDf(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		debug_print("%1.5f*************\n",env->GetPathLength(solutionPath));
		solutionCost = q_f.Lookup(nodeid).g;
		data.solutionInterval.lowerBound = solutionCost;
		debug_print("the solution cost is %1.5f has value is %lld\n",solutionCost, env->GetStateHash(q_f.Lookup(nodeid).data));
		return false;
	}
	
	debug_print("Expanding: %d with hash: %d with f-value %1.5f \n",q_f.Lookup(nodeid).data,env->GetStateHash(q_f.Lookup(nodeid).data), q_f.Lookup(nodeid).g + q_f.Lookup(nodeid).h);
	
	env->GetSuccessors(q_f.Lookup(nodeid).data, neighbors);
	double bestH = q_f.Lookup(nodeid).h;
	double lowHC = DBL_MAX;

	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(q_f.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(q_f.Lookup(nodeid).data, neighbors[x]));
		if (useBPMX)
		{
			if (neighborLoc.back() != kNotFound)
			{
				
				bestH = std::max(bestH, q_f.Lookup(theID).h-edgeCosts.back());
				lowHC = std::min(lowHC, q_f.Lookup(theID).h+edgeCosts.back());
			}
			else {
				double tmpH = h->HCost(neighbors[x], goal);
				bestH = std::max(bestH, tmpH-edgeCosts.back());
				lowHC = std::min(lowHC, tmpH+edgeCosts.back());
			}
		}
	}

	if (useBPMX) // propagate best child to parent
	{
	
		q_f.Lookup(nodeid).h = std::max(q_f.Lookup(nodeid).h, bestH);
		q_f.Lookup(nodeid).h = std::max(q_f.Lookup(nodeid).h, lowHC);
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		double childGCost = q_f.Lookup(nodeid).g+edgeCosts[x];
		double childFCost = childGCost+h->HCost(neighbors[x], goal);
		uint64_t hk = env->GetStateHash(neighbors[x]);
		uint64_t ok_child;
	//  std::cout << "looking at child with hash : " << hk << "and f-cost "<<childFCost<<std::endl;
	//	std::cout << "looking at child with hash : " << env->GetStateHash(neighbors[x]) << "and g-cost"<<childGCost<<std::endl;
		sd.f_above = std::min(sd.f_above, childFCost); //this is not used in bgs
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				debug_print("Closed node\n",1);
				if (useBPMX) // propagate parent to child - do this before potentially re-opening
				{
					if (fless(q_f.Lookup(neighborID[x]).h, bestH-edgeCosts[x]))
					{
						auto &i = q_f.Lookup(neighborID[x]);
						i.h = bestH-edgeCosts[x];
						q_g.Lookup(hk,ok_child);
						auto &j = q_g.Lookup(ok_child);
						j.h = bestH-edgeCosts[x];
					}
				}
				if(fless(childGCost,q_f.Lookup(neighborID[x]).g))
				{
					q_f.Lookup(neighborID[x]).parentID = nodeid;
					q_f.Lookup(neighborID[x]).g = childGCost;
					q_f.Reopen(neighborID[x]);
					q_f.Lookup(neighborID[x]).data = neighbors[x];
				
                    // Reopen it is q_g as well
                    assert(q_g.Lookup(hk,ok_child) == kClosedList);
                    q_g.Lookup(ok_child).parentID = ok_parent;
					q_g.Lookup(ok_child).g = childGCost;
					q_g.Reopen(ok_child);

				}
				break;
			case kOpenList:
			debug_print("Open node\n",1);
				if (fless(childGCost, q_f.Lookup(neighborID[x]).g))
				{
					debug_print("better path found\n",1);
					q_f.Lookup(neighborID[x]).parentID = nodeid;
					q_f.Lookup(neighborID[x]).g = childGCost;
					q_f.KeyChanged(neighborID[x]);

                    // Adjust the key in q_g
                    assert(q_g.Lookup(hk,ok_child) == kOpenList);
                    q_g.Lookup(ok_child).parentID = ok_parent;
					q_g.Lookup(ok_child).g = childGCost;
					q_g.KeyChanged(ok_child);

				}
				if (useBPMX) // propagate best child to parent
				{
					if (fgreater(bestH-edgeCosts[x], q_f.Lookup(neighborID[x]).h))
					{
						auto &i = q_f.Lookup(neighborID[x]);
						i.h = std::max(i.h, bestH-edgeCosts[x]);
						q_f.KeyChanged(neighborID[x]);

						q_g.Lookup(hk,ok_child);
						auto &j = q_g.Lookup(ok_child);
						j.h = std::max(j.h, bestH-edgeCosts[x]);
						q_g.KeyChanged(ok_child);
					}
				}
				break;
			case kNotFound:
				debug_print("Node not found\n",1);
				if(useBPMX)
				{
					q_f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  std::max(h->HCost(neighbors[x], goal), q_f.Lookup(nodeid).h-edgeCosts[x]),
							  nodeid);
					
					q_g.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  std::max(h->HCost(neighbors[x], goal), q_f.Lookup(ok_parent).h-edgeCosts[x]),
							  nodeid);
				}
				else{
					q_f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  nodeid);
                    
                    // Add node to q_g as well
                    assert(q_g.Lookup(hk,ok_child) == kNotFound);
                    q_g.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  ok_parent);
				}
				
			}
		}
	return false;
	
}

/**
 * Does Step Iteration bounded by g-cost.
 * 
 */



template <class state, class action>
bool ImprovedBGS2<state, action>::StepIterationUsingG()
{

	// Close the node
	uint64_t nodeid = q_g.Close();
    

	// To check if the heap is balanced - - use only for debugging
	/*
	hp.clear();
	for(int m = 0 ; m < q_f.OpenSize();m++){
		uint64_t temp_ok = q_f.GetOpenItem(m);
	    debug_print("heap element %d is %lld with f value %1.5f\n",m,env->GetStateHash(q_f.Lookup(temp_ok).data),q_f.Lookup(temp_ok).g+q_f.Lookup(temp_ok).h);
		hp.push_back(q_f.Lookup(temp_ok).g+q_f.Lookup(temp_ok).h);

	}
    assert((isHeap(0,hp.size()-1)) == true);
	*/
    // Close the node by objkey from q_f
    uint64_t ok_parent;
    uint64_t hk = env->GetStateHash(q_g.Lookup(nodeid).data);
    assert(q_f.Lookup(hk,ok_parent) == kOpenList);
    q_f.Close(ok_parent);
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	env->GetSuccessors(q_g.Lookup(nodeid).data, neighbors);
	nodesExpanded++;

	// Check if we are done
	if (env->GoalTest(q_g.Lookup(nodeid).data, goal))
	{
		ExtractPathToStartFromIDg(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		debug_print("%1.5f\n",env->GetPathLength(solutionPath));
		solutionCost = q_g.Lookup(nodeid).g;
		data.solutionInterval.lowerBound = solutionCost;
		debug_print("the solution cost is %1.5f has value is %lld\n",solutionCost, env->GetStateHash(q_g.Lookup(nodeid).data));
		return false;
	}
    debug_print("Expanding: %d with hash: %d with g-value %1.5f \n",q_g.Lookup(nodeid).data,env->GetStateHash(q_g.Lookup(nodeid).data), q_g.Lookup(nodeid).g + q_g.Lookup(nodeid).h);
	
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(q_g.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(q_g.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		double childGCost = q_g.Lookup(nodeid).g+edgeCosts[x];
		double childFCost = childGCost+h->HCost(neighbors[x], goal);
		uint64_t hk = env->GetStateHash(neighbors[x]);
		
		// std::cout << "looking at child with hash : " << hk << "and f-cost "<<childFCost<<std::endl;
		switch (neighborLoc[x])
		{
			case kClosedList:
				debug_print("Closed node\n",1);
				if(fless(childGCost,q_g.Lookup(neighborID[x]).g))
				{
					debug_print("better path found for closed\n",1);
					q_g.Lookup(neighborID[x]).parentID = nodeid;
					q_g.Lookup(neighborID[x]).g = childGCost;
					q_g.Reopen(neighborID[x]);

                    // Reopen it is q_f as well
					uint64_t ok_child;
                    assert(q_f.Lookup(hk,ok_child) == kClosedList);
                    q_f.Lookup(ok_child).parentID = ok_parent;
					q_f.Lookup(ok_child).g = childGCost;
					q_f.Reopen(ok_child);
					q_f.Lookup(ok_child).data = neighbors[x];
					
				}
				break;
			case kOpenList:
				debug_print("Open node\n",1);
				if (fless(childGCost, q_g.Lookup(neighborID[x]).g))
				{
					debug_print("better path found for open\n",1);
					q_g.Lookup(neighborID[x]).parentID = nodeid;
					q_g.Lookup(neighborID[x]).g = childGCost;
					q_g.KeyChanged(neighborID[x]);

                    // Adjust the key in q_f
					uint64_t ok_child;
                    assert(q_f.Lookup(hk,ok_child) == kOpenList);
                    q_f.Lookup(ok_child).parentID = ok_parent;
					q_f.Lookup(ok_child).g = childGCost;
					q_f.KeyChanged(ok_child);
				}
				break;
			case kNotFound:
				{	debug_print("Node not found\n",1);
					q_g.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  nodeid);

                    // Add node to q_f as well
					uint64_t ok_child;
                    assert(q_f.Lookup(hk,ok_child) == kNotFound);
                    q_f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  ok_parent);
				}
		}
	}
	return false;
	
}


template <class state, class action>
bool ImprovedBGS2<state, action>::StepIterationUsingG(int i)
{
	uint64_t nodeid_m;
	uint64_t nodeid_c;
    debug_print("checking heap in G size of open g is %d\n",q_g.OpenSize());
	nodeid_m = q_g.Close();
	debug_print("checking heap in G size of open g is %d\n",q_g.OpenSize());
	uint64_t hk = env->GetStateHash(q_g.Lookup(nodeid_m).data);
	assert(q_f.Lookup(hk,nodeid_c) == kOpenList);
	assert(env->GetStateHash(q_g.Lookup(nodeid_m).data) == env->GetStateHash(q_f.Lookup(nodeid_c).data));
	hp.clear();
	for(int m = 0 ; m < q_f.OpenSize();m++){
		uint64_t temp_ok = q_f.GetOpenItem(m);
		hp.push_back(q_f.Lookup(temp_ok).g+q_f.Lookup(temp_ok).h);

	}
	assert((isHeap(0,hp.size()-1)) == true);
	debug_print("checking heap in G size of open f is %d\n",q_f.OpenSize());
	q_f.Close(nodeid_c);
	assert(q_f.Lookup(hk,nodeid_c) == kClosedList);
	hp.clear();
	for(int m = 0 ; m < q_f.OpenSize();m++){
		uint64_t temp_ok = q_f.GetOpenItem(m);
		hp.push_back(q_f.Lookup(temp_ok).g+q_f.Lookup(temp_ok).h);

	}
	debug_print("checking heap in G size of open f is %d\n",q_f.OpenSize());
    assert((isHeap(0,hp.size()-1)) == true);
	
	neighbors_m.resize(0);
	edgeCosts_m.resize(0);
	neighborID_m.resize(0);
	neighborLoc_m.resize(0);
	
	env->GetSuccessors(q_g.Lookup(nodeid_m).data, neighbors_m);

	neighbors_c.resize(0);
	edgeCosts_c.resize(0);
	neighborID_c.resize(0);
	neighborLoc_c.resize(0);

    env->GetSuccessors(q_f.Lookup(nodeid_c).data, neighbors_c);
	nodesExpanded++;

	// Check if we are done
	if (env->GoalTest(q_g.Lookup(nodeid_m).data, goal))
	{
		ExtractPathToStartFromIDg(nodeid_m, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		debug_print("%1.5f*************\n",env->GetPathLength(solutionPath));
		solutionCost = q_g.Lookup(nodeid_m).g;
		data.solutionInterval.lowerBound = solutionCost;
		// printf("the solution cost is %1.5f has value is %lld\n",solutionCost, env->GetStateHash(q_g.Lookup(nodeid).data));
		return false;
	}


   // debug_print("Expanding: %d with hash: %d with g-value %1.5f \n",q_g.Lookup(nodeid).data,env->GetStateHash(q_g.Lookup(nodeid).data), q_g.Lookup(nodeid).g + q_g.Lookup(nodeid).h);
	
	
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors_m.size(); x++)
	{
		uint64_t theID;
		neighborLoc_m.push_back(q_g.Lookup(env->GetStateHash(neighbors_m[x]), theID));
		neighborID_m.push_back(theID);
		edgeCosts_m.push_back(env->GCost(q_g.Lookup(nodeid_m).data, neighbors_m[x]));
	}


	for (unsigned int x = 0; x < neighbors_c.size(); x++)
	{
		uint64_t theID;
		neighborLoc_c.push_back(q_f.Lookup(env->GetStateHash(neighbors_c[x]), theID));
		neighborID_c.push_back(theID);
		edgeCosts_c.push_back(env->GCost(q_f.Lookup(nodeid_c).data, neighbors_c[x]));
	}




	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors_m.size(); x++)
	{
		double childGCost = q_g.Lookup(nodeid_m).g+edgeCosts_m[x];
		double childFCost = childGCost+h->HCost(neighbors_m[x], goal);
		uint64_t hk = env->GetStateHash(neighbors_m[x]);
		
		// std::cout << "looking at child with hash : " << hk << "and f-cost "<<childFCost<<std::endl;
		switch (neighborLoc_m[x])
		{
			case kClosedList:
				debug_print("Closed node\n",1);
				if(fless(childGCost,q_g.Lookup(neighborID_m[x]).g))
				{
					debug_print("better path found for closed\n",1);
					q_g.Lookup(neighborID_m[x]).parentID = nodeid_m;
					q_g.Lookup(neighborID_m[x]).g = childGCost;
					q_g.Reopen(neighborID_m[x]);
					
				}
				break;
			case kOpenList:
				debug_print("Open node\n",1);
				if (fless(childGCost, q_g.Lookup(neighborID_m[x]).g))
				{
					debug_print("better path found for open\n",1);
					q_g.Lookup(neighborID_m[x]).parentID = nodeid_m;
					q_g.Lookup(neighborID_m[x]).g = childGCost;
					q_g.KeyChanged(neighborID_m[x]);

				}
				break;
			case kNotFound:
				{	debug_print("Node not found\n",1);
					q_g.AddOpenNode(neighbors_m[x],
							  env->GetStateHash(neighbors_m[x]),
							  childGCost,
							  h->HCost(neighbors_m[x], goal),
							  nodeid_m);

				}
		}
	}
    

	for (int x = 0; x < neighbors_c.size(); x++)
	{
		double childGCost = q_f.Lookup(nodeid_c).g+edgeCosts_c[x];
		double childFCost = childGCost+h->HCost(neighbors_c[x], goal);
		uint64_t hk = env->GetStateHash(neighbors_c[x]);
		
		// std::cout << "looking at child with hash : " << hk << "and f-cost "<<childFCost<<std::endl;
		switch (neighborLoc_c[x])
		{
			case kClosedList:
				debug_print("Closed node\n",1);
				if(fless(childGCost,q_f.Lookup(neighborID_c[x]).g))
				{
					debug_print("better path found for closed\n",1);
					q_f.Lookup(neighborID_c[x]).parentID = nodeid_c;
					q_f.Lookup(neighborID_c[x]).g = childGCost;
					q_f.Reopen(neighborID_c[x]);

					
				}
				break;
			case kOpenList:
				debug_print("Open node\n",1);
				if (fless(childGCost, q_f.Lookup(neighborID_c[x]).g))
				{
					debug_print("better path found for open\n",1);
					q_f.Lookup(neighborID_c[x]).parentID = nodeid_c;
					q_f.Lookup(neighborID_c[x]).g = childGCost;
					q_f.KeyChanged(neighborID_c[x]);

				}
				break;
			case kNotFound:
				{	debug_print("Node not found\n",1);
					q_f.AddOpenNode(neighbors_c[x],
							  env->GetStateHash(neighbors_c[x]),
							  childGCost,
							  h->HCost(neighbors_c[x], goal),
							  nodeid_c);

				}
		}
	}


	return false;
	
}


template <class state, class action>
bool ImprovedBGS2<state, action>::StepIterationUsingF(int i)
{
	uint64_t nodeid_m;
	uint64_t nodeid_c;
    
	nodeid_m = q_f.Close();
	uint64_t hk = env->GetStateHash(q_f.Lookup(nodeid_m).data);
	assert(q_g.Lookup(hk,nodeid_c) == kOpenList);
	q_g.Close(nodeid_c);
	debug_print("checking heap in F\n",1);
	hp.clear();
	for(int m = 0 ; m < q_f.OpenSize();m++){
		uint64_t temp_ok = q_f.GetOpenItem(m);
	    // printf("heap element %d is %lld with f value %1.5f\n",m,env->GetStateHash(q_f.Lookup(temp_ok).data),q_f.Lookup(temp_ok).g+q_f.Lookup(temp_ok).h);
		hp.push_back(q_f.Lookup(temp_ok).g+q_f.Lookup(temp_ok).h);

	}
    assert((isHeap(0,hp.size()-1)) == true);
	
	neighbors_m.resize(0);
	edgeCosts_m.resize(0);
	neighborID_m.resize(0);
	neighborLoc_m.resize(0);
	
	env->GetSuccessors(q_f.Lookup(nodeid_m).data, neighbors_m);

	neighbors_c.resize(0);
	edgeCosts_c.resize(0);
	neighborID_c.resize(0);
	neighborLoc_c.resize(0);

    env->GetSuccessors(q_g.Lookup(nodeid_c).data, neighbors_c);
	if(q_f.Lookup(nodeid_m).reopened == true)
	{
		nodesReexpanded++;
	}
	else{
		nodesExpanded++;
	}

	// Check if we are done
	if (env->GoalTest(q_f.Lookup(nodeid_m).data, goal))
	{
		ExtractPathToStartFromIDg(nodeid_m, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		debug_print("%1.5f\n",env->GetPathLength(solutionPath));
		solutionCost = q_f.Lookup(nodeid_m).g;
		data.solutionInterval.lowerBound = solutionCost;
		debug_print("the solution cost is %1.5f has value is %lld\n",solutionCost, env->GetStateHash(q_g.Lookup(nodeid).data));
		return false;
	}


    debug_print("Expanding: %d with hash: %d with g-value %1.5f \n",q_g.Lookup(nodeid).data,env->GetStateHash(q_g.Lookup(nodeid).data), q_g.Lookup(nodeid).g + q_g.Lookup(nodeid).h);
	
	
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors_m.size(); x++)
	{
		uint64_t theID;
		neighborLoc_m.push_back(q_f.Lookup(env->GetStateHash(neighbors_m[x]), theID));
		neighborID_m.push_back(theID);
		edgeCosts_m.push_back(env->GCost(q_f.Lookup(nodeid_m).data, neighbors_m[x]));
	}


	for (unsigned int x = 0; x < neighbors_c.size(); x++)
	{
		uint64_t theID;
		neighborLoc_c.push_back(q_g.Lookup(env->GetStateHash(neighbors_c[x]), theID));
		neighborID_c.push_back(theID);
		edgeCosts_c.push_back(env->GCost(q_g.Lookup(nodeid_c).data, neighbors_c[x]));
	}




	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors_m.size(); x++)
	{
		double childGCost = q_f.Lookup(nodeid_m).g+edgeCosts_m[x];
		double childFCost = childGCost+h->HCost(neighbors_m[x], goal);
		uint64_t hk = env->GetStateHash(neighbors_m[x]);
		
		// std::cout << "looking at child with hash : " << hk << "and f-cost "<<childFCost<<std::endl;
		switch (neighborLoc_m[x])
		{
			case kClosedList:
				debug_print("Closed node\n",1);
				if(fless(childGCost,q_f.Lookup(neighborID_m[x]).g))
				{
					debug_print("better path found for closed\n",1);
					q_f.Lookup(neighborID_m[x]).parentID = nodeid_m;
					q_f.Lookup(neighborID_m[x]).g = childGCost;
					q_f.Reopen(neighborID_m[x]);
					
				}
				break;
			case kOpenList:
				debug_print("Open node\n",1);
				if (fless(childGCost, q_f.Lookup(neighborID_m[x]).g))
				{
					debug_print("better path found for open\n",1);
					q_f.Lookup(neighborID_m[x]).parentID = nodeid_m;
					q_f.Lookup(neighborID_m[x]).g = childGCost;
					q_f.KeyChanged(neighborID_m[x]);

				}
				break;
			case kNotFound:
				{	debug_print("Node not found\n",1);
					q_f.AddOpenNode(neighbors_m[x],
							  env->GetStateHash(neighbors_m[x]),
							  childGCost,
							  h->HCost(neighbors_m[x], goal),
							  nodeid_m);

				}
		}
	}
    

	for (int x = 0; x < neighbors_c.size(); x++)
	{
		double childGCost = q_g.Lookup(nodeid_c).g+edgeCosts_c[x];
		double childFCost = childGCost+h->HCost(neighbors_c[x], goal);
		uint64_t hk = env->GetStateHash(neighbors_c[x]);
		
		// std::cout << "looking at child with hash : " << hk << "and f-cost "<<childFCost<<std::endl;
		switch (neighborLoc_c[x])
		{
			case kClosedList:
				debug_print("Closed node\n",1);
				if(fless(childGCost,q_g.Lookup(neighborID_c[x]).g))
				{
					debug_print("better path found for closed\n",1);
					q_g.Lookup(neighborID_c[x]).parentID = nodeid_c;
					q_g.Lookup(neighborID_c[x]).g = childGCost;
					q_g.Reopen(neighborID_c[x]);

					
				}
				break;
			case kOpenList:
				debug_print("Open node\n",1);
				if (fless(childGCost, q_g.Lookup(neighborID_c[x]).g))
				{
					debug_print("better path found for open\n",1);
					q_g.Lookup(neighborID_c[x]).parentID = nodeid_c;
					q_g.Lookup(neighborID_c[x]).g = childGCost;
					q_g.KeyChanged(neighborID_c[x]);

				}
				break;
			case kNotFound:
				{	debug_print("Node not found\n",1);
					q_g.AddOpenNode(neighbors_c[x],
							  env->GetStateHash(neighbors_c[x]),
							  childGCost,
							  h->HCost(neighbors_c[x], goal),
							  nodeid_c);

				}
		}
	}


	return false;
	
}


template <class state, class action>
void ImprovedBGS2<state, action>::Draw(Graphics::Display &disp) const
{
	/* TODO : see how to set for both the queues*/
	double transparency = 1.0; 
	if (q_f.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (q_f.OpenSize() > 0)
	{
		top = q_f.Peek();
	}
	for (unsigned int x = 0; x < q_f.size(); x++)
	{
		const auto &data = q_f.Lookat(x);
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

template <class state, class action>
void ImprovedBGS2<state, action>::OpenGLDraw()
{
}

template <class state, class action>
void ImprovedBGS2<state, action>::PrintQueueStatus()
{
int c1 = 0;
std::vector< std::pair<uint64_t,float> > g_list;
std::vector< std::pair<uint64_t,float> > f_list;
	for(int i = 0; i < q_g.size();i++)
	{
		state s = q_g.Lookup(i).data;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;

		if(q_g.Lookup(hk,ok) == kOpenList)
		{
			printf("hash value %lld is open at %lld and in g with f-value %1.5f\n",hk,ok,q_g.Lookup(i).g + q_g.Lookup(i).h);
			g_list.push_back(std::make_pair(hk,q_g.Lookup(i).g + q_g.Lookup(i).h));
		}
		else if(q_g.Lookup(hk,ok) == kClosedList)
		{
			printf("hash value %lld is closed at %lld and in g with f-value %1.5f\n",hk,ok,q_g.Lookup(i).g + q_g.Lookup(i).h);
			g_list.push_back(std::make_pair(hk,q_g.Lookup(i).g + q_g.Lookup(i).h));
		}
		else {
			printf("hash value %lld is notfound in g\n",hk);
		}
		if(q_g.Lookup(hk,ok) != kNotFound)
		{
			c1++;
		}
	}
	int c2 = 0;
	for(int i = 0; i < q_f.size();i++)
	{
		state s = q_f.Lookup(i).data;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kOpenList)
		{
			printf("hash value %lld is open at %lld and in f with f-value %1.5f\n",hk,ok,q_f.Lookup(i).g + q_f.Lookup(i).h);
			f_list.push_back(std::make_pair(hk,q_f.Lookup(i).g + q_f.Lookup(i).h));
		}
		else if(q_f.Lookup(hk,ok) == kClosedList)
		{
			printf("hash value %lld is closed at %lld and in f with f-value %1.5f\n",hk,ok,q_f.Lookup(i).g + q_f.Lookup(i).h);
			f_list.push_back(std::make_pair(hk,q_f.Lookup(i).g + q_f.Lookup(i).h));
		}
		else {
			printf("hash value %lld is notfound in f\n",hk);
		}
		if(q_f.Lookup(hk,ok) != kNotFound)
		{
			c2++;
		}
	}
	
	std::sort(g_list.begin(), g_list.end());
	std::sort(f_list.begin(), f_list.end());
	
}

#endif /* ImprovedBGS_h */
