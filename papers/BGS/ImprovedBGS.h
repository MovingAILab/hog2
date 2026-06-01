//
//  ImprovedBGS.h
//  hog2
//
//  This is an incremental version of Budgeted Graph Search - IBEX on trees with Bounded Dijkstra at the low level.
//  This code isn't meant for production - just to animate how the algorithm works as a comparison to IDA*.
//  See (Helmert et al, IJCAI 2019) for details of the algorithm
//
// This implementation contains working of Algo2 for the efficient BGS. 
// The search can start from the current f-cost without discarding any nodes in the queue.
// There is only need to update the budget values correctly.

#ifndef ImprovedBGS_h
#define ImprovedBGS_h

#include "Heuristic.h"
#include "AStarOpenClosed.h"

template <class state, class action>
class ImprovedBGS {
public:
	ImprovedBGS(double initialBound = 0) :bound(initialBound), initialBound(initialBound), nextBound(initialBound)
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
	int b_num = 10;
	const double k = 20;
	const int infiniteWorkBound = -1;
	void GetGlobalCostInterval(double &lower, double &upper)
	{ lower = data.solutionInterval.lowerBound; upper = data.solutionInterval.upperBound; }
	void GetNodeInterval(uint64_t &lower, uint64_t &upper)
	{ lower = data.nodeLB*c1; upper = data.workBound; }
	std::string stage;
	std::string fEquation;
	void SetUseBPMX();
	void SetB(int a);
	double GetSolutionCost(){return solutionCost;}
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
	bool IterationCompleteG() { return q_g.OpenSize() == 0; }
	bool IterationCompleteF() { return q_f.OpenSize() == 0; }
	unsigned long nodesTouched;
	
	void SetupIterationF(double cost);
	bool StepIterationUsingF();
	bool StepIterationUsingG();
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	void ExtractPathToStartFromIDF(uint64_t node, std::vector<state> &thePath);
	void ExtractPathToStartFromIDG(uint64_t node, std::vector<state> &thePath);
    void GetNodesFromGq();
	void GetAllNodesinG();
    void GetNodeswithBoundinG();
	void GetNodeswithBoundinGAboveinF();
	void FindtheBoundAndNextBoundF();
	void FindtheBoundAndNextBoundG();
	void FindtheCurrentBoundF();
	bool MainIterationComplete();
	void PrintQueueStatus();
	std::vector<std::pair<state, double>> history;
	std::vector<state> solutionPath;
	state start, goal;
	double previousBound;
	double bound;
	double initialBound;
	double nextBound;
	double bound_g;
	bool case3 = 0;
	bool case2 = 0;
	bool useBPMX = false;
	SearchEnvironment<state, action> *env;
	Heuristic<state> *h;
	std::vector<state> succ;
	uint64_t newNodeCount, newNodesLastIteration;
	double fc;
	double fc_next;
	int MInodesexpanded;
	int MInodesreexpanded;
	unsigned long nodesExpanded;
	unsigned long nodesReexpanded;
	bool MODE = 0;
	bool SetupExponentialBinaryIteration = false;
	int cnt = 0;
	bool iterationComplete = false;
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
	AStarOpenClosed<state, BFHSCompare> q;
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	std::vector<state> solutionStates;
};

template <class state, class action>
void ImprovedBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											Heuristic<state> *h, std::vector<state> &thePath)
{
	if (InitializeSearch(e, from, to, h, thePath))
		return;
	while (!DoSingleSearchStep(thePath))
	{}
}

template <class state, class action>
void ImprovedBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool ImprovedBGS<state, action>::InitializeSearch(SearchEnvironment<state, action> *e, state from, state to, Heuristic<state> *h,
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

	/// Setup BTS control data
/*	data.nodesExpanded = 0;
	data.workBound = infiniteWorkBound;
	data.nodeLB = 0;
	data.solutionInterval.lowerBound = h->HCost(start, goal);
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 0;
	fEquation = to_string_with_precision(h->HCost(start, goal), 0);
	solutionCost = DBL_MAX;
	SetupIteration(h->HCost(start, goal));
	stage = "NEW ITERATION";
*/
	//nodeLB = nodesExpanded ;
	data.nodesExpanded = 0;
	data.nodesReexpanded = 0;
	data.workBound = infiniteWorkBound;
	data.nodeLB = 0;
	data.solutionInterval.lowerBound = h->HCost(start, goal);  //this is the f-cost for the next iteration
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 1;
	fEquation = to_string_with_precision(h->HCost(start, goal), 0);
    q_g.Reset(env->GetMaxHash());
	q_f.Reset(env->GetMaxHash());
	fc = DBL_MAX;
	fc_next = DBL_MAX;
	MInodesexpanded = 0;
	MInodesreexpanded = 0;
	SetupIterationF(h->HCost(start, goal));
	solutionCost = DBL_MAX;
	stage = "INITIALIZE SEARCH";
	nodesExpanded = 0;
	nodesReexpanded = 0;
	return false;
}
template <class state, class action>
bool ImprovedBGS<state, action>::MainIterationComplete()
{
	if(data.solutionInterval.upperBound != DBL_MAX){
		return true;
	}
	else{
		return false;
	}
}

template <class state, class action>
void ImprovedBGS<state, action>::SetUseBPMX()
{
	useBPMX = true;
}


template <class state, class action>
void ImprovedBGS<state, action>::SetB(int a)
{

    if( a == 1){
		b_num = 1;
	}
	else if(a == 2){
		b_num = 2;
	}
	else if(a == 3){
		b_num = 3;
	}
	else if(a == 4){
		b_num = 4;
	}
	else{
		b_num = 5;
	}
}

template <class state, class action>
void ImprovedBGS<state, action>::SetupIterationF(double cost)
{

	/* if the g list is empty, get the start node, otherwise use the cost value to put all the nodes from g to f */
    if(q_g.empty()){
		q_f.Reset(env->GetMaxHash());
		q_f.AddOpenNode(start, env->GetStateHash(start), 0.0, h->HCost(start, goal));
	}
	else{
		GetAllNodesinG();
		GetNodesFromGq();
	}

	previousBound = bound;
	bound = cost;//nextBound;
	nextBound = -1;
	//printf("Starting iteration bound %1.1f\n", bound);
}

template <class state, class action>
void ImprovedBGS<state, action>::ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
{
	/* TODO */
	thePath.clear();
	/*do {
		thePath.push_back(q.Lookup(node).data);
		node = q.Lookup(node).parentID;
	} while (q.Lookup(node).parentID != node);
	thePath.push_back(q.Lookup(node).data);
	*/
	
}
template <class state, class action>
void ImprovedBGS<state, action>::ExtractPathToStartFromIDF(uint64_t node, std::vector<state> &thePath)
{
	
	
}
template <class state, class action>
void ImprovedBGS<state, action>::ExtractPathToStartFromIDG(uint64_t node, std::vector<state> &thePath)
{
	
	
}


template <class state, class action>
void ImprovedBGS<state, action>::GetNodeswithBoundinGAboveinF(){

	for(uint64_t i = 0; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t node1;
		uint64_t node2;
		uint64_t node3;
		uint64_t parentId = q_f.Lookup(i).parentID;
		if(q_f.Lookup(hk,node1) == kOpenList && flesseq(f_value,bound_g)){
			if(q_g.Lookup(hk,node2) == kNotFound){
				q_g.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
				q_f.Remove(env->GetStateHash(s));
			}
			else if(q_g.Lookup(hk,node2) == kOpenList && fless(g_value, q_g.Lookup(node2).g)){
					q_g.Lookup(node2).parentID = parentId;
					q_g.Lookup(node2).g = g_value;
					q_g.KeyChanged(node2);
					q_f.Remove(env->GetStateHash(s));
				}
			else if(q_g.Lookup(hk,node2) == kOpenList && !fless(g_value, q_g.Lookup(node2).g)){
					q.Remove(hk);
			}
			}
		else if(q_f.Lookup(hk,node1) == kOpenList && fgreater(f_value,bound_g)){
			if(q.Lookup(hk,node3) == kNotFound){
				q.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
				q_f.Remove(env->GetStateHash(s));
			}
			else if(q.Lookup(hk,node3) == kOpenList && fless(g_value, q.Lookup(node3).g)){
					q.Lookup(node3).parentID = parentId;
					q.Lookup(node3).g = g_value;
					q.KeyChanged(node3);
					q_f.Remove(env->GetStateHash(s));
				}
			else if(q.Lookup(hk,node3) == kOpenList && !fless(g_value, q.Lookup(node3).g)){
					q.Remove(hk);
			}
			}
	}
}


template <class state, class action>
void ImprovedBGS<state, action>::GetNodesFromGq()
{
	for(uint64_t i = 0; i < q_g.size();i++){
		state s = q_g.Lookup(i).data;
		double g_value = q_g.Lookup(i).g;
		double h_value = q_g.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t nodeid_g;
		uint64_t nodeid_f;
		uint64_t parentId = q_g.Lookup(i).parentID;

		if(q_g.Lookup(hk,nodeid_g) == kOpenList){
			if(q_f.Lookup(hk,nodeid_f) == kNotFound){
				q_f.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
				q_g.Remove(hk);
			}
			else if(q_f.Lookup(hk,nodeid_f) == kOpenList && fless(g_value, q_f.Lookup(nodeid_f).g)){
					q_f.Lookup(nodeid_f).parentID = parentId;
					q_f.Lookup(nodeid_f).g = g_value;
					q_f.KeyChanged(nodeid_f);
					q_g.Remove(hk);
			}
			else if(q_f.Lookup(hk,nodeid_f) == kClosedList  &&fless(g_value, q_f.Lookup(nodeid_f).g)){
					q_f.Lookup(nodeid_f).parentID = i;
					q_f.Lookup(nodeid_f).g = g_value;
					q_f.Reopen(nodeid_f);
					q_g.Remove(hk);
			}
			else if(q_f.Lookup(hk,nodeid_f) != kNotFound  &&  !fless(g_value, q_f.Lookup(nodeid_f).g)){
				    q_g.Remove(hk);

			}
		}
		else if(q_g.Lookup(hk,nodeid_g) == kClosedList){
			if(q_f.Lookup(hk,nodeid_f) == kNotFound){
				q_f.AddClosedNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
			}
			else if(q_f.Lookup(hk,nodeid_f) == kOpenList){
				    q_f.Lookup(nodeid_f).parentID = parentId;
					q_f.Lookup(nodeid_f).g = g_value;
					q_f.KeyChanged(nodeid_f);
			}
			else if(q_f.Lookup(hk,nodeid_f) == kClosedList){
					q_f.Lookup(nodeid_f).parentID = i;
					q_f.Lookup(nodeid_f).g = g_value;
					q_f.Reopen(nodeid_f);
			}
		}
		}
	assert(q_g.OpenSize() == 0);
	q_g.Reset();

}

template <class state, class action>
void ImprovedBGS<state, action>::GetAllNodesinG()
{
	for(uint64_t i = 0; i < q.size();i++){
		state s = q.Lookup(i).data;
		double g_value = q.Lookup(i).g;
		double h_value = q.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		uint64_t parentId = q.Lookup(i).parentID;
		if(q.Lookup(hk,ok) == kOpenList){
			if(q_g.Lookup(hk,ok) == kNotFound){
				q_g.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
			    q.Remove(hk);
			}
			else if(q_g.Lookup(hk,ok) == kOpenList && fless(g_value, q_g.Lookup(ok).g)){
					q_g.Lookup(ok).parentID = parentId;
					q_g.Lookup(ok).g = g_value;
					q_g.KeyChanged(ok);
					q.Remove(hk);
			}
			else if(q_g.Lookup(hk,ok) == kOpenList && !fless(g_value, q_g.Lookup(ok).g)){
					q.Remove(hk);
			}
			else if(q_g.Lookup(hk,ok) == kClosedList){
				q.Remove(hk);
			}	
		}
	}
   assert(q.OpenSize()==0);
   q.Reset(env->GetMaxHash());
}

template <class state, class action>
void ImprovedBGS<state, action>::GetNodeswithBoundinG()
{
	//all the opennodes with updated g value are sitting in q
	for(uint64_t i = 0; i < q.size();i++){
		state s = q.Lookup(i).data;
		double g_value = q.Lookup(i).g;
		double h_value = q.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		uint64_t parentId = q.Lookup(i).parentID;
		if(q.Lookup(hk,ok) == kOpenList  && flesseq(f_value,bound_g)){
			if(q_g.Lookup(hk,ok) == kNotFound){
				q_g.AddOpenNode(s,
						env->GetStateHash(s),
						g_value,
						h_value,
						parentId);
				q.Remove(env->GetStateHash(s)); //this is very important so that these nodes are not added agin
			}
			else if(q_g.Lookup(hk,ok) == kOpenList && fless(g_value, q_g.Lookup(ok).g)){
					q_g.Lookup(ok).parentID = parentId;
					q_g.Lookup(ok).g = g_value;
					q_g.KeyChanged(ok);
					q.Remove(hk);
			}
			else if(q_g.Lookup(hk,ok) == kOpenList && !fless(g_value, q_g.Lookup(ok).g)){
					q.Remove(hk);
			}
			else if(q_g.Lookup(hk,ok) == kClosedList){
				q.Remove(hk);
			}	
			
		}
	}
}


template <class state, class action>
void ImprovedBGS<state, action>::FindtheBoundAndNextBoundF(){

	double tmp = DBL_MAX;
	double tmp2 = DBL_MAX;
	double tmp3 = 0;
	for(uint64_t i = 0; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kClosedList && fgreater(f_value,tmp3)){
				tmp3 = f_value;
		}
	}
	previousBound = tmp3;   
	for(uint64_t i = 0; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kOpenList){
			if(fgreatereq(f_value,tmp3) && fless(f_value,tmp)){
				tmp = f_value;
			}
		}
	}
	if(fequal(tmp,DBL_MAX)){
		tmp = tmp3;
	}
	bound = tmp;    //bound is the next f-cost that is to be expanded or the largest f-value expanded otherwise
	for(uint64_t i = 0; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kOpenList){
			if(fgreater(f_value,tmp)){
				if(fless(f_value,tmp2)){
				tmp2 = f_value;
			}
			}
		}
	}
	nextBound = tmp2;    
	//printf("previous bound %1.5f bound is %1.5f and nextBound is %1.5f\n",tmp3,bound,nextBound);
}

/* Finds the largest f-cost expanded in the iteration */
template <class state, class action>
void ImprovedBGS<state, action>::FindtheCurrentBoundF(){

	double tmp = 0.0;
	double tmp2 = DBL_MAX;
	for(uint64_t i = 0; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		double g_value = q_f.Lookup(i).g;
		double h_value = q_f.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kClosedList){
			if(fgreater(f_value,tmp)){
				tmp = f_value;
			}
		}
	}
	previousBound = tmp;
	//printf("nodes with f-value %1.5f have been closed\n",previousBound);
}



template <class state, class action>
void ImprovedBGS<state, action>::FindtheBoundAndNextBoundG(){

    double tmp = DBL_MAX;
	double tmp2 = DBL_MAX;
	//all the nodes in g are closed; find the next open from f
	
	for(uint64_t i = 0; i < q.size();i++){
		state s = q.Lookup(i).data;
		double g_value = q.Lookup(i).g;
		double h_value = q.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q.Lookup(hk,ok) == kOpenList){
			if(fless(f_value,tmp)){
				tmp = f_value;
			}
		}
	}
	if(fequal(tmp,DBL_MAX)){
		tmp = bound_g;  //the bound used to close the last nodes
	}
	bound = tmp;
	for(uint64_t i = 0; i < q.size();i++){
		state s = q.Lookup(i).data;
		double g_value = q.Lookup(i).g;
		double h_value = q.Lookup(i).h;
		double f_value = g_value+h_value;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q.Lookup(hk,ok) == kOpenList){
			if(fgreater(f_value,tmp)){
				if(fless(f_value,tmp2)){
				tmp2 = f_value;
			}
			}
		}
	}
	nextBound = tmp2;
}




template <class state, class action>
bool ImprovedBGS<state, action>::StepIterationUsingF()
{

	//PrintQueueStatus();
	uint64_t nodeid = q_f.Close();
	//printf("StepIteration - f\n");
	
	if(q_f.Lookup(nodeid).reopened == true){
		//std::cout << "reExpanding: " << q_f.Lookup(nodeid).data << " with hash: " << env->GetStateHash(q_f.Lookup(nodeid).data) << "with f-value " << q_f.Lookup(nodeid).g + h->HCost(q_f.Lookup(nodeid).data, goal)<<std::endl;
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
		ExtractPathToStartFromID(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		solutionCost = q_f.Lookup(nodeid).g;
		data.solutionInterval.lowerBound = solutionCost;
		printf("the solution cost is %1.5f has value is %lld\n",solutionCost, env->GetStateHash(q_f.Lookup(nodeid).data));
		return false;
	}
	
	//std::cout << "Expanding: " << q_f.Lookup(nodeid).data << " with hash: " << env->GetStateHash(q_f.Lookup(nodeid).data) << "with f-value " << q_f.Lookup(nodeid).g + h->HCost(q_f.Lookup(nodeid).data, goal)<<std::endl;
	
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
		uint64_t ok;
	//	std::cout << "looking at child with hash : " << env->GetStateHash(neighbors[x]) << "and g-cost"<<childGCost<<std::endl;
		sd.f_above = std::min(sd.f_above, childFCost); //this is not used in bgs
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				if (useBPMX) // propagate parent to child - do this before potentially re-opening
				{
					if (fless(q_f.Lookup(neighborID[x]).h, bestH-edgeCosts[x]))
					{
						auto &i = q_f.Lookup(neighborID[x]);
						i.h = bestH-edgeCosts[x];
					}
				}
				if(fless(childGCost,q_f.Lookup(neighborID[x]).g)){
					q_f.Lookup(neighborID[x]).parentID = nodeid;
					q_f.Lookup(neighborID[x]).g = childGCost;
					q_f.Reopen(neighborID[x]);
				}
				break;
			case kOpenList:
				if (fless(childGCost, q_f.Lookup(neighborID[x]).g))
				{
					q_f.Lookup(neighborID[x]).parentID = nodeid;
					q_f.Lookup(neighborID[x]).g = childGCost;
					q_f.KeyChanged(neighborID[x]);
				}
				if (useBPMX) // propagate best child to parent
				{
					if (fgreater(bestH-edgeCosts[x], q_f.Lookup(neighborID[x]).h))
					{
						auto &i = q_f.Lookup(neighborID[x]);
						i.h = std::max(i.h, bestH-edgeCosts[x]);
						q_f.KeyChanged(neighborID[x]);
					}
				}
				break;
			case kNotFound:
				
				if(useBPMX){
					q_f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  std::max(h->HCost(neighbors[x], goal), q_f.Lookup(nodeid).h-edgeCosts[x]),
							  nodeid);
				}
				else{
					q_f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  nodeid);
				}
				
			}
		}
	return false;
	
}


template <class state, class action>
bool ImprovedBGS<state, action>::StepIterationUsingG()
{
	//PrintQueueStatus();
	uint64_t nodeid = q_g.Close();
	
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	
	env->GetSuccessors(q_g.Lookup(nodeid).data, neighbors);
	nodesExpanded++;
	// Check if we are done
	if (env->GoalTest(q_g.Lookup(nodeid).data, goal))
	{
		ExtractPathToStartFromID(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		solutionCost = q_g.Lookup(nodeid).g;
		data.solutionInterval.lowerBound = solutionCost;
		printf("the solution cost is %1.5f has value is %lld\n",solutionCost, env->GetStateHash(q_g.Lookup(nodeid).data));
		return false;
	}
	//std::cout << "Expanding: " << q_g.Lookup(nodeid).data << " with hash: " << env->GetStateHash(q_g.Lookup(nodeid).data) << "with f-value " << q_g.Lookup(nodeid).g + h->HCost(q_g.Lookup(nodeid).data, goal)<<std::endl;
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
		uint64_t ok;
		if (fgreater(childFCost, bound_g))
		{

			if(q.Lookup(hk,ok) == kNotFound){
				q.AddOpenNode(neighbors[x],
						env->GetStateHash(neighbors[x]),
						childGCost,
						h->HCost(neighbors[x], goal),
						nodeid);
			}
			else if(q.Lookup(hk,ok) == kOpenList && fless(childGCost, q.Lookup(ok).g)){
					q.Lookup(ok).parentID = nodeid;
					q.Lookup(ok).g = childGCost;
					q.KeyChanged(ok);
				}
			continue;
		}
		switch (neighborLoc[x])
		{
			case kClosedList:
				if(fless(childGCost,q_g.Lookup(neighborID[x]).g)){
					q_g.Lookup(neighborID[x]).parentID = nodeid;
					q_g.Lookup(neighborID[x]).g = childGCost;
					q_g.Reopen(neighborID[x]);
				}
				break;
			case kOpenList:
				if (fless(childGCost, q_g.Lookup(neighborID[x]).g))
				{
					q_g.Lookup(neighborID[x]).parentID = nodeid;
					q_g.Lookup(neighborID[x]).g = childGCost;
					q_g.KeyChanged(neighborID[x]);
				}
				break;
			case kNotFound:
				{
					
					q_g.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  nodeid);
				}
		}
	}
	return false;
	
}



template <class state, class action>
bool ImprovedBGS<state, action>::DoSingleSearchStep(std::vector<state> &thePath)
{

    int b = k * data.nodeLB;
	if( b_num == 1){
		b = k * data.nodeLB;
	}
	else if(b_num == 2){
		b = k * data.nodeLB;
		b = sqrt(b);
	}
	else if(b_num == 3){
		b = k * data.nodeLB;
	    b = cbrt(b);
	}
	else if(b_num == 4){
		b = 0;
	}
	else{
		b = 10000;
	}
  

	cnt+=1;
	if(!MainIterationComplete()){
		if (flesseq(solutionCost, data.solutionInterval.lowerBound))
				{
					thePath = solutionPath;
					//printf("Found solution cost %1.5f\n", solutionCost);    //if the solution is found
					return true;
				}
		if(!MODE){
			if(data.nodesReexpanded <= b && data.nodesExpanded <= c1*data.nodeLB){    //case 1 : Run A* when within the re-expansion budget
				int temp1 = nodesExpanded;
				int temp2 = nodesReexpanded;
				if(!IterationCompleteF()){
					StepIterationUsingF();
				}
				else {
					GetAllNodesinG();
					GetNodesFromGq();
					StepIterationUsingF();
				}
		
				data.nodesExpanded += nodesExpanded-temp1;
				data.nodesReexpanded += nodesReexpanded -temp2;
				return false;
				}
			else if(data.nodesReexpanded <= b && data.nodesExpanded > c1*data.nodeLB){  //case 2 : Expanded nodes by a factor, Run A* 
				if(!case2){
				    FindtheCurrentBoundF();
					case2 = 1;
					return false;
				}
				else {
					if(!q_f.empty()){
						uint64_t node = q_f.Peek();
						state s = q_f.Lookup(node).data;
						double g_value = q_f.Lookup(node).g;
						double h_value = q_f.Lookup(node).h;
						double f_value = g_value+h_value;
						//printf("f-value is %1.5f anf previous bound is %1.5f\n",f_value,previousBound);
						if(flesseq(f_value,previousBound)){      //expand all the nodes with the previousBound
							int temp1 = nodesExpanded;
							int temp2 = nodesReexpanded;
							StepIterationUsingF();
							data.nodesExpanded += nodesExpanded-temp1;
							data.nodesReexpanded += nodesReexpanded -temp2;
							return false;
						}
						data.solutionInterval.lowerBound = previousBound;   
						data.solutionInterval.upperBound = previousBound;
						FindtheBoundAndNextBoundF();   //find the previousBound, bound and nextBound
						return false;
				   }
				data.solutionInterval.lowerBound = previousBound;
				data.solutionInterval.upperBound = previousBound;
				FindtheBoundAndNextBoundF();  //find the previousBound, bound and nextBound
				return false;
				}
				

			}
			else if(data.nodesReexpanded > b && data.nodesExpanded > c1*data.nodeLB) {  //case 3: Grew by a factor but can not afford any more re-expansions
				if(!case3){
					FindtheBoundAndNextBoundF();
					bound_g = previousBound;
				    GetNodeswithBoundinGAboveinF();
					case3 = 1;
					return false;
				}
				else{
				    if(!IterationCompleteG()){
						int temp1 = nodesExpanded;
						StepIterationUsingG();
						data.nodesExpanded += nodesExpanded-temp1;
						return false;
				    }
					data.solutionInterval.lowerBound = previousBound;
					data.solutionInterval.upperBound = previousBound;
					GetAllNodesinG();
					GetNodesFromGq();
					FindtheBoundAndNextBoundF();   // end of main interation
					return false;
				}
			}
			else {
				if(!SetupExponentialBinaryIteration){    //case 4: Exponential Search
					FindtheCurrentBoundF();
					FindtheBoundAndNextBoundF();
					if(fequal(previousBound,bound)){
						bound_g = previousBound;
					}
					else{
						bound_g = bound;
					}
					GetNodeswithBoundinGAboveinF(); //move from q_f to q_g and q once bound_g is set
					SetupExponentialBinaryIteration = true;
					iterationComplete = false;
					MODE = 1;
					return false;
			}
			}
		}
	else{      
				if (flesseq(solutionCost, data.solutionInterval.lowerBound))
				{
					thePath = solutionPath;
					//printf("Found solution cost %1.5f\n", solutionCost);
					return true;
				}
				//printf("--> Iteration- %" PRId64 " expanded; target [%" PRId64 ", %" PRId64 ")\n", data.nodesExpanded, c1*data.nodeLB, c2*data.nodeLB);
				if(!IterationCompleteG()  && data.nodesExpanded < data.workBound){
					
					int temp1 = nodesExpanded;
					StepIterationUsingG();
					data.nodesExpanded += nodesExpanded-temp1;
					return false;

				}
				
				// Invariant - iteration complete *or* exceeded work limit
				// last search completed - set the interval from this search
				if (data.nodesExpanded >= data.workBound)
				{
					data.solutionInterval.lowerBound = 0;
					data.solutionInterval.upperBound = sd.f_below;
				}
				else if (solutionCost != DBL_MAX && fgreatereq(sd.f_below, solutionCost))
				{
					data.solutionInterval.lowerBound = solutionCost;
					data.solutionInterval.upperBound = solutionCost;
				}
				else {
					//FindtheBoundAndNextBoundG();
					data.solutionInterval.lowerBound = bound_g;
					data.solutionInterval.upperBound = DBL_MAX;
				}
			    //printf("--> Iteration complete - %" PRId64 " expanded; target [%" PRId64 ", %" PRId64 ")\n", data.nodesExpanded, c1*data.nodeLB, c2*data.nodeLB);
				
				// Move to next iteration
				if (data.nodesExpanded >= c1*data.nodeLB && data.workBound == infiniteWorkBound)
				{
					//printf("Expanded %" PRId64 " - needed at least %" PRId64 "\n", data.nodesExpanded, c1*data.nodeLB);
					if (data.solutionInterval.lowerBound == DBL_MAX) // No new nodes in iteration
						printf("[HIT]--Critical f in [%1.5f, %1.5f]\n", solutionCost, solutionCost);
					else
						printf("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
					data.solutionInterval.upperBound = bound_g;
					fEquation = to_string_with_precision(nextBound, 0);
					previousBound = bound_g;
					FindtheBoundAndNextBoundG();
					GetAllNodesinG();  //move everything from temp to g 
					GetNodesFromGq();  //move everything from g to f
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
						printf("    ]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
					else
						printf("    ]--Critical f in [%1.5f, %1.5f]\n", data.solutionInterval.lowerBound, data.solutionInterval.upperBound);
						
					double nextCost;
					data.delta *= gamma;
					if (data.solutionInterval.upperBound == DBL_MAX)
					{

						//nextCost = data.solutionInterval.lowerBound+pow(gamma, data.delta);
						nextCost = data.solutionInterval.lowerBound * data.delta;
						fEquation = to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+to_string_with_precision(gamma, 0)+"^"+to_string_with_precision(data.delta, 0)+"="+to_string_with_precision(nextCost, 0);
						stage = "EXP";
					}
					else {
						nextCost = (data.solutionInterval.lowerBound+data.solutionInterval.upperBound)/2.0;
						fEquation = "("+to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+ to_string_with_precision(data.solutionInterval.upperBound, 0)+")/2"+"="+to_string_with_precision(nextCost, 0);
						stage = "BIN";
					}
					//data.delta += 1;
					data.workBound = c2*data.nodeLB;
					previousBound = bound;
					bound = nextCost; 
					bound_g = bound;
					nextBound = -1;
					iterationComplete = false;
					GetNodeswithBoundinG(); //move nodes from temp to g
					//printf("-> Starting new iteration with f: %f; node limit: %" PRId64 "\n", nextCost, data.workBound);

					return false;
				}
				
				if (data.solutionInterval.lowerBound == DBL_MAX)
					printf("[HIT]--Critical f in [∞, ∞]\n");
				else
					printf("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
				

				data.solutionInterval.upperBound = bound_g;
				previousBound = bound_g;
				FindtheBoundAndNextBoundG();
				GetAllNodesinG();  //move evrerything from temp to g 
				GetNodesFromGq();
				bound_g = bound;
				fEquation = to_string_with_precision(nextBound, 0);
				stage = "NEW ITERATION";
				return false;
				}
	}
	else{
		//printf("main iteration complete\n");
		data.solutionInterval.lowerBound = bound;
		data.solutionInterval.upperBound = DBL_MAX;
		data.nodeLB = nodesExpanded;
		data.nodesExpanded = 0;
		data.nodesReexpanded = 0;
		data.workBound = infiniteWorkBound;
		data.delta = 1;
		SetupExponentialBinaryIteration = false;
		MODE = 0;
		iterationComplete = false;
		case2= 0;
		case3=0;
		q.Reset(env->GetMaxHash());
		return false;
	}
}

template <class state, class action>
void ImprovedBGS<state, action>::Draw(Graphics::Display &disp) const
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
void ImprovedBGS<state, action>::OpenGLDraw()
{
}

template <class state, class action>
void ImprovedBGS<state, action>::PrintQueueStatus()
{
int c1 = 0;
	for(int i = 0; i < q_g.size();i++){
		state s = q_g.Lookup(i).data;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;

		if(q_g.Lookup(hk,ok) == kOpenList){
			printf("hash value %lld is open at %lld and in g with f-value %1.5f\n",hk,ok,q_g.Lookup(i).g + h->HCost(q_g.Lookup(i).data, goal));
		}
		else if(q_g.Lookup(hk,ok) == kClosedList){
			printf("hash value %lld is closed at %lld and in g with f-value %1.5f\n",hk,ok,q_g.Lookup(i).g + h->HCost(q_g.Lookup(i).data, goal));
		}
		else {
			printf("hash value %lld is notfound in g\n",hk);
		}
		if(q_g.Lookup(hk,ok) != kNotFound){
			c1++;
		}
	}
	int c2 = 0;
	for(int i = 0; i < q_f.size();i++){
		state s = q_f.Lookup(i).data;
		uint64_t hk = env->GetStateHash(s);
		uint64_t ok;
		if(q_f.Lookup(hk,ok) == kOpenList){
			printf("hash value %lld is open at %lld and in f with f-value %1.5f\n",hk,ok,q_f.Lookup(i).g + h->HCost(q_f.Lookup(i).data, goal));
		}
		else if(q_f.Lookup(hk,ok) == kClosedList){
			printf("hash value %lld is closed at %lld and in f with f-value %1.5f\n",hk,ok,q_f.Lookup(i).g + h->HCost(q_f.Lookup(i).data, goal));
		}
		else {
			printf("hash value %lld is notfound in f\n",hk);
		}
		if(q_f.Lookup(hk,ok) != kNotFound){
			c2++;
		}
	}
	printf("size of g-list is %d and size of f-list is %d\n",c1,c2);
}

#endif /* ImprovedBGS_h */
