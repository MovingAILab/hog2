//
//  IBEX.h
//  hog2
//
//  This code contains an implementation of IBEX / BTS / BGS
//  See also:
//  https://webdocs.cs.ualberta.ca/~nathanst/papers/IBEX.pdf
//  https://www.movingai.com/SAS/BTS/
//

#ifndef IBEX_h
#define IBEX_h

#include "vectorCache.h"
#include "AStarOpenClosed.h"
#ifndef DEBUG 
#define DEBUG 0 // set debug mode
#endif

#if DEBUG
#define debug_print(fmt, ...) printf(fmt,__VA_ARGS__)
#else
#define debug_print(fmt, ...) do {} while (0)
#endif
namespace IBEX {
	
	template <class state>
	struct BFHSCompare {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g, i2.g));
		}
	};
	
	const int infiniteWorkBound = -1;
	
	template <class state, class action, class environment, bool DFS = true>
	class IBEX {
	public:
		IBEX(uint64_t minGrow, uint64_t maxGrow, double growthRate, bool exponential)
		:c1(minGrow), c2(maxGrow), gamma(growthRate), oracle(false), exponentialGrowth(exponential)  {}
		void GetPath(environment *env, state from, state to,
					 std::vector<action> &thePath);
		void GetPath(environment *env, Heuristic<state> *heuristic, state from, state to,
					 std::vector<action> &thePath);
		void Dovetail(environment *env, Heuristic<state> *heuristic, state from, state to,
					  std::vector<action> &thePath);
		double RedoMinWork();
		
		uint64_t GetNodesExpanded() { return totalNodesExpanded; }
		uint64_t GetNodesTouched() { return totalNodesTouched; }
		void ResetNodeCount() { totalNodesExpanded = totalNodesTouched = 0; }
	private:
		struct searchBounds {
			double f_below; // value below the actual limit
			double f_above;  // value above the actual limit
			uint64_t nodes; // nodes used to search
		};
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
		IBEX<state, action, environment, DFS>::costInterval LowLevelSearch(double costLimit, uint64_t nodeLimit, uint64_t &nodesUsed);
		
		// Functions for DF Search
		double GCost(const state &s1, const state &s2)
		{ return env->GCost(s1, s2); }
		double GCost(const state &s, const action &a)
		{ return env->GCost(s, a); }
		double HCost(const state &s)
		{ return h->HCost(s, goal); }
		IBEX<state, action, environment, DFS>::costInterval DFBNB(double costLimit, uint64_t nodeLimit, uint64_t &nodesUsed);
		IBEX<state, action, environment, DFS>::searchBounds DFBNBHelper(state &currState, double pathCost, double costLimit,
																	  searchBounds &sd, uint64_t nodeLimit, action forbidden);
		
		// Functions for BFHS
		IBEX<state, action, environment, DFS>::costInterval BFHS(double costLimit, uint64_t nodeLimit, uint64_t &nodesUsed);
		void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
		
		
		uint64_t totalNodesExpanded, totalNodesTouched;
		Heuristic<state> *h;
		environment *env;
		std::vector<action> solutionPath, currPath;
		double solutionCost;
		vectorCache<action> actCache;
		state start, goal;
		uint64_t c1, c2;
		double gamma;
		double dfsLowerBound;
		bool oracle;
		bool exponentialGrowth;
		
		// Data for BFHS
		AStarOpenClosed<state, BFHSCompare<state>> q;
		std::vector<state> neighbors;
		std::vector<uint64_t> neighborID;
		std::vector<double> edgeCosts;
		std::vector<dataLocation> neighborLoc;
		std::vector<state> solutionStates;
	};
	
	template <class state, class action, class environment, bool DFS>
	void IBEX<state, action, environment, DFS>::GetPath(environment *env, state from, state to,
														std::vector<action> &thePath)
	{
		GetPath(env, env, from, to, thePath);
	}
	
	template <class state, class action, class environment, bool DFS>
	void IBEX<state, action, environment, DFS>::GetPath(environment *env, Heuristic<state> *heuristic, state from, state to,
														std::vector<action> &thePath)
	{		
		this->env = env;
		h = heuristic;
		start = from;
		goal = to;
		solutionPath.clear();
		solutionCost = DBL_MAX;
		ResetNodeCount();
		
		uint64_t nodeLB = 1;
		costInterval solutionInterval;
		uint64_t currentNodesUsed;
		solutionInterval.lowerBound = HCost(from);
		solutionInterval.upperBound = DBL_MAX;
		while (fgreater(solutionCost, solutionInterval.lowerBound))
		{
			double delta = 1;
			debug_print("IBEX: Base search: f: %1.5f, cost limit ∞, target [%" PRId64 ", %" PRId64 "]\n", solutionInterval.lowerBound, c1*nodeLB, c2*nodeLB);
			dfsLowerBound = solutionInterval.lowerBound;
			solutionInterval &= LowLevelSearch(solutionInterval.lowerBound, infiniteWorkBound, currentNodesUsed);
			
			// Move to next iteration
			if (currentNodesUsed >= c1*nodeLB)
			{
				nodeLB = currentNodesUsed;
				solutionInterval.upperBound = DBL_MAX;
				continue;
			}
			
			while (!(fequal(solutionInterval.upperBound, solutionInterval.lowerBound) ||
					 (currentNodesUsed >= c1*nodeLB && currentNodesUsed < c2*nodeLB)))
			{
				if (solutionInterval.upperBound == DBL_MAX)
				{
					debug_print("    ]--Critical f in [%1.5f, ∞]\n", solutionInterval.lowerBound);
				}
					
				else
				{
					debug_print("    ]--Critical f in [%1.5f, %1.5f]\n", solutionInterval.lowerBound, solutionInterval.upperBound);
				}
					

				double nextCost;
				delta *= gamma;
				if (solutionInterval.upperBound == DBL_MAX)
				{
					if (exponentialGrowth)
						nextCost = solutionInterval.lowerBound+delta;
//					nextCost = baseHCost+(solutionInterval.lowerBound-baseHCost) * gamma;
					else
						nextCost = solutionInterval.lowerBound * gamma;
				}
				else
					nextCost = (solutionInterval.lowerBound+solutionInterval.upperBound)/2.0;

				dfsLowerBound = solutionInterval.lowerBound;
				solutionInterval &= LowLevelSearch(nextCost, c2*nodeLB, currentNodesUsed);
			}
			
			nodeLB = std::max(currentNodesUsed, c1*nodeLB);
			solutionInterval.upperBound = DBL_MAX;

			if (fequal(solutionInterval.lowerBound, solutionCost))
				break;
		}
		thePath = solutionPath;
		debug_print("Found solution cost %1.5f\n", solutionCost);
	}
	
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::costInterval IBEX<state, action, environment, DFS>::DFBNB(double costLimit, uint64_t nodeLimit, uint64_t &nodesUsed)
	{
		state currState = start;
		if (nodeLimit == infiniteWorkBound)
		{
			debug_print("    --+DFBnB f: %1.5f; nodes: ∞; ", costLimit);
		}
			
		else
		{
			debug_print("    --+DFBnB f: %1.5f; nodes: %" PRId64 "; ", costLimit, nodeLimit);
		}
			
		currPath.clear();
		searchBounds sd;
		sd.f_below = 0;
		sd.f_above = DBL_MAX;
		sd.nodes = 0;
		action a;
		sd = DFBNBHelper(currState, 0, costLimit, sd, nodeLimit, a);
		totalNodesExpanded += sd.nodes;
		
		costInterval v;
		if (sd.nodes >= nodeLimit)
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
			assert(fgreater(sd.f_above, costLimit));
		}
		nodesUsed = sd.nodes;
		if (v.upperBound == DBL_MAX)
		{
			debug_print("%" PRId64 " (new) %" PRId64 " (total), solution range: [%1.5f, ∞] ", nodesUsed, totalNodesExpanded, v.lowerBound);
		}
		else
		{
			debug_print("%" PRId64 " (new) %" PRId64 " (total), solution range: [%1.5f, %1.5f] ", nodesUsed, totalNodesExpanded, v.lowerBound, v.upperBound);
		}
			
		if (solutionCost != DBL_MAX)
		{
			debug_print("sol: %1.5f\n", solutionCost);
		}	
		else
		{
			debug_print("\n",1);
		}
			
		return v;
		//return sd;
	}
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::searchBounds IBEX<state, action, environment, DFS>::DFBNBHelper(state &currState, double pathCost, double costLimit,
																												  searchBounds &sd, uint64_t nodeLimit, action forbidden)
	{
		double currF = pathCost+HCost(currState);
		//	printf("-------->%f [%f]\n", currF, pathCost);
		if (fequal(dfsLowerBound, solutionCost) && !oracle)
		{
			return sd;
		}
		if (fgreater(currF, costLimit))
		{
			sd.f_above = std::min(sd.f_above, currF);
			return sd;
		}
		else if (fgreatereq(currF, solutionCost))
		{
			sd.f_below = solutionCost;
			return sd;
		}
		else {
			sd.f_below = std::max(currF, sd.f_below);
		}
		
		if (sd.nodes >= nodeLimit)
			return sd;
		if (env->GoalTest(currState, goal))
		{
			if (fless(currF, solutionCost))
			{
				solutionPath = currPath;
				solutionCost = currF;
			}
			return sd;
		}
		
		std::vector<action> &acts = *actCache.getItem();
		env->GetActions(currState, acts);
		sd.nodes++;
		totalNodesTouched+=acts.size();
		
		for (size_t x = 0; x < acts.size(); x++)
		{
			if (acts[x] == forbidden && currPath.size() > 0)
				continue;
			double edgeCost = GCost(currState, acts[x]);
			env->ApplyAction(currState, acts[x]);
			currPath.push_back(acts[x]);
			action rev = acts[x];
			env->InvertAction(rev);
			sd = DFBNBHelper(currState, pathCost+edgeCost, costLimit, sd, nodeLimit, rev);
			currPath.pop_back();
			env->UndoAction(currState, acts[x]);
		}
		actCache.returnItem(&acts);
		return sd;
	}
	
	template <class state, class action, class environment, bool DFS>
	double IBEX<state, action, environment, DFS>::RedoMinWork()
	{
		ResetNodeCount();
		debug_print("IBEX Validation:\n",1);
		oracle = true;
		uint64_t nodesUsed;
		LowLevelSearch(solutionCost, -1, nodesUsed);
		oracle = false;
		return solutionCost;
	}
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::costInterval IBEX<state, action, environment, DFS>::BFHS(double costLimit, uint64_t nodeLimit, uint64_t &nodesUsed)
	{
		if (nodeLimit == -1ull && costLimit == DBL_MAX)
		{
			debug_print("    --+BFHS f: ∞; nodes: ∞; ",1);
		}
		else if (nodeLimit == -1)
		{
			debug_print("    --+BFHS f: %1.5f; nodes: ∞; ", costLimit);
		}
		else
		{
			debug_print("    --+BFHS f: %1.5f; nodes: %" PRId64 "; ", costLimit, nodeLimit);
		}
		
		searchBounds sd;
		sd.f_below = 0;
		sd.f_above = DBL_MAX;
		sd.nodes = 0;
		
		q.Reset(env->GetMaxHash());

		// put start in open
		q.AddOpenNode(start, env->GetStateHash(start), 0.0, 0.0, (uint64_t)0);
		
		while (sd.nodes < nodeLimit && q.OpenSize() > 0)
		{
			// expand by low g until
			// (1) we find the goal
			// (2) we hit the node limit
			// (3) we exhaust all states
			uint64_t nodeid = q.Close();
			sd.nodes++;
			totalNodesExpanded++;
			
			double nextf = q.Lookup(nodeid).g + HCost(q.Lookup(nodeid).data);
			if (fgreater(nextf, costLimit))
			{
				// case shouldn't happen - pruned elsewhere
				assert(false);
			} else {
				sd.f_below = std::max(sd.f_below, nextf);
			}

			
			if (env->GoalTest(q.Lookup(nodeid).data, goal))
			{
				solutionCost = q.Lookup(nodeid).g;
				ExtractPathToStartFromID(nodeid, solutionStates);
				// Path is backwards - reverse
				reverse(solutionStates.begin(), solutionStates.end());
				// f, nextF, failedF, nodes
				for (int x = 0; x < solutionStates.size()-1; x++)
				{
					solutionPath.push_back(env->GetAction(solutionStates[x], solutionStates[x+1]));
				}
				// TODO: return range here
				nodesUsed = sd.nodes;
				return {q.Lookup(nodeid).g, q.Lookup(nodeid).g};
			}
			
			neighbors.resize(0);
			edgeCosts.resize(0);
			neighborID.resize(0);
			neighborLoc.resize(0);
			
//				std::cout << "Expanding: " << q.Lookup(nodeid).data << " with f:";
//				std::cout << q.Lookup(nodeid).g << std::endl;
			
			env->GetSuccessors(q.Lookup(nodeid).data, neighbors);
			
			// 1. load all the children
			for (size_t x = 0; x < neighbors.size(); x++)
			{
				uint64_t theID;
				neighborLoc.push_back(q.Lookup(env->GetStateHash(neighbors[x]), theID));
				neighborID.push_back(theID);
				edgeCosts.push_back(GCost(q.Lookup(nodeid).data, neighbors[x]));
			}
			
			// iterate again updating costs and writing out to memory
			for (size_t x = 0; x < neighbors.size(); x++)
			{
				totalNodesTouched++;
			
				double childGCost = q.Lookup(nodeid).g+edgeCosts[x];
				double childFCost = childGCost+HCost(neighbors[x]);
				if (fgreater(childFCost, costLimit) || fgreatereq(childFCost, solutionCost))
				{
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
									  0.0,
									  nodeid);
					}
				}
			}
		}
		// f, nextF, failedF, nodes
		
		// TODO: return range here
		costInterval v;
		if (sd.nodes >= nodeLimit)
		{
			v.lowerBound = 0;
			v.upperBound = sd.f_below;
		}
		else {
			v.lowerBound = sd.f_above;
			v.upperBound = DBL_MAX;
		}
		//v.nodes = sd.nodes;
		nodesUsed = sd.nodes;
		
		if (v.upperBound == DBL_MAX)
		{
			debug_print("%" PRId64 " (new) %" PRId64 " (total), solution range: [%1.5f, ∞]\n", nodesUsed, totalNodesExpanded, v.lowerBound);
		}
			
		else
		{
			debug_print("%" PRId64 " (new) %" PRId64 " (total), solution range: [%1.5f, %1.5f]\n", nodesUsed, totalNodesExpanded, v.lowerBound, v.upperBound);
		}
			

		return v;
	}
	
	template <class state, class action,class environment,bool DFS>
	void IBEX<state, action, environment, DFS>::ExtractPathToStartFromID(uint64_t node,
																		 std::vector<state> &thePath)
	{
		thePath.clear();
		do {
			thePath.push_back(q.Lookup(node).data);
			node = q.Lookup(node).parentID;
		} while (q.Lookup(node).parentID != node);
		thePath.push_back(q.Lookup(node).data);
	}
	
	
	template <class state, class action, class environment, bool DFS>
	typename IBEX<state, action, environment, DFS>::costInterval IBEX<state, action, environment, DFS>::LowLevelSearch(double costLimit, uint64_t nodeLimit, uint64_t &nodesUsed)
	{
		if (DFS)
			return DFBNB(costLimit, nodeLimit, nodesUsed);
		else
			return BFHS(costLimit, nodeLimit, nodesUsed);
	}

	/* Node used by the UBS search */
	class UBSNode {
	public:
		UBSNode(int k, int r) : r(r), k(k) {}
		int r;
		int k;
		friend bool operator<(const UBSNode &n1, const UBSNode &n2) {return n1.T() > n2.T();}
		int T() const {return r * pow(2, k);}
	};
	
	template <class state, class action, class environment, bool DFS>
	void IBEX<state, action, environment, DFS>::Dovetail(environment *env, Heuristic<state> *heuristic, state from, state to,
														 std::vector<action> &thePath)
	{
		this->env = env;
		h = heuristic;
		start = from;
		goal = to;
		solutionPath.clear();
		solutionCost = DBL_MAX;
		ResetNodeCount();

		//Graph &graph, int alpha, float gamma, Data &data)
		int alpha=c2;
		
		/* lookup table maps programs k to uppder bounds.
		 Lower bound is tracked globally */
		std::map<int,double> lookup;
		/* priority queue for storing the UBS nodes */
		std::priority_queue<UBSNode> q;
		q.push(UBSNode(1,1));
		/* b_low is a lower bound on the optimal budget */
		uint64_t b_low = 0;
		double low = HCost(from);
		while (!q.empty()) {
			/* get top element from queue */
			auto n = q.top();
			q.pop();
			int k = n.k;
			int r = n.r;
			/* set budget */
			int b = pow(alpha,k);

			/* update the UBS */
			if (n.r == 1) {
				q.push(UBSNode(n.k+1,1));
			}
			
			/* skip the query if the budget is known to be insufficient
			 to find a solution */
			if (b <= b_low) {
				continue;
			}
			

			/* check to see if upper bound has been found yet for this program */
			if (lookup.find(k) == lookup.end()) {
				lookup[k] = DBL_MAX;//std::numeric_limits<float>::max();
			}
			double high = lookup[k];

			if (high != DBL_MAX)
			{
				debug_print("Running (k=%d, r=%d) with budget %d; [global] low = %f, [loca] high = %f\n", k, r, b, low, high);
			}
			else
			{
				debug_print("Running (k=%d, r=%d) with budget %d; [global] low = %f, [loca] high = ∞\n", k, r, b, low);
			}
				

			
			/* compute the cost threshhold for the query */
			float C;
			if (exponentialGrowth)
				C = low + (1<<(r-1));
			else
				C = gamma * low;  /* by default, in the binary search mode */
			if (r == 1)
			{   /* start with an infinite budget search */
				C = low;
				b = infiniteWorkBound;//std::numeric_limits<int>::max();
			}
			else if (high != DBL_MAX) { /* exponential search */
				C = (low + high) / 2.0;
			}
			/* make the query */
			uint64_t nodesUsed;
			costInterval i = LowLevelSearch(C, b, nodesUsed);
			
			/* perform the intersection */
			lookup[k] = std::min(i.upperBound, high);
			low = std::max(low, i.lowerBound);
			
			/* the budget was sufficient and no solution found, so update the minimal budget */
			if (i.upperBound == DBL_MAX) {
				b_low = nodesUsed;
			}
			
			/* update the UBS */
			q.push(UBSNode(n.k, n.r+1));

			if (flesseq(solutionCost, low))
			{
				thePath = solutionPath;
				debug_print("proven solution cost %f\n", solutionCost);
				break;
			}
		}
	}
	
}

#endif /* IBEX_h */
