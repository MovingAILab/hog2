//
//  FocalAdd.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 1/8/20.
//

#ifndef FocalAdd_h
#define FocalAdd_h


#include <iostream>
#include "FPUtil.h"
//#include <unordered_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
#include "TemplateAStar.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"
#include <algorithm> // for vector reverse
#include "GenericSearchAlgorithm.h"
#include <functional>
#include <unordered_map>
#include "Treap.h"

//	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
//	{
//		if (fequal(i1.h, i2.h))
//		{
//			return (fless(i1.g, i2.g));
//		}
//		return (fgreater(i1.h, i2.h));
//	}
//};


/**
 * A generic focal list algorithm that returns linearly bounded solutions
 * Open is sorted by f; provide a function for sorting focal
 * Provide a weight w
 * Given minimum f in open, anything up to w*minf goes into focal
 * Expand from focal until empty; otherwise expand from open
 */

template <class state, class action, class environment>
class FocalAdd : public GenericSearchAlgorithm<state,action,environment> {
public:
	FocalAdd(double optimalBound = 2)
	{
		ResetNodeCount(); env = 0; bound = optimalBound; theHeuristic = 0;
		phi_open = ([=](double h, double g){return g+h;});
		//phi_focal = ([=](double h, double g){return g+(2*bound-1)*h;});
	}
	virtual ~FocalAdd() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
	// uses admissible heuristic (regular A* search)
	//AStarOpenClosed<state, AStarCompare<state>> f;
	// uses inadmissible heuristic
	//AStarOpenClosed<state, GBFSCompare<state>> focal;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; focal.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(size_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	state CheckNextOpenNode();
	state CheckNextFocalNode();
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetFocalListGCost(const state &val, double &gCost) const;

//	bool GetClosedListGCost(const state &val, double &gCost) const;
//	bool GetClosedItem(const state &s, AStarOpenClosedData<state> &);
	//{ return focal.Lookat(focal.GetOpenItem(which)); }
	inline const int GetNumItems() { return states.size(); }
//	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return states[which]; }
//	bool HaveExpandedState(const state &val)
//	{ uint64_t key; return focal.Lookup(env->GetStateHash(val), key) != kNotFound; }
//	dataLocation GetStateLocation(const state &val)
//	{ uint64_t key; return focal.Lookup(env->GetStateHash(val), key); }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
//	void OpenGLDraw() const;
	void Draw(Graphics::Display &d) const;
	
	void SetOptimalityBound(double w) {bound = w; /*phi_focal = ([=](double h, double g){return g+(2*bound-1)*h;});*/}
	double GetOptimalityBound() { return bound; }
	enum stateLoc {
		kClosedList,
		kOpenList,
		kOpenFocalList
	};
	struct SearchState {
		double f_open;
		double f_focal;
		double g, h;
		state s;
		stateLoc where; // which data structures
		size_t parent; // location in states
		bool reopened;
	};

	unsigned int GetNumOpenItems() { return open.Size(); }
	unsigned int GetNumFocalItems() { return focal.Size(); }
	inline const FocalAdd<state,action,environment>::SearchState &GetOpenItem(unsigned int which)
	{
		return states[open.GetNode(which).location];
	}
	//{ return f.Lookat(f.GetOpenItem(which)); }
	inline const FocalAdd<state,action,environment>::SearchState &GetFocalItem(unsigned int which)
	{
		return states[focal.GetNode(which).location];
	}
private:
	struct OpenTreapItem {
		size_t location;
		std::vector<SearchState> *dataVector;
		bool operator<(const OpenTreapItem &i)
		{
			auto &me = (*dataVector)[location];
			auto &other = (*(i.dataVector))[i.location];
			
			if (fequal(me.f_open, other.f_open))
			{
				if (fequal(me.h, other.h))
					return me.s<other.s;
				return fless(me.h, other.h);
			}
			return fless(me.f_open, other.f_open);
		}
		bool operator==(const OpenTreapItem &i) const
		{
			return location==i.location;
		}
		bool operator>(double i)
		{
			auto &me = (*dataVector)[location];
			return fgreatereq(me.f_open, i);
		}
		bool operator<=(double i)
		{
			auto &me = (*dataVector)[location];
			return flesseq(me.f_open, i);
		}

		friend std::ostream& operator<<(std::ostream& o, OpenTreapItem const& obj )
        {
			o << obj.dataVector->at(obj.location).s;
            return o;
        }
	};
	struct FocalTreapItem {
		size_t location;
		std::vector<SearchState> *dataVector;
		bool operator<(FocalTreapItem &i)
		{
			auto &me = (*dataVector)[location];
			auto &other = (*(i.dataVector))[i.location];
			
			if (fequal(me.f_focal, other.f_focal))
			{
				if (fequal(me.h, other.h))
					return me.s<other.s;
				return fless(me.h, other.h);
			}
			return fless(me.f_focal, other.f_focal);
		}
		bool operator==(const FocalTreapItem &i) const
		{
			return location==i.location;
		}
        friend std::ostream& operator<<(std::ostream& o, FocalTreapItem const& obj )
        {
			o << obj.dataVector->at(obj.location).s;
            return o;
        }
	};
	SearchState &PeekOpen()
	{
		auto i = open.Peek();
		return (*i.dataVector)[i.location];
	}
	SearchState &PeekFocal()
	{
		auto i = focal.Peek();
		return (*i.dataVector)[i.location];
	}
	void Add(state &s, double g, double h, size_t parent);
	Treap<FocalTreapItem> focal;
	Treap<OpenTreapItem> open;
	std::unordered_map<state, size_t> hash; // map from state to index in states vector
	std::vector<SearchState> states;
	std::function<double(double, double)> phi_focal;
	std::function<double(double, double)> phi_open;

	void DrawOpen(Graphics::Display &d) const;
	void DrawFocal(Graphics::Display &d) const;
	void ExpandOpen();
	void ExpandFocal();
	uint64_t nodesTouched, nodesExpanded;
	
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;

	std::vector<state> solution;
	environment *env;
	double bestSolution;
	double bound;
	uint64_t uniqueNodesExpanded;
	double minF;
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
const char *FocalAdd<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "Focal[%1.2f]", bound);
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
void FocalAdd<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	//discardcount=0;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{
		//		if (0 == nodesExpanded%100000)
		//			printf("%" PRId64 " nodes expanded\n", nodesExpanded);
	}
}

template <class state, class action, class environment>
void FocalAdd<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
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
bool FocalAdd<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	bestSolution = DBL_MAX;
	
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	focal.Reset();
	open.Reset();
	solution.clear();
	ResetNodeCount();
	hash.clear();
	start = from;
	goal = to;
	minF = 0;
	
	if (env->GoalTest(from, to)) //assumes that from and to are valid states
	{
		return false;
	}
	
	states.resize(0);
	double init_h = theHeuristic->HCost(start, goal);
	phi_focal = ([=](double h, double g){return g+h+bound*min(h, init_h)/init_h;});
	Add(start, 0, theHeuristic->HCost(start, goal), states.size()); // should be 0
//	focal.AddOpenNode(start, env->GetStateHash(start), 0, theHeuristic->HCost(start, goal));
//	f.AddOpenNode(start, env->GetStateHash(start), 0, theHeuristic->HCost(start, goal));
	return true;
}

///**
// * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
// * @author Nathan Sturtevant
// * @date 01/06/08
// */
//template <class state, class action, class environment>
//void Focal<state,action,environment>::AddAdditionalStartState(state& newState)
//{
//	focal.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
//	f.AddOpenNode(newState, env->GetStateHash(newState), 0, theHeuristic->HCost(start, goal));
//}
//
///**
// * Add additional start state to the search. This should only be called after Initialize Search
// * @author Nathan Sturtevant
// * @date 09/25/10
// */
//template <class state, class action, class environment>
//void Focal<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
//{
//	focal.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
//	f.AddOpenNode(newState, env->GetStateHash(newState), cost, theHeuristic->HCost(start, goal));
//}

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
bool FocalAdd<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (solution.size() > 0)
	{
		thePath = solution;
		return true;
	}
	if (open.Size() == 0)
	{
		printf("No path\n");
		thePath.resize(0); // no path found!
		return true;
	}
	
	uint64_t nodeOnfocal;
	uint64_t nodeOnF;
	// only reopen states taken from f, not fhat
	bool reopen = true;

	size_t bestOpenLoc = open.Peek().location;
	double regularF = states[bestOpenLoc].f_open; //f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h;
	if (fless(minF, regularF)) // minimum cost increased!
	{
		std::function<void (const OpenTreapItem &)> func = [&](const OpenTreapItem &i)
		{ // Move item to focal
			FocalTreapItem f = {i.location, i.dataVector};
			if (states[i.location].where == kOpenList)
			{
//				std::cout << "-->Moving " << states[i.location].s << "(f:" << states[i.location].f_open << ") to focal.\n";
				focal.Add(f);
				states[i.location].where = kOpenFocalList;
			}
		};
		// Add all states in the bound to focal
		open.Iterate(minF+bound, regularF+bound, func);
		minF = regularF;
	}

	if (focal.Size() > 0)
	{
		size_t bestFocalLoc = focal.Peek().location;
		double focalF = states[bestFocalLoc].f_focal;//focal.Lookat(focal.Peek()).g+focal.Lookat(focal.Peek()).h;
		if (flesseq(states[bestFocalLoc].f_open, bound+regularF))
		{
//			printf(" Expanding focal\n");
			ExpandFocal();
		}
		else {
//			printf("Min f: %1.2f; next focal f: %1.2f. Max allowable: %1.2f.\n", regularF, states[bestFocalLoc].f_open, regularF*bound);
			printf("ERROR: Expanding open\n");
			ExpandOpen();
		}
	}
	else {
//		printf("-->Expanding open\n");
		ExpandOpen();
	}
	
	if (solution.size() > 0)
	{
		thePath = solution;
		return true;
	}

//	if (fless(focal.Lookat(focal.Peek()).g+focal.Lookat(focal.Peek()).h, bestSolution))
//	{
//		nodeOnfocal = focal.Close();
//		reopen = false;
//		dataLocation d = f.Lookup(env->GetStateHash(focal.Lookup(nodeOnfocal).data), nodeOnF);
//		assert(d != kNotFound);
////		f.Close(nodeOnF);
//	}
//	else {
//		nodeOnF = f.Close();
//		reopen = true;
//		dataLocation d = focal.Lookup(env->GetStateHash(f.Lookup(nodeOnF).data), nodeOnfocal);
//		assert(d != kNotFound);
//		if (d == kOpenList)
//		{
//			focal.Close(nodeOnfocal);
//			d = focal.Lookup(env->GetStateHash(f.Lookup(nodeOnF).data), nodeOnfocal);
//			assert(d == kClosedList);
//		}
//	}
//
//	if (!fequal(oldF, f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h))
//	{
//		printf("Best solution %1.2f\n", bestSolution);
//		printf("Best on open %1.2f - lower bound is %1.2f\n", f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h,
//			   (f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h)*bound);
//	}
//	if (!focal.Lookup(nodeOnfocal).reopened)
//		uniqueNodesExpanded++;
//	nodesExpanded++;
//
//	//	if n is a goal then
//	//		incumbent ← n
//	if (env->GoalTest(focal.Lookup(nodeOnfocal).data, goal))
//	{
//		// Actually extract the path, since the cost may not actually be the g-cost
//		ExtractPathToStartFromID(nodeOnfocal, thePath);
//		bestSolution = env->GetPathLength(thePath);
//		thePath.resize(0);
//		printf("Best solution %1.2f\n", bestSolution);
//		printf("Best on open %1.2f - lower bound is %1.2f\n", f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h,
//			   (f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h)*bound);
//
//		return false; // check on next iteration through the loop
//	}

	
	//	std::cout << "Expanding: " << focal.Lookup(nodeOnfocal).data << " with f:";
	//	std::cout << focal.Lookup(nodeid).g+focal.Lookup(nodeOnfocal).h << std::endl;
	
	return false;
}

/**
 * Expands a single state from the given open list
 * @author Nathan Sturtevant
 * @date 01/27/19
 */
template <class state, class action, class environment>
void FocalAdd<state,action,environment>::ExpandOpen()
{
	auto i = open.RemoveSmallest();
//	std::cout << states[i.location].s << " expanded from open\n";
	if (!states[i.location].reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;
	states[i.location].where = kClosedList;
	
	if (env->GoalTest(states[i.location].s, goal))
	{
		ExtractPathToStartFromID(i.location, solution);
		// Path is backwards - reverse
		reverse(solution.begin(), solution.end());
		return;
	}
	
	neighbors.resize(0);
	env->GetSuccessors(states[i.location].s, neighbors);
	for (unsigned int x = 0; x < neighbors.size(); x++)
		Add(neighbors[x], states[i.location].g+env->GCost(states[i.location].s, neighbors[x]),
			env->HCost(neighbors[x], goal), i.location);

//	//	env->GetSuccessors(f.Lookup(nodeid).data, neighbors);
//	// 1. load all the children
//	for (unsigned int x = 0; x < neighbors.size(); x++)
//	{
//		uint64_t theID;
//		neighborLoc.push_back(f.Lookup(env->GetStateHash(neighbors[x]), theID));
//		neighborID.push_back(theID);
//		edgeCosts.push_back(env->GCost(f.Lookup(nodeid).data, neighbors[x]));
//	}
//
//	// iterate again updating costs and writing out to memory
//	for (int x = 0; x < neighbors.size(); x++)
//	{
//		nodesTouched++;
//
//		switch (neighborLoc[x])
//		{
//			case kClosedList:
////				if (reopenNodes)
////				{
////					if (fless(f.Lookup(nodeid).g+edgeCosts[x], f.Lookup(neighborID[x]).g))
////					{
////						f.Lookup(neighborID[x]).parentID = nodeid;
////						f.Lookup(neighborID[x]).g = f.Lookup(nodeid).g+edgeCosts[x];
////						f.Reopen(neighborID[x]);
////						// This line isn't normally needed, but in some state spaces we might have
////						// equality but different meta information, so we need to make sure that the
////						// meta information is also copied, since this is the most generic A* implementation
////						f.Lookup(neighborID[x]).data = neighbors[x];
////					}
////				}
//				break;
//			case kOpenList:
//				//edgeCost = env->GCost(f.Lookup(nodeid).data, neighbors[x]);
//				if (fless(f.Lookup(nodeid).g+edgeCosts[x], f.Lookup(neighborID[x]).g))
//				{
//					f.Lookup(neighborID[x]).parentID = nodeid;
//					f.Lookup(neighborID[x]).g = f.Lookup(nodeid).g+edgeCosts[x];
//					// This line isn't normally needed, but in some state spaces we might have
//					// equality but different meta information, so we need to make sure that the
//					// meta information is also copied, since this is the most generic A* implementation
//					f.Lookup(neighborID[x]).data = neighbors[x];
//					f.KeyChanged(neighborID[x]);
//					//					std::cout << " Reducing cost to " << f.Lookup(nodeid).g+edgeCosts[x] << "\n";
//					// TODO: unify the KeyChanged calls.
//				}
//				else {
//					//					std::cout << " no cheaper \n";
//				}
//				break;
//			case kNotFound:
//			{
//				f.AddOpenNode(neighbors[x],
//							  env->GetStateHash(neighbors[x]),
//							  f.Lookup(nodeid).g+edgeCosts[x],
//							  theHeuristic->HCost(neighbors[x], goal),
//							  nodeid);
//			}
//		}
//	}
}

template <class state, class action, class environment>
void FocalAdd<state,action,environment>::ExpandFocal()
{
	auto i = focal.RemoveSmallest();
//	std::cout << states[i.location].s << " expanded from focal\n";
	if (!states[i.location].reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;

	states[i.location].where = kOpenList;
	
	if (env->GoalTest(states[i.location].s, goal))
	{
		ExtractPathToStartFromID(i.location, solution);
		// Path is backwards - reverse
		reverse(solution.begin(), solution.end());
		return;
	}
	
	neighbors.resize(0);
	env->GetSuccessors(states[i.location].s, neighbors);
	for (unsigned int x = 0; x < neighbors.size(); x++)
		Add(neighbors[x], states[i.location].g+env->GCost(states[i.location].s, neighbors[x]),
			env->HCost(neighbors[x], goal), i.location);
//
//
//	uint64_t nodeid = focal.Close();
//	if (!focal.Lookup(nodeid).reopened)
//		uniqueNodesExpanded++;
//	nodesExpanded++;
//
//	if (env->GoalTest(focal.Lookup(nodeid).data, goal))
//	{
//		ExtractPathToStartFromID(nodeid, solution);
//		// Path is backwards - reverse
//		reverse(solution.begin(), solution.end());
//		return;
//	}
//
//	neighbors.resize(0);
//	edgeCosts.resize(0);
//	neighborID.resize(0);
//	neighborLoc.resize(0);
//
//	std::cout << "Expanding: " << focal.Lookup(nodeid).data << " with f:";
//	std::cout << focal.Lookup(nodeid).g+focal.Lookup(nodeid).h << std::endl;
//
//	env->GetSuccessors(focal.Lookup(nodeid).data, neighbors);
//	double bestH = focal.Lookup(nodeid).h;
//	double lowHC = DBL_MAX;
//	// 1. load all the children
//	for (unsigned int x = 0; x < neighbors.size(); x++)
//	{
//		uint64_t theID;
//		neighborLoc.push_back(focal.Lookup(env->GetStateHash(neighbors[x]), theID));
//		neighborID.push_back(theID);
//		edgeCosts.push_back(env->GCost(focal.Lookup(nodeid).data, neighbors[x]));
//	}
//
//	// iterate again updating costs and writing out to memory
//	for (int x = 0; x < neighbors.size(); x++)
//	{
//		nodesTouched++;
//
//		switch (neighborLoc[x])
//		{
//			case kClosedList:
//				break;
//			case kOpenList:
//				//edgeCost = env->GCost(focal.Lookup(nodeid).data, neighbors[x]);
//				if (fless(focal.Lookup(nodeid).g+edgeCosts[x], focal.Lookup(neighborID[x]).g))
//				{
//					focal.Lookup(neighborID[x]).parentID = nodeid;
//					focal.Lookup(neighborID[x]).g = focal.Lookup(nodeid).g+edgeCosts[x];
//					// This line isn't normally needed, but in some state spaces we might have
//					// equality but different meta information, so we need to make sure that the
//					// meta information is also copied, since this is the most generic A* implementation
//					focal.Lookup(neighborID[x]).data = neighbors[x];
//					focal.KeyChanged(neighborID[x]);
//					//					std::cout << " Reducing cost to " << focal.Lookup(nodeid).g+edgeCosts[x] << "\n";
//					// TODO: unify the KeyChanged calls.
//				}
//				else {
//					//					std::cout << " no cheaper \n";
//				}
//				break;
//			case kNotFound:
//			{
//				focal.AddOpenNode(neighbors[x],
//							  env->GetStateHash(neighbors[x]),
//							  focal.Lookup(nodeid).g+edgeCosts[x],
//							  theHeuristic->HCost(neighbors[x], goal),
//							  nodeid);
//			}
//		}
//	}
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue).
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The first state in the open list.
 */
//template <class state, class action, class environment>
//state Focal<state, action,environment>::CheckNextNode()
//{
//	uint64_t key = focal.Peek();
//	return focal.Lookup(key).data;
//	//assert(false);
//	//return openQueue.top().currNode;
//}


/**
 * Get the path from a goal state to the start state
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment>
void FocalAdd<state, action,environment>::ExtractPathToStartFromID(size_t node,
																std::vector<state> &thePath)
{
	do {
		thePath.push_back(states[node].s);
		node = states[node].parent;
	} while (states[node].parent != node);
	thePath.push_back(states[node].s);
}

template <class state, class action,class environment>
const state &FocalAdd<state, action,environment>::GetParent(const state &s)
{
	assert(false);
	return s;
//	uint64_t theID;
//	focal.Lookup(env->GetStateHash(s), theID);
//	theID = focal.Lookup(theID).parentID;
//	return focal.Lookup(theID).data;
}

/**
 * A function that prints the number of states in the closed list and open
 * queue.
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment>
void FocalAdd<state, action,environment>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)focal.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)focal.OpenSize());
}

/**
 * Return the amount of memory used by Focal
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment>
int FocalAdd<state, action,environment>::GetMemoryUsage()
{
	return focal.size();
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
//template <class state, class action, class environment>
//bool Focal<state, action,environment>::GetClosedListGCost(const state &val, double &gCost) const
//{
//	uint64_t theID;
//	dataLocation loc = focal.Lookup(env->GetStateHash(val), theID);
//	if (loc == kClosedList)
//	{
//		gCost = focal.Lookat(theID).g;
//		return true;
//	}
//	return false;
//}

template <class state, class action, class environment>
state FocalAdd<state, action,environment>::CheckNextOpenNode()
{
	return states[open.Peek().location].s;
}

template <class state, class action, class environment>
state FocalAdd<state, action,environment>::CheckNextFocalNode()
{
	return states[focal.Peek().location].s;
}


template <class state, class action, class environment>
bool FocalAdd<state, action,environment>::GetOpenListGCost(const state &val, double &gCost) const
{
	auto i = hash.find(val);
	if (i == hash.end())
		return false;
	if (states[i->second].where != kClosedList)
	{
		gCost = states[i->second].g;
		return true;
	}
	return false;
}

template <class state, class action, class environment>
bool FocalAdd<state, action,environment>::GetFocalListGCost(const state &val, double &gCost) const
{
	auto i = hash.find(val);
	if (i == hash.end())
		return false;
	if (states[i->second].where == kOpenFocalList)
	{
		gCost = states[i->second].g;
		return true;
	}
	return false;
}
//
//
//template <class state, class action, class environment>
//bool Focal<state, action,environment>::GetClosedItem(const state &s, AStarOpenClosedData<state> &result)
//{
//	uint64_t theID;
//	dataLocation loc = focal.Lookup(env->GetStateHash(s), theID);
//	if (loc == kClosedList)
//	{
//		result = focal.Lookat(theID);
//		return true;
//	}
//	return false;
//
//}

template <class state, class action, class environment>
void FocalAdd<state, action,environment>::Add(state &s, double g, double h, size_t parent)
{
	auto i = hash.find(s);
	if (i == hash.end())
	{
		stateLoc loc = kOpenList;
		if (open.Size() > 0)
		{
			auto item = open.Peek();
			if (flesseq(g+h, states[item.location].f_open+bound))
			{
				loc = kOpenFocalList;
			}
		}
		else
			loc = kOpenFocalList;
		
		hash[s] = states.size();
		states.push_back({
			phi_open(h, g),   //double f_open;
			phi_focal(h, g),  // double f_focal;
			g, h,             //double g, h;
			s,                //state s
			loc,              // location (open/closed/focal)
			parent,           //size_t parent; // location in states
			false             //reopened
		});
		if (loc == kOpenList || loc == kOpenFocalList)
		{
			OpenTreapItem openItem = {states.size()-1, &states};
//			std::cout << "Adding " << s << " to open\n";
			open.Add(openItem);
		}
		if (loc == kOpenFocalList)
		{
			FocalTreapItem focalItem = {states.size()-1, &states};
//			std::cout << "Adding " << s << " to focal\n";
			focal.Add(focalItem);
		}
	}
	else if (fless(g, states[i->second].g)) { // update on open/focal
		OpenTreapItem openItem = {i->second, &states};
		FocalTreapItem focalItem = {i->second, &states};
		states[i->second].parent = parent;
		switch (states[i->second].where)
		{
			case kOpenFocalList:
			{
				bool result = open.Remove(openItem);
//				std::cout << s << " removed from open\n";
				if (result == false)
				{
					open.Print();
					assert(result == true);
				}
				result = focal.Remove(focalItem);
//				std::cout << s << " removed from focal\n";
				if (result == false)
				{
					focal.Print();
					assert(result == true);
				}

				states[i->second].g = g;
				states[i->second].f_open = phi_open(h, g);
				states[i->second].f_focal = phi_focal(h, g);
				open.Add(openItem);
//				std::cout << "Adding " << s << " to open\n";
				focal.Add(focalItem);
//				std::cout << "Adding " << s << " to focal\n";
				break;
			}
			case kOpenList:
			{
				bool result = open.Remove(openItem);
//				std::cout << s << " removed from open\n";
				if (result == false)
				{
					open.Print();
					assert(result == true);
				}

				states[i->second].g = g;
				states[i->second].f_open = phi_open(h, g);
				states[i->second].f_focal = phi_focal(h, g);
				open.Add(openItem);
//				std::cout << "Adding " << s << " to open\n";
				auto item = open.Peek();

				// DON'T move back to focal if it is still on open
				if (0&&flesseq(g+h, states[item.location].f_open+bound))
				{
//					std::cout << "Adding " << s << " to focal\n";
					focal.Add(focalItem);
					states[i->second].where = kOpenFocalList;
				}
				break;
			}
			case kClosedList:
			{
				// Put back on open/focal
				states[i->second].g = g;
				states[i->second].f_open = phi_open(h, g);
				states[i->second].f_focal = phi_focal(h, g);
				states[i->second].reopened = true;
				open.Add(openItem);
//				std::cout << "Adding " << s << " to open\n";
				states[i->second].where = kOpenList;
				auto item = open.Peek();
				if (flesseq(g+h, states[item.location].f_open+bound))
				{
//					std::cout << "Adding " << s << " to focal\n";
					focal.Add(focalItem);
					states[i->second].where = kOpenFocalList;
				}
				break;
			}
		}
	}
}

///**
// * Draw the open/closed list
// * @author Nathan Sturtevant
// * @date 03/12/09
// *
// */
//template <class state, class action, class environment>
//void FocalAdd<state, action,environment>::OpenGLDraw() const
//{
//}

template <class state, class action, class environment>
void FocalAdd<state, action,environment>::Draw(Graphics::Display &d) const
{
	double transparency = 1.0;
	for (unsigned int x = 0; x < states.size(); x++)
	{
		const auto &data = states[x].s;
		if ((states[x].where == kOpenList) && (states[x].reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(d, data);
		}
		else if (states[x].where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(d, data);
		}
		else if ((states[x].where == kClosedList) && (states[x].reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(d, data);
		}
		else if (states[x].where == kClosedList)
		{
//			if (states[x].parent == x)
//				env->SetColor(1.0, 0.5, 0.5, transparency);
//			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			env->Draw(d, data);
		}
		else if ((states[x].where == kOpenFocalList) && (states[x].reopened))
		{
			env->SetColor(0.5, 1.0, 0.5, transparency);
			env->Draw(d, data);
		}
		else if (states[x].where == kOpenFocalList)
		{
			env->SetColor(0.0, 0.0, 1.0, transparency);
			env->Draw(d, data);
		}
	}
}

#endif /* Focal_h */
