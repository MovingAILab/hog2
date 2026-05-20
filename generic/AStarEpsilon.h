//
//  AStarEpsilon.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/24/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef AStarEpsilon_h
#define AStarEpsilon_h


#include <iostream>
#include "FPUtil.h"
#include <unordered_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
#include "TemplateAStar.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"
#include <algorithm> // for vector reverse
#include "GenericSearchAlgorithm.h"

template <class state>
struct GBFSCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		if (fequal(i1.h, i2.h))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(i1.h, i2.h));
	}
};


/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment>
class AStarEpsilon : public GenericSearchAlgorithm<state,action,environment> {
public:
	AStarEpsilon(double optimalBound = 2) {
		ResetNodeCount(); env = 0; bound = optimalBound; theHeuristic = 0;
		phi = [optimalBound](double h, double g){ return g/optimalBound+h; };
	}
	virtual ~AStarEpsilon() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
//	// uses admissible heuristic (regular A* search)
//	AStarOpenClosed<state, AStarCompare<state>> f;
//	// uses inadmissible heuristic
//	AStarOpenClosed<state, GBFSCompare<state>> focal;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath);
	void ExtractPathToStartFromID(size_t node, std::vector<state> &thePath);
	virtual const char *GetName();
	
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
//	int GetMemoryUsage();
//
	size_t GetNumItems() const { return allStates.size(); }
	bool IsOpen(size_t item) const { return allStates[item].open; }
	bool IsFocal(size_t item) const
	{ return (flesseq(Phi(allStates[item].hCost, allStates[item].gCost), minf)); }
	state GetItem(size_t item) const { return allStates[item].s; }
	float GetItemGCost(size_t item) const { return allStates[item].gCost; }
	float GetItemHCost(size_t item) const { return allStates[item].hCost; }
	float GetMinF() const { return minf; }
	//	state CheckNextOpenNode();
//	state CheckNextFocalNode();
//	bool GetOpenListGCost(const state &val, double &gCost) const;
//	bool GetFocalListGCost(const state &val, double &gCost) const;
//
//	bool GetClosedListGCost(const state &val, double &gCost) const;
//	bool GetClosedItem(const state &s, AStarOpenClosedData<state> &);
//	unsigned int GetNumOpenItems() { return f.OpenSize(); }
//	unsigned int GetNumFocalItems() { return focal.OpenSize(); }
//	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return f.Lookat(f.GetOpenItem(which)); }
//	inline const AStarOpenClosedData<state> &GetFocalItem(unsigned int which) { return focal.Lookat(focal.GetOpenItem(which)); }
//	inline const int GetNumItems() { return focal.size(); }
//	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return focal.Lookat(which); }
//	bool HaveExpandedState(const state &val)
//	{ uint64_t key; return focal.Lookup(env->GetStateHash(val), key) != kNotFound; }
//	dataLocation GetStateLocation(const state &val)
//	{ uint64_t key; return focal.Lookup(env->GetStateHash(val), key); }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
//
//	void OpenGLDraw() const;
	void Draw(Graphics::Display &d) const;
	
	void SetPhi(std::function<double(double, double)> p)
	{
		phi = p;
	}
	double Phi(double h, double g) const
	{
		return phi(h, g);
	}
	void SetOptimalityBound(double w)
	{
		phi = [w](double h, double g){ return g/w+h; };
		bound = w;
	}
	double GetOptimalityBound() { return bound; }
private:
	void GetMinFOnOpen() const;
	size_t GetBestStateOnFocal() const;

	void DrawOpen(Graphics::Display &d) const;
	void DrawFocal(Graphics::Display &d) const;
	void Expand(size_t which);
	void UpdateCost(size_t next, size_t parent);
	void AddState(const state &s, size_t parent);
	size_t Lookup(const state &s);
	uint64_t nodesTouched, nodesExpanded;
	
	struct item {
		state s;
		float gCost;
		float hCost;
		bool open;
		size_t parent;
	};
	std::vector<item> allStates;
//	std::vector<item> closed;
	std::vector<state> neighbors;
//	std::vector<uint64_t> neighborID;
//	std::vector<double> edgeCosts;
//	std::vector<dataLocation> neighborLoc;

	std::vector<state> solution;
	environment *env;
//	double bestSolution;
	double bound;
	mutable float minf;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;

	std::function<double(double, double)> phi;
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
const char *AStarEpsilon<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "AStarEpsilon[%1.2f]", bound);
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
void AStarEpsilon<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
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
void AStarEpsilon<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
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
bool AStarEpsilon<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
//	bestSolution = DBL_MAX;
	
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
//	focal.resize(0);
	allStates.resize(0);
//	closed.resize(0);
//	focal.Reset(env->GetMaxHash());
//	f.Reset(env->GetMaxHash());
	solution.clear();
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to)) //assumes that from and to are valid states
	{
		return false;
	}
	
//	focal.AddOpenNode(start, env->GetStateHash(start), 0, theHeuristic->HCost(start, goal));
//	f.AddOpenNode(start, env->GetStateHash(start), 0, theHeuristic->HCost(start, goal));
	allStates.push_back({start, 0, (float)theHeuristic->HCost(start, goal), true, 0});
	
	return true;
}

///**
// * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
// * @author Nathan Sturtevant
// * @date 01/06/08
// */
//template <class state, class action, class environment>
//void AStarEpsilon<state,action,environment>::AddAdditionalStartState(state& newState)
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
//void AStarEpsilon<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
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
bool AStarEpsilon<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (solution.size() > 0)
	{
		thePath = solution;
		return true;
	}

	// get minimum f-cost
	GetMinFOnOpen();
	if (minf == FLT_MAX)
	{
		printf("No path\n");
		thePath.resize(0); // no path found!
		return true;
	}
	
	// get state with best epsilon
	size_t which = GetBestStateOnFocal();
	
	if (env->GoalTest(allStates[which].s, goal))
	{
		ExtractPathToStartFromID(which, solution);
		// Path is backwards - reverse
		reverse(solution.begin(), solution.end());
		return true;
	}
	else {
		Expand(which);
	}
	
//	uint64_t nodeOnfocal;
//	uint64_t nodeOnF;
//	// only reopen states taken from f, not fhat
//	bool reopen = true;
//
//	double regularF = f.Lookat(f.Peek()).g+f.Lookat(f.Peek()).h;
//	double focalF = focal.Lookat(focal.Peek()).g+focal.Lookat(focal.Peek()).h;
//	printf("Min f: %1.2f; next focal f: %1.2f. Max allowable: %1.2f.", regularF, focalF, regularF*bound);
//	if (flesseq(focalF, bound*regularF))
//	{
//		//printf(" Expanding focal\n");
//		ExpandFocal();
//	}
//	else {
//		//printf(" Expanding open\n");
//		ExpandOpen();
//	}
//
//	if (solution.size() > 0)
//	{
//		thePath = solution;
//		return true;
//	}

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

template <class state, class action, class environment>
void AStarEpsilon<state,action,environment>::GetMinFOnOpen() const
{
	minf = FLT_MAX;
	for (auto &i : allStates)
	{
		if (i.open)
			minf = std::min(minf, i.gCost+i.hCost);
	}
}

template <class state, class action, class environment>
size_t AStarEpsilon<state,action,environment>::GetBestStateOnFocal() const
{
	float focalCost = FLT_MAX;
	size_t which = allStates.size();
	for (size_t x = 0; x < allStates.size(); x++)
	{
		if (allStates[x].open && fless(allStates[x].hCost, focalCost) && (flesseq(Phi(allStates[x].hCost, allStates[x].gCost), minf)))
		{
			focalCost = allStates[x].hCost;
			which = x;
		}
	}
	return which;
}

template <class state, class action, class environment>
void AStarEpsilon<state,action,environment>::UpdateCost(size_t next, size_t parent)
{
	float newCost = allStates[parent].gCost+env->GCost(allStates[parent].s, allStates[next].s);
	if (fless(newCost, allStates[next].gCost))
	{
		// If we update the cost it's always on open now - TODO: count re-openings
		allStates[next].open = true;
		allStates[next].gCost = newCost;
		allStates[next].parent = parent;
	}
}

template <class state, class action, class environment>
void AStarEpsilon<state,action,environment>::AddState(const state &s, size_t parent)
{
	float cost = (float)allStates[parent].gCost+env->GCost(allStates[parent].s, s);
	allStates.push_back({s, cost, (float)theHeuristic->HCost(s, goal), true, parent});
}


/**
 * Expands a single state from the given open
 * @author Nathan Sturtevant
 * @date 01/27/19
 */
template <class state, class action, class environment>
void AStarEpsilon<state,action,environment>::Expand(size_t which)
{
	env->GetSuccessors(allStates[which].s, neighbors);
	allStates[which].open = false;
	nodesExpanded++;
	for (int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		size_t next = Lookup(neighbors[x]);
		if (next < allStates.size())
			UpdateCost(next, which);
		else
			AddState(neighbors[x], which);
	}
}

template <class state, class action,class environment>
size_t AStarEpsilon<state, action,environment>::Lookup(const state &s)
{
	size_t x = 0;
	for (; x < allStates.size(); x++)
		if (allStates[x].s == s)
			break;
	return x;
}

/**
 * Get the path from a goal state to the start state
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param node the state from which the path is extracted
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment>
void AStarEpsilon<state, action,environment>::ExtractPathToStartFromID(size_t node,
																	   std::vector<state> &thePath)
{
	do {
		thePath.push_back(allStates[node].s);
		node = allStates[node].parent;
	} while (allStates[node].parent != node);
	// start state
	thePath.push_back(allStates[node].s);
}

template <class state, class action,class environment>
void AStarEpsilon<state, action,environment>::ExtractPathToStart(state &node, std::vector<state> &thePath)
{
	ExtractPathToStartFromID(Lookup(node), thePath);
}

	



/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 *
 */
//template <class state, class action, class environment>
//void AStarEpsilon<state, action,environment>::OpenGLDraw() const
//{
//	double transparency = 1.0;
//	//GetMinFOnOpen();
//
//	for (unsigned int x = 0; x < allStates.size(); x++)
//	{
//		if (allStates[x].open)
//		{
//			if (flesseq(Phi(allStates[x].hCost, allStates[x].gCost), minf))
//				env->SetColor(0.0, 1.0, 1.0, transparency);
//			else
//				env->SetColor(0.0, 1.0, 0.0, transparency);
//		}
//		else {
//			env->SetColor(1.0, 0.0, 0.0, transparency);
//		}
//		if (allStates[x].parent == x)
//			env->SetColor(1.0, 0.5, 0.5, transparency);
//		env->OpenGLDraw(allStates[x].s);
//
//	}
//	env->SetColor(1.0, 0.5, 1.0, 0.5);
//	env->OpenGLDraw(goal);
//}

template <class state, class action, class environment>
void AStarEpsilon<state, action,environment>::Draw(Graphics::Display &d) const
{
	double transparency = 1.0;
	//float minf = GetMinFOnOpen();

	for (unsigned int x = 0; x < allStates.size(); x++)
	{
		if (allStates[x].open)
		{
			if (flesseq(Phi(allStates[x].hCost, allStates[x].gCost), minf))
				env->SetColor(0.0, 1.0, 1.0, transparency); // on focal
			else
				env->SetColor(0.0, 1.0, 0.0, transparency); // only on open
		}
		else {
			env->SetColor(1.0, 0.0, 0.0, transparency);
		}
		if (allStates[x].parent == x)
			env->SetColor(1.0, 0.5, 0.5, transparency);
		env->Draw(d, allStates[x].s);

	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(d, goal);
}

#endif /* AStarEpsilon_h */
