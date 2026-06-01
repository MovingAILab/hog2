//
//  ParallelAStarIDA.h
//
//  Created by Nathan Sturtevant on 6/19/24.
//

#ifndef ParallelAStarIDAStar_h
#define ParallelAStarIDAStar_h

#include <iostream>
#include "SearchEnvironment.h"
#include <unordered_map>
#include "FPUtil.h"
#include "vectorCache.h"
#include "SharedQueue.h"
#include <thread>
#include "TemplateAStar.h"

template <class environment, class state, class action>
class ParallelAStarIDAStar {
public:
	ParallelAStarIDAStar(uint64_t nodeLimit, int numThreads)
	:nodeLimit(nodeLimit), numThreads(numThreads), q(numThreads*2)
	{ storedHeuristic = false; finishAfterSolution=false;}
	virtual ~ParallelAStarIDAStar() {}
	//	void GetPath(environment *env, state from, state to,
	//				 std::vector<state> &thePath);
	void GetPath(environment *env1, environment *env2, state from, state to,
				 std::vector<action> &thePath);
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void SetHeuristic(Heuristic<state> *heur) { heuristic = heur; if (heur != 0) storedHeuristic = true;}
	void SetFinishAfterSolution(bool fas) { this->finishAfterSolution=fas;}
private:
	unsigned long long nodesExpanded, nodesTouched;
	
	void StartThreadedIteration(environment env, double bound);
	void StartTally();
	void DoIteration(environment *env,
					 action parentAction, state &currState,
					 std::vector<action> &thePath, double bound, double &nextBound, double g,
					 uint64_t &generated, uint64_t &expanded,
					 vectorCache<action> &cache);
	
	void UpdateNextBound(double currBound, double fCost);
	uint64_t nodeLimit;
	int numThreads;
	state goal;
	double nextBound, bestBound;
	//vectorCache<action> actCache;
	bool storedHeuristic;
	Heuristic<state> *heuristic;
	std::vector<std::thread*> threads;
	std::thread *tallyThread;
	SharedQueue<int> q;
	SharedQueue<int64_t> expansions;
	SharedQueue<int64_t> generations;
	SharedQueue<double> nextF;
	bool foundSolution;
	bool finishAfterSolution;
	TemplateAStar<state, action, environment> astar;
	std::mutex m;
};

//template <class state, class action>
//void ParallelAStarIDAStar<environment, state, action>::GetPath(environment *env,
//											 state from, state to,
//											 std::vector<state> &thePath)
//{
//	assert(!"Not Implemented");
//}

template <class environment, class state, class action>
void ParallelAStarIDAStar<environment, state, action>::GetPath(environment *env1, environment *env2,
														  state from, state to,
														  std::vector<action> &thePath)
{
	if (!storedHeuristic)
		heuristic = env1;
	bestBound = nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	foundSolution = false;
	
	// Set class member
	goal = to;
	if (env1->GoalTest(from, to))
		return;

	thePath.resize(0);
	std::vector<state> path;
	astar.SetHeuristic(heuristic);
	if (astar.InitializeSearch(env1, from, to, path) != true)
		return; // shouldn't happen - this was tested above
	for (uint64_t x = 0; x < nodeLimit; x++)
	{
		if (astar.DoSingleSearchStep(path)) // search complete
		{
			// TODO: need to return path
			return;
		}
	}
	printf("A* iteration complete. %" PRId64 " expanded. Current f: %1.2f\n", astar.GetNodesExpanded(), astar.GetOpenItem(0).f);
	nodesExpanded += astar.GetNodesExpanded();
	nodesTouched += astar.GetNodesTouched();
//	std::vector<action> act;
//	env1->GetActions(from, act);
	
	UpdateNextBound(0, astar.GetOpenItem(0).f);
	
	while (!foundSolution)
	{
		threads.resize(0);
		
		printf("Starting iteration with bound %f; %" PRId64 " expanded, %" PRId64 " generated\n", nextBound, nodesExpanded, nodesTouched);
		fflush(stdout);
		
		tallyThread = new std::thread(&ParallelAStarIDAStar<environment, state, action>::StartTally, this);
		for (size_t x = 0; x < numThreads; x++)
		{
			threads.push_back(new std::thread(&ParallelAStarIDAStar<environment, state, action>::StartThreadedIteration, this,
												 *env2, nextBound));
		}
		for (size_t x = 0; x < astar.GetNumOpenItems(); x++)
		{
			q.WaitAdd(x);
		}
		for (int x = 0; x < threads.size(); x++) // tell threads to stop; work is done
			q.WaitAdd(-1);
		for (int x = 0; x < threads.size(); x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		tallyThread->join();
		delete tallyThread;
		tallyThread = 0;

//		double bestBound = (nextBound+1)*10; // FIXME: Better ways to do bounds
//		for (int x = 0; x < work.size(); x++)
//		{
//			if (work[x].nextBound > nextBound && work[x].nextBound < bestBound)
//			{
//				bestBound = work[x].nextBound;
//			}
//			nodesExpanded += work[x].expanded;
//			nodesTouched += work[x].touched;
//			if (work[x].solution.size() != 0)
//			{
////				printf(">>Solution histogram>>\n");
////				PrintGHistogram();
////				printf("<<Solution histogram<<\n");
//				thePath = work[x].solution;
//			}
//		}
		nextBound = bestBound;
//		printf(">>Full histogram>>\n");
//		PrintGHistogram();
//		printf("<<Full histogram<<\n");
		if (thePath.size() != 0)
			return;
	}
}

template <class environment, class state, class action>
void ParallelAStarIDAStar<environment, state, action>::StartTally()
{
	bestBound = std::numeric_limits<double>::max();
	int done = 0;
	int64_t exp;
	double f;
	do {
		expansions.WaitRemove(exp);
		if (exp == -1)
		{
			done++;
		}
		else {
			nodesExpanded += exp;
			generations.WaitRemove(exp);
			nodesTouched += exp;
			nextF.WaitRemove(f);
			if (fgreater(f, nextBound) && fless(f, bestBound))
				bestBound = f;
		}
	} while (done < numThreads);
}


template <class environment, class state, class action>
void ParallelAStarIDAStar<environment, state, action>::StartThreadedIteration(environment env, double bound)
{
	vectorCache<action> actCache;
	std::vector<action> thePath;
	std::vector<state> path;
	uint64_t generated = 0, expanded = 0;
	double nextBound = std::numeric_limits<double>::max();
	while (true)
	{
		int nextValue;
		// All values put in before threads start. Once the queue is empty we're done
		q.WaitRemove(nextValue);
		if (nextValue == -1)
		{
			break;
		}
		
		// Reconstruct path from start to current state
		thePath.clear();
		auto i = astar.GetOpenItem(nextValue);
		astar.ExtractPathToStart(i.data, path);
		std::reverse(path.begin(), path.end());
		state startState = path[0];
		for (int x = 0; x < path.size()-1; x++)
		{
			thePath.push_back(env.GetAction(path[x], path[x+1]));
			env.ApplyAction(startState, thePath.back());
			assert(startState == path[x+1]);
		}
		assert(startState == i.data);
		//env.GetAction(path[0], path[1]);
		const state &parent = astar.GetItem(i.parentID).data;
//		m.lock();
//		std::cout << "Thread: " << nextValue << "\n";// << parent << "\n";
//		m.unlock();
		action a = env.GetAction(parent, i.data);
		assert(thePath.back() == a);
		//env.InvertAction(a);
		DoIteration(&env, a, i.data, thePath, bound, nextBound, i.g, generated, expanded, actCache);
//		m.lock();
//		std::cout << expanded << " expanded; next bound " << nextBound << "\n";
//		m.unlock();
		expansions.Add(expanded);
		expanded = 0;
		generations.Add(generated);
		generated = 0;
		nextF.Add(nextBound);
	}
	// signal completion to tally thread; main thread exits when it is done
	expansions.Add(-1);
}


template <class environment, class state, class action>
void ParallelAStarIDAStar<environment, state, action>::DoIteration(environment *env,
																   action parentAction, state &currState,
																   std::vector<action> &thePath, double bound, double &nextBound, double g,
																   uint64_t &generated, uint64_t &expanded,
																   vectorCache<action> &cache)
{
	double h = heuristic->HCost(currState, goal);//, parentH); // TODO: restore code that uses parent h-cost
	
	if (fgreater(g+h, bound))
	{
		if (g+h < nextBound)
			nextBound = g+h;
		return;
	}

	// must do this after we check the f-cost bound
	if (env->GoalTest(currState, goal))
	{
		foundSolution = true; // TODO: consider avoid using a shared variable here
//		return;
	}
	
	std::vector<action> &actions = *cache.getItem();
	//env->GetActions(currState, actions);
	env->GetActions(currState, actions, parentAction); // parent pruning is in this function
	generated += actions.size();
	expanded++;
//	w.gHistogram[g]++;
//	w.fHistogram[g+h]++;

	for (unsigned int x = 0; x < actions.size(); x++)
	{
//		if (actions[x] == forbiddenAction)
//			continue;
		
		thePath.push_back(actions[x]);
		
		double edgeCost = env->GCost(currState, actions[x]);
		env->ApplyAction(currState, actions[x]);
		action a = actions[x];
		//env->InvertAction(a);
		DoIteration(env, a, currState, thePath, bound, nextBound, g+edgeCost, generated, expanded, cache);
		env->UndoAction(currState, actions[x]);
		thePath.pop_back();
//		if (foundSolution)// <= w.unitNumber)
//			break;
	}
	cache.returnItem(&actions);
}


template <class environment, class state, class action>
void ParallelAStarIDAStar<environment, state, action>::UpdateNextBound(double currBound, double fCost)
{
	if (!fgreater(nextBound, currBound))
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
	else if (fgreater(fCost, currBound) && fless(fCost, nextBound))
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
}

#endif
