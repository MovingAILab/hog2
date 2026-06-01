//
//  BSStar.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/14/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef BSStar_h
#define BSStar_h

#include "AStarOpenClosed.h"
#include "FPUtil.h"
#include "Timer.h"
#include <unordered_map>

template <class state, int epsilon = 1>
struct BSCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double p1 = i1.g+i1.h;
		double p2 = i2.g+i2.h;
		if (fequal(p1, p2))
		{
			//return (fgreater(i1.g, i2.g)); // low g-cost over high
			return (fless(i1.g, i2.g)); // high g-cost over low
		}
		return (fgreater(p1, p2)); // low priority over high
	}
};

template <class state, class action, class environment, class priorityQueue = AStarOpenClosed<state, BSCompare<state>> >
class BSStar {
public:
	BSStar() { forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount(); }
	virtual ~BSStar() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	virtual const char *GetName() { return "MM"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = uniqueNodesExpanded = 0; }
	
	inline const int GetNumForwardItems() { return forwardQueue.size(); }
	inline const AStarOpenClosedData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return backwardQueue.size(); }
	inline const AStarOpenClosedData<state> &GetBackwardItem(unsigned int which) { return backwardQueue.Lookat(which); }
	
	uint64_t GetUniqueNodesExpanded() const { return uniqueNodesExpanded; }
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const;

	void OpenGLDraw() const;
	
	//	void SetWeight(double w) {weight = w;}
private:
	void Trim();
	void Nip(const state &, priorityQueue &reverse);

	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
	{ uint64_t theID; backwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToGoalFromID(theID, thePath); }
	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(backwardQueue.Lookup(node).data);
			node = backwardQueue.Lookup(node).parentID;
		} while (backwardQueue.Lookup(node).parentID != node);
		thePath.push_back(backwardQueue.Lookup(node).data);
	}
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; forwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(forwardQueue.Lookup(node).data);
			node = forwardQueue.Lookup(node).parentID;
		} while (forwardQueue.Lookup(node).parentID != node);
		thePath.push_back(forwardQueue.Lookup(node).data);
	}
	
	void OpenGLDraw(const priorityQueue &queue) const;
	
	void Expand(priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic,
				const state &target);
	priorityQueue forwardQueue, backwardQueue;
	state goal, start;
//	std::unordered_map<std::pair<double, double>, int> dist;
//	std::unordered_map<std::pair<double, double>, int> f, b;
	uint64_t nodesTouched, nodesExpanded, uniqueNodesExpanded;
	state middleNode;
	double currentCost;
	double lastMinForwardG;
	double lastMinBackwardG;
	double epsilon;
	
	std::vector<state> neighbors;
	environment *env;
	Timer t;
	Heuristic<state> *forwardHeuristic;
	Heuristic<state> *backwardHeuristic;
	
	double oldp1;
	double oldp2;
};

template <class state, class action, class environment, class priorityQueue>
void BSStar<state, action, environment, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
															Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	t.StartTimer();
	while (!DoSingleSearchStep(thePath))
	{ }
}

template <class state, class action, class environment, class priorityQueue>
bool BSStar<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																	 Heuristic<state> *forward, Heuristic<state> *backward,
																	 std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentCost = DBL_MAX;
	forwardQueue.Reset();
	backwardQueue.Reset();
	ResetNodeCount();
	thePath.resize(0);
	start = from;
	goal = to;
	if (start == goal)
		return false;
	oldp1 = oldp2 = 0;
	lastMinForwardG = 0;
	lastMinBackwardG = 0;
	forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
	backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start));
	return true;
}

template <class state, class action, class environment, class priorityQueue>
bool BSStar<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
	{
		if (currentCost != DBL_MAX)
		{
			std::vector<state> pFor, pBack;
			ExtractPathToGoal(middleNode, pBack);
			ExtractPathToStart(middleNode, pFor);
			reverse(pFor.begin(), pFor.end());
			thePath = pFor;
			thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
		}
		return true;
	}
	
	if (forwardQueue.OpenSize() > backwardQueue.OpenSize())
		Expand(backwardQueue, forwardQueue, backwardHeuristic, start);
	else
		Expand(forwardQueue, backwardQueue, forwardHeuristic, goal);
	return false;
}

template <class state, class action, class environment, class priorityQueue>
void BSStar<state, action, environment, priorityQueue>::Expand(priorityQueue &current,
														   priorityQueue &opposite,
														   Heuristic<state> *heuristic, const state &target)
{
	uint64_t nextID;
	
	bool success = false;
	while (current.OpenSize() > 0) {
		nextID = current.Close();
		uint64_t reverseLoc;
		auto loc = opposite.Lookup(env->GetStateHash(current.Lookup(nextID).data), reverseLoc);
		if (loc != kClosedList)
		{
			success = true;
			break;
		}
		// We are doing this lazily. It isn't clear from the BS* paper what it means to remove the
		// descendants. If you have an explicit graph it is handled differently than when the states
		// only exist in open/closed.
//		// 1. if current closed in opposite direction
//		// 1a. Remove descendents of current in open
//		Nip(current.Lookup(nextID).data, opposite);
	}
	if (!success)
		return;
	
	// 2. Else expand as usual on current direction
	// 2a. Check for bidirectional solution
	// 2b. Set trim flag
	
	// 3. If trim, remove any states with f >= best solution from open (Except start/goal?)

	bool foundBetterSolution = false;
	nodesExpanded++;
	if (current.Lookup(nextID).reopened == false)
		uniqueNodesExpanded++;

	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		uint64_t hash = env->GetStateHash(succ);
		auto loc = current.Lookup(hash, childID);
		auto &childData = current.Lookup(childID);
		auto &parentData = current.Lookup(nextID);
		
		double edgeCost = env->GCost(parentData.data, succ);

		// ignore states with greater cost than best solution
		if (fgreatereq(parentData.g+edgeCost, currentCost))
			continue;
		
		switch (loc)
		{
			case kClosedList: // ignore
				if (fless(parentData.g+edgeCost, childData.g))
				{
					childData.h = std::max(childData.h, parentData.h-edgeCost);
					childData.parentID = nextID;
					childData.g = parentData.g+edgeCost;
					current.Reopen(childID);
				}
				break;
			case kOpenList: // update cost if needed
			{
				if (fless(parentData.g+edgeCost, childData.g))
				{
					childData.parentID = nextID;
					childData.g = parentData.g+edgeCost;
					current.KeyChanged(childID);
					
					
					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(hash, reverseLoc);
					if (loc == kOpenList)
					{
						if (fless(parentData.g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							foundBetterSolution = true;
//							printf("Potential updated solution found, cost: %1.2f + %1.2f = %1.2f\n",
//								   parentData.g+edgeCost,
//								   opposite.Lookup(reverseLoc).g,
//								   parentData.g+edgeCost+opposite.Lookup(reverseLoc).g);
							currentCost = parentData.g+edgeCost + opposite.Lookup(reverseLoc).g;
							middleNode = succ;
//							PrintOpenStats(f);
//							PrintOpenStats(b);
						}
					}
				}
			}
				break;
			case kNotFound:
			{
				double g = parentData.g+edgeCost;
				double h = std::max(heuristic->HCost(succ, target), parentData.h-edgeCost);
				
				// Ignore nodes that don't have lower f-cost than the incumbant solution
				if (!fless(g+h, currentCost))
					break;
				
				current.AddOpenNode(succ, // This may invalidate our references
									hash,
									g,
									h,
									nextID);
				
				// check for solution
				uint64_t reverseLoc;
				auto loc = opposite.Lookup(hash, reverseLoc);
				if (loc == kOpenList)
				{
					if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
					{
						foundBetterSolution = true;
//						printf("Potential solution found, cost: %1.2f + %1.2f = %1.2f\n",
//							   current.Lookup(nextID).g+edgeCost,
//							   opposite.Lookup(reverseLoc).g,
//							   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g);
						currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
						middleNode = succ;
//						PrintOpenStats(f);
//						PrintOpenStats(b);
					}
				}
			}
		}
	}
	
	if (foundBetterSolution)
		Trim();
}

template <class state, class action, class environment, class priorityQueue>
void BSStar<state, action, environment, priorityQueue>::Nip(const state &s, priorityQueue &queue)
{
	assert(!"Not using this code currently - the correct implementation of 'remove' is unclear from BS*");
	// At this point parent has been removed from open
	// Need to find any successors that have a parent id of parent & recursively remove them from open

	std::vector<state> n;

	uint64_t parentID;
	auto loc = queue.Lookup(env->GetStateHash(s), parentID);
	assert(loc == kClosedList);
	env->GetSuccessors(s, n);
	for (auto &succ : n)
	{
		uint64_t childID;
		uint64_t hash = env->GetStateHash(succ);
		auto loc = queue.Lookup(hash, childID);
		auto &childData = queue.Lookup(childID);
		if (loc == kClosedList && childData.parentID == parentID)
		{
			Nip(childData.data, queue);
		}
		else if (loc == kOpenList && (childData.parentID == parentID))// && (childData.data != middleNode))
		{
			if (childData.data == middleNode)
			{
				std::cout << "Error - removing middle node\n";
				if (&queue == &forwardQueue)
					std::cout << "In backward search - removing from for\n";
				else
					std::cout << "In forward search - removing from back\n";
				std::cout << "Parent: " << s << "\n";
				std::cout << "Middle: " << middleNode << "\n";
				std::vector<state> pFor, pBack, final;
				ExtractPathToGoal(middleNode, pBack);
				ExtractPathToStart(middleNode, pFor);
				reverse(pFor.begin(), pFor.end());
				std::cout << "Path forward: \n";
				
				for (auto &s : pFor)
					std::cout << s << "\n";
				std::cout << "Path backward: \n";
				for (auto &s : pBack)
					std::cout << s << "\n";

				exit(0);
			}
			queue.Remove(env->GetStateHash(childData.data));
		}
	}
}

template <class state, class action, class environment, class priorityQueue>
void BSStar<state, action, environment, priorityQueue>::Trim()
{
	for (unsigned int x = 0; x < forwardQueue.OpenSize(); x++)
	{
		const AStarOpenClosedData<state> &data = forwardQueue.Lookat(forwardQueue.GetOpenItem(x));
		if (data.where == kOpenList && fgreatereq(data.g + data.h, currentCost) && (data.data != middleNode)) // and not start or goal
		{
			forwardQueue.Remove(env->GetStateHash(data.data));
		}
	}

	for (unsigned int x = 0; x < backwardQueue.OpenSize(); x++)
	{
		const AStarOpenClosedData<state> &data = backwardQueue.Lookat(backwardQueue.GetOpenItem(x));
		if (data.where == kOpenList && fgreatereq(data.g + data.h, currentCost) && (data.data != middleNode)) // and not start or goal
		{
			backwardQueue.Remove(env->GetStateHash(data.data));
		}
	}

}

template <class state, class action, class environment, class priorityQueue>
uint64_t BSStar<state, action, environment, priorityQueue>::GetNecessaryExpansions() const
{
	uint64_t count = 0;
	for (unsigned int x = 0; x < forwardQueue.size(); x++)
	{
		const AStarOpenClosedData<state> &data = forwardQueue.Lookat(x);
		if ((data.where == kClosedList) && (fless(data.g+data.h, currentCost)))
			count++;
	}
	for (unsigned int x = 0; x < backwardQueue.size(); x++)
	{
		const AStarOpenClosedData<state> &data = backwardQueue.Lookat(x);
		if ((data.where == kClosedList) && (fless(data.g+data.h, currentCost)))
			count++;
	}
	return count;
}

template <class state, class action, class environment, class priorityQueue>
void BSStar<state, action, environment, priorityQueue>::OpenGLDraw() const
{
	OpenGLDraw(forwardQueue);
	OpenGLDraw(backwardQueue);
}

template <class state, class action, class environment, class priorityQueue>
void BSStar<state, action, environment, priorityQueue>::OpenGLDraw(const priorityQueue &queue) const
{
	double transparency = 0.9;
	if (queue.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (queue.OpenSize() > 0)
	{
		top = queue.Peek();
	}
	for (unsigned int x = 0; x < queue.size(); x++)
	{
		const AStarOpenClosedData<state> &data = queue.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
	}
}


#endif /* BSStar_h */
