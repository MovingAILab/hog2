//
//  MM.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/27/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef MM_h
#define MM_h

#include "AStarOpenClosed.h"
#include "FPUtil.h"
#include "Timer.h"
#include <unordered_map>
#include <cinttypes>

template <class state, int epsilon = 1>
struct MMCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		double p1 = std::max(i1.g+i1.h, i1.g*2+epsilon);
		double p2 = std::max(i2.g+i2.h, i2.g*2+epsilon);
		if (fequal(p1, p2))
		{
			//return (fgreater(i1.g, i2.g)); // low g-cost over high
			return (fless(i1.g, i2.g)); // high g-cost over low
		}
		return (fgreater(p1, p2)); // low priority over high
	}
};

namespace std {
	template <> struct hash<std::pair<double, double>>
	{
		size_t operator()(const std::pair<double, double> & x) const
		{
			return std::hash<double>()(x.first)^(std::hash<double>()(x.second)<<16);
		}
	};
}


template <class state, class action, class environment, class priorityQueue = AStarOpenClosed<state, MMCompare<state>> >
class MM {
public:
	MM(double epsilon = 1.0):epsilon(epsilon) { forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount(); }
	virtual ~MM() {}
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
	//void FullBPMX(uint64_t nodeID, int distance);
	
	std::string SVGDraw() const;
//	void OpenGLDraw() const;
	void PrintHDist()
	{
		std::vector<uint64_t> d;
		for (auto i = dist.begin(); i != dist.end(); i++)
		{
			if (i->first.first < i->first.second)
			{
				int h = (int)i->first.second;
				if (h >= d.size())
					d.resize(h+1);
				d[h] += i->second;
			}
		}
		printf("MM Dynamic Distribution\n");
		for (int x = 0; x < d.size(); x++)
		{
			if (d[x] != 0)
				printf("%d\t%" PRId64 "\n", x, d[x]);
		}
	}
	void PrintOpenStats(std::unordered_map<std::pair<double, double>, int>  &s)
	{
		printf("Search distributions: (%s)\n", ((&s)==(&f))?"forward":"backward");
		for (auto i = s.begin(); i != s.end(); i++)
		{
			if (i->second > 0)
			{
				bool ignore = false;
				ignore = (i->first.first+i->first.second >= currentCost);
				printf("%c g: %1.1f h: %1.1f count: %d\n", ignore?'*':' ',
					   i->first.first, i->first.second, i->second);
			}
		}
	}

//	void SetWeight(double w) {weight = w;}
private:
	
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

//	void OpenGLDraw(const priorityQueue &queue) const;
	std::string SVGDraw(const priorityQueue &queue) const;

	void Expand(priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic,
				const state &target,
				std::unordered_map<std::pair<double, double>, int> &count);
//				std::unordered_map<double, int> &ming,
//				std::unordered_map<double, int> &minf);
	priorityQueue forwardQueue, backwardQueue;
	state goal, start;
	std::unordered_map<std::pair<double, double>, int> dist;
	std::unordered_map<std::pair<double, double>, int> f, b;
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
	bool recheckPath;
};

template <class state, class action, class environment, class priorityQueue>
void MM<state, action, environment, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
			 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	t.StartTimer();
	while (!DoSingleSearchStep(thePath))
	{ }
}

template <class state, class action, class environment, class priorityQueue>
bool MM<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
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
	f.clear();
	b.clear();
	recheckPath = false;
	return true;
}

template <class state, class action, class environment, class priorityQueue>
bool MM<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
	{
		return true;
	}
	
//	if (forwardQueue.OpenSize() == 0)
//		//Expand(backwardQueue, forwardQueue, backwardHeuristic, start, g_b, f_b);
//		Expand(backwardQueue, forwardQueue, backwardHeuristic, start, b);
//	
//	if (backwardQueue.OpenSize() == 0)
//		//Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, g_f, f_f);
//		Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, f);

	uint64_t forward = forwardQueue.Peek();
	uint64_t backward = backwardQueue.Peek();
	
	const AStarOpenClosedData<state> &nextForward = forwardQueue.Lookat(forward);
	const AStarOpenClosedData<state> &nextBackward = backwardQueue.Lookat(backward);

	double p1 = std::max(nextForward.g+nextForward.h, nextForward.g*2);
	double p2 = std::max(nextBackward.g+nextBackward.h, nextBackward.g*2);
	if (p1 > oldp1)
	{
//		printf("Forward priority to %1.2f [%" PRId64 " expanded - %1.2fs]\n", p1, GetNodesExpanded(), t.EndTimer());
		oldp1 = p1;
		//PrintOpenStats(f);
	}
	if (p2 > oldp2)
	{
//		printf("Backward priority to %1.2f [%" PRId64 " expanded - %1.2fs]\n", p2, GetNodesExpanded(), t.EndTimer());
		oldp2 = p2;
		//PrintOpenStats(b);
	}
	
	if (fless(p1, p2))
	{
		//Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, g_f, f_f);
		Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, f);
	}
	else if (fless(p2, p1))
	{
		//Expand(backwardQueue, forwardQueue, backwardHeuristic, start, g_b, f_b);
		Expand(backwardQueue, forwardQueue, backwardHeuristic, start, b);
	}
	else { // equal priority
		if (fless(nextForward.g, nextBackward.g))
		{
			//Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, g_f, f_f);
			Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, f);
		}
		else if (fless(nextBackward.g, nextForward.g))
		{
			//Expand(backwardQueue, forwardQueue, backwardHeuristic, start, g_b, f_b);
			Expand(backwardQueue, forwardQueue, backwardHeuristic, start, b);
		}
		else {
			Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, f);
			//Expand(forwardQueue, backwardQueue, forwardHeuristic, goal, g_f, f_f);
		}
	}
	// check if we can terminate
	if (recheckPath)
	{
		recheckPath = false;
		// TODO: make this more efficient
		double minForwardG = DBL_MAX;
		double minBackwardG = DBL_MAX;
		double minForwardF = DBL_MAX;
		double minBackwardF =  DBL_MAX;
		double forwardP;
		double backwardP;
		
		for (auto i = f.begin(); i != f.end(); i++)
		{
			if (i->second > 0) // some elements
			{
				if ((i->first.first + i->first.second < currentCost) && // termination only stopped by lower f-cost
					(i->first.first + lastMinBackwardG + 1.0 < currentCost))
				{
					minForwardG = std::min(minForwardG, i->first.first);
					minForwardF = std::min(minForwardF, i->first.first+i->first.second);
				}
			}
		}
		for (auto i = b.begin(); i != b.end(); i++)
		{
			if (i->second > 0) // some elements
			{
				if ((i->first.first + i->first.second < currentCost) && // termination only stopped by lower f-cost
					(i->first.first + lastMinForwardG + 1.0 < currentCost))
				{
					minBackwardG = std::min(minBackwardG, i->first.first);
					minBackwardF = std::min(minBackwardF, i->first.first+i->first.second);
				}
			}
		}
		
		{
			auto iB = backwardQueue.Lookat(backwardQueue.Peek());
			backwardP = std::max(iB.g+iB.h, iB.g*2);
			auto iF = forwardQueue.Lookat(forwardQueue.Peek());
			forwardP = std::max(iF.g+iF.h, iF.g*2);
		}
		bool done = false;
		if (minForwardF == DBL_MAX)
		{
			minForwardF = minForwardG = currentCost+1;
		}
		if (minBackwardF == DBL_MAX)
		{
			minBackwardF = minBackwardG = currentCost+1;
		}
		if (!fgreater(currentCost, minForwardF))
		{
//			printf("Terminated on forwardf (%f >= %f)\n", minForwardF, currentCost);
			done = true;
		}
		if (!fgreater(currentCost, minBackwardF))
		{
//			printf("Terminated on backwardf (%f >= %f)\n", minBackwardF, currentCost);
			done = true;
		}
		if (!fgreater(currentCost, minForwardG+minBackwardG+epsilon)) // TODO: epsilon
		{
//			printf("Terminated on g+g+epsilon (%f+%f+%f >= %f)\n", minForwardG, minBackwardG, epsilon, currentCost);
			done = true;
		}
		if (!fgreater(currentCost, std::min(forwardP, backwardP)))
		{
//			printf("Terminated on forwardP/backwardP (min(%f, %f) >= %f)\n", forwardP, backwardP, currentCost);
			done = true;
		}
//		if (!fgreater(currentCost, backwardP))
//		{
//			printf("Terminated on backwardP\n");
//			done = true;
//		}
		// for now, always terminate
		lastMinBackwardG = minBackwardG;
		lastMinForwardG = minForwardG;
		if (done)
		{
//			PrintOpenStats(f);
//			PrintOpenStats(b);
			
			std::vector<state> pFor, pBack;
			ExtractPathToGoal(middleNode, pBack);
			ExtractPathToStart(middleNode, pFor);
			reverse(pFor.begin(), pFor.end());
			thePath = pFor;
			thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
			
			return true;
		}
	}
	return false;
}

template <class state, class action, class environment, class priorityQueue>
void MM<state, action, environment, priorityQueue>::Expand(priorityQueue &current,
														   priorityQueue &opposite,
														   Heuristic<state> *heuristic, const state &target,
														   std::unordered_map<std::pair<double, double>, int> &count)
//														   std::unordered_map<double, int> &ming,
//														   std::unordered_map<double, int> &minf)
{
	uint64_t nextID = current.Close();
	nodesExpanded++;
	if (current.Lookup(nextID).reopened == false)
		uniqueNodesExpanded++;

	// decrease count from parent
	{
		auto &parentData = current.Lookup(nextID);
		count[{parentData.g,parentData.h}]--;
		if (count[{parentData.g,parentData.h}] == 0 && currentCost < DBL_MAX)
		{
			recheckPath = true;
		}
	}
//	std::cout << "Expanding " << current.Lookup(nextID).data << "\n";
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
		switch (loc)
		{
			case kClosedList: // ignore
				if (fless(parentData.g+edgeCost, childData.g))
				{
					childData.h = std::max(childData.h, parentData.h-edgeCost);
					childData.parentID = nextID;
					childData.g = parentData.g+edgeCost;
					count[{childData.g,childData.h}]++;
					dist[{childData.g,childData.h}]++;
					current.Reopen(childID);
				}
				break;
			case kOpenList: // update cost if needed
			{
				// 1-step BPMX
				parentData.h = std::max(childData.h-edgeCost, parentData.h);

				if (fgreater(parentData.h-edgeCost, childData.h))
				{
					count[{childData.g,childData.h}]--;
					dist[{childData.g,childData.h}]--;
					//minf[childData.g+childData.h]--;
					childData.h = parentData.h-edgeCost;
					//minf[childData.g+childData.h]++;
					count[{childData.g,childData.h}]++;
					dist[{childData.g,childData.h}]++;
				}
				if (fless(parentData.g+edgeCost, childData.g))
				{
					count[{childData.g,childData.h}]--;
					dist[{childData.g,childData.h}]--;
					childData.parentID = nextID;
					childData.g = parentData.g+edgeCost;
					current.KeyChanged(childID);
					count[{childData.g,childData.h}]++;
					dist[{childData.g,childData.h}]++;
					
					
					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(hash, reverseLoc);
					if (loc == kOpenList)
					{
						if (fless(parentData.g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							recheckPath = true;
							// TODO: store current solution
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
//				ming[g]++;
//				minf[g+h]++;
				count[{g,h}]++;
				dist[{g,h}]++;
				// 1-step BPMX
				parentData.h = std::max(h-edgeCost, parentData.h);

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
						recheckPath = true;
						// TODO: store current solution
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
}

template <class state, class action, class environment, class priorityQueue>
uint64_t MM<state, action, environment, priorityQueue>::GetNecessaryExpansions() const
{
	uint64_t count = 0;
	for (unsigned int x = 0; x < forwardQueue.size(); x++)
	{
		const AStarOpenClosedData<state> &data = forwardQueue.Lookat(x);
		if ((data.where == kClosedList) && (fless(data.g+data.h, currentCost)) && fless(data.g*2, currentCost))
			count++;
	}
	for (unsigned int x = 0; x < backwardQueue.size(); x++)
	{
		const AStarOpenClosedData<state> &data = backwardQueue.Lookat(x);
		if ((data.where == kClosedList) && (fless(data.g+data.h, currentCost)) && (fless(data.g*2, currentCost)))
			count++;
	}
	return count;
}

//template <class state, class action, class environment, class priorityQueue>
//void MM<state, action, environment, priorityQueue>::OpenGLDraw() const
//{
//	OpenGLDraw(forwardQueue);
//	OpenGLDraw(backwardQueue);
//}

//template <class state, class action, class environment, class priorityQueue>
//void MM<state, action, environment, priorityQueue>::OpenGLDraw(const priorityQueue &queue) const
//{
//	double transparency = 0.9;
//	if (queue.size() == 0)
//		return;
//	uint64_t top = -1;
//	//	double minf = 1e9, maxf = 0;
//	if (queue.OpenSize() > 0)
//	{
//		top = queue.Peek();
//	}
//	for (unsigned int x = 0; x < queue.size(); x++)
//	{
//		const AStarOpenClosedData<state> &data = queue.Lookat(x);
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
//			env->SetColor(1.0, 0.0, 0.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
//	}
//}

template <class state, class action, class environment, class priorityQueue>
std::string MM<state, action, environment, priorityQueue>::SVGDraw() const
{
	std::string s;
	s += SVGDraw(forwardQueue);
	s += SVGDraw(backwardQueue);
	return s;
}

template <class state, class action, class environment, class priorityQueue>
std::string MM<state, action, environment, priorityQueue>::SVGDraw(const priorityQueue &queue) const
{
	std::string s;
	double transparency = 1.0;
	if (queue.size() == 0)
		return s;
	uint64_t top = -1;
	
	if (queue.OpenSize() > 0)
	{
		top = queue.Peek();
	}
	for (unsigned int x = 0; x < queue.size(); x++)
	{
		const auto &data = queue.Lookat(x);
		
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

#endif /* MM_h */
