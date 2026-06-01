//
//  FFBDS.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/20/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef FFBDS_h
#define FFBDS_h

#include "AStarOpenClosed.h"

template <class state>
class FFBDSData {
public:
	FFBDSData() {}
	FFBDSData(double gCost, const state &parent, dataLocation location)
	:g(gCost), parent(parent), where(location) { }
	double g;
	state parent;
	dataLocation where;
};


template <class state, class action, class environment>
class FFBDS {
public:
	FFBDS()
	{
		heuristic = 0; env = 0; ResetNodeCount();
	}
	virtual ~FFBDS() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *h, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *h, std::vector<state> &thePath);
	bool ExpandAPair(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	
	bool GetNextPair(state &nForward, state &nBackward);
	
	virtual const char *GetName() { return "FFBDS"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	
//	inline const int GetNumForwardItems() { return queue.forwardQueue.size(); }
//	inline const BDOpenClosedData<state> &GetForwardItem(unsigned int which) { return queue.forwardQueue.Lookat(which); }
//	inline const int GetNumBackwardItems() { return queue.backwardQueue.size(); }
//	inline const BDOpenClosedData<state> &GetBackwardItem(unsigned int which) { return queue.backwardQueue.Lookat(which); }
	
//	void SetForwardHeuristic(Heuristic<state> *h) { forwardHeuristic = h; }
//	void SetBackwardHeuristic(Heuristic<state> *h) { backwardHeuristic = h; }

	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetDoubleExpansions() const;
	double GetSolutionCost() const { return currentCost; }
	
	void OpenGLDraw() const;
	
	//	void SetWeight(double w) {weight = w;}
private:
	
	typedef std::unordered_map<state, FFBDSData<state>> priorityQueue;
	priorityQueue forwardQueue;
	priorityQueue backwardQueue;

	
//	void ExtractFromMiddle(std::vector<state> &thePath);
//	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
//	{ uint64_t theID; queue.backwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToGoalFromID(theID, thePath); }
//	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
//	{
//		do {
//			thePath.push_back(queue.backwardQueue.Lookup(node).data);
//			node = queue.backwardQueue.Lookup(node).parentID;
//		} while (queue.backwardQueue.Lookup(node).parentID != node);
//		thePath.push_back(queue.backwardQueue.Lookup(node).data);
//	}
//	
//	void ExtractPathToStart(state &node, std::vector<state> &thePath)
//	{ uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
//	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
//	{
//		do {
//			thePath.push_back(queue.forwardQueue.Lookup(node).data);
//			node = queue.forwardQueue.Lookup(node).parentID;
//		} while (queue.forwardQueue.Lookup(node).parentID != node);
//		thePath.push_back(queue.forwardQueue.Lookup(node).data);
//	}
	
	void OpenGLDraw(const priorityQueue &queue) const;
	
	void Expand(state &next,
				priorityQueue &current,
				priorityQueue &opposite);

	//direction ==0 forward; 1 backward
	//void Expand(int direction);
	uint64_t nodesTouched, nodesExpanded;
	state middleNode;
	double currentCost;
	double currentSolutionEstimate;
	std::vector<state> neighbors;
	environment *env;
	
	//dataStructure queue;
	//	priorityQueue queue.forwardQueue, queue.backwardQueue;
	//priorityQueue2 queue.forwardQueue, queue.backwardQueue;
	
	state goal, start;
	
	Heuristic<state> *heuristic;
	
	//keep track of whether we expand a node or put it back to open
	bool expand;
	
	double currentPr;
	
	
};

template <class state, class action, class environment>
void FFBDS<state, action, environment>::GetPath(environment *env, const state& from, const state& to,
																			Heuristic<state> *h, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, h, thePath) == false)
		return;
	
	while (!ExpandAPair(thePath))
	{ }
}

template <class state, class action, class environment>
bool FFBDS<state, action, environment>::InitializeSearch(environment *env, const state& from, const state& to, Heuristic<state> *h, std::vector<state> &thePath)
{
	this->env = env;
	heuristic = h;
	currentSolutionEstimate = 0;
	currentCost = DBL_MAX;
	forwardQueue.clear();
	backwardQueue.clear();
	//queue.Reset();
	//	queue.forwardQueue.Reset();
	//	queue.backwardQueue.Reset();
	ResetNodeCount();
	thePath.resize(0);
	start = from;
	goal = to;
	if (start == goal)
		return false;
	
	forwardQueue[start] = FFBDSData<state>(0, start, kOpenList);
	backwardQueue[goal] = FFBDSData<state>(0, goal, kOpenList);
	
	return true;
}

template <class state, class action, class environment>
bool FFBDS<state, action, environment>::ExpandAPair(std::vector<state> &thePath)
{
	state nForward, nBackward;
	bool result = GetNextPair(nForward, nBackward);
	// if failed, see if we have optimal path (but return)
	if (result == false)
	{
//		if (currentCost == DBL_MAX)
//		{
//			thePath.resize(0);
//			return true;
//		}
//		ExtractFromMiddle(thePath);
		return true;
	}
//	else if (queue.forwardQueue.Lookup(nForward).data == queue.backwardQueue.Lookup(nBackward).data) // if success, see if nodes are the same (return path)
//	{
//		ExtractFromMiddle(thePath);
//		return true;
//	}
//	else if (!fless(queue.GetLowerBound(), currentCost))
//	{
//		ExtractFromMiddle(thePath);
//		return true;
//	}
	
//	counts[queue.GetLowerBound()]+=2;
	Expand(nForward, forwardQueue, backwardQueue);
	Expand(nBackward, backwardQueue, forwardQueue);
	return false;
}

//template <class state, class action, class environment>
//void FFBDS<state, action, environment>::ExtractFromMiddle(std::vector<state> &thePath)
//{
//	std::vector<state> pFor, pBack;
//	ExtractPathToGoal(middleNode, pBack);
//	ExtractPathToStart(middleNode, pFor);
//	reverse(pFor.begin(), pFor.end());
//	thePath = pFor;
//	thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
//}


template <class state, class action, class environment>
bool FFBDS<state, action, environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	return ExpandAPair(thePath);
}


template <class state, class action, class environment>
void FFBDS<state, action, environment>::Expand(state &next,
											   priorityQueue &current,
											   priorityQueue &opposite)
{
	current[next].where = kClosedList;
	nodesExpanded++;
	env->GetSuccessors(next, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		auto locData = current.find(succ);
		dataLocation loc;
		if (locData == current.end())
			loc = kNotFound;
		else
			loc = current[succ].where;
		
		double edgeCost = env->GCost(next, succ);
		// screening
		if (fgreatereq(current[next].g+edgeCost, currentCost))
			continue;
		
		switch (loc)
		{
			case kClosedList: // ignore
				break;
			case kOpenList: // update cost if needed
			{
				if (fless(current[next].g+edgeCost, current[succ].g))
				{
					//double oldGCost = current[succ].g;
					current[succ].parent = succ;
					current[succ].g = current[next].g+edgeCost;
					
					// TODO: check if we improved the current solution?
					auto oppData = opposite.find(succ);
					if (oppData != opposite.end() && oppData->second.where == kOpenList)
					{
						if (fless(current[succ].g + opposite[succ].g, currentCost))
						{
							// TODO: store current solution
							printf("FFBDS Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
								   current[succ].g,
								   opposite[succ].g,
								   current[succ].g+opposite[succ].g,
								   nodesExpanded);
							currentCost = current[succ].g + opposite[succ].g;
							middleNode = succ;
						}
					}
					else if (loc == kClosedList)
					{
//						opp.erase(succ
//						current.Remove(childID);
					}
				}
			}
				break;
			case kNotFound:
			{
				auto oppData = opposite.find(succ);
				if (oppData != opposite.end() && oppData->second.where == kClosedList)
				{
					break;			//do nothing. do not put this node to open
				}
				else {
					current[succ] = FFBDSData<state>(current[next].g+edgeCost, next, kOpenList);
					if (oppData != opposite.end() && oppData->second.where == kOpenList)
					{
						if (fless(current[succ].g + opposite[succ].g, currentCost))
						{
							// TODO: store current solution
							printf("FFBDS Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
								   current[succ].g,
								   opposite[succ].g,
								   current[succ].g+opposite[succ].g,
								   nodesExpanded);
							currentCost = current[succ].g + opposite[succ].g;
							middleNode = succ;
						}
						
					}
				}
				
			}
				break;
		}
	}
}

template <class state, class action, class environment>
bool FFBDS<state, action, environment>::GetNextPair(state &nForward, state &nBackward)
{
	double bestCost = DBL_MAX;
	for (auto i : forwardQueue)
	{
		if (i.second.where != kOpenList)
			continue;
		for (auto j : backwardQueue)
		{
			if (j.second.where != kOpenList)
				continue;
			double val = i.second.g + j.second.g + heuristic->HCost(i.first, j.first);
			if (fless(val, bestCost))
			{
				bestCost = val;
				nForward = i.first;
				nBackward = j.first;
			}
		}
	}
	if (fgreatereq(bestCost, currentCost))
	{
		printf("Done with cost %1.2f\n", currentCost);
		return false;
	}
	//std::cout << nForward << " and " << nBackward << " cost: " << bestCost << "\n";
	return true;
}


template <class state, class action, class environment>
uint64_t FFBDS<state, action, environment>::GetDoubleExpansions() const
{
//	uint64_t doubles = 0;
//	for (unsigned int x = 0; x < queue.forwardQueue.size(); x++)
//	{
//		uint64_t key;
//		const auto &data = queue.forwardQueue.Lookat(x);
//		if (data.where == kClosed)
//		{
//			auto loc = queue.backwardQueue.Lookup(env->GetStateHash(data.data), key);
//			if (loc == kClosed)
//				doubles++;
//		}
//		
//	}
//	return doubles;
	return 0;
}

template <class state, class action, class environment>
void FFBDS<state, action, environment>::OpenGLDraw() const
{
	OpenGLDraw(forwardQueue);
	OpenGLDraw(backwardQueue);
}

template <class state, class action, class environment>
void FFBDS<state, action, environment>::OpenGLDraw(const priorityQueue &queue) const
{
	double transparency = 0.9;
	if (queue.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	for (auto &d : queue)
	{
		if (d.second.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(d.first);
		}
		else if (d.second.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(d.first);
		}
	}
}


#endif /* FFBDS_h */
