//
//	BOBA.h
//	This file derived from MM.h by Nathan Sturtevant
//	The following is the original claim
//
//  MM.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/27/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef BOBA_H
#define BOBA_H

#include "BDOpenClosed.h"
#include "FPUtil.h"
#include <unordered_map>
#include "BOBAQueue.h"
#include "BOBAQueueGF.h"

#define EPSILON 1

using std::cout;


template <class state, class action, class environment, class dataStructure = BOBAQueue<state>,
          class priorityQueue = BDOpenClosed<state, BOBACompareOpenReady<state>, BOBACompareOpenWaiting<state>>>
class BOBA {
public:
	BOBA()
	{
		forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount();
	}
	virtual ~BOBA() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool ExpandAPair(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	
	
	
	virtual const char *GetName() { return "BOBA"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; counts.clear(); }
	
	inline const int GetNumForwardItems() { return queue.forwardQueue.size(); }
	inline const BDOpenClosedData<state> &GetForwardItem(unsigned int which) { return queue.forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return queue.backwardQueue.size(); }
	inline const BDOpenClosedData<state> &GetBackwardItem(unsigned int which) { return queue.backwardQueue.Lookat(which); }
	
	void SetForwardHeuristic(Heuristic<state> *h) { forwardHeuristic = h; }
	void SetBackwardHeuristic(Heuristic<state> *h) { backwardHeuristic = h; }
	stateLocation GetNodeForwardLocation(const state &s)
	{
		uint64_t childID;
		auto l = queue.forwardQueue.Lookup(env->GetStateHash(s), childID);
		return l;
	}
	stateLocation GetNodeBackwardLocation(const state &s)
	{
		uint64_t childID;
		return queue.backwardQueue.Lookup(env->GetStateHash(s), childID);
	}
	double GetNodeForwardG(const state& s)
	{
		
		uint64_t childID;
		auto l = queue.forwardQueue.Lookup(env->GetStateHash(s), childID);
		if (l != kUnseen)
			return queue.forwardQueue.Lookat(childID).g;
		return -1;
	}
	double GetNodeBackwardG(const state& s)
	{
		
		uint64_t childID;
		auto l = queue.backwardQueue.Lookup(env->GetStateHash(s), childID);
		if (l != kUnseen)
			return queue.backwardQueue.Lookat(childID).g;
		return -1;
	}
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const {
		uint64_t necessary = 0;
		for (const auto &i : counts)
		{
			if (i.first < currentCost)
				necessary+=i.second;
		}
		return necessary;
	}
	double GetSolutionCost() const { return currentCost; }
	
	void OpenGLDraw() const;
	
	//	void SetWeight(double w) {weight = w;}
private:
	void ExtractFromMiddle(std::vector<state> &thePath);
	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.backwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToGoalFromID(theID, thePath); }
	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.backwardQueue.Lookup(node).data);
			node = queue.backwardQueue.Lookup(node).parentID;
		} while (queue.backwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.backwardQueue.Lookup(node).data);
	}
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.forwardQueue.Lookup(node).data);
			node = queue.forwardQueue.Lookup(node).parentID;
		} while (queue.forwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.forwardQueue.Lookup(node).data);
	}
	
	void OpenGLDraw(const priorityQueue &queue) const;
	
	void Expand(uint64_t nextID,
				priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic, const state &target);
	//direction ==0 forward; 1 backward
	//void Expand(int direction);
	uint64_t nodesTouched, nodesExpanded;
	state middleNode;
	double currentCost;
	double currentSolutionEstimate;
	std::vector<state> neighbors;
	environment *env;
	std::unordered_map<double, int> counts;
	
	dataStructure queue;
	//	priorityQueue queue.forwardQueue, queue.backwardQueue;
	//priorityQueue2 queue.forwardQueue, queue.backwardQueue;
	
	state goal, start;
	
	Heuristic<state> *forwardHeuristic;
	Heuristic<state> *backwardHeuristic;
	
	//keep track of whether we expand a node or put it back to open
	bool expand;
	
	double currentPr;
	
	
};

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void BOBA<state, action, environment, dataStructure, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
																			 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	
	while (!ExpandAPair(thePath))
	{ }
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool BOBA<state, action, environment, dataStructure, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																					  Heuristic<state> *forward, Heuristic<state> *backward,
																					  std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentSolutionEstimate = 0;
	currentCost = DBL_MAX;
	queue.Reset();
//	queue.forwardQueue.Reset();
//	queue.backwardQueue.Reset();
	ResetNodeCount();
	thePath.resize(0);
	start = from;
	goal = to;
	if (start == goal)
		return false;
	
	queue.forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
	queue.backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start));
	
	return true;
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool BOBA<state, action, environment, dataStructure, priorityQueue>::ExpandAPair(std::vector<state> &thePath)
{
//	if (queue.forwardQueue.OpenSize() == 0 || queue.backwardQueue.OpenSize() == 0)
//	{
//		//		printf("No more pairs, terminate! (%f)\n", currentCost);
//		if (currentCost != DBL_MAX)
//		{
//			std::vector<state> pFor, pBack;
//			ExtractPathToGoal(middleNode, pBack);
//			ExtractPathToStart(middleNode, pFor);
//			reverse(pFor.begin(), pFor.end());
//			thePath = pFor;
//			thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
//		}
//		return true;
//	}
//	
//	uint64_t nextIDForward;
//	uint64_t nextIDBackward;
//	BDOpenClosedData<state> iFReady, iBReady, iFWaiting, iBWaiting;
//	
//	if (queue.forwardQueue.OpenReadySize() == 0)
//		queue.forwardQueue.PutToReady();
//	if (queue.backwardQueue.OpenReadySize() == 0)
//		queue.backwardQueue.PutToReady();
//	
//	if (queue.forwardQueue.OpenWaitingSize() > 0)
//	{
//		iFWaiting = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenWaiting));
//		while (iFWaiting.g+iFWaiting.h < currentSolutionEstimate)
//		{
//			queue.forwardQueue.PutToReady();
//			if (queue.forwardQueue.OpenWaitingSize()  == 0)
//				break;
//			iFWaiting = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenWaiting));
//		}
//	}
//	if (queue.backwardQueue.OpenWaitingSize() > 0)
//	{
//		iBWaiting = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenWaiting));
//		while (iBWaiting.g+iBWaiting.h < currentSolutionEstimate)
//		{
//			queue.backwardQueue.PutToReady();
//			if (queue.backwardQueue.OpenWaitingSize()  == 0)
//				break;
//			iBWaiting = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenWaiting));
//		}
//	}
//	
//	iFReady = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenReady));
//	iBReady = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenReady));
//	
//	if (queue.forwardQueue.OpenWaitingSize() != 0 || queue.backwardQueue.OpenWaitingSize() != 0)
//	{
//		double fBound, gBound;
//		
//		gBound = iFReady.g + iBReady.g + EPSILON;
//		if (queue.forwardQueue.OpenWaitingSize() == 0)
//		{
//			iBWaiting = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenWaiting));
//			fBound = iBWaiting.g + iBWaiting.h;
//		}
//		else if (queue.backwardQueue.OpenWaitingSize() == 0)
//		{
//			iFWaiting = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenWaiting));
//			fBound = iFWaiting.g + iFWaiting.h;
//		}
//		else//queue.forwardQueue.OpenWaitingSize() > 0 && queue.backwardQueue.OpenWaitingSize() > 0
//		{
//			iFWaiting = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenWaiting));
//			iBWaiting = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenWaiting));
//			fBound = std::min(iFWaiting.g + iFWaiting.h, iBWaiting.g + iBWaiting.h);
//		}
//		
//		while (fgreater(gBound, fBound))
//		{
//			if (queue.forwardQueue.OpenWaitingSize() == 0)
//				queue.backwardQueue.PutToReady();
//			else if (queue.backwardQueue.OpenWaitingSize() == 0)
//				queue.forwardQueue.PutToReady();
//			else
//			{
//				nextIDForward = queue.forwardQueue.Peek(kOpenWaiting);
//				nextIDBackward = queue.backwardQueue.Peek(kOpenWaiting);
//				auto iF = queue.forwardQueue.Lookat(nextIDForward);
//				auto iB = queue.backwardQueue.Lookat(nextIDBackward);
//				if (fless(iF.g + iF.h, iB.g + iB.h))
//					queue.forwardQueue.PutToReady();
//				else
//					queue.backwardQueue.PutToReady();
//			}
//			
//			if (queue.forwardQueue.OpenWaitingSize() == 0 && queue.backwardQueue.OpenWaitingSize() == 0)
//				break;
//			else if (queue.forwardQueue.OpenWaitingSize() == 0)
//			{
//				iBWaiting = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenWaiting));
//				
//				fBound = iBWaiting.g + iBWaiting.h;
//			}
//			else if (queue.backwardQueue.OpenWaitingSize() == 0)
//			{
//				iFWaiting = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenWaiting));
//				fBound = iFWaiting.g + iFWaiting.h;
//			}
//			else //queue.forwardQueue.OpenWaitingSize() > 0 && queue.backwardQueue.OpenWaitingSize() > 0
//			{
//				iFWaiting = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenWaiting));
//				iBWaiting = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenWaiting));
//				
//				fBound = std::min(iFWaiting.g + iFWaiting.h, iBWaiting.g + iBWaiting.h);
//				//TODO epsilon
//			}
//			iFReady = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenReady));
//			iBReady = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenReady));
//			gBound = iFReady.g + iBReady.g + EPSILON;
//		}
//		currentSolutionEstimate = fBound;
//		iFReady = queue.forwardQueue.Lookat(queue.forwardQueue.Peek(kOpenReady));
//		iBReady = queue.backwardQueue.Lookat(queue.backwardQueue.Peek(kOpenReady));
//	}
//	
//	
//	if (iFReady.data == iBReady.data) // terminate - fronts equal
//	{
//		if (currentCost != DBL_MAX)
//		{
//			std::vector<state> pFor, pBack;
//			ExtractPathToGoal(middleNode, pBack);
//			ExtractPathToStart(middleNode, pFor);
//			reverse(pFor.begin(), pFor.end());
//			thePath = pFor;
//			thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
//		}
//		return true;
//	}
//	double minPr = std::max(iFReady.g + iFReady.h, iBReady.g + iBReady.h);
//	minPr = std::max(minPr, iFReady.g + iBReady.g + EPSILON);
//	if (!fless(minPr, currentCost)) // terminate - priority >= incumbant solution
//	{
//		if (currentCost != DBL_MAX)
//		{
//			std::vector<state> pFor, pBack;
//			ExtractPathToGoal(middleNode, pBack);
//			ExtractPathToStart(middleNode, pFor);
//			reverse(pFor.begin(), pFor.end());
//			thePath = pFor;
//			thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
//		}
//		return true;
//	}
//	counts[minPr]+=2; // 2 expansions
	//printf("Expanding F_f = %f; F_b = %f; g+g+epsilon=%f\n", iFReady.g+iFReady.h, iBReady.g + iBReady.h, iFReady.g + iBReady.g + EPSILON);
	uint64_t nForward, nBackward;
	bool result = queue.GetNextPair(nForward, nBackward);
	// if failed, see if we have optimal path (but return)
	if (result == false)
	{
		if (currentCost == DBL_MAX)
		{
			thePath.resize(0);
			return true;
		}
		ExtractFromMiddle(thePath);
		return true;
	}
	else if (queue.forwardQueue.Lookup(nForward).data == queue.backwardQueue.Lookup(nBackward).data) // if success, see if nodes are the same (return path)
	{
		ExtractFromMiddle(thePath);
		return true;
	}
	else if (!fless(queue.GetLowerBound(), currentCost))
	{
		ExtractFromMiddle(thePath);
		return true;
	}
	// else expand both
	//	Expand(nForward, kForward);
	//	Expand(nForward, kBackward);
	
	counts[queue.GetLowerBound()]+=2;
	Expand(nForward, queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
	Expand(nBackward, queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
	return false;
	
	
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void BOBA<state, action, environment, dataStructure, priorityQueue>::ExtractFromMiddle(std::vector<state> &thePath)
{
	std::vector<state> pFor, pBack;
	ExtractPathToGoal(middleNode, pBack);
	ExtractPathToStart(middleNode, pFor);
	reverse(pFor.begin(), pFor.end());
	thePath = pFor;
	thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool BOBA<state, action, environment, dataStructure, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	return ExpandAPair(thePath);
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
void BOBA<state, action, environment, dataStructure, priorityQueue>::Expand(uint64_t nextID,
																			priorityQueue &current,
																			priorityQueue &opposite,
																			Heuristic<state> *heuristic, const state &target)
{
	
	//	uint64_t nextID = current.Peek(kOpenReady);
	//
	uint64_t tmp = current.Close();
	assert(tmp == nextID);
	
	//this can happen when we expand a single node instead of a pair
	if (!fless(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost))
		return;
	
	nodesExpanded++;
	//	if (current.Lookup(nextID).data.x == 201 &&
	//		(current.Lookup(nextID).data.y == 135 || current.Lookup(nextID).data.y == 136 || current.Lookup(nextID).data.y == 134))
	//	{
	//		printf("Expanding (%d, %d)\n", current.Lookup(nextID).data.x, current.Lookup(nextID).data.y);
	//	}
	//	else if (&current == &queue.backwardQueue){
	//		printf("Next f: %f g: %f\n", current.Lookup(nextID).g+current.Lookup(nextID).h, current.Lookup(nextID).g);
	//	}
	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		auto loc = current.Lookup(env->GetStateHash(succ), childID);
		switch (loc)
		{
			case kClosed: // ignore
				break;
			case kOpenReady: // update cost if needed
			case kOpenWaiting:
			{
				double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
				if (fless(current.Lookup(nextID).g+edgeCost, current.Lookup(childID).g))
				{
					double oldGCost = current.Lookup(childID).g;
					current.Lookup(childID).parentID = nextID;
					current.Lookup(childID).g = current.Lookup(nextID).g+edgeCost;
					current.KeyChanged(childID);
					
					//					if (current.Lookup(childID).data.x == 201 &&
					//						current.Lookup(childID).data.y == 136)
					//					{
					//						printf("(201, 136) updated from %f to %f (%f)\n", oldGCost, current.Lookup(childID).g, current.Lookup(childID).g+current.Lookup(childID).h);
					//					}
					//					if (current.Lookup(childID).data.x == 201 &&
					//						current.Lookup(childID).data.y == 135)
					//					{
					//						printf("(201, 135) updated from %f to %f (%f)\n", oldGCost, current.Lookup(childID).g, current.Lookup(childID).g+current.Lookup(childID).h);
					//					}
					//					if (current.Lookup(childID).data.x == 201 &&
					//						current.Lookup(childID).data.y == 134)
					//					{
					//						printf("(201, 134) updated from %f to %f (%f)\n", oldGCost, current.Lookup(childID).g, current.Lookup(childID).g+current.Lookup(childID).h);
					//					}
					
					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							// TODO: store current solution
//							printf("BOBA Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
//								   current.Lookup(nextID).g+edgeCost,
//								   opposite.Lookup(reverseLoc).g,
//								   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g,
//								nodesExpanded);
							currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
							
							middleNode = succ;
						}
					}
					else if (loc == kClosed)
					{
						//current.Lookup(childID).h = opposite.Lookup(reverseLoc).g;
						//current.Lookup(childID).g = 100000;
						//current.KeyChanged(childID);
						current.Remove(childID);
					}
				}
			}
				break;
			case kUnseen:
			{
				uint64_t reverseLoc;
				auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
				if (loc == kClosed)// then
				{
					break;			//do nothing. do not put this node to open
				}
				else//loc == kUnseen
				{
					double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
					//if(fless(current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target),currentPr))
					//	current.AddOpenNode(succ,
					//		env->GetStateHash(succ),
					//		current.Lookup(nextID).g + edgeCost,
					//		heuristic->HCost(succ, target),
					//		nextID,0);
					//else
					double newNodeF = current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target);
					if (fless(newNodeF , currentCost))
					{
						if (fless(newNodeF, queue.GetLowerBound()))
							current.AddOpenNode(succ,
												env->GetStateHash(succ),
												current.Lookup(nextID).g + edgeCost,
												heuristic->HCost(succ, target),
												nextID, kOpenReady);
						else
							current.AddOpenNode(succ,
												env->GetStateHash(succ),
												current.Lookup(nextID).g + edgeCost,
												heuristic->HCost(succ, target),
												nextID, kOpenWaiting);
						//						if (succ.x == 201 &&
						//							succ.y == 136)
						//						{
						//							printf("(201, 136) added with %f + %f = %f\n",
						//								   current.Lookup(nextID).g + edgeCost,
						//								   heuristic->HCost(succ, target),
						//								   current.Lookup(nextID).g + edgeCost+heuristic->HCost(succ, target));
						//						}
						//						if (succ.x == 201 &&
						//							succ.y == 135)
						//						{
						//							printf("(201, 135) added with %f + %f = %f\n",
						//								   current.Lookup(nextID).g + edgeCost,
						//								   heuristic->HCost(succ, target),
						//								   current.Lookup(nextID).g + edgeCost+heuristic->HCost(succ, target));
						//						}
						//						if (succ.x == 201 &&
						//							succ.y == 134)
						//						{
						//							printf("(201, 134) added with %f + %f = %f\n",
						//								   current.Lookup(nextID).g + edgeCost,
						//								   heuristic->HCost(succ, target),
						//								   current.Lookup(nextID).g + edgeCost+heuristic->HCost(succ, target));
						//						}
					}
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
						if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							// TODO: store current solution
//							printf("BOBA Potential solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
//								current.Lookup(nextID).g + edgeCost,
//								opposite.Lookup(reverseLoc).g,
//								current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g,
//								nodesExpanded);
							currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
							
							middleNode = succ;
						}
						
					}
				}
				
			}
				break;
		}
	}
}



template <class state, class action, class environment, class dataStructure, class priorityQueue>
void BOBA<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw() const
{
	OpenGLDraw(queue.forwardQueue);
	OpenGLDraw(queue.backwardQueue);
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void BOBA<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw(const priorityQueue &queue) const
{
	double transparency = 0.9;
	if (queue.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (queue.OpenReadySize() > 0)
	{
		top = queue.Peek(kOpenReady);
	}
	for (unsigned int x = 0; x < queue.size(); x++)
	{
		const auto &data = queue.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		if (data.where == kOpenWaiting)
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kOpenReady)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kClosed)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
	}
}

#endif /* BOBA_h */
