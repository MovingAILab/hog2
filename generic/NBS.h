//
//	NBS.h
//	This file derived from MM.h by Nathan Sturtevant
//	The following is the original claim
//
//  MM.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/27/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef NBS_H
#define NBS_H

#include "BDOpenClosed.h"
#include "BDIndexOpenClosed.h"
#include "FPUtil.h"
#include <unordered_map>
#include "NBSQueue.h"
//#include "NBSQueueGF.h"
#include "Graphics.h"
//#define EPSILON 1

// ADMISSIBLE is defined at "BDOpenClosed.h"


using std::cout;


template <class state, class action, class environment, class dataStructure = NBSQueue<state, 1>, class priorityQueue = BDOpenClosed<state, NBSCompareOpenReady<state, BDOpenClosedData<state>>, NBSCompareOpenWaiting<state, BDOpenClosedData<state>>>>
class NBS {
public:
	NBS()
	{
		forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount();
	}
	virtual ~NBS() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool ExpandAPair(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	
	
	
	virtual const char *GetName() { return "NBS"; }
	
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
	uint64_t GetDoubleExpansions() const;
	uint64_t GetNecessaryExpansions() const
	{
		uint64_t necessary = 0;
		for (const auto &i : counts)
		{
			if (i.first < currentCost)
				necessary+=i.second;
		}
		return necessary;
	}
	// returns 0...1 for the percentage of the optimal path length on each frontier
	float GetMeetingPoint()
	{
		uint64_t fID, bID;
		queue.backwardQueue.Lookup(env->GetStateHash(middleNode), bID);
		queue.forwardQueue.Lookup(env->GetStateHash(middleNode), fID);
		assert (fequal(queue.backwardQueue.Lookup(bID).g+queue.forwardQueue.Lookup(fID).g, currentCost));
		return queue.backwardQueue.Lookup(bID).g/currentCost;
	}
	double GetSolutionCost() const { return currentCost; }
	
//	void OpenGLDraw() const;
	void Draw(Graphics::Display &d) const;
	void DrawBipartiteGraph(Graphics::Display &d) const;
	
	//	void SetWeight(double w) {weight = w;}
private:
	void ExtractFromMiddle(std::vector<state> &thePath);
	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
	{
		uint64_t theID;
		queue.backwardQueue.Lookup(env->GetStateHash(node), theID);
		ExtractPathToGoalFromID(theID, thePath);
	}
	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.backwardQueue.Lookup(node).data);
			node = queue.backwardQueue.Lookup(node).parentID;
		} while (queue.backwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.backwardQueue.Lookup(node).data);
	}
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{
		uint64_t theID;
		queue.forwardQueue.Lookup(env->GetStateHash(node), theID);
		ExtractPathToStartFromID(theID, thePath);
	}

	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.forwardQueue.Lookup(node).data);
			node = queue.forwardQueue.Lookup(node).parentID;
		} while (queue.forwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.forwardQueue.Lookup(node).data);
	}
	
//	void OpenGLDraw(const priorityQueue &queue) const;
	void Draw(Graphics::Display &d, const priorityQueue &queue) const;
	
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
void NBS<state, action, environment, dataStructure, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
																			Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	
	while (!ExpandAPair(thePath))
	{ }
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool NBS<state, action, environment, dataStructure, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																					 Heuristic<state> *forward, Heuristic<state> *backward,
																					 std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentSolutionEstimate = 0;
	currentCost = DBL_MAX;
	queue.Reset(env->GetMaxHash());
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
bool NBS<state, action, environment, dataStructure, priorityQueue>::ExpandAPair(std::vector<state> &thePath)
{
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
//		if (queue.TerminateOnG())
//			printf("NBS: Lower Bound on C* from g+g (gsum)\n");
		ExtractFromMiddle(thePath);
		return true;
	}
	else if (!fless(queue.GetLowerBound(), currentCost))
	{
		ExtractFromMiddle(thePath);
		return true;
	}
	
	counts[queue.GetLowerBound()]+=2;
	Expand(nForward, queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
	Expand(nBackward, queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
	return false;
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void NBS<state, action, environment, dataStructure, priorityQueue>::ExtractFromMiddle(std::vector<state> &thePath)
{
	std::vector<state> pFor, pBack;
	//	std::cout << "Extracting from " << middleNode << "\n";
	ExtractPathToGoal(middleNode, pBack);
	//	std::cout << "And from: Extracting from " << middleNode << "\n";
	ExtractPathToStart(middleNode, pFor);
	reverse(pFor.begin(), pFor.end());
	thePath = pFor;
	thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool NBS<state, action, environment, dataStructure, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	return ExpandAPair(thePath);
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
void NBS<state, action, environment, dataStructure, priorityQueue>::Expand(uint64_t nextID,
																		   priorityQueue &current,
																		   priorityQueue &opposite,
																		   Heuristic<state> *heuristic, const state &target)
{
	
	//	uint64_t nextID = current.Peek(kOpenReady);
	//
	uint64_t tmp = current.Close();
	assert(tmp == nextID);
	//if (currentCost != DBL_MAX)
	//std::cout << "Expanding " << current.Lookup(nextID).data << "\n";
	
	//this can happen when we expand a single node instead of a pair
	if (fgreatereq(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost))
		return;
	
	nodesExpanded++;
	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		//std::cout << "--Child " << succ << "\n";
		nodesTouched++;
		uint64_t childID;
		auto loc = current.Lookup(env->GetStateHash(succ), childID);
		
		// screening
		double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
		if (fgreatereq(current.Lookup(nextID).g+edgeCost, currentCost))
			continue;
		
		switch (loc)
		{
			case kClosed: // ignore
				//				break;
#ifdef ADMISSIBLE
				if (fless(current.Lookup(nextID).g + edgeCost, current.Lookup(childID).g))
				{
					//					std::cout << "Re-opening node from closed " << current.Lookup(nextID).data << "\n";
					double oldGCost = current.Lookup(childID).g;
					current.Lookup(childID).parentID = nextID;
					current.Lookup(childID).g = current.Lookup(nextID).g + edgeCost;
					current.Reopen(childID);
					
					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							if (currentCost == DBL_MAX)
							{
								//								printf("NBS: first solution %" PRId64 "\n", nodesExpanded);
								//								std::cout << "Through " << succ << " (not here)\n";
							}
							// TODO: store current solution
							//							printf("NBS Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%" PRId64 " nodes)\n",
							//								   current.Lookup(nextID).g+edgeCost,
							//								   opposite.Lookup(reverseLoc).g,
							//								   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g,
							//								   nodesExpanded);
							currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
							
							middleNode = succ;
						}
					}
				}
#endif
				break;
			case kOpenReady: // update cost if needed
			case kOpenWaiting:
			{
				if (fless(current.Lookup(nextID).g+edgeCost, current.Lookup(childID).g))
				{
					current.Lookup(childID).parentID = nextID;
					current.Lookup(childID).g = current.Lookup(nextID).g+edgeCost;
					current.KeyChanged(childID);
					
					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							//							if (currentCost == DBL_MAX)
							//							{
							//								printf("NBS: first solution %" PRId64 "\n", nodesExpanded);
							//								std::cout << "Through " << succ << " (better)\n";
							//							}
							// TODO: store current solution
							//							printf("NBS Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%" PRId64 " nodes)\n",
							//								   current.Lookup(nextID).g+edgeCost,
							//								   opposite.Lookup(reverseLoc).g,
							//								   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g,
							//								nodesExpanded);
							currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
							
							middleNode = succ;
						}
					}
#ifdef ADMISSIBLE
					
#else
					else if (loc == kClosed)
					{
						current.Remove(childID);
					}
#endif // !ADMISSIBLE
				}
			}
				break;
			case kUnseen:
			{
				uint64_t reverseLoc;
				auto locReverse = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
#ifdef ADMISSIBLE
				
#else
				if (locReverse == kClosed)// then
				{
					break;			//do nothing. do not put this node to open
				}
				else//loc == kUnseen or kOpen
#endif // ADMISSIBLE
				{
					//double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
					//if(fless(current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target),currentPr))
					//	current.AddOpenNode(succ,
					//		env->GetStateHash(succ),
					//		current.Lookup(nextID).g + edgeCost,
					//		heuristic->HCost(succ, target),
					//		nextID,0);
					//else
					double newNodeF = current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target);
					if (fless(newNodeF, currentCost))
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
						
						if (locReverse == kOpenReady || locReverse == kOpenWaiting)
						{
							//double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
							if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
							{
								if (currentCost == DBL_MAX)
								{
									//									if (&current == &queue.forwardQueue)
									//										std::cout << "Searching forward\n";
									//									else
									//										std::cout << "Searching backward\n";
									//									printf("NBS: first solution %" PRId64 " ", nodesExpanded);
									//
									//									std::cout << "Through " << succ << " (first) \n";
									//
									//									uint64_t theID;
									//									auto loc = queue.forwardQueue.Lookup(env->GetStateHash(succ), theID);
									//									std::cout << "Forward:\n";
									//									switch (loc)
									//									{
									//										case kOpenReady: std::cout << "Initially in open ready\n"; break;
									//										case kOpenWaiting: std::cout << "Initially in open waiting\n"; break;
									//										case kClosed: std::cout << "Initially in closed\n"; break;
									//										case kUnseen: std::cout << "Initially in UNSEEN\n"; break;
									//									}
									//									loc = queue.backwardQueue.Lookup(env->GetStateHash(succ), theID);
									//									std::cout << "Backward:\n";
									//									switch (loc)
									//									{
									//										case kOpenReady: std::cout << "Initially in open ready\n"; break;
									//										case kOpenWaiting: std::cout << "Initially in open waiting\n"; break;
									//										case kClosed: std::cout << "Initially in closed\n"; break;
									//										case kUnseen: std::cout << "Initially in UNSEEN\n"; break;
									//									}
									
								}
								// TODO: store current solution
								//								printf("NBS Potential solution found, cost: %1.2f + %1.2f = %1.2f (%" PRId64 " nodes)\n",
								//									   current.Lookup(nextID).g + edgeCost,
								//									   opposite.Lookup(reverseLoc).g,
								//									   current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g,
								//									   nodesExpanded);
								currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
								
								middleNode = succ;
								//								std::cout << "One more time, solution passes through " << middleNode << " (first) \n";
								
							}
							
						}
						
					}
					else {
						//std::cout << "***Not adding " << succ << " to open because cost is worse than current of " << currentCost << "\n";
					}
				}
				
			}
				break;
		}
	}
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
uint64_t NBS<state, action, environment, dataStructure, priorityQueue>::GetDoubleExpansions() const
{
	uint64_t doubles = 0;
	for (unsigned int x = 0; x < queue.forwardQueue.size(); x++)
	{
		uint64_t key;
		const auto &data = queue.forwardQueue.Lookat(x);
		if (data.where == kClosed)
		{
			auto loc = queue.backwardQueue.Lookup(env->GetStateHash(data.data), key);
			if (loc == kClosed)
				doubles++;
		}
		
	}
	return doubles;
}

//template <class state, class action, class environment, class dataStructure, class priorityQueue>
//void NBS<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw() const
//{
//	OpenGLDraw(queue.forwardQueue);
//	OpenGLDraw(queue.backwardQueue);
//}
//
//template <class state, class action, class environment, class dataStructure, class priorityQueue>
//void NBS<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw(const priorityQueue &q) const
//{
//	double transparency = 0.9;
//	if (q.size() == 0)
//		return;
//	uint64_t top = -1;
//	//	double minf = 1e9, maxf = 0;
//	if (q.OpenReadySize() > 0)
//	{
//		top = q.Peek(kOpenReady);
//	}
//	for (unsigned int x = 0; x < q.size(); x++)
//	{
//		const auto &data = q.Lookat(x);
//		if (x == top)
//		{
//			env->SetColor(1.0, 1.0, 0.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
//		if (data.where == kOpenWaiting)
//		{
//			env->SetColor(0.0, 0.5, 0.5, transparency);
//			env->OpenGLDraw(data.data);
//		}
//		else if (data.where == kOpenReady)
//		{
//			env->SetColor(0.0, 1.0, 0.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
//		else if (data.where == kClosed)
//		{
//			if (&q == &queue.backwardQueue)
//				env->SetColor(1.0, 0.0, 1.0, transparency);
//			else
//				env->SetColor(1.0, 0.0, 0.0, transparency);
//			env->OpenGLDraw(data.data);
//		}
//	}
//}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void NBS<state, action, environment, dataStructure, priorityQueue>::Draw(Graphics::Display &d) const
{
	Draw(d, queue.forwardQueue);
	Draw(d, queue.backwardQueue);
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void NBS<state, action, environment, dataStructure, priorityQueue>::Draw(Graphics::Display &d, const priorityQueue &q) const
{
	double transparency = 0.9;
	if (q.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (q.OpenReadySize() > 0)
	{
		top = q.Peek(kOpenReady);
	}
	for (unsigned int x = 0; x < q.size(); x++)
	{
		const auto &data = q.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(d, data.data);
		}
		else if (data.where == kOpenWaiting)
		{
			env->SetColor(1.0, 0.50, 0.25, transparency);
			env->Draw(d, data.data);
		}
		else if (data.where == kOpenReady)
		{
			env->SetColor(1.0, 0.75, 0.25, transparency);
			env->Draw(d, data.data);
		}
		else if (data.where == kClosed)
		{
			if (&q == &queue.backwardQueue)
				env->SetColor(0.25, 0.5, 1.0, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			env->Draw(d, data.data);
		}
	}
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void NBS<state, action, environment, dataStructure, priorityQueue>::DrawBipartiteGraph(Graphics::Display &d) const
{
	double val = queue.GetLowerBound();
	//currentCost
	assert(!"Implementaion incomplete");
	//	Draw(d, queue.forwardQueue);
	//	Draw(d, queue.backwardQueue);
}
//void DrawBipartiteGraph(Graphics::Display &d);


#endif /* NBS_h */
