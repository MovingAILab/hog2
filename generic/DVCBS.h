//
//  DVCBS.h
//  hog2 mac native demos
//
//  Created by Shahaf S. Shperberg
//

#ifndef DVCBS_h
#define DVCBS_h

#include "DVCBSOpenClosed.h"
#include "FPUtil.h"
#include <unordered_map>
#include "DVCBSQueue.h"
//#include "NBSQueueGF.h"
#include <algorithm>

using std::cout;


template <class state, class action, class environment, class dataStructure = DVCBSQueue<state>,
class priorityQueue = DVCBSOpenClosed<state> >
class DVCBS {
public:
	DVCBS(int tieBreaking, bool allSolutions = false, bool leq = false)
	{
		forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount();
		tieBreakingPolicy = tieBreaking;
		isAllSolutions = allSolutions;
		isLEQ = leq;
	}
	DVCBS(bool allSolutions = false, bool leq = false)
	{
		forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount();
		tieBreakingPolicy = 4;
		isAllSolutions = allSolutions;
		isLEQ = leq;
	}
	virtual ~DVCBS() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool ExpandAVertexCover(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	uint64_t getForwardUnnecessaryNodesInPath(){
		return forwardUnnecessaryNodesInPath;
	}
	
	uint64_t getBackwardUnnecessaryNodesInPath(){
		return backwardUnnecessaryNodesInPath;
	}
	
	uint64_t getForwardMeetingPoint(){
		return forwardMeetingPoint;
	}
	
	uint64_t getBackwardMeetingPoint(){
		return backwardMeetingPoint;
	}
	
	uint64_t getOptimalNumberOfExpantions(){
		printf("curr Cost: %d\n",currentCost);
		return queue.getMinimalVertexCover(currentCost);
	}
	
	
	virtual const char *GetName() { return "DVCBS"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; counts.clear(); }
	
	inline const int GetNumForwardItems() { return queue.forwardQueue.size(); }
	inline const DVCBSOpenClosedData<state> &GetForwardItem(unsigned int which) { return queue.forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return queue.backwardQueue.size(); }
	inline const DVCBSOpenClosedData<state> &GetBackwardItem(unsigned int which) { return queue.backwardQueue.Lookat(which); }
	
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
	uint64_t GetNecessaryExpansions() const {
		uint64_t necessary = 0;
		for (const auto &i : counts)
		{
			if (isAllSolutions){
				if (i.first <= currentCost)
					necessary+=i.second;
			}
			else{
				if (i.first < currentCost)
					necessary+=i.second;
			}
		}
		return necessary;
	}
	double GetSolutionCost() const { return currentCost; }
	double GetExpansionUntilFirstSolution() const { return expansionsUntilSolution; }
	
	
//	void OpenGLDraw() const;
	
	//	void SetWeight(double w) {weight = w;}
private:
	void ExtractFromMiddle(std::vector<state> &thePath);
	double ExtractCostFromMiddle();
	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.backwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToGoalFromID(theID, thePath); }
	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.backwardQueue.Lookup(node).data);
			backwardMeetingPoint++;
			if (queue.backwardQueue.Lookup(node).g+queue.backwardQueue.Lookup(node).h == currentCost){
				backwardUnnecessaryNodesInPath++;
			}
			node = queue.backwardQueue.Lookup(node).parentID;
		} while (queue.backwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.backwardQueue.Lookup(node).data);
		
	}
	
	double ExtractCostToGoal(state &node)
	{ uint64_t theID; queue.backwardQueue.Lookup(env->GetStateHash(node), theID); return ExtractCostToGoalFromID(theID); }
	double ExtractCostToGoalFromID(uint64_t node)
	{
		double cost = 0;
		do {
			cost += queue.backwardQueue.Lookup(node).g;
			node = queue.backwardQueue.Lookup(node).parentID;
		} while (queue.backwardQueue.Lookup(node).parentID != node);
		cost += queue.backwardQueue.Lookup(node).g;
		return cost;
	}
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.forwardQueue.Lookup(node).data);
			forwardMeetingPoint++;
			if (queue.forwardQueue.Lookup(node).g+queue.forwardQueue.Lookup(node).h == currentCost){
				forwardUnnecessaryNodesInPath++;
			}
			node = queue.forwardQueue.Lookup(node).parentID;
		} while (queue.forwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.forwardQueue.Lookup(node).data);
	}
	
	double  ExtractCostToStart(state &node)
	{ uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(node), theID); return ExtractCostToStartFromID(theID); }
	double ExtractCostToStartFromID(uint64_t node)
	{
		double cost = 0;
		do {
			cost += queue.forwardQueue.Lookup(node).g;
			printf("cost: %d ",cost);
			node = queue.forwardQueue.Lookup(node).parentID;
		} while (queue.forwardQueue.Lookup(node).parentID != node);
		cost += queue.forwardQueue.Lookup(node).g;
		return cost;
	}
	
//	void OpenGLDraw(const priorityQueue &queue) const;
	
	void Expand(uint64_t nextID,
				priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic, const state &target);
	uint64_t nodesTouched, nodesExpanded;
	state middleNode;
	double currentCost;
	double expansionsUntilSolution;
	double currentSolutionEstimate;
	std::vector<state> neighbors;
	environment *env;
	std::unordered_map<double, int> counts;
	bool isAllSolutions;
	bool isLEQ;
	
	dataStructure queue;
	
	state goal, start;
	
	Heuristic<state> *forwardHeuristic;
	Heuristic<state> *backwardHeuristic;
	
	bool expand;
	
	double currentPr;
	
	int tieBreakingPolicy;
	
	uint64_t forwardUnnecessaryNodesInPath;
	uint64_t backwardUnnecessaryNodesInPath;
	uint64_t forwardMeetingPoint;
	uint64_t backwardMeetingPoint;
	
};

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void DVCBS<state, action, environment, dataStructure, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
																			  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	
	while (!ExpandAVertexCover(thePath))
	{ }
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool DVCBS<state, action, environment, dataStructure, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																					   Heuristic<state> *forward, Heuristic<state> *backward,
																					   std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentSolutionEstimate = 0;
	forwardUnnecessaryNodesInPath = 0;
	backwardUnnecessaryNodesInPath = 0;
	forwardMeetingPoint = 0;
	backwardMeetingPoint = 0;
	currentCost = DBL_MAX;
	expansionsUntilSolution = 0;
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
bool DVCBS<state, action, environment, dataStructure, priorityQueue>::ExpandAVertexCover(std::vector<state> &thePath)
{
	std::vector<uint64_t> nForward, nBackward;
	bool result = queue.getVertexCover(nForward, nBackward,tieBreakingPolicy);
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
	
	
	if ((!isAllSolutions && !fless(queue.GetLowerBound(), currentCost)) || (isAllSolutions && !flesseq(queue.GetLowerBound(), currentCost))){
		ExtractFromMiddle(thePath);
		return true;
	}
	
	else if (nForward.size() > 0 &&
			 nBackward.size()> 0)
	{
		std::unordered_map<state*,bool> mapData;
		for (int i =0; i< nForward.size(); i++){
			mapData[&(queue.forwardQueue.Lookup(nForward[i]).data)] = true;
		}
		for (int j =0; j< nBackward.size(); j++){
			if (mapData.find(&(queue.backwardQueue.Lookup(nBackward[j]).data)) != mapData.end()){
				ExtractFromMiddle(thePath);
				return true;
			}
		}
		
	}
	struct compareBackward {
		compareBackward(dataStructure currQueue) : queue(currQueue) {}
		bool operator () (uint64_t i, uint64_t j) { return (queue.backwardQueue.Lookup(i).h<queue.backwardQueue.Lookup(j).h); }
		dataStructure queue;
	};
	struct compareForward {
		compareForward(dataStructure currQueue) : queue(currQueue) {}
		bool operator () (uint64_t i, uint64_t j) { return (queue.forwardQueue.Lookup(i).h<queue.forwardQueue.Lookup(j).h); }
		dataStructure queue;
	};
	double currentLowerBound = queue.GetLowerBound();
	
	if (nForward.size() == 0){
		for (int j =0; j< ((int)nBackward.size());j++){
			double oldKey = queue.backwardQueue.getFirstKey(kOpenReady);
			if (queue.backwardQueue.Lookup(nBackward[j]).where != kClosed){
				counts[currentLowerBound]++;
				Expand(nBackward[j], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
			}
			if ((!isAllSolutions && !fless(queue.GetLowerBound(), currentCost)) || (isAllSolutions && !flesseq(queue.GetLowerBound(), currentCost))){
				ExtractFromMiddle(thePath);
				return true;
			}
			if (currentLowerBound != queue.GetLowerBound() || oldKey != queue.backwardQueue.getFirstKey(kOpenReady)){
				return false;
			}
		}
	}
	
	else if (nBackward.size() == 0){
		for (int i =0; i< ((int)nForward.size());i++){
			double oldKey = queue.forwardQueue.getFirstKey(kOpenReady);
			if (queue.forwardQueue.Lookup(nForward[i]).where != kClosed){
				counts[currentLowerBound]++;
				Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
			}
			if ((!isAllSolutions && !fless(queue.GetLowerBound(), currentCost)) || (isAllSolutions && !flesseq(queue.GetLowerBound(), currentCost))){
				ExtractFromMiddle(thePath);
				return true;
			}
			if (currentLowerBound != queue.GetLowerBound() || oldKey != queue.forwardQueue.getFirstKey(kOpenReady)){
				return false;
			}
		}
	}
	else{
		int i = nForward.size()-1;
		int j = nBackward.size()-1;
		while (i >= 0 || j >=0 ){
			if ((!isAllSolutions && !fless(queue.GetLowerBound(), currentCost)) || (isAllSolutions && !flesseq(queue.GetLowerBound(), currentCost)))
			{
				ExtractFromMiddle(thePath);
				return true;
			}
			bool expandForward;
			if (i < 0){
				expandForward = false;
			}
			else if (j < 0){
				expandForward = true;
			}
			else {
				if (queue.forwardQueue.Lookup(nForward[i]).g >= queue.backwardQueue.Lookup(nBackward[j]).g){
					expandForward = true;
				}
				else{
					expandForward = false;
				}
			}
			if (expandForward){
				if (queue.forwardQueue.Lookup(nForward[i]).where != kClosed){
					counts[currentLowerBound]++;
					Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
				}
				i--;
			}
			else{
				if (queue.backwardQueue.Lookup(nBackward[j]).where != kClosed){
					counts[currentLowerBound]++;
					Expand(nBackward[j], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
				}
				j--;
			}
			if (currentLowerBound != queue.GetLowerBound()){
				return false;
			}
		}
	}
	return false;
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
double DVCBS<state, action, environment, dataStructure, priorityQueue>::ExtractCostFromMiddle()
{
	double cost = 0;
	printf("cost1: %d",cost);
	cost += ExtractCostToGoal(middleNode);
	printf("cost2: %d",cost);
	cost += ExtractCostToStart(middleNode);
	printf("cost3: %d",cost);
	return cost;
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void DVCBS<state, action, environment, dataStructure, priorityQueue>::ExtractFromMiddle(std::vector<state> &thePath)
{
	
	std::vector<state> pFor, pBack;
	ExtractPathToGoal(middleNode, pBack);
	ExtractPathToStart(middleNode, pFor);
	reverse(pFor.begin(), pFor.end());
	thePath = pFor;
	thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool DVCBS<state, action, environment, dataStructure, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	return ExpandAVertexCover(thePath);
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
void DVCBS<state, action, environment, dataStructure, priorityQueue>::Expand(uint64_t nextID,
																			 priorityQueue &current,
																			 priorityQueue &opposite,
																			 Heuristic<state> *heuristic, const state &target)
{
	if (current.Lookup(nextID).where == kClosed){
		return;
	}
	
	uint64_t tmp = current.CloseAtIndex(nextID);
	assert(tmp == nextID);
	
	//this can happen when we expand a single node instead of a pair
	if ( (!isAllSolutions && fgreatereq(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost)) || (isAllSolutions && fgreater(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost))){
		return;
	}
	
	
	nodesExpanded++;
	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		auto loc = current.Lookup(env->GetStateHash(succ), childID);
		
		// screening
		double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
		if ( (!isAllSolutions && fgreatereq(current.Lookup(nextID).g+edgeCost, currentCost)) || (isAllSolutions && fgreater(current.Lookup(nextID).g+edgeCost, currentCost))){
			continue;
		}
		
		switch (loc)
		{
			case kClosed: // ignore
				break;
			case kOpenReady: // update cost if needed
			case kOpenWaiting:
			{
				if (fless(current.Lookup(nextID).g+edgeCost, current.Lookup(childID).g))
				{
					double oldGCost = current.Lookup(childID).g;
					current.Lookup(childID).parentID = nextID;
					current.Lookup(childID).g = current.Lookup(nextID).g+edgeCost;
					if (loc == kOpenWaiting){
						current.KeyChanged(childID,oldGCost+current.Lookup(childID).h);
					}
					else{
						current.KeyChanged(childID,oldGCost);
					}
					
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
							expansionsUntilSolution = nodesExpanded;
							middleNode = succ;
						}
					}
					else if (loc == kClosed)
					{
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
					break;
				}
				else//loc == kUnseen
				{
					double newNodeF = current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target);
					if ((!isAllSolutions && fless(newNodeF , currentCost)) || (isAllSolutions && flesseq(newNodeF , currentCost)))
					{
						
						if (fless(newNodeF , queue.GetLowerBound()) || (isLEQ && flesseq(newNodeF , queue.GetLowerBound())))
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
					}
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
							expansionsUntilSolution = nodesExpanded;
							
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
uint64_t DVCBS<state, action, environment, dataStructure, priorityQueue>::GetDoubleExpansions() const
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
//void DVCBS<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw() const
//{
//	OpenGLDraw(queue.forwardQueue);
//	OpenGLDraw(queue.backwardQueue);
//}
//
//template <class state, class action, class environment, class dataStructure, class priorityQueue>
//void DVCBS<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw(const priorityQueue &q) const
//{
//	{
//		double transparency = 0.9;
//		if (q.size() == 0)
//			return;
//		uint64_t top = -1;
//		//	double minf = 1e9, maxf = 0;
////		if (q.OpenReadySize() > 0)
////		{
////			top = q.Peek(kOpenReady);
////		}
//		for (unsigned int x = 0; x < q.size(); x++)
//		{
//			const auto &data = q.Lookat(x);
////			if (x == top)
////			{
////				env->SetColor(1.0, 1.0, 0.0, transparency);
////				env->OpenGLDraw(data.data);
////			}
//			if (data.where == kOpenWaiting)
//			{
//				env->SetColor(0.0, 0.5, 0.5, transparency);
//				env->OpenGLDraw(data.data);
//			}
//			else if (data.where == kOpenReady)
//			{
//				env->SetColor(0.0, 1.0, 0.0, transparency);
//				env->OpenGLDraw(data.data);
//			}
//			else if (data.where == kClosed)
//			{
//				if (&q == &queue.backwardQueue)
//					env->SetColor(1.0, 0.0, 1.0, transparency);
//				else
//					env->SetColor(1.0, 0.0, 0.0, transparency);
//				env->OpenGLDraw(data.data);
//			}
//		}
//	}
//	
//}
//

#endif /* DVCBS_h */
