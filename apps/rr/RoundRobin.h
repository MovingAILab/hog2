//
//  RoundRobin.h
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 8/9/24.
//  Copyright © 2024 MovingAI. All rights reserved.
//

#ifndef RoundRobin_h
#define RoundRobin_h

#pragma mark -
#pragma mark RoundRobin Class Implementation, Adapted from Tianyi Gu [link needed]
#pragma mark -

#include "PriorityQueue.h"
#include "RBTree.h"
#include "Node.h"
#include "Graphics.h"

template <class state, class action, class environment>
class RoundRobin {
	enum class Qtype
	{
		undefined,
		focal,
		open,
		openfhat,
		openAndOpenFhat
	};
	
public:
#pragma mark public
	RoundRobin() {}
	
	~RoundRobin()
	{
		// delete all of the nodes from the last expansion phase
		for (typename unordered_map<state, Node<state>*>::iterator it = closed.begin(); it != closed.end(); it++)
			delete it->second;

		closed.clear();
	}
	
	bool InitializeSearch(environment *e_, state start_, state goal_)
	{
		epsilonDSum      = 0;
		epsilonHSum      = 0;
		epsilonHSumSq    = 0;
		expansionCounter = 0;
		curEpsilonD      = 0;
		curEpsilonH      = 0;
		curEpsilonHVar   = 0;
		xesExp = 1;
		this->env = e_;
		start = start_;
		goal = goal_;

		fhatminVar = 100;
		fhatminSum = 0;
		fhatminSumSq = 0;
		fhatminCounter = 0;

		{
			for (typename unordered_map<state, Node<state>*>::iterator it = closed.begin(); it != closed.end(); it++)
				delete it->second;

			closed.clear();
		}
		
	
		open.swapComparator(Node<state>::compareNodesF);
		open.swapCursorValueFn(Node<state>::getNodeF);
		focal.swapComparator(Node<state>::compareNodesExpectedEffort);
		
		auto inith = env->HCost(start, goal);
		auto initD = env->DCost(start, goal);
		
		// Get the start node
		Node<state>* initNode = new Node<state>(
												0, inith, initD, epsilonHGlobal(),
												epsilonDGlobal(), epsilonHVarGlobal(),
												start, NULL);
		
		Node<state>* weightedInitNode =
		new Node<state>(Node<state>::weight * initNode->getGValue(),
						Node<state>::weight * initNode->getHValue(),
						Node<state>::weight * initNode->getDValue(),
						epsilonHGlobal(), epsilonDGlobal(),
						epsilonHVarGlobal(), start, NULL);
		
		fmin        = initNode->getFValue();
		fhatmin     = initNode->getFHatValue();
		fhatminNode = initNode;
		pushFhatmin();
		
		open.insert(initNode);
		openfhat.push(initNode);
		bool isIncrement;
		open.updateCursor(weightedInitNode, isIncrement);
		//		res.initialH = inith;
		nodesExpanded = 0;
		nodesGenerated = 0;
		return false;
	}
	
	bool DoSingleSearchStep(std::vector<state> &path)
	{
		// Expand until find the goal
		if (!open.empty()) {
			// cout << "open size " << open.getSize() << "\n";
			// cout << "openfhat size " << openfhat.size() << "\n";
			
			Qtype nodeFrom = Qtype::undefined;
			Node<state>* cur      = selectNode(nodesExpanded, nodeFrom);
			
			// Check if current node is goal
			if (env->GoalTest(cur->getState(), goal))
			{
				this->getSolutionPath(cur, path);
				return true;
			}
			
			nodesExpanded++;
			
			cur->close();
			
			vector<state> children;
			env->GetSuccessors(cur->getState(), children);
			nodesGenerated += children.size();
			
			state bestChild;
			Cost  bestF = numeric_limits<double>::infinity();
			
			for (state child : children)
			{
				auto newG = cur->getGValue() + env->GCost(cur->getState(), child);
				auto newH = env->HCost(child, goal);
				auto newD = env->DCost(child, goal);
				
				Node<state>* childNode =
				new Node<state>(newG, newH, newD, epsilonHGlobal(),
								epsilonDGlobal(),
								epsilonHVarGlobal(), child, cur);
				
				bool dup = duplicateDetection(childNode);
				
				if (!dup && childNode->getFValue() < bestF) {
					bestF     = childNode->getFValue();
					bestChild = child;
				}
				
				// Duplicate detection
				if (!dup) {
					
					childNode->computeExpectedEffortValue(fhatminNode,
														  fhatminVar);
					
					open.insert(childNode);
					openfhat.push(childNode);
					// if (childNode->getFValue() <= Node::weight * fmin) {
					
					// if (res.nodesExpanded > 100 &&
					if (childNode->getFValue() <= Node<state>::weight * fmin) {
						focal.push(childNode);
					}
					
					closed[child] = childNode;
				} else
					delete childNode;
			}
			
			// Learn one-step error
			if (bestF != numeric_limits<double>::infinity()) {
				Cost epsD =
				(1 + env->DCost(bestChild, goal)) - cur->getDValue();
				Cost epsH = (env->GCost(cur->getState(), bestChild) +
							 env->HCost(bestChild, goal)) -
				cur->getHValue();
				
				pushEpsilonHGlobal(epsH);
				pushEpsilonDGlobal(epsD);
				updateEpsilons();
			}
			
			// update fhatmin
			// if (nodeFrom == Qtype::openfhat ||
			// nodeFrom == Qtype::openAndOpenFhat) {
			// cout << "update fhatmin fffffffffffffffffffffhatmin" << endl;
			fhatminNode = openfhat.top();
			// auto oldfhatmin = fhatmin; // reevalutate code
			fhatmin = fhatminNode->getFHatValue();
			//}
			
			pushFhatmin();
			
			// update fmin
			auto fminNode = open.getMinItem();
			
			if (fmin != fminNode->getFValue())
			{
				
				fmin = fminNode->getFValue();
				
				Node<state>* weightedFMinNode = new Node<state>(
																Node<state>::weight * fminNode->getGValue(),
																Node<state>::weight * fminNode->getHValue(),
																Node<state>::weight * fminNode->getDValue(),
																epsilonHGlobal(), epsilonDGlobal(),
																epsilonHVarGlobal(), state(), NULL);
				
				bool isIncrease;
				auto itemsNeedUpdate =
				open.updateCursor(weightedFMinNode, isIncrease);
				
				for (auto item : itemsNeedUpdate)
				{
					if (isIncrease)
					{
						// reevaluateNode(item); // reevalutate code
						focal.push(item);
						
					}
					else {
						focal.remove(item);
					}
				}
			}
			assert(open.getSize() == openfhat.size());
			// cout << "--------------------expansion end-------" << endl;
		}
		return false; // didn't finish search yet
	}
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	void getSolutionPath(Node<state>* goal, std::vector<state> &path)
	{
		auto cur = goal;
		
		while (cur)
		{
			path.push_back(cur->getState());
			cur = cur->getParent();
		}
		std::reverse(path.begin(), path.end());
		//		res.soltuionPath = p;
	}

	void Draw(Graphics::Display &display) const
	{
		for (auto i = closed.begin(); i != closed.end(); i++)
		{
			env->SetColor(Colors::red);
			env->Draw(display, i->second->stateRep);
		}
		for (auto i : focal)
		{
			env->SetColor(Colors::blue);
			env->Draw(display, i->stateRep);
		}
		for (auto i : openfhat)
		{
			env->SetColor(Colors::pink);
			env->Draw(display, i->stateRep);
		}
//		PriorityQueue<Node<state>*>              focal;
//		PriorityQueue<Node<state>*>              openfhat;
//		unordered_map<state, Node<state>*> closed;

	}

#pragma mark private
private:
	Node<state>* selectNode(size_t expansionNum, Qtype& nodeFrom)
	{
		Node<state>* cur;

		//open.prettyPrint();
		auto qtype = roundRobinGetQtype(expansionNum);
		if (qtype == Qtype::focal && !focal.empty()) {
			
			// cout << "pop from focal\n";
			cur = focal.top();
			
			auto isOpenTop     = cur == open.getMinItem();
			auto isOpenFhatTop = cur == openfhat.top();
			
			nodeFrom = Qtype::focal;
			
			if (isOpenTop && isOpenFhatTop)
			{
				nodeFrom = Qtype::openAndOpenFhat;
			}
			else if (isOpenTop)
			{
				nodeFrom = Qtype::open;
			}
			else if (isOpenFhatTop)
			{
				nodeFrom = Qtype::openfhat;
			}
			focal.pop();
			
			open.deleteNode(cur);
			openfhat.remove(cur);
			return cur;
		}
		else if (qtype == Qtype::openfhat && !openfhat.empty())
		{
			cur = openfhat.top();
			
			nodeFrom       = Qtype::openfhat;
			auto isOpenTop = cur == open.getMinItem();
			if (isOpenTop)
			{
				nodeFrom = Qtype::openAndOpenFhat;
			}
			
			openfhat.pop();
			focal.remove(cur);
			open.deleteNode(cur);
			return cur;
		}
		
		cur = open.getMinItem();
		
		nodeFrom = Qtype::open;
		if (cur == openfhat.top()) {
			nodeFrom = Qtype::openAndOpenFhat;
		}
		
		// cout << "pop from open\n";
		focal.remove(cur);
		open.deleteNode(cur);
		openfhat.remove(cur);
		return cur;
	}
	
	bool duplicateDetection(Node<state>* node)
	{
		// Check if this state exists
		typename unordered_map<state, Node<state>*>::iterator it =
		closed.find(node->getState());
		
		if (it == closed.end()) // return not found
			return false;

		// if the new node is better, update it on close
		if (node->getGValue() >= it->second->getGValue()) // return found, no updating needed
			return true;

		// This state has been generated before,
		// check if its node is on OPEN
		if (it->second->onOpen()) {
			// This node is on OPEN and cleanup, keep the better
			// g-value cout << "dup on open " << it->second << "\n";
			open.deleteNode(it->second);
			
			it->second->setGValue(node->getGValue());
			it->second->setParent(node->getParent());
			it->second->setHValue(node->getHValue());
			it->second->setDValue(node->getDValue());
			it->second->setEpsilonH(node->getEpsilonH());
			it->second->setEpsilonHVar(node->getEpsilonHVar());
			it->second->setEpsilonD(node->getEpsilonD());
			it->second->setState(node->getState());
			
			open.insert(it->second);
			focal.update(it->second);
			openfhat.update(it->second);
		} else {
			it->second->reopen();
			
			it->second->setGValue(node->getGValue());
			it->second->setParent(node->getParent());
			it->second->setHValue(node->getHValue());
			it->second->setDValue(node->getDValue());
			it->second->setEpsilonH(node->getEpsilonH());
			it->second->setEpsilonHVar(node->getEpsilonHVar());
			it->second->setEpsilonD(node->getEpsilonD());
			it->second->setState(node->getState());
			
			open.insert(it->second);
			openfhat.push(it->second);
			if (it->second->getFHatValue() <= Node<state>::weight * fmin) {
				focal.push(it->second);
			}
		}
		return true;
	}
	
	Qtype roundRobinGetQtype(size_t expansion)
	{
//		auto curNum = static_cast<int>(expansion) % (xesExp + 1 + 1);
//		if (curNum <= xesExp) {
//			return Qtype::focal;
//		}
//		if (curNum == xesExp + 1) {
//			return Qtype::open;
//		}
//		return Qtype::openfhat;
		
		// Perhaps more correct code
		auto curNum = static_cast<int>(expansion) % (xesExp + 1 + 1);
		if (curNum < xesExp) {
			return Qtype::focal;
		}
		if (curNum == xesExp) {
			return Qtype::open;
		}
		return Qtype::openfhat;
	}
	
	void pushFhatmin()
	{
		fhatminSum += fhatmin;
		fhatminSumSq += fhatmin * fhatmin;
		fhatminCounter++;
		
		if (fhatminCounter < 2) {
			fhatminVar = 100;
			return;
		}
		
		fhatminVar =
		(fhatminSumSq - (fhatminSum * fhatminSum) / fhatminCounter) /
		(fhatminCounter - 1.0);
	}
	
	RBTree<Node<state>*>              open;
	Cost                              fhatmin;
	Cost                              fmin;
	PriorityQueue<Node<state>*>              focal;
	PriorityQueue<Node<state>*>              openfhat;
	Node<state>*                      fhatminNode;
	unordered_map<state, Node<state>*> closed;
	
	double fhatminVar;
	double fhatminSum;
	double fhatminSumSq;
	double fhatminCounter;
	uint64_t nodesExpanded;
	uint64_t nodesGenerated;
	int xesExp;
	environment *env;
	state start, goal;
	

	double epsilonHSum;
	double epsilonHSumSq;
	double epsilonDSum;
	double curEpsilonH;
	double curEpsilonD;
	double curEpsilonHVar;
	double expansionCounter;

	Cost epsilonHGlobal() { return curEpsilonH; }
	Cost epsilonDGlobal() { return curEpsilonD; }
	Cost epsilonHVarGlobal() { return curEpsilonHVar; }
	void updateEpsilons()
	{
		if (expansionCounter < 100) {
			curEpsilonD    = 0;
			curEpsilonH    = 0;
			curEpsilonHVar = 0;

			return;
		}

		curEpsilonD = epsilonDSum / expansionCounter;

		curEpsilonH = epsilonHSum / expansionCounter;

		curEpsilonHVar =
		  (epsilonHSumSq - (epsilonHSum * epsilonHSum) / expansionCounter) /
		  (expansionCounter - 1);

		assert(curEpsilonHVar > 0);
	}

	void pushEpsilonHGlobal(double eps)
	{
		/*if (eps < 0)*/
		// eps = 0;
		// else if (eps > 1)
		/*eps = 1;*/

		epsilonHSum += eps;
		epsilonHSumSq += eps * eps;
		expansionCounter++;
	}

	void pushEpsilonDGlobal(double eps)
	{
		/*if (eps < 0)*/
		// eps = 0;
		// else if (eps > 1)
		/*eps = 1;*/

		epsilonDSum += eps;
		expansionCounter++;
	}
};


#endif /* RoundRobin_h */
