/*
 *  $Id: aStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/22/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */


#ifndef ASTAR3_H
#define ASTAR3_H

#include "SearchAlgorithm.h"
#include "Path.h"
#include "GraphAbstraction.h"
#include "FPUtil.h"
#include "OpenClosedList.h"

#include <unordered_map>

namespace AStar3Util
{
	class SearchNode {
public:
		SearchNode(double _fCost=0, double _gCost=0, edge *_e=0, node *curr=0, node *prev=0)
		:fCost(_fCost), gCost(_gCost), e(_e), currNode(curr), prevNode(prev) {}
		
		SearchNode(node *curr)
		:fCost(0), gCost(0), e(0), currNode(curr), prevNode(0) {}

		double fCost;
		double gCost;
//		double steps;
		edge *e;
		node *currNode;
		node *prevNode;
	};
	
	
	struct NodeEqual {
		bool operator()(const node *i1, const node *i2) const
		{ return (i1->getUniqueID() == i2->getUniqueID()); } };
	
	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{ return (i1.currNode->getUniqueID() == i2.currNode->getUniqueID()); } };
	
	struct SearchNodeCompare {
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} };
	
	struct NodeHash {
		size_t operator()(const node *x) const
		{ return (size_t)(x->getUniqueID()); }
	};
	
	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode->getUniqueID()); }
	};
	
	typedef OpenClosedList<AStar3Util::SearchNode, AStar3Util::SearchNodeHash,
		AStar3Util::SearchNodeEqual, AStar3Util::SearchNodeCompare> PQueue;
	
	typedef std::unordered_map<node*, AStar3Util::SearchNode,
		AStar3Util::NodeHash, AStar3Util::NodeEqual > NodeLookupTable;
	
	typedef std::unordered_map<node*, bool,
		AStar3Util::NodeHash, AStar3Util::NodeEqual > Corridor;
}

class aStar : public SearchAlgorithm {
public:
	aStar() {}
	virtual ~aStar() {}
	path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *GetName();
	
	double getHVal(node *whence);
	void setCorridor(path *corridor, int width);
	
	void printStats();
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void resetNodeCount() { nodesExpanded = nodesTouched = 0; }
	int getMemoryUsage();
private:
	//	long nodesTouched, nodesExpanded;
	inline node *ABSNode(node *n) { return abstr->GetNthParent(n, absLevel); }
	path *getPathToNode(node *target, reservationProvider *rp);
	path *extractPathToStart(Graph *g, node *n);
	node *getNextNode();
	void updateWeight(node *currOpenNode, node *neighbor, edge *e);
	void addToOpenList(node *currOpenNode, node *neighbor, edge *e);
	bool nodeInCorridor(node *n);
	void addNeighborsToCorridor(Graph *g, node *n, int windowSize);
	void buildCorridor(path *p, int windowSize);
	double internalHeuristic(node *from, node *to);
	AStar3Util::PQueue openQueue;
	AStar3Util::NodeLookupTable closedList;
	node *goal, *start;
	Graph *g;
	GraphAbstraction *abstr;
	AStar3Util::Corridor eligibleNodes;
	int absLevel;
	//	AStarHeuristic *abstraction;
};

#endif
