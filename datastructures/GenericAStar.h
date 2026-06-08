/*
 *  $Id: GenericAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/30/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */


#ifndef GENERICSTAR_H
#define GENERICSTAR_H

#define __STDC_CONSTANT_MACROS
#include <stdint.h>
// this is defined in stdint.h, but it doesn't always get defined correctly
// even when __STDC_CONSTANT_MACROS is defined before including stdint.h
// because stdint might be included elsewhere first...
#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#include "FPUtil.h"
#include "OpenClosedList.h"
#include "OldSearchEnvironment.h" // for the SearchEnvironment class

#include <unordered_map>

namespace GenericAStarUtil
{
	class SearchNode {
public:
		SearchNode(double _fCost=0, double _gCost=0, uint32_t curr=0, uint32_t prev=0)
		:fCost(_fCost), gCost(_gCost), currNode(curr), prevNode(prev) {}

		SearchNode(uint32_t curr)
		:fCost(0), gCost(0), currNode(curr), prevNode(0) {}

		double fCost;
		double gCost;
		uint32_t currNode;
		uint32_t prevNode;
	};
		
	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{ return (i1.currNode == i2.currNode); } };
	
	struct SearchNodeCompare {
		bool operator()(const SearchNode &i1, const SearchNode &i2) const
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} };
		
	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode); }
	};
	
	typedef OpenClosedList<GenericAStarUtil::SearchNode, GenericAStarUtil::SearchNodeHash,
		GenericAStarUtil::SearchNodeEqual, GenericAStarUtil::SearchNodeCompare> PQueue;
	
	typedef std::unordered_map<uint32_t, GenericAStarUtil::SearchNode > NodeLookupTable;
	
	typedef std::unordered_map<uint32_t, bool > Corridor;
}


typedef GenericAStarUtil::NodeLookupTable::const_iterator closedList_iterator;

class GenericAStar {
public:
	GenericAStar() {}
	virtual ~GenericAStar() {}
	void GetPath(OldSearchCode::SearchEnvironment *env, uint32_t from, uint32_t to,
							 std::vector<uint32_t> &thePath);

	bool InitializeSearch(OldSearchCode::SearchEnvironment *env, uint32_t from, uint32_t to,
												std::vector<uint32_t> &thePath);
	bool DoSingleSearchStep(std::vector<uint32_t> &thePath);
	uint32_t CheckNextNode();
	void ExtractPathToStart(uint32_t n, std::vector<uint32_t> &thePath);

	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	int GetMemoryUsage();

	closedList_iterator GetClosedListIter() const;
  uint32_t ClosedListIterNext(closedList_iterator&) const;

private:
		uint64_t nodesTouched, nodesExpanded;

	uint32_t GetNextNode();
	void UpdateWeight(uint32_t currOpenNode, uint32_t neighbor);
	void AddToOpenList(uint32_t currOpenNode, uint32_t neighbor);
	GenericAStarUtil::PQueue openQueue;
	GenericAStarUtil::NodeLookupTable closedList; //openList
	uint32_t goal, start;

	std::vector<uint32_t> neighbors;
	OldSearchCode::SearchEnvironment *env;
	GenericAStarUtil::Corridor eligibleNodes;
};

#endif
