/*
 *  GenericIDAStar.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 1/3/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef GENERICIDASTAR_H
#define GENERICIDASTAR_H

#include "OldSearchEnvironment.h" // for the SearchEnvironment class

#include <unordered_map>

typedef std::unordered_map<uint32_t, double> NodeHashTable;

class GenericIDAStar {
public:
	GenericIDAStar() {}
	virtual ~GenericIDAStar() {}
	void GetPath(OldSearchCode::SearchEnvironment *env, uint32_t from, uint32_t to,
							 std::vector<uint32_t> &thePath);

	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void SetUseBDPathMax(bool val) { usePathMax = val; }
private:
	unsigned long nodesExpanded, nodesTouched;

	double DoIteration(OldSearchCode::SearchEnvironment *env,
										 uint32_t parent, uint32_t currState, uint32_t goal,
										 std::vector<uint32_t> &thePath, double bound, double g,
										 double maxH);
	void UpdateNextBound(double currBound, double fCost);
	double nextBound;
	NodeHashTable nodeTable;
	bool usePathMax;
};	

#endif
