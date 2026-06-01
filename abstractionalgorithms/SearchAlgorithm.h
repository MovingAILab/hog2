/*
 *  $Id: SearchAlgorithm.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef SEARCHALGORITHM_H
#define SEARCHALGORITHM_H

#include "Graph.h"
#include "Path.h"
#include "MapAbstraction.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"

/**
 * A generic algorithm which can be used for pathfinding.
 */

class SearchAlgorithm {
public:
	SearchAlgorithm() { nodesExpanded = nodesTouched = 0; }
	virtual ~SearchAlgorithm() {}
	virtual const char *GetName() = 0;
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0) = 0;
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	virtual void LogFinalStats(StatCollection *) {}

	//protected:
	uint32_t nodesExpanded;
	uint32_t nodesTouched;
};

extern void DoRandomPath(GraphAbstraction *aMap, SearchAlgorithm *sa, bool repeat = false);

#endif
