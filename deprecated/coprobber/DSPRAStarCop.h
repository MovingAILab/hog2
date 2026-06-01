#include "PRAStar.h"
#include "GraphAbstraction.h"
#include "GraphEnvironment.h"

#ifndef DSPRASTARCOP_H
#define DSPRASTARCOP_H

/*
	Cop move policy. The cop chooses a move due to PRA* and
	follows it for cop_speed steps
*/
class DSPRAStarCop {

	public:

	// constructor and destructor
	DSPRAStarCop( GraphAbstraction *graphabs, unsigned int cop_speed = 1 );
	~DSPRAStarCop();

	graphState MakeMove( graphState &robber, graphState &cop, float &gcost );

	// follow 1/pathfraction of the path computed by PRA* before recomputing a new path
	// default set by constructor is 2
	void SetPathFraction( unsigned int _pathfraction ) { pathfraction = _pathfraction; };

	unsigned int nodesExpanded;
	unsigned int nodesTouched;

	protected:

	GraphAbstraction *graphabs;
	unsigned int cop_speed;
	unsigned int pathfraction;
	Graph *g;
	praStar *pra;
	std::vector<graphState> pathcache;
	std::vector<float> gcosts;

};

#endif
