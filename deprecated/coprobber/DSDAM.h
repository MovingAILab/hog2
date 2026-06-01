#include <vector>
#include "MapAbstraction.h"
#include "DSMinimax.h"
#include "PRAStar.h"
#include "MaximumNormAbstractGraphMapHeuristic.h"
#include "DSRobberAlgorithm.h"

#ifndef DSDAM_H
#define DSDAM_H

/*
	dynamic abstract minimax (DAM)
	code functions with DSCREnvironment, thus possibly faster cop

	original DAM implementation with additional start_level_fraction parameter
*/

class DSDAM: public DSRobberAlgorithm<graphState,graphMove> {

	public:

	// constructor
	DSDAM( MapAbstraction *gabs, bool canPause = true, unsigned int cop_speed = 1, bool useAbstraction = true );
	~DSDAM();

	// if m is the level where cop and robber position abstract together, then computation starts
	// on level m*start_level_fraction
	void dam( node* pos_robber, node* pos_cop, std::vector<node*> &path, bool minFirst = true, double depth = 5., double start_level_fraction = 0.5 );

	node* MakeMove( node* pos_robber, node* pos_cop, bool minFirst = true, double depth = 5., double start_level_fraction = 0.5 );

	graphState MakeMove( graphState pos_robber, graphState pos_cop, unsigned int );

	// statistic variables that get resetted everytime dam(...) is called
	unsigned int nodesExpanded, nodesTouched;

	protected:

	MapAbstraction *gabs;
	bool canPause;
	unsigned int cop_speed;
	bool useAbstraction;

	praStar *pra;

	// cache the minimax objects and graph environments for each level of abstraction
	std::vector<DSMinimax<graphState,graphMove>* > dsminimax;
	std::vector<GraphEnvironment*> graphenvironments;
	std::vector<MaximumNormAbstractGraphMapHeuristic*> graphmapheuristics;
};

#endif
