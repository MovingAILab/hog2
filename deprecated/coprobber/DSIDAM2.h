#include <vector>
#include "MapAbstraction.h"
#include "DSMinimax.h"
#include "PRAStar.h"
#include "MaximumNormAbstractGraphMapHeuristic.h"
#include "DSRobberAlgorithm.h"

#ifndef DSIDAM2_H
#define DSIDAM2_H

/*
	improved dynamic abstract minimax (IDAM)
	code functions with DSCREnvironment, thus possibly faster cop

	this version only takes the next abstract move of the robber
	and refines it into the ground level

	node: this code only works for the robber!!!
*/

class DSIDAM2:
	praStar, // because we want to use the refinement routines from PRA*
	public DSRobberAlgorithm<graphState,graphMove>
{

	public:

	// constructor
	DSIDAM2( MapAbstraction *gabs, bool canPause = true, unsigned int cop_speed = 1, bool useAbstraction = true );
	~DSIDAM2();

	void dam( node* pos_robber, node* pos_cop, std::vector<node*> &path, bool minFirst = true, double depth = 5., double start_level_fraction = 0.5 );

	node* MakeMove( node* pos_robber, node* pos_cop, bool minFirst = true, double depth = 5., double start_level_fraction = 0.5 );

	graphState MakeMove( graphState pos_robber, graphState pos_cop, unsigned int );

	// statistic variables that get resetted everytime dam(...) is called
	unsigned int myNodesExpanded, myNodesTouched;

	protected:

	MapAbstraction *gabs;
	bool canPause;
	unsigned int cop_speed;
	bool useAbstraction;

	// cache the minimax objects and graph environments for each level of abstraction
	std::vector<DSMinimax<graphState,graphMove>* > dsminimax;
	std::vector<GraphEnvironment*> graphenvironments;
	std::vector<MaximumNormAbstractGraphMapHeuristic*> graphmapheuristics;
};

#endif
