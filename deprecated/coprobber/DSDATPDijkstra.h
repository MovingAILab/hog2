#include <vector>
#include "MapAbstraction.h"
#include "DSTPDijkstra.h"
#include "PRAStar.h"
#include "MaximumNormAbstractGraphMapHeuristic.h"
#include "DSRobberAlgorithm.h"

#ifndef DSDATPDIJKSTRA_H
#define DSDATPDIJKSTRA_H

/*
	dynamic abstract two player dijkstra
	code mainly copied from DSDAM.h but modified to function with
	two player dijkstra

	note: These routines only compute move strategies for the robber!!!
	note: players can always pass their turn
*/

class DSDATPDijkstra:
	praStar, public DSRobberAlgorithm<graphState,graphMove> // because we want to use the refinement routines from PRA*
{

	public:

	// constructor
	DSDATPDijkstra( MapAbstraction *gabs, unsigned int cop_speed = 1, bool useAbstraction = true );
	~DSDATPDijkstra();

	void datpdijkstra( node* pos_robber, node* pos_cop, std::vector<node*> &path, bool minFirst = true, double min_escape_length = 5. );

	node* MakeMove( node* pos_robber, node* pos_cop, bool minFirst = true, double min_escape_length = 5. );

	graphState MakeMove( graphState pos_robber, graphState pos_cop, unsigned int );

	// statistic variables that get resetted everytime datpdijkstra(...) is called
	unsigned int myNodesExpanded, myNodesTouched;

	protected:

	MapAbstraction *gabs;
	unsigned int cop_speed;
	bool useAbstraction;

	// cache the minimax objects and graph environments for each level of abstraction
	std::vector<DSTPDijkstra<graphState,graphMove>* > dstpdijkstra;
	std::vector<GraphEnvironment*> graphenvironments;
	std::vector<MaximumNormAbstractGraphMapHeuristic*> graphmapheuristics;
};

#endif
