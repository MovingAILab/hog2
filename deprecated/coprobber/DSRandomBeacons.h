#include <vector>
#include "GraphAbstraction.h"
#include "GraphEnvironment.h"
#include "DSCREnvironment.h"
#include "MaximumNormAbstractGraphMapHeuristic.h"
#include "PRAStar.h"
#include "DSRobberAlgorithm.h"

#ifndef DSRANDOMBEACONS_H
#define DSRANDOMBEACONS_H

/*
	Implentation of a strategy that randomly places beacons on the map
	and then evaluates these beacons heuristically. It then produces
	a path towards the beacon with the best evaluate (via PRA*).
*/
class DSRandomBeacons: public DSRobberAlgorithm<graphState,graphMove> {

	public:

	// constructor & destructor
	DSRandomBeacons( MapAbstraction *mabs, bool canPass = true,
		unsigned int cop_speed = 1 );
	virtual ~DSRandomBeacons();

	// this algorithm only functions for the robber and produces a path
	// that he should take
	void GetPath( graphState pos_robber, graphState pos_cop, unsigned int num_beacons,
		std::vector<graphState> &path );

	graphState MakeMove( graphState pos_robber, graphState pos_cop, unsigned int );

	unsigned int nodesExpanded, nodesTouched;

	protected:

	MapAbstraction *mabs;
	MaximumNormAbstractGraphMapHeuristic *gh;
	GraphEnvironment *env;
	// we need this for heuristic evaluation
	DSCREnvironment<graphState,graphMove> *dscrenv;
	praStar *pra;

};


#endif
