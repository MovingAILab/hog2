#include <vector>
#include "MultiAgentEnvironment.h"
#include "DSCREnvironment.h"
#include "DSRobberAlgorithm.h"

#ifndef DSHEURISTICGREEDY_H
#define DSHEURISTICGREEDY_H

/*
	Implementation of a greedy algorithm for the robber/cop
	This algorithm selects the best move according to
	the heuristic (octile distance)
*/
template<class state,class action>
class DSHeuristicGreedy: public DSRobberAlgorithm<state,action> {

	public:

	// constructor & destructor
	DSHeuristicGreedy( SearchEnvironment<state,action> *env,
		bool canPass = true, unsigned int cop_speed = 1 );
	virtual ~DSHeuristicGreedy();

	// makes a move as described above
	state MakeMove( state pos_robber, state pos_cop, bool minFirst );

	state MakeMove( state pos_robber, state pos_cop, unsigned int );

	unsigned int nodesExpanded, nodesTouched;

	protected:

	DSCREnvironment<state,action> *dscrenv;

};

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

template<class state,class action>
DSHeuristicGreedy<state,action>::DSHeuristicGreedy( SearchEnvironment<state,action> *env, bool canPass, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<state,action>( env, canPass, cop_speed ) )
{ };

template<class state,class action>
DSHeuristicGreedy<state,action>::~DSHeuristicGreedy() {
	delete dscrenv;
};

template<class state,class action>
state DSHeuristicGreedy<state,action>::MakeMove( state pos_robber, state pos_cop, bool minFirst ) {

	nodesExpanded = 0; nodesTouched = 0;

	// find out all the possible moves
	std::vector<state> neighbors;
	state nextmove = minFirst?pos_cop:pos_robber;
	double heuristic_value, temp;

	if( minFirst ) {
		dscrenv->GetCopSuccessors( pos_cop, neighbors );
		nodesExpanded++; nodesTouched++;
		heuristic_value = DBL_MAX;
		// for each successor state find the one that is best concerning the heuristic
		for( typename std::vector<state>::iterator it = neighbors.begin();
			it != neighbors.end(); it++ ) {
			nodesTouched++;
			temp = dscrenv->HCost( pos_robber, *it );
			if( temp < heuristic_value ) {
				nextmove = *it;
				heuristic_value = temp;
			}
		}

	} else {
		dscrenv->GetRobberSuccessors( pos_robber, neighbors );
		nodesExpanded++; nodesTouched++;
		heuristic_value = -DBL_MAX;
		for( typename std::vector<state>::iterator it = neighbors.begin();
			it != neighbors.end(); it++ ) {
			nodesTouched++;
			temp = dscrenv->HCost( *it, pos_cop );
			if( temp > heuristic_value ) {
				nextmove = *it;
				heuristic_value = temp;
			}
		}
	}

	return nextmove;
};

template<class state,class action>
state DSHeuristicGreedy<state,action>::MakeMove( state pos_robber, state pos_cop, unsigned int ) {
	return( MakeMove( pos_robber, pos_cop, false ) );
};


#endif
