#include "Dijkstra.h"

Dijkstra::Dijkstra( GraphEnvironment *_env, unsigned int _number_of_cops, bool _canPass ):
	env(_env), number_of_cops(_number_of_cops), canPass(_canPass) {

	crg = new CopRobberGame( env, number_of_cops, false, canPass );
}

Dijkstra::~Dijkstra() {
	delete crg;
}


void Dijkstra::dijkstra() {
	QueueEntry qe, qtemp;

	push_end_states_on_queue();

	while( !queue.empty() ) {

		// get the element from the queue
		qe = queue.top(); queue.pop();

		// verbose
		//fprintf( stdout, "minFirst = %d, pos = (%u,%u), value = %f\n", qe.minFirst, qe.pos[0], qe.pos[1], qe.value );

		unsigned int num = crg->GetNumberByState( qe.pos );

		if( qe.minFirst ) {

			if( min_cost[num] != DBL_MAX ) {
				// if the value for this position has been set yet
				// sanity check: check that the value for the position is smaller than the current value that
				// we expanded
				assert( min_cost[num] <= qe.value );
			} else {

				min_cost[num] = qe.value;

				// the cops moved in this state, thus we have to find the
				// opposite actions of the robber here
				// note: this cannot be implemented with the CopRobberGame
				//   interface since that doesn't allow us to move away from the cops
				//   after capture (CopRobberOccupancy)
				std::vector<graphState> myneighbors;
				env->GetSuccessors( qe.pos[0], myneighbors );
				if( canPass )
					myneighbors.push_back( qe.pos[0] );

				// now, for all successor states
				for( std::vector<graphState>::iterator it = myneighbors.begin();
				     it != myneighbors.end(); it++ ) {
					// build the state
					qtemp = qe;
					qtemp.pos[0] = *it;


					// check whether its value is \infty
					if( max_cost[ crg->GetNumberByState( qtemp.pos ) ] == DBL_MAX ) {
						qtemp.value = compute_target_value( qtemp.pos );
						if( qtemp.value != DBL_MAX ) {
							qtemp.minFirst = !qe.minFirst; // !qe.minFirst == false
							queue.push( qtemp );
						}
					}
				}
			}

		} else {

			if( max_cost[num] != DBL_MAX ) {
				// sanity check: we should only set the value of a max position once
				assert( max_cost[ num ] == qe.value );
			} else {

				max_cost[ num ] = qe.value;

				// the robber moved in this state, thus we have to find the
				// actions of the cops leading to this state
				std::vector<CRMove> cop_actions;
				crg->GetPossibleOpponentActions( 0, qe.pos, cop_actions );
				for( std::vector<CRMove>::iterator it = cop_actions.begin();
				     it != cop_actions.end(); it++ ) {

					// build the next state
					qtemp.pos = qe.pos;
					// the robber sits out - this should actually not be neccessary because CopRobberGame takes care of it
					(*it)[0] = CRAction();
					crg->ApplyAction( qtemp.pos, *it );

					// check agains infinity
					if( min_cost[ crg->GetNumberByState( qtemp.pos ) ] == DBL_MAX ) {
						qtemp.minFirst = !qe.minFirst; // !qe.minFirst == true
						qtemp.value    = qe.value + MinGCost( qtemp.pos, qe.pos );
						queue.push( qtemp );
					}
				}
			}
		}

	}

}

double Dijkstra::compute_target_value( CRState &s ) {
	double result = 0.;

	CRState temp = s;
	double tempvalue;
	std::vector<graphState> myneighbors;
	env->GetSuccessors( s[0], myneighbors );
	if( canPass )
		myneighbors.push_back( s[0] );

	// now, for all successor states
	for( std::vector<graphState>::iterator it = myneighbors.begin();
	     it != myneighbors.end(); it++ ) {
	
		// build the state
		temp[0] = *it;
		tempvalue = MinGCost( temp, s ) + min_cost[ crg->GetNumberByState( temp ) ];
		if( tempvalue == DBL_MAX ) return DBL_MAX;
		if( tempvalue > result ) result = tempvalue;
	}
	return result;
}


// this function loops through all possible states for now
// instead of generating the beginning states directly from the map
void Dijkstra::push_end_states_on_queue() {
	QueueEntry qe;

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );
	// get the number of joint states
	unsigned int num_states = crg->GetNumStates();
	min_cost.assign( num_states, DBL_MAX );
	max_cost.assign( num_states, DBL_MAX );
	// for all states
	for( unsigned int i = 0; i < num_states; i++ ) {

		qe.pos = crg->GetStateByNumber( i );

		if( crg->GoalTest( qe.pos, qe.pos ) ) {
			qe.minFirst = false;
			qe.value    = 0.;
			queue.push( qe );

			if( canPass ) {
				// In case the players can pass their turn, it will always be the cop who does the last move.
				// Thus, we only have to push the qe's onto the queue where qe.minFirst = false.

				// set the value for the min costs (h_p)
				min_cost[i] = 0.;
			} else {
				// In the case where players cannot pass their turn, we have to push all end states onto the queue
				qe.minFirst = true;
				queue.push( qe );
			}

		}
	}
	return;
}



// this version is a faster variant of the above function for two players (one cop)
/*
void Dijkstra::push_end_states_on_queue() {
	QueueEntry qe;
	node_iterator nit;
	Graph *g = env->GetGraph();
	node *n;
	unsigned int i;

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );
	min_cost.assign( crg->GetNumStates(), DBL_MAX );
	max_cost.assign( crg->GetNumStates(), DBL_MAX );

	// loop through the graph
	nit = g->getNodeIter();
	n = g->nodeIterNext( nit );
	while( n != NULL ) {

		qe.pos.clear();
		for( i = 0; i <= number_of_cops; i++ ) {
			qe.pos.push_back( n->GetNum() );
		}
		qe.minFirst = false;
		qe.value = 0.; // Terminal Cost
		queue.push( qe );

		if( canPass ) {
			// set the value for the min costs (h_p)
			min_cost[ crg->GetNumberByState( qe.pos ) ] = 0.;
		} else {
			qe.minFirst = true;
			queue.push( qe );
		}

		n = g->nodeIterNext( nit );
	}
	
};
*/



double Dijkstra::MinGCost( CRState&, CRState& ) {
	return 1.;
};


void Dijkstra::WriteValuesToDisk( const char* filename ) {
	FILE *fhandler;

	unsigned int tns = crg->GetNumStates(), tnp = crg->GetNumPlayers(), i, j;
	CRState s;

	fhandler = fopen( filename, "w" );
	fprintf( fhandler, "number of players: %u\n", tnp );
	fprintf( fhandler, "states in space: %u\n", tns );
	fprintf( fhandler, "expected rewards if cops (player >= 1) move first:\n" );
	fprintf( fhandler, "\n\n" );
	fprintf( fhandler, "positions:\n" );
	for( i = 0; i < tnp; i++ ) {
		fprintf( fhandler, "player%u ", i );
	}
	fprintf( fhandler, "expected reward\n" );
	for( i = 0; i < tns; i++ ) {
		s = crg->GetStateByNumber( i );
		for( j = 0; j < tnp; j++ ) {
			fprintf( fhandler, "%lu ", s[j] );
//			fprintf( fhandler, "(%u,%u) ", env->GetGraph()->GetNode(s[j])->GetLabelL(GraphSearchConstants::kMapX),
//			env->GetGraph()->GetNode(s[j])->GetLabelL(GraphSearchConstants::kMapY));
		}
		fprintf( fhandler, "%g\n", min_cost[i] );
	}
	fclose( fhandler );
	return;
}
