#include "MinimaxAStar.h"


// MINIMAX A*

MinimaxAStarMulti::MinimaxAStarMulti( GraphEnvironment *_env, unsigned int _number_of_cops, bool _canPass ):
	env(_env), number_of_cops(_number_of_cops), canPass(_canPass) {

	crg = new CopRobberGame( env, number_of_cops, false, canPass );
}

MinimaxAStarMulti::~MinimaxAStarMulti() {
	delete crg;
}


double MinimaxAStarMulti::astar( CRState goal_pos, bool goal_minFirst, double weight ) {
	QueueEntry qe, qtemp;
	MyClosedList::iterator mclit;

	nodesExpanded = 0; nodesTouched = 0;


	if( crg->GoalTest( goal_pos, goal_pos ) )
		return 0.;

	push_end_states_on_queue( goal_pos, goal_minFirst, weight );

	while( !queue.empty() ) {

		// get the element from the queue
		qe = queue.top(); queue.pop();

		nodesTouched++;

		// recursion break
		if( qe.pos == goal_pos && qe.minFirst == goal_minFirst ) {
			clear_cache();
			return qe.gvalue;
		}

		// verbose
		//fprintf( stdout, "minFirst = %d, pos = (%u,%u), fvalue = %f, gvalue = %f\n", qe.minFirst, qe.pos[0], qe.pos[1], qe.fvalue, qe.gvalue );

		if( qe.minFirst ) {

			mclit = min_cost.find( qe.pos );

			if( mclit != min_cost.end() ) {
				// if the gvalue for this position has been set yet
				// sanity check: check that the gvalue for the position is smaller than the current gvalue that
				// we expanded
				assert( mclit->second <= qe.gvalue );
			} else {
				nodesExpanded++;

				min_cost[qe.pos] = qe.gvalue;

				// the cops moved in this state, thus we have to find the
				// opposite actions of the robber here
				// note: this cannot be implemented with the CopRobberGame
				//   interface since that doesn't allow us to move away from the cops
				//   after capture (CopRobberOccupancy)
				std::vector<graphState> myneighbors;
				env->GetSuccessors( qe.pos[0], myneighbors );
				if( canPass )
					myneighbors.push_back( qe.pos[0] );

				qtemp.pos = qe.pos;

				// now, for all successor states
				for( std::vector<graphState>::iterator it = myneighbors.begin();
				     it != myneighbors.end(); it++ ) {
					nodesTouched++;
					// build the state
					qtemp.pos[0] = *it;


					// check whether its value is \infty
					if( max_cost.find( qtemp.pos ) == max_cost.end() ) {
						qtemp.gvalue = compute_target_value( qtemp.pos );
						if( qtemp.gvalue != DBL_MAX ) {
							qtemp.minFirst = !qe.minFirst; // !qe.minFirst == false
							qtemp.fvalue = qtemp.gvalue + weight * HCost( goal_pos, goal_minFirst, qtemp.pos, qtemp.minFirst );
							queue.push( qtemp );

							// verbose
							//fprintf( stdout, "pushing up: %d, (%u,%u), %f, %f\n", qtemp.minFirst, qtemp.pos[0], qtemp.pos[1], qtemp.fvalue, qtemp.gvalue );
						}
					}
				}
			}

		} else {

			mclit = max_cost.find( qe.pos );

			if( mclit != max_cost.end() ) {
				// sanity check: we should only set the value of a max position once
				//fprintf( stdout, "qe.pos = %u,%u ", qe.pos[0], qe.pos[1] );
				//fprintf( stdout, "max_cost[qe.pos] = %f, qe.gvalue = %f\n", mclit->second, qe.gvalue );
				assert( mclit->second == qe.gvalue );
			} else {
				nodesExpanded++;

				max_cost[qe.pos] = qe.gvalue;

				// the robber moved in this state, thus we have to find the
				// actions of the cops leading to this state
				std::vector<CRMove> cop_actions;
				crg->GetPossibleOpponentActions( 0, qe.pos, cop_actions );
				for( std::vector<CRMove>::iterator it = cop_actions.begin();
				     it != cop_actions.end(); it++ ) {

					nodesTouched++;

					// build the next state
					qtemp.pos = qe.pos;
					// the robber sits out - this should actually not be neccessary because CopRobberGame takes care of it
					(*it)[0] = CRAction();
					crg->ApplyAction( qtemp.pos, *it );

					// check agains infinity
					if( min_cost.find( qtemp.pos ) == min_cost.end() ) {
						qtemp.minFirst = !qe.minFirst; // !qe.minFirst == true
						qtemp.gvalue   = qe.gvalue + MinGCost( qe.pos, qtemp.pos );
						qtemp.fvalue   = qtemp.gvalue + weight * HCost( goal_pos, goal_minFirst, qtemp.pos, qtemp.minFirst );
						queue.push( qtemp );

						// verbose
						//fprintf( stdout, "pushing up: %d, (%u,%u), %f, %f\n", qtemp.minFirst, qtemp.pos[0], qtemp.pos[1], qtemp.fvalue, qtemp.gvalue );
					}
				}
			}
		}

	}

	clear_cache();
	return DBL_MAX;
}

double MinimaxAStarMulti::compute_target_value( CRState &s ) {
	double result = 0.;

	CRState temp = s;
	double tempvalue;
	MyClosedList::iterator mclit;
	std::vector<graphState> myneighbors;
	env->GetSuccessors( s[0], myneighbors );
	if( canPass )
		myneighbors.push_back( s[0] );
	//nodesExpanded++;

	// now, for all successor states
	for( std::vector<graphState>::iterator it = myneighbors.begin();
	     it != myneighbors.end(); it++ ) {
		//nodesTouched++;
	
		// build the state
		temp[0] = *it;
		mclit = min_cost.find( temp );
		if( mclit != min_cost.end() )
			tempvalue = mclit->second + MinGCost( temp, s );
		else
			return DBL_MAX;
		if( tempvalue > result ) result = tempvalue;
	}
	return result;
}

// this function loops through all possible states for now
// instead of generating the beginning states directly from the map
void MinimaxAStarMulti::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst, double &weight ) {
	QueueEntry qe;
	CRState pos;

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );
	// get the number of joint states
	unsigned int num_states = crg->GetNumStates();
	// for all states
	for( unsigned int i = 0; i < num_states; i++ ) {

		pos = crg->GetStateByNumber( i );
		nodesTouched++;

		if( crg->GoalTest( pos, pos ) ) {

			min_cost[pos] = 0.;
			max_cost[pos] = 0.;

			// now push all the states on the queue that are one step away
			// from the goal
			//
			// if cop moves last
			std::vector<CRMove> cop_actions;
			crg->GetPossibleOpponentActions( 0, pos, cop_actions );
			nodesExpanded++;

			for( std::vector<CRMove>::iterator it = cop_actions.begin();
			     it != cop_actions.end(); it++ ) {

				nodesTouched++;

				// build the next state
				qe.pos = pos;
				// the robber sits out - this should actually not be neccessary because CopRobberGame takes care of it
				(*it)[0] = CRAction();
				crg->ApplyAction( qe.pos, *it );
				qe.minFirst = true;
				qe.gvalue   = MinGCost( qe.pos, pos );
				qe.fvalue   = qe.gvalue + weight * HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
				queue.push( qe );

				// verbose
				//fprintf( stdout, "pushing up: %d, (%u,%u), %f, %f\n", qe.minFirst, qe.pos[0], qe.pos[1], qe.fvalue, qe.gvalue );
			}
		}
	}

	if( !canPass ) {
		// In the case where players cannot pass their turn, we also have to consider the end states where the
		// cop moves last
		for( unsigned int i = 0; i < num_states; i++ ) {

			pos = crg->GetStateByNumber( i );
			nodesTouched++;

			if( crg->GoalTest( pos, pos ) ) {

				std::vector<graphState> myneighbors;
				env->GetSuccessors( pos[0], myneighbors );
				nodesExpanded++;

				qe.pos = pos;

				// now, for all successor states
				for( std::vector<graphState>::iterator it = myneighbors.begin();
				     it != myneighbors.end(); it++ ) {
					nodesTouched++;
					// build the state
					qe.pos[0] = *it;

					qe.gvalue = compute_target_value( qe.pos );
					if( qe.gvalue != DBL_MAX ) {
						qe.minFirst = false;
						qe.fvalue = qe.gvalue + weight * HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
						queue.push( qe );

						// verbose
						//fprintf( stdout, "pushing up: %d, (%u,%u), %f, %f\n", qe.minFirst, qe.pos[0], qe.pos[1], qe.fvalue, qe.gvalue );
					}
				}
			}
		}
	}
	return;
}


/*
// this version is a faster variant of the above function for two players (one cop)
void MinimaxAStarMulti::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst, double &weight ) {
	QueueEntry qe;
	CRState pos;
	std::vector<graphState> neighbors;
	node_iterator nit;
	Graph *g = env->GetGraph();
	node *n;
	unsigned int i;

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );

	// loop through the graph
	nit = g->getNodeIter();
	n = g->nodeIterNext( nit );
	while( n != NULL ) {

		nodesExpanded++;nodesTouched++;

		pos.clear();
		for( i = 0; i < 2; i++ ) {
			pos.push_back( n->GetNum() );
		}

		min_cost[pos] = 0.;
		max_cost[pos] = 0.;

		// now push all the states on the queue that are one step away
		// from the goal
		//
		// if cop moves last
		env->GetSuccessors( pos[1], neighbors );
		qe.pos = pos;
		qe.minFirst = true;
		for( std::vector<graphState>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;

				qe.pos[1] = *it;
				qe.gvalue   = MinGCost( qe.pos, pos );
				qe.fvalue   = qe.gvalue + weight * HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
				queue.push( qe );
		}

		n = g->nodeIterNext( nit );
	}
	
};
*/


double MinimaxAStarMulti::MinGCost( CRState&, CRState& ) {
	return 1.;
};


// note: this HCost implementation relies on all edge costs 1 and MaximumNormGraphMapHeuristic in the GraphEnvironment
// furthermore, it only makes sense with the above definition of MinGCost===1
double MinimaxAStarMulti::HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 ) {
	double hmax = env->HCost( pos1[0], pos2[0] );
	double hmin = 0.;
	double h;

	for( unsigned int i = 1; i < pos1.size(); i++ ) {
		h = env->HCost( pos1[i], pos2[i] );
		hmin = max( hmin, h );
	}

	if( minFirst1 == minFirst2 )
		return( 2. * max(hmax,hmin) );

	if( hmax == hmin )
		return( 2. * hmax + 1. );

	if( hmax < hmin ) {
		// robber has less way to go then one of the cops

		if( minFirst1 && !minFirst2 ) // cops starts and ends
			return( 2. * hmin - 1. );
		else // !minFirst1 && minFirst2 // robber starts and ends
			return( 2. * hmin + 1. );
	} else {
		// robber has more way to go then all of the cops
		if( minFirst1 && !minFirst2 )
			return( 2. * hmax + 1. );
		else
			return( 2. * hmax - 1. );
	}

}


void MinimaxAStarMulti::clear_cache() {
	min_cost.clear();
	max_cost.clear();
	queue = MyPriorityQueue();
	return;
};
