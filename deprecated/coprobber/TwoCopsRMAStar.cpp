#include "TwoCopsRMAStar.h"
#include <math.h>
#include "FloydWarshall.h"

/*------------------------------------------------------------------------------
| Constructor
------------------------------------------------------------------------------*/
TwoCopsRMAStar::TwoCopsRMAStar( GraphEnvironment *_env ):
	env(_env), numnodes( env->GetGraph()->GetNumNodes() ), useHeuristic( true ), usePerfectDistanceHeuristic( false )
{
	if( numnodes * (numnodes + 1) / 2 * numnodes > UINT32_MAX ) {
		fprintf( stderr, "ERROR: More nodes in graph than can be encoded in hash\n" );
		exit( 1 );
	}
};


/*------------------------------------------------------------------------------
| Hashing (copied from TwoCopsDijkstra)
------------------------------------------------------------------------------*/
TwoCopsRMAStar::Position TwoCopsRMAStar::CRHash_MemOptim( CRState &s ) {
	// sanity check of order
	assert( s[1] <= s[2] );

	return( s[0] * numnodes*(numnodes+1)/2 + s[1]*(s[1]+1)/2 + s[1]*(numnodes-s[1]-1) + s[2] );

	//return( s[0] * numnodes * numnodes + s[1] * numnodes + s[2] );
};

void TwoCopsRMAStar::MemOptim_Hash_To_CRState( Position &hash, CRState &s ) {
	s[0] = hash / (numnodes*(numnodes+1)/2);
	Position h = hash % (numnodes*(numnodes+1)/2);

	s[1] = (unsigned int) floor( numnodes + 0.5 - sqrt( (numnodes+0.5)*(numnodes+0.5) - 2 * h ) );
	s[2] = h - s[1]*(s[1]+1)/2 - s[1]*(numnodes-s[1]-1);
	//s[0] = hash / (numnodes * numnodes);
	//s[1] = (hash % numnodes) / numnodes;
	//s[2] = hash % numnodes;
	return;
};

/*------------------------------------------------------------------------------
| Neighbor generation
------------------------------------------------------------------------------*/
void TwoCopsRMAStar::GetNeighbors( Position &pos, bool minFirst, std::set<Position> &neighbors ) {

	// clear the arguments
	neighbors.clear();

	// dehash the position
	CRState crpos;
	MemOptim_Hash_To_CRState( pos, crpos );

	// sanity checks
	assert( crpos[0] < numnodes );
	assert( crpos[1] < numnodes );
	assert( crpos[2] < numnodes );

	if( minFirst ) {
		// move generation for the cops

		// backup copy from original position
		graphState backup_cop1 = crpos[1];
		graphState backup_cop2 = crpos[2];

		// find the neighbors for both cops
		std::vector<graphState> cop1_neighbors, cop2_neighbors;
		env->GetSuccessors( crpos[1], cop1_neighbors );
		env->GetSuccessors( crpos[2], cop2_neighbors );

		// loop over neighbors of cop 1
		for( std::vector<graphState>::iterator it1 = cop1_neighbors.begin();
		     it1 != cop1_neighbors.end(); it1++ ) {
			
			// loop over neighbors of cop 2
			for( std::vector<graphState>::iterator it2 = cop2_neighbors.begin();
			     it2 != cop2_neighbors.end(); it2++ ) {

				crpos[1] = *it1;
				crpos[2] = *it2;
				// order the cops' positions
				if( crpos[1] > crpos[2] ) {
					graphState temp = crpos[2];
					crpos[2] = crpos[1];
					crpos[1] = temp;
				}
				neighbors.insert( CRHash_MemOptim( crpos ) );
			}

			// the second cop could also just pass
			crpos[1] = *it1;
			crpos[2] = backup_cop2;
			if( crpos[1] > crpos[2] ) { graphState temp = crpos[2]; crpos[2] = crpos[1]; crpos[1] = temp; }
			neighbors.insert( CRHash_MemOptim( crpos ) );
		}

		// or the first player passes
		// that only makes sense though when they're not at the same position
		if( backup_cop1 != backup_cop2 ) {
			for( std::vector<graphState>::iterator it2 = cop2_neighbors.begin();
			     it2 != cop2_neighbors.end(); it2++ ) {

				crpos[1] = backup_cop1;
				crpos[2] = *it2;
				if( crpos[1] > crpos[2] ) { graphState temp = crpos[2]; crpos[2] = crpos[1]; crpos[1] = temp; }
				neighbors.insert( CRHash_MemOptim( crpos ) );
			}
		}


	} else {
		// move generation for the robber
		
		// the robber can pass
		neighbors.insert( pos );
		// or move to an adjacent vertice
		std::vector<graphState> robber_neighbors;
		env->GetSuccessors( crpos[0], robber_neighbors );
		for( std::vector<graphState>::iterator it = robber_neighbors.begin(); it != robber_neighbors.end(); it++ ) {
			crpos[0] = *it;
			neighbors.insert( CRHash_MemOptim( crpos ) );
		}
	}
	return;
};


/*------------------------------------------------------------------------------
| Dijkstra implementation
------------------------------------------------------------------------------*/
/*
void TwoCopsRMAStar::AddToOpenList( QueueEntry &qe ) {
	if( queue.IsIn( qe ) ) {
		QueueEntry qold = queue.find( qe );
		if( qold.fvalue > qe.fvalue )
			queue.DecreaseKey( qe );
	} else
		queue.Add( qe );
	return;
};
*/


unsigned int TwoCopsRMAStar::rmastar( graphState &r, graphState &c1, graphState &c2, bool minFirst ) {

	nodesExpanded = 0;
	nodesTouched  = 0;

	// check for goal
	if( r == c1 || r == c2 ) return 0;

	// generate goal position
	current_goal_minFirst = minFirst;
	current_goal[0] = r;
	if( c1 < c2 ) {
		current_goal[1] = c1;
		current_goal[2] = c2;
	} else {
		current_goal[1] = c2;
		current_goal[2] = c1;
	}
	Position goal_pos = CRHash_MemOptim( current_goal );



	QueueEntry qe;
	MyClosedList::iterator mclit;
	std::set<Position> neighbors;
	std::set<Position>::iterator it;

	push_end_states_on_queue();

	while( !queue.empty() ) {

		// pop
		qe = queue.top();queue.pop();
		nodesTouched++;

		if( qe.pos == goal_pos && qe.minFirst == current_goal_minFirst ) {
			clear_cache();
			return qe.gvalue;
		}

		if( qe.minFirst ) {
			// it is the cop's turn in qe

			mclit = min_cost.find( qe.pos );

			// if value isn't set yet
			if( mclit == min_cost.end() || (mclit != min_cost.end() && mclit->second > qe.gvalue) ) {

				min_cost[qe.pos] = qe.gvalue;
				// find all the positions the robber could have come from
				GetNeighbors( qe.pos, false, neighbors );
				nodesExpanded++;

				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = *it;
					qe.gvalue = compute_target_value( qe.pos );

					if( qe.gvalue != UINT_MAX ) {

						mclit = max_cost.find( qe.pos );

						if( mclit == max_cost.end() || (mclit != max_cost.end() && mclit->second > qe.gvalue) ) {
							qe.minFirst = false;
							qe.fvalue = qe.gvalue + HCost( qe.pos, qe.minFirst );
							//AddToOpenList( qe );
							queue.push( qe );
						}
					}
				} // for all previous robber positions

			}

		} else {
			// it is the robber's turn in qe

			mclit = max_cost.find( qe.pos );

			//assert( mclit == max_cost.end() || (mclit != max_cost.end() && mclit->second >= qe.gvalue) );

			// if value isn't set yet
			if( mclit == max_cost.end() || (mclit != max_cost.end() && mclit->second > qe.gvalue ) ) {

				max_cost[qe.pos] = qe.gvalue;
				unsigned int backup_value = qe.gvalue;
				// find all the positions the cops could have come from
				GetNeighbors( qe.pos, true, neighbors );
				nodesExpanded++;

				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = *it;
					qe.gvalue = backup_value + 1;
					mclit = min_cost.find( qe.pos );
					if( mclit == min_cost.end() || (mclit != min_cost.end() && mclit->second > qe.gvalue ) ) {
						qe.minFirst = true;
						qe.fvalue   = qe.gvalue + HCost( qe.pos, qe.minFirst );
						//AddToOpenList( qe );
						queue.push( qe );
					}
				}

			}

		} // minFirst
	} // while loop

	clear_cache();

	return UINT_MAX;
};



unsigned int TwoCopsRMAStar::compute_target_value( Position &pos ) {

	unsigned int result = 0;
	unsigned int temp;
	MyClosedList::iterator mclit;
	std::set<Position> neighbors;

	// get the possible positions the robber could move to
	GetNeighbors( pos, false, neighbors );
	//nodesExpanded++;

	for( std::set<Position>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
		//nodesTouched++;

		mclit = min_cost.find( *it );
		if( mclit == min_cost.end() ) return UINT_MAX;

		temp = mclit->second + 1;
		if( temp > result ) result = temp;
	}
	return result;
};



void TwoCopsRMAStar::push_end_states_on_queue() {
	QueueEntry qe;
	CRState crpos;
	Position pos;
	std::set<Position> neighbors;
	std::set<Position>::iterator it;

	// sanity check
	assert( queue.empty() );

	// set initial values
	qe.minFirst = true;
	qe.gvalue    = 1;

	// loop through the graph nodes - cop1
	for( unsigned int i = 0; i < numnodes; i++ ) {
		// loop through the rest of the graph nodes - cop2
		for( unsigned int j = i; j < numnodes; j++ ) {
			nodesTouched++;

			// case 1: robber is caught under cop1
			crpos[0] = i;
			crpos[1] = i;
			crpos[2] = j;
			pos = CRHash_MemOptim( crpos );
			min_cost[pos] = 0; // shouldn't be neccessary
			max_cost[pos] = 0;

			// neighbors
			GetNeighbors( pos, true, neighbors );
			nodesExpanded++;
			for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;
				qe.pos = *it;
				qe.fvalue = qe.gvalue + HCost( qe.pos, qe.minFirst );
				//AddToOpenList( qe );
				queue.push( qe );
			}

			// case 2: robber is caught under cop2
			// if cop1 and cop2 are at same position this is not a new case
			if( i != j ) {
				nodesTouched++;
				crpos[0] = j;
				crpos[1] = i;
				crpos[2] = j;
				pos = CRHash_MemOptim( crpos );
				min_cost[pos] = 0; // shouldn't be neccessary
				max_cost[pos] = 0;

				// neighbors
				GetNeighbors( pos, true, neighbors );
				nodesExpanded++;
				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = *it;
					qe.fvalue = qe.gvalue + HCost( qe.pos, qe.minFirst );
					//AddToOpenList( qe );
					queue.push( qe );
				}
			}

		} // for loop cop2
	} // for loop cop1

	return;
};


unsigned int TwoCopsRMAStar::HCost( Position &pos, bool &minFirst ) {

	if( !useHeuristic ) return 0;

	// get the underlying positions for each agent
	CRState crpos;
	MemOptim_Hash_To_CRState( pos, crpos );

	double hmax, hmin;

	if( usePerfectDistanceHeuristic ) {
		hmax = distance_heuristic[current_goal[0]][crpos[0]];
		double cost11 = distance_heuristic[current_goal[1]][crpos[1]];
		double cost12 = distance_heuristic[current_goal[1]][crpos[2]];
		double cost21 = distance_heuristic[current_goal[2]][crpos[1]];
		double cost22 = distance_heuristic[current_goal[2]][crpos[2]];
		hmin = min( max( cost11, cost22 ), max( cost12, cost21 ) );
	} else {
		hmax = env->HCost( current_goal[0], crpos[0] );
		double cost11 = env->HCost( current_goal[1],crpos[1] );
		double cost12 = env->HCost( current_goal[1],crpos[2] );
		double cost21 = env->HCost( current_goal[2],crpos[1] );
		double cost22 = env->HCost( current_goal[2],crpos[2] );
		hmin = min( max( cost11, cost22 ), max( cost12, cost21 ) );
	}

	if( current_goal_minFirst == minFirst )
		return( (unsigned int) floor( 2. * max( hmax, hmin ) ) );
	
	if( fequal( hmax, hmin ) )
		return( (unsigned int) floor( 2. * hmax + 1. ) );

	if( hmax < hmin ) {
		// robber has less way to go then one of the cops
		if( current_goal_minFirst && !minFirst ) // cops start and end
			return( (unsigned int) floor( 2. * hmin - 1. ) );
		else // robber starts and ends (recall that current_goal_minFirst == minFirst has been dealt with already)
			return( (unsigned int) floor( 2. * hmin + 1. ) );
	} else {
		// robber has more way to go than all the cops (hmax==hmin has been dealt with already)
		if( current_goal_minFirst && !minFirst )
			return( (unsigned int) floor( 2. * hmax + 1. ) );
		else
			return( (unsigned int) floor( 2. * hmax - 1. ) );
	}
};



void TwoCopsRMAStar::clear_cache() {
	min_cost.clear();
	max_cost.clear();
	queue = MyPriorityQueue();
	return;
};

void TwoCopsRMAStar::set_usePerfectDistanceHeuristic( bool set ) {
	usePerfectDistanceHeuristic = set;

	if( set ) {
		Graph *g = env->GetGraph();
		edge_iterator eit = g->getEdgeIter();
		edge *e = g->edgeIterNext( eit );
		while( e ) {
			e->setWeight( 1. );
			e = g->edgeIterNext( eit );
		}
		FloydWarshall( g, distance_heuristic );
	}
};
