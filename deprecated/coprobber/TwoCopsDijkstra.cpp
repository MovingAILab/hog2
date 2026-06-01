#include "TwoCopsDijkstra.h"
#include <math.h>

/*------------------------------------------------------------------------------
| Constructor
------------------------------------------------------------------------------*/
TwoCopsDijkstra::TwoCopsDijkstra( GraphEnvironment *_env ):
	env(_env), numnodes( env->GetGraph()->GetNumNodes() )
{
	if( numnodes*numnodes*numnodes > UINT32_MAX ) {
		fprintf( stderr, "ERROR: More nodes in graph than can be encoded in hash\n" );
		exit( 1 );
	}
};


/*------------------------------------------------------------------------------
| Hashing
------------------------------------------------------------------------------*/
TwoCopsDijkstra::Position TwoCopsDijkstra::CRHash_MemOptim( CRState &s ) {
	// sanity check of order
	assert( s[1] <= s[2] );

	return( s[0] * numnodes*(numnodes+1)/2 + s[1]*(s[1]+1)/2 + s[1]*(numnodes-s[1]-1) + s[2] );

	//return( s[0] * numnodes * numnodes + s[1] * numnodes + s[2] );
};

void TwoCopsDijkstra::MemOptim_Hash_To_CRState( Position &hash, CRState &s ) {
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
void TwoCopsDijkstra::GetNeighbors( Position &pos, bool minFirst, std::set<Position> &neighbors ) {

	// clear the arguments
	neighbors.clear();

	// dehash the position
	CRState crpos;
	MemOptim_Hash_To_CRState( pos, crpos );

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
void TwoCopsDijkstra::dijkstra() {
	nodesExpanded = 0; nodesTouched = 0;
	QueueEntry qe;
	std::set<Position> neighbors;
	std::set<Position>::iterator it;

	// cop1 and cop2 can have maximally numnodes*(numnodes+1)/2 joint positions
	// robber can be on any vertex => numnodes
	min_cost.assign( numnodes * (numnodes + 1) / 2 * numnodes, UINT_MAX );
	max_cost.assign( numnodes * (numnodes + 1) / 2 * numnodes, UINT_MAX );
	//min_cost.assign( numnodes * numnodes * numnodes, UINT_MAX );
	//max_cost.assign( numnodes * numnodes * numnodes, UINT_MAX );

	push_end_states_on_queue();

	while( !queue.empty() ) {

		// pop
		qe = queue.front();
		queue.pop();
		nodesTouched++;

		if( qe.minFirst ) {
			// it is the cop's turn in qe

			// if value isn't set yet
			if( min_cost[qe.pos] == UINT_MAX ) {
				min_cost[qe.pos] = qe.value;
				// find all the positions the robber could have come from
				GetNeighbors( qe.pos, false, neighbors );
				nodesExpanded++;
				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = *it;
					if( max_cost[qe.pos] == UINT_MAX ) {
						qe.value = compute_target_value( qe.pos );
						if( qe.value != UINT_MAX ) {
							qe.minFirst = false;
							queue.push( qe );
						}
					}
				} // for all previous robber positions

			}

		} else {
			// it is the robber's turn in qe

			// if value isn't set yet
			if( max_cost[qe.pos] == UINT_MAX ) {
				max_cost[qe.pos] = qe.value;
				unsigned int backup_value = qe.value;
				// find all the positions the cops could have come from
				GetNeighbors( qe.pos, true, neighbors );
				nodesExpanded++;
				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = *it;
					if( min_cost[qe.pos] == UINT_MAX ) {
						qe.minFirst = true;
						qe.value = backup_value + 1;
						queue.push( qe );
					}
				}

			}

		} // minFirst
	} // while loop

	return;
};



unsigned int TwoCopsDijkstra::compute_target_value( Position &pos ) {

	unsigned int result = 0;
	unsigned int temp;
	std::set<Position> neighbors;

	// get the possible positions the robber could move to
	GetNeighbors( pos, false, neighbors );
	//nodesExpanded++;
	for( std::set<Position>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
		//nodesTouched++;
		temp = min_cost[*it];
		if( temp == UINT_MAX ) return UINT_MAX;
		temp += 1;
		if( temp > result ) result = temp;
	}
	return result;
};



void TwoCopsDijkstra::push_end_states_on_queue() {
	QueueEntry qe;
	CRState crpos;
	Position pos;
	std::set<Position> neighbors;
	std::set<Position>::iterator it;

	// sanity check
	assert( queue.empty() );

	// set initial values
	qe.minFirst = true;
	qe.value    = 1;

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
			min_cost[pos] = 0;
			max_cost[pos] = 0; // shouldn't be necessary

			// neighbors
			GetNeighbors( pos, true, neighbors );
			nodesExpanded++;
			for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;
				qe.pos = *it;
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
				min_cost[pos] = 0;
				max_cost[pos] = 0; // shouldn't be necessary

				// neighbors
				GetNeighbors( pos, true, neighbors );
				nodesExpanded++;
				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = *it;
					queue.push( qe );
				}
			}

		} // for loop cop2
	} // for loop cop1

	return;
};

/*------------------------------------------------------------------------------
| Cop win? and access to the computed values
------------------------------------------------------------------------------*/
bool TwoCopsDijkstra::is_two_cop_win() {
	// in case dijkstra hasn't been computed yet
	if( min_cost.empty() ) dijkstra();

	for( ClosedList::iterator it = min_cost.begin(); it != min_cost.end(); it++ ) {
		if( *it == UINT_MAX )
			return false;
	}
	return true;
};

unsigned int TwoCopsDijkstra::Value( graphState &r, graphState &c1, graphState &c2 ) {

	assert( !min_cost.empty() );
	// failsafe
	if( min_cost.empty() ) return 0;

	CRState pos;
	pos[0] = r;
	pos[1] = c1;
	pos[2] = c2;
	if( pos[1] > pos[2] ) {
		graphState temp = pos[2];
		pos[2] = pos[1];
		pos[1] = temp;
	}
	return min_cost[CRHash_MemOptim( pos )];
};

/*------------------------------------------------------------------------------
| Output
------------------------------------------------------------------------------*/
void TwoCopsDijkstra::WriteValuesToDisk( const char* filename ) {
	FILE *fhandler;

	CRState pos;

	fhandler = fopen( filename, "w" );
	for( unsigned int r = 0; r < numnodes; r++ ) {
		pos[0] = r;
		for( unsigned int c1 = 0; c1 < numnodes; c1++ ) {
			for( unsigned int c2 = 0; c2 < numnodes; c2++ ) {
				pos[1] = c1;
				pos[2] = c2;
				if( pos[1] > pos[2] ) { graphState temp = pos[2]; pos[2] = pos[1]; pos[1] = temp; };

				fprintf( fhandler, "%u %u %u %u\n", r, c1, c2, min_cost[CRHash_MemOptim( pos )] );
			}
		}
	}
	fclose( fhandler );
	return;
};
