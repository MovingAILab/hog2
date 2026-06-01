#include "TwoCopsDijkstra2.h"
#include <math.h>

/*------------------------------------------------------------------------------
| Constructor
------------------------------------------------------------------------------*/
TwoCopsDijkstra2::TwoCopsDijkstra2( GraphEnvironment *_env ):
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
TwoCopsDijkstra2::Position TwoCopsDijkstra2::CRHash_MemOptim( CRState &s ) {
	// sanity check of order
	assert( s[1] <= s[2] );

	return( s[0] * numnodes*(numnodes+1)/2 + s[1]*(s[1]+1)/2 + s[1]*(numnodes-s[1]-1) + s[2] );

	//return( s[0] * numnodes * numnodes + s[1] * numnodes + s[2] );
};

void TwoCopsDijkstra2::MemOptim_Hash_To_CRState( Position &hash, CRState &s ) {
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
void TwoCopsDijkstra2::GetNeighbors( Position &pos, bool minFirst, NeighborSet &neighbors ) {

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
		float gcost = 0.;

		// loop over neighbors of cop 1
		for( std::vector<graphState>::iterator it1 = cop1_neighbors.begin();
		     it1 != cop1_neighbors.end(); it1++ ) {
			
			// loop over neighbors of cop 2
			for( std::vector<graphState>::iterator it2 = cop2_neighbors.begin();
			     it2 != cop2_neighbors.end(); it2++ ) {

				crpos[1] = *it1;
				crpos[2] = *it2;

				// determine the gcost of this move
				gcost = env->GCost( crpos[1], backup_cop1 ) + env->GCost( crpos[2], backup_cop2 );

				// order the cops' positions
				if( crpos[1] > crpos[2] ) {
					graphState temp = crpos[2];
					crpos[2] = crpos[1];
					crpos[1] = temp;
				}
				neighbors.insert( std::pair<Position,float>(CRHash_MemOptim( crpos ),gcost) );
			}

			// the second cop could also just pass
			crpos[1] = *it1;
			crpos[2] = backup_cop2;
			gcost = env->GCost( crpos[1], backup_cop1 );
			if( crpos[1] > crpos[2] ) { graphState temp = crpos[2]; crpos[2] = crpos[1]; crpos[1] = temp; }
			neighbors.insert( std::pair<Position,float>( CRHash_MemOptim( crpos ), gcost ) );
		}

		// or the first player passes
		// that only makes sense though when they're not at the same position
		if( backup_cop1 != backup_cop2 ) {
			for( std::vector<graphState>::iterator it2 = cop2_neighbors.begin();
			     it2 != cop2_neighbors.end(); it2++ ) {

				crpos[1] = backup_cop1;
				crpos[2] = *it2;
				gcost = env->GCost( crpos[2], backup_cop2 );
				if( crpos[1] > crpos[2] ) { graphState temp = crpos[2]; crpos[2] = crpos[1]; crpos[1] = temp; }
				neighbors.insert( std::pair<Position,float>( CRHash_MemOptim( crpos ), gcost ) );
			}
		}


	} else {
		// move generation for the robber
		
		// the robber can pass
		neighbors.insert( std::pair<Position,float>( pos, 0. ) );
		// or move to an adjacent vertice
		std::vector<graphState> robber_neighbors;
		env->GetSuccessors( crpos[0], robber_neighbors );
		for( std::vector<graphState>::iterator it = robber_neighbors.begin(); it != robber_neighbors.end(); it++ ) {
			crpos[0] = *it;
			// we do not count the robber's move
			neighbors.insert( std::pair<Position,float>( CRHash_MemOptim( crpos ), 0. ) );
		}
	}
	return;
};



/*------------------------------------------------------------------------------
| Dijkstra implementation
------------------------------------------------------------------------------*/
void TwoCopsDijkstra2::dijkstra() {
	nodesExpanded = 0; nodesTouched = 0;
	QueueEntry qe;
	NeighborSet neighbors;
	NeighborSet::iterator it;

	// cop1 and cop2 can have maximally numnodes*(numnodes+1)/2 joint positions
	// robber can be on any vertex => numnodes
	min_cost.assign( numnodes * (numnodes + 1) / 2 * numnodes, FLT_MAX );
	max_cost.assign( numnodes * (numnodes + 1) / 2 * numnodes, FLT_MAX );

	push_end_states_on_queue();

	while( !queue.empty() ) {

		// pop
		qe = queue.top();
		queue.pop();
		nodesTouched++;

		if( qe.minFirst ) {
			// it is the cop's turn in qe

			// if value isn't set yet
			if( min_cost[qe.pos] == FLT_MAX ) {
				min_cost[qe.pos] = qe.value;
				// find all the positions the robber could have come from
				GetNeighbors( qe.pos, false, neighbors );
				nodesExpanded++;
				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = (*it).first;
					if( max_cost[qe.pos] == FLT_MAX ) {
						qe.value = compute_target_value( qe.pos );
						if( qe.value != FLT_MAX ) {
							qe.minFirst = false;
							queue.push( qe );
						}
					}
				} // for all previous robber positions

			}

		} else {
			// it is the robber's turn in qe

			// if value isn't set yet
			if( max_cost[qe.pos] == FLT_MAX ) {
				max_cost[qe.pos] = qe.value;
				float backup_value = qe.value;
				// find all the positions the cops could have come from
				GetNeighbors( qe.pos, true, neighbors );
				nodesExpanded++;
				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qe.pos = (*it).first;
					if( min_cost[qe.pos] == FLT_MAX ) {
						qe.minFirst = true;
						qe.value = backup_value + (*it).second;
						queue.push( qe );
					}
				}

			}

		} // minFirst
	} // while loop

	return;
};



float TwoCopsDijkstra2::compute_target_value( Position &pos ) {

	float result = 0.;
	float temp;
	NeighborSet neighbors;

	// get the possible positions the robber could move to
	GetNeighbors( pos, false, neighbors );
	//nodesExpanded++;
	for( NeighborSet::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
		//nodesTouched++;
		temp = min_cost[(*it).first];
		if( temp == FLT_MAX ) return FLT_MAX;
		temp += (*it).second;
		result = max( result, temp );
	}
	return result;
};



void TwoCopsDijkstra2::push_end_states_on_queue() {
	QueueEntry qe;
	CRState crpos;
	Position pos;
	NeighborSet neighbors;
	NeighborSet::iterator it;

	// sanity check
	assert( queue.empty() );

	// set initial values
	qe.minFirst = true;
	qe.value    = 0.;

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
				qe.pos = (*it).first;
				qe.value = (*it).second;
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
					qe.pos = (*it).first;
					qe.value = (*it).second;
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
bool TwoCopsDijkstra2::is_two_cop_win() {
	// in case dijkstra hasn't been computed yet
	if( min_cost.empty() ) dijkstra();

	for( ClosedList::iterator it = min_cost.begin(); it != min_cost.end(); it++ ) {
		if( *it == FLT_MAX )
			return false;
	}
	return true;
};

float TwoCopsDijkstra2::Value( graphState &r, graphState &c1, graphState &c2 ) {

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
void TwoCopsDijkstra2::WriteValuesToDisk( const char* filename ) {
	FILE *fhandler;

	CRState pos;

	fhandler = fopen( filename, "w" );
	for( unsigned int r = 0; r < numnodes; r++ ) {
		pos[0] = r;
		for( unsigned int c1 = 0; c1 < numnodes; c1++ ) {
			//for( unsigned int c2 = 0; c2 < numnodes; c2++ ) {
			// Isaza comparison
			for( unsigned int c2 = c1; c2 < numnodes; c2++ ) {
				pos[1] = c1;
				pos[2] = c2;

				//fprintf( fhandler, "%u %u %u %u\n", r, c1, c2, min_cost[CRHash_MemOptim( pos )] );

				// Isaza comparison
				node *rnode  = env->GetGraph()->GetNode( r );
				node *c1node = env->GetGraph()->GetNode( c1 );
				node *c2node = env->GetGraph()->GetNode( c2 );
				unsigned int h = CRHash_MemOptim( pos );
				fprintf( fhandler, "(%ld,%ld) (%ld,%ld) (%ld,%ld) %g %g\n",
					rnode->GetLabelL(GraphSearchConstants::kMapX), rnode->GetLabelL(GraphSearchConstants::kMapY),
					c1node->GetLabelL(GraphSearchConstants::kMapX), c1node->GetLabelL(GraphSearchConstants::kMapY),
					c2node->GetLabelL(GraphSearchConstants::kMapX), c2node->GetLabelL(GraphSearchConstants::kMapY),
					max_cost[h], min_cost[h] );
			}
		}
	}
	fclose( fhandler );
	return;
};
