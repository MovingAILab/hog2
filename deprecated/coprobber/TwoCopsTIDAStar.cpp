#include "TwoCopsTIDAStar.h"
#include "FloydWarshall.h"

TwoCopsTIDAStar::TwoCopsTIDAStar( GraphEnvironment *_env ):
	env(_env), numnodes(env->GetGraph()->GetNumNodes()),usePerfectDistanceHeuristic(false)
{
	if( numnodes * (numnodes+1)/2*numnodes > UINT32_MAX ) {
		fprintf( stderr, "ERROR: More nodes in graph than can be encoded in hash\n" );
		exit( 1 );
	}
};


/*------------------------------------------------------------------------------
| hash functions (copied from TwoCopsRMAStar.cpp)
------------------------------------------------------------------------------*/
TwoCopsTIDAStar::Position TwoCopsTIDAStar::CRHash_MemOptim( CRState &s ) {
	// sanity check of order
	assert( s[1] <= s[2] );

	return( s[0] * numnodes*(numnodes+1)/2 + s[1]*(s[1]+1)/2 + s[1]*(numnodes-s[1]-1) + s[2] );
};

void TwoCopsTIDAStar::MemOptim_Hash_To_CRState( Position &hash, const CRState &s ) {
	s[0] = hash / (numnodes*(numnodes+1)/2);
	Position h = hash % (numnodes*(numnodes+1)/2);

	s[1] = (unsigned int) floor( numnodes + 0.5 - sqrt( (numnodes+0.5)*(numnodes+0.5) - 2 * h ) );
	s[2] = h - s[1]*(s[1]+1)/2 - s[1]*(numnodes-s[1]-1);
	return;
};

/*------------------------------------------------------------------------------
| Neighbor generation (copied from TwoCopsRMAStar.cpp)
------------------------------------------------------------------------------*/
void TwoCopsTIDAStar::GetNeighbors( Position &pos, bool minFirst, std::set<Position> &neighbors ) {

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
| Bound Cache clearance
------------------------------------------------------------------------------*/
void TwoCopsTIDAStar::clear_bounds_cache() {
	min_lcache.clear();
	max_lcache.clear();
	min_ucache.clear();
	max_ucache.clear();
	return;
};

/*------------------------------------------------------------------------------
| TIDA* implementation
------------------------------------------------------------------------------*/
unsigned int TwoCopsTIDAStar::tida( graphState &r, graphState &c1, graphState &c2, bool minFirst ) {

	// put together the state
	CRState crpos;
	crpos[0] = r;
	if( c1 < c2 ) {
		crpos[1] = c1;
		crpos[2] = c2;
	} else {
		crpos[1] = c2;
		crpos[2] = c1;
	}
	Position pos = CRHash_MemOptim( crpos );

	unsigned int b, c = HCost( pos, minFirst );

	unsigned int sumNodesTouched = 0, sumNodesExpanded = 0;
	iteration_nodesExpanded.clear();
	iteration_nodesTouched.clear();

	do {
		nodesExpanded = 0; nodesTouched = 0;
		b = c;
		//fprintf( stdout, "set bound to b = %f\n", b );
		c = tida_update( pos, b, minFirst );

		iteration_nodesExpanded.push_back( nodesExpanded );
		iteration_nodesTouched.push_back( nodesTouched );
		sumNodesExpanded += nodesExpanded;
		sumNodesTouched  += nodesTouched;

	} while( c > b ); // until c <= b

	nodesExpanded = sumNodesExpanded;
	nodesTouched  = sumNodesTouched;

	assert( c == b );

	// cleanup
	clear_bounds_cache();
	return c;
}


/*------------------------------------------------------------------------------
| TIDA* subroutine
------------------------------------------------------------------------------*/
unsigned int TwoCopsTIDAStar::tida_update( Position &pos, unsigned int bound, bool minFirst )
{
	unsigned int result, temp;
	std::set<Position> neighbors;
	std::set<Position>::iterator it;
	Position neighbor;

	nodesTouched++;

	// goal test
	if( GoalTest( pos ) ) return 0;

	// upper bound cache lookup
	BoundCache::iterator hcit;
	BoundCache *current_bcache = minFirst?&min_ucache:&max_ucache;
	hcit = current_bcache->find( pos );
	if( hcit != current_bcache->end() ) {
		if( hcit->second <= bound ) return hcit->second;
	}

	// lower bound cache lookup
	current_bcache = minFirst?&min_lcache:&max_lcache;
	hcit = current_bcache->find( pos );
	if( hcit != current_bcache->end() ) {
		if( bound < hcit->second ) return hcit->second;
	} else {
		// heuristic pruning
		temp = HCost( pos, minFirst );
		if( bound < temp ) return temp;
	}

	GetNeighbors( pos, minFirst, neighbors );
	nodesExpanded++;

	// in case we are the cop/min player
	if( minFirst ) {

		result = UINT_MAX;

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			neighbor = *it;
			temp = 1 + tida_update( neighbor, bound - 1, !minFirst );

			if( result > temp ) result = temp; // result = min( result, temp );

			// alpha prune
			if( result <= bound ) break;
		}

	// in case we are the robber/max player
	} else {

		result = 0; // UINT_MIN

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			neighbor = *it;
			temp = 1 + tida_update( neighbor, bound - 1, !minFirst );

			if( result < temp ) result = temp; // result = max( result, temp );

			// beta pruning
			if( result > bound ) break;
		}

	}

	// update heuristic
	if( bound < result ) {
		current_bcache = minFirst?&min_lcache:&max_lcache;
		(*current_bcache)[pos] = result;
	} else {
		current_bcache = minFirst?&min_ucache:&max_ucache;
		(*current_bcache)[pos] = result;
	}

	return result;
};


bool TwoCopsTIDAStar::GoalTest(const  Position &pos ) {
	CRState crpos;
	MemOptim_Hash_To_CRState( pos, crpos );
	return( crpos[0] == crpos[1] || crpos[0] == crpos[2] );
};


unsigned int TwoCopsTIDAStar::HCost( Position &pos, bool &minFirst ) {

	CRState crpos;
	MemOptim_Hash_To_CRState( pos, crpos );

	// goal test - since we need crpos afterwards we do not call GoalTest here!
	if( crpos[0] == crpos[1] || crpos[0] == crpos[2] ) return 0;

	// compute the minimal distance from the cops to the robber
	double dist;
	if( usePerfectDistanceHeuristic ) {
		dist = min( distance_heuristic[crpos[0]][crpos[1]], distance_heuristic[crpos[0]][crpos[2]] );
	} else {
		dist = min( env->HCost(crpos[0],crpos[1]), env->HCost(crpos[0],crpos[2]) );
	}

	return( (unsigned int)floor( 2. * dist - (minFirst?1.:0.) ) );
}


void TwoCopsTIDAStar::set_usePerfectDistanceHeuristic( bool set ) {
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
