#include "DSDijkstra.h"

template<>
void DSDijkstra<graphState,graphMove,GraphEnvironment>::push_end_states_on_queue() {
	QueueEntry qe;
	CRState pos;
	std::vector<graphState> neighbors;
	node_iterator nit;
	Graph *g = env->GetGraph();
	node *n;

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );

	// loop through the graph
	nit = g->getNodeIter();
	n = g->nodeIterNext( nit );
	while( n != NULL ) {

		nodesExpanded++;nodesTouched++;

		pos.clear();
		pos.push_back( n->GetNum() );
		pos.push_back( n->GetNum() );

		min_cost[pos] = 0.;
		max_cost[pos] = 0.;

		// now push all the states on the queue that are one step away
		// from the goal
		//
		// if cop moves last
		neighbors.clear();
		dscrenv->GetCopSuccessors( pos, neighbors );
		qe.pos = pos;
		qe.minFirst = true;
		for( std::vector<graphState>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;

				qe.pos[1] = *it;
				qe.value   = dscrenv->CopGCost( qe.pos, pos );
				queue.push( qe );
				//min_olis.insert( CRHash<graphState>( qe.pos ) );
		}

		n = g->nodeIterNext( nit );
	}
	
};



template<>
void DSDijkstra<xyLoc,tDirection,MapEnvironment>::push_end_states_on_queue() {
	QueueEntry qe;
	CRState pos;
	std::vector<xyLoc> neighbors;
	Map *m = env->GetMap();

	// sanity check: is the queue empty or are there rests?
	assert( queue.empty() );

	for( long width = 0; width < m->GetMapWidth(); width++ ) {
		for( long height = 0; height < m->GetMapHeight(); height++ ) {

			if( m->GetTerrainType( width, height ) != kGround ) continue;

			nodesExpanded++;nodesTouched++;

			pos.clear();
			pos.push_back( xyLoc( width, height ) );
			pos.push_back( xyLoc( width, height ) );

			min_cost[pos] = 0.;
			max_cost[pos] = 0.;

			// now push all the states on the queue that are one step away
			// from the goal
			//
			// if cop moves last
			neighbors.clear();
			dscrenv->GetCopSuccessors( pos, neighbors );
			qe.pos = pos;
			qe.minFirst = true;
			for( std::vector<xyLoc>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;

					qe.pos[1] = *it;
					qe.value   = dscrenv->CopGCost( qe.pos, pos );
					queue.push( qe );
					//min_olis.insert( CRHash<xyLoc>( qe.pos ) );
					//printf( "pushed (%u,%u) (%u,%u) on queue\n", qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y );
			}

		}
	}
	
};


template<>
void DSDijkstra<xyLoc,tDirection,MapEnvironment>::WriteValuesToDisk( const char* filename ) {
	FILE *fhandler;

	ClosedList::iterator it;

	fhandler = fopen( filename, "w" );
	fprintf( fhandler, "states in space: %lu\n", min_cost.size() );
	fprintf( fhandler, "expected rewards if cop moves first:\n" );
	fprintf( fhandler, "\n\n" );
	for( it = min_cost.begin(); it != min_cost.end(); it++ ) {
		fprintf( fhandler, "(%u,%u) (%u,%u) %g\n", it->first[0].x, it->first[0].y,
		         it->first[1].x, it->first[1].y, it->second );
	}
	fclose( fhandler );
	return;
};

template<>
void DSDijkstra<graphState,graphMove,GraphEnvironment>::WriteValuesToDisk( const char* filename ) {
	FILE *fhandler;

	ClosedList::iterator it;

	fhandler = fopen( filename, "w" );
	fprintf( fhandler, "states in space: %lu\n", min_cost.size() );
	fprintf( fhandler, "expected rewards if cop moves first:\n" );
	fprintf( fhandler, "\n\n" );
	for( it = min_cost.begin(); it != min_cost.end(); it++ ) {
		fprintf( fhandler, "%lu %lu %g\n", it->first[0], it->first[1], it->second );
	}
	fclose( fhandler );
	return;
};
