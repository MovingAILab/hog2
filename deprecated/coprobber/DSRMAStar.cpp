#include "DSRMAStar.h"


template<>
void DSRMAStar<graphState,graphMove,GraphEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst ) {
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

		pos.assign( 2, n->GetNum() );

		min_cost[pos] = 0.;
		max_cost[pos] = 0.;

		// now push all the states on the queue that are one step away
		// from the goal
		//
		// if cop moves last
		dscrenv->GetCopSuccessors( pos, neighbors );
		qe.pos = pos;
		qe.minFirst = true;
		for( std::vector<graphState>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;

				qe.pos[1] = *it;
				qe.gvalue   = dscrenv->CopGCost( qe.pos, pos );
				qe.fvalue   = qe.gvalue + HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
				queue.push( qe );
		}

		n = g->nodeIterNext( nit );
	}
	
};



template<>
void DSRMAStar<xyLoc,tDirection,MapEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst ) {
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

			pos.assign( 2, xyLoc( width, height ) );

			min_cost[pos] = 0.;
			max_cost[pos] = 0.;

			// now push all the states on the queue that are one step away
			// from the goal
			//
			// if cop moves last
			dscrenv->GetCopSuccessors( pos, neighbors );
			qe.pos = pos;
			qe.minFirst = true;
			for( std::vector<xyLoc>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;

					qe.pos[1] = *it;
					qe.gvalue   = dscrenv->CopGCost( qe.pos, pos );
					qe.fvalue   = qe.gvalue + HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
					queue.push( qe );
					//fprintf( stdout, "pushed up (%u,%u)(%u,%u) %d g=%f f=%f\n", qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.minFirst, qe.gvalue, qe.fvalue );
			}

		}
	}
	
};



template<>
double DSRMAStar<xyLoc,tDirection,MapEnvironment>::HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 ) {
	// case where edge costs are all 1
	double hmax = max(abs(pos1[0].x-pos2[0].x),abs(pos1[0].y-pos2[0].y));
	double hmin = max(abs(pos1[1].x-pos2[1].x),abs(pos1[1].y-pos2[1].y));

	hmin = ceil( hmin / (double)dscrenv->GetCopSpeed() );

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

/*
	// case where edge costs are 1 and diagonals are 1.5
	int a = abs(pos1[0].x-pos2[0].x);
	int b = abs(pos1[0].y-pos2[0].y);
	int    maxturns = (a>b)?a:b;
	double hmax     = (a>b)?(1.5*b+a-b):(1.5*a+b-a);

	a = abs(pos1[1].x-pos2[1].x);
	b = abs(pos1[1].y-pos2[1].y);
	int    minturns = (a>b)?a:b;
	double hmin     = (a>b)?(1.5*b+a-b):(1.5*a+b-a);

	hmin = ceil( hmin / (double)dscrenv->GetCopSpeed() );

	if( minFirst1 == minFirst2 )
		return( hmax + hmin + MinGCost( pos2, pos2 ) * abs(minturns-maxturns) );

	if( maxturns == minturns )
		return( hmax + hmin + MinGCost( pos2, pos2 ) );

	if( maxturns < minturns ) {
		if( minFirst1 && !minFirst2 )
			return( hmax + hmin + MinGCost( pos2, pos2 ) * (minturns - maxturns - 1) );
		else
			return( hmax + hmin + MinGCost( pos2, pos2 ) * (minturns - maxturns + 1) );
	} else {
		if( minFirst1 && !minFirst2 )
			return( hmax + hmin + MinGCost( pos2, pos2 ) * (maxturns - minturns + 1) );
		else
			return( hmax + hmin + MinGCost( pos2, pos2 ) * (maxturns - minturns - 1) );
	}
*/

};

