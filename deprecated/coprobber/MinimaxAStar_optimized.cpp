#include "MinimaxAStar_optimized.h"


template<>
void MinimaxAStar<graphState,graphMove,GraphEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst ) {
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
		neighbors.clear();
		env->GetSuccessors( pos[1], neighbors );
		qe.pos = pos;
		qe.minFirst = true;
		for( std::vector<graphState>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
				nodesTouched++;

				qe.pos[1] = *it;
				qe.gvalue   = MinGCost( qe.pos, pos );
				qe.fvalue   = qe.gvalue + HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
				queue.push( qe );
		}

		n = g->nodeIterNext( nit );
	}
	
};



template<>
void MinimaxAStar<xyLoc,tDirection,MapEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst ) {
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

			pos.clear(); pos.push_back( xyLoc( width, height ) ); pos.push_back( xyLoc( width, height ) );

			min_cost[pos] = 0.;
			max_cost[pos] = 0.;

			// now push all the states on the queue that are one step away
			// from the goal
			//
			// if cop moves last
			neighbors.clear();
			env->GetSuccessors( pos[1], neighbors );
			qe.pos = pos;
			qe.minFirst = true;
			for( std::vector<xyLoc>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;

					qe.pos[1] = *it;
					qe.gvalue   = MinGCost( qe.pos, pos );
					qe.fvalue   = qe.gvalue + HCost( goal_pos, goal_minFirst, qe.pos, qe.minFirst );
					queue.push( qe );
					//fprintf( stdout, "pushed up (%u,%u)(%u,%u) %d g=%f f=%f\n", qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.minFirst, qe.gvalue, qe.fvalue );
			}

		}
	}
	
};



template<>
double MinimaxAStar<xyLoc,tDirection,MapEnvironment>::HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 ) {

	if( !useHeuristic ) return 0.;

/*
	if( minFirst1 == minFirst2 ) {
		if( my_minheuristic.find( pos2 ) != my_minheuristic.end() )
			return 0.;
	} else {
		if( my_maxheuristic.find( pos2 ) != my_maxheuristic.end() )
			return 0.;
	}


	if( random() < RAND_MAX/2 ) {
		if( minFirst1==minFirst2) my_minheuristic[pos2] = 0.;
		else my_maxheuristic[pos2] = 0.;
		return 0.;
	}
*/


	// case where edge costs are all 1
	double hmax, hmin;
	if( usePerfectDistanceHeuristic ) {
		hmax = distance_heuristic[env->GetMap()->GetNodeNum(pos1[0].x,pos1[0].y)][env->GetMap()->GetNodeNum(pos2[0].x,pos2[0].y)];
		hmin = distance_heuristic[env->GetMap()->GetNodeNum(pos1[1].x,pos1[1].y)][env->GetMap()->GetNodeNum(pos2[1].x,pos2[1].y)];
	} else {
		hmax = max(abs(pos1[0].x-pos2[0].x),abs(pos1[0].y-pos2[0].y));
		hmin = max(abs(pos1[1].x-pos2[1].x),abs(pos1[1].y-pos2[1].y));
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


template<>
double MinimaxAStar<xyLoc, tDirection, MapEnvironment>::MinGCost( CRState &p1, CRState &p2 ) {
//	if( abs(p1[0].x-p2[0].x)==1 && abs(p1[0].y-p2[0].y)==1 ) return 1.5;
//	if( abs(p1[1].x-p2[1].x)==1 && abs(p1[1].y-p2[1].y)==1 ) return 1.5;
	return 1.;
}

// test wise, compute the exact distance heuristic (TODO: make this work for every kind of environment)
template<>
void MinimaxAStar<xyLoc,tDirection,MapEnvironment>::set_usePerfectDistanceHeuristic( bool set ) {
	usePerfectDistanceHeuristic = set;

	if( set ) {
		Map *m = env->GetMap();
		Graph *g = GraphSearchConstants::GetGraph( m );
		// set all weights in the graph to 1.
		edge_iterator eit = g->getEdgeIter();
		edge *e = g->edgeIterNext( eit );
		while( e ) {
			e->setWeight( 1. );
			e = g->edgeIterNext( eit );
		}
		FloydWarshall( g, distance_heuristic );
	}
}
