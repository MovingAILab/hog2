#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include "DSBestResponse.h"

/*------------------------------------------------------------------------------
| Hashing
------------------------------------------------------------------------------*/
// squeze the entire joint state into a 32bit integer
uint32_t DSBestResponse::CRHash_MemOptim( CRState &s ) {
	return( s[0] * numnodes + s[1] );
//	return( ( (uint32_t)s[0] )<<16 | (((uint32_t)s[1])<<16)>>16 );
};

// get the joint state back from the hash
void DSBestResponse::MemOptim_Hash_To_CRState( uint32_t &hash, DSBestResponse::CRState &s ) {
	s.resize( 2, 0 );
	s[0] = (graphState) (hash/numnodes);
	s[1] = (graphState) (hash%numnodes);
//	s.push_back( (graphState) (hash>>16) );
//	s.push_back( (graphState) ( (hash<<16)>>16 ) );
	return;
};


/*------------------------------------------------------------------------------
| DSDijkstra Memory Optimized - Implementation
------------------------------------------------------------------------------*/
DSBestResponse::DSBestResponse( GraphEnvironment *_env, DSRobberAlgorithm<graphState,graphMove> *_ralg, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<graphState,graphMove>( _env, true, cop_speed ) ),
	env(_env),
	ralg(_ralg)
{
	numnodes = env->GetGraph()->GetNumNodes();
	// this is just to keep errors in computation out
	if( numnodes*numnodes > UINT32_MAX ) {
		fprintf( stderr, "ERROR: More nodes in graph than can be encoded in hash\n" );
		exit(1);
	}
};

DSBestResponse::~DSBestResponse() {
	delete dscrenv;
};


float DSBestResponse::compute_target_value( CRState &s ) {

	CRState temp = s;
	// verbose
	//std::cout << "position: (" << temp[0] << "," << temp[1] << ") => ";
	uint32_t hash = CRHash_MemOptim( s );
	if( ralg_moves[hash] >= numnodes ) {
		ralg_moves[hash] = ralg->MakeMove( s[0], s[1], numnodes );
		ralgCalls++;
	}
	temp[0] = ralg_moves[hash];
	// verbose
	//std::cout << temp[0] << std::endl;

	return( dscrenv->RobberGCost( s, temp ) + min_cost[ CRHash_MemOptim( temp ) ] );
};

void DSBestResponse::compute_robber_moves() {
	ralg_moves.assign( numnodes*numnodes, 0 );

	CRState s;
	for( unsigned int i = 0; i < numnodes*numnodes; i++ ) {
		MemOptim_Hash_To_CRState( i, s );

		ralg_moves[i] = ralg->MakeMove( s[0], s[1], numnodes );
		ralgCalls++;
	}
	return;
};


void DSBestResponse::compute_best_response() {
	QueueEntry qe, qtemp;
	std::vector<graphState>::iterator it;
	CRState pos, postemp;

	// failproof
	assert( queue.empty() );

	// assign space neccessary
	min_cost.assign( numnodes*numnodes, FLT_MAX );
	max_cost.assign( numnodes*numnodes, FLT_MAX );

	nodesExpanded = 0;
	nodesTouched  = 0;
	ralgCalls = 0;

	ralg_moves.assign( numnodes*numnodes, numnodes );
//	compute_robber_moves();

	push_end_states_on_queue();

	while( !queue.empty() ) {

		// get the element from the queue
		qe = queue.top(); queue.pop();
		MemOptim_Hash_To_CRState( qe.pos_hash, pos );

		// verbose
		//fprintf( stdout, "minFirst = %d, pos = (%u,%u)(%u,%u), value = %f\n", qe.minFirst, qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.value );

		if( qe.minFirst ) {

			if( min_cost[qe.pos_hash] == FLT_MAX ) {

				min_cost[qe.pos_hash] = qe.value;

				std::vector<graphState> myneighbors;
				dscrenv->GetRobberSuccessors( pos, myneighbors, false );
				nodesExpanded++;

				// now, for all successor states
				for( it = myneighbors.begin(); it != myneighbors.end(); it++ ) {

					nodesTouched++;

					// build the state
					pos[0] = *it;
					qe.pos_hash = CRHash_MemOptim( pos );

					// check whether its value is not yet set
					if( max_cost[qe.pos_hash] == FLT_MAX ) {
						qe.value = compute_target_value( pos );
						if( qe.value != FLT_MAX ) {
							qe.minFirst = false;
							queue.push( qe );
						}
					}
				}
			}

		} else {

			if( max_cost[qe.pos_hash] == FLT_MAX ) {

				max_cost[qe.pos_hash] = qe.value;
				postemp = pos;
				max_max_cost = qe.value;

				// get neighbors
				std::vector<graphState> myneighbors;
				//std::vector<float> gcosts; // real edge costs
				//dscrenv->GetCopSuccessors( postemp[1], myneighbors, gcosts ); // real edge costs
				dscrenv->GetCopSuccessors( postemp[1], myneighbors );
				nodesExpanded++;

				for( it = myneighbors.begin(); it != myneighbors.end(); it++ ) {
				//for( unsigned int i = 0; i < myneighbors.size(); i++ ) { // real edge costs

					nodesTouched++;

					// build the next state
					postemp[1] = *it;
					//postemp[1] = myneighbors[i]; // real edge costs
					qtemp.pos_hash = CRHash_MemOptim( postemp );

					// check again whether already set or not
					if( min_cost[ qtemp.pos_hash ] == FLT_MAX ) {
						qtemp.minFirst = true;
						qtemp.value = qe.value + dscrenv->CopGCost( postemp, pos );
						//qtemp.value = qe.value + gcosts[i]; // real edge costs
						//printf( "pushed %d (%u,%u)(%u,%u) on queue with %g\n", qtemp.minFirst,
						//	qtemp.pos[0].x, qtemp.pos[0].y, qtemp.pos[1].x, qtemp.pos[1].y, qtemp.value );
						queue.push( qtemp );
					}
				}
			}
		}

	}

}


float DSBestResponse::Value( CRState &pos, bool minFirst ) {
	if( minFirst ) {
		return min_cost[CRHash_MemOptim( pos )];
	} else {
		return max_cost[CRHash_MemOptim( pos )];
	}
};

// this implementation only supports cop_speed = 2 because I was to lazy to implement
// an entire Dijkstra algorithm here
//
// note: this only works well because of the move ordering, if we used a wait
// as the first move (from DSCREnvironment) this routine would screw up at the end
void DSBestResponse::MakeSingleStepsCopMove( CRState &pos, std::vector<graphState> &moves ) {
	moves.clear();
	if( dscrenv->GetCopSpeed() != 2 ) std::cerr << "ERROR: speed != 2 not supported." << std::endl;

	std::vector<graphState> neighbors1, neighbors2;
	float value = max_cost[CRHash_MemOptim( pos )];
	graphState first_move = pos[1], second_move = pos[1];
	CRState temppos = pos;

	dscrenv->GetCopSuccessors( temppos, neighbors1, 1 );
	for( std::vector<graphState>::iterator it1 = neighbors1.begin(); it1 != neighbors1.end(); it1++ ) {
		dscrenv->GetCopSuccessors( *it1, neighbors2, 1 );
		for( std::vector<graphState>::iterator it2 = neighbors2.begin(); it2 != neighbors2.end(); it2++ ) {
			temppos[1] = *it2;
			float temp = max_cost[ CRHash_MemOptim( temppos ) ];
			if( value > temp ) {
				value = temp;
				first_move = *it1;
				second_move = *it2;
			}
		}
	}
	moves.push_back( first_move );
	moves.push_back( second_move );
	return;
};

graphState DSBestResponse::MakeMove( CRState &pos, bool minFirst ) {

	CRState temppos = pos;
	std::vector<graphState> neighbors;
	float temp;
	graphState result = minFirst?temppos[1]:temppos[0];
	std::vector<graphState>::iterator it;

	// get the available moves
	if( minFirst ) {
		float value = max_cost[ CRHash_MemOptim( temppos ) ];
		dscrenv->GetCopSuccessors( temppos, neighbors );

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			temppos[1] = *it;
			temp = max_cost[ CRHash_MemOptim( temppos ) ];
			if( value > temp ) {
				value = temp;
				result = *it;
			}
		}
	} else {
		float value = min_cost[ CRHash_MemOptim( temppos ) ];
		dscrenv->GetRobberSuccessors( temppos, neighbors );

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			temppos[0] = *it;
			temp = min_cost[ CRHash_MemOptim( temppos ) ];
			if( value < temp ) {
				value = temp;
				result = *it;
			}
		}
	}

	return result;
};


void DSBestResponse::push_end_states_on_queue() {
	QueueEntry qe;
	CRState pos, postemp;
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

		pos.clear();
		pos.push_back( n->GetNum() );
		pos.push_back( n->GetNum() );

		min_cost[CRHash_MemOptim( pos )] = 0.;
		max_cost[CRHash_MemOptim( pos )] = 0.;

		// now push all the states on the queue that are one step away
		// from the goal
		//
		// if cop moves last
		neighbors.clear();
		//std::vector<float> gcosts; // real edge costs
		//dscrenv->GetCopSuccessors( pos[1], neighbors, gcosts ); // real edge costs
		dscrenv->GetCopSuccessors( pos, neighbors );
		nodesExpanded++;
		nodesTouched++;

		postemp = pos;
		qe.minFirst = true;
		for( std::vector<graphState>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
		//for( unsigned int i = 0; i < neighbors.size(); i++ ) { // real edge costs
				nodesTouched++;

				postemp[1]  = *it;
				//postemp[1]  = neighbors[i]; // real edge costs
				qe.pos_hash = CRHash_MemOptim( postemp );
				qe.value = dscrenv->CopGCost( postemp, pos );
				//qe.value = gcosts[i]; // real edge costs
				queue.push( qe );
		}

		n = g->nodeIterNext( nit );
	}
	
};


void DSBestResponse::WriteValuesToDisk( const char* filename ) {
	FILE *fhandler;

	CRState pos;

	fhandler = fopen( filename, "w" );
	fprintf( fhandler, "states in space: %lu\n", min_cost.size() );
	fprintf( fhandler, "expected rewards if cop moves first:\n" );
	fprintf( fhandler, "\n\n" );

	for( unsigned int i = 0; i < min_cost.size(); i++ ) {
		MemOptim_Hash_To_CRState( i, pos );
		//fprintf( fhandler, "%lu %lu %g %g\n", pos[0], pos[1], min_cost[i], max_cost[i] );
		fprintf( fhandler, "%lu %lu %g\n", pos[0], pos[1], min_cost[i] );
	}

	fclose( fhandler );
	return;
};



void DSBestResponse::ReadValuesFromDisk( const char* filename ) {
	FILE *fhandler;
	max_max_cost = -FLT_MAX;

	fhandler = fopen( filename, "r" );

	if( fhandler == NULL ) {
		std::cerr << "ERROR: could not open dijkstra file." << std::endl;
		exit( 1 );
	}

	unsigned int num_states;
	fscanf( fhandler, "states in space: %d\n", &num_states );
	if( num_states != numnodes*numnodes ) {
		std::cerr << "ERROR: input file does not have the same amount of states as the map requires." << std::endl;
		exit( 1 );
	}
	fscanf( fhandler, "expected rewards if cop moves first:\n" );
	fscanf( fhandler, "\n\n" );

	min_cost.assign( numnodes*numnodes, FLT_MAX );
	max_cost.assign( numnodes*numnodes, FLT_MAX );

	while( !feof( fhandler ) ) {
		float min_c, max_c;
		unsigned int pos_0, pos_1;
		fscanf( fhandler, "%d %d %g %g\n", &pos_0, &pos_1, &min_c, &max_c );
		CRState pos; pos.push_back( pos_0 ); pos.push_back( pos_1 );
		size_t hash = CRHash_MemOptim( pos );
		min_cost[hash] = min_c;
		max_cost[hash] = max_c;
		if( max_c > max_max_cost ) max_max_cost = max_c;
	}
	fclose( fhandler );
	return;
};



void DSBestResponse::DrawCopRobberEdges( bool minFirst, graphState pos_opponent ) {

	Graph *g = env->GetGraph();
	CRState pos; pos.assign( 2, 0 );
	pos[minFirst?0:1] = pos_opponent;
	edge_iterator eit = g->getEdgeIter();
	edge *e = g->edgeIterNext( eit );

	glBegin( GL_LINES );
	double value;

	// color all edges due to their respective distance to the robber
	while( e != NULL ) {

		// readme:
		// within the following code, value is always the solution length
		// when the opponent starts from the submitted position and I would
		// be in a variable position (that's why we loop over the edges)

		node *n = g->GetNode( e->getFrom() );
		if( minFirst ) {
			pos[1]  = e->getFrom();
			value   = max_cost[ CRHash_MemOptim( pos ) ];
			//glColor3f( value/max_max_cost, 1.-value/max_max_cost, 1.-fabs(0.5-value/max_max_cost) );
			glColor3f( 0., 1.-value/max_max_cost, 0. );
		} else {
			pos[0] = e->getFrom();
			value  = min_cost[ CRHash_MemOptim( pos ) ];
			//glColor3f( 1.-fabs(0.5-value/max_max_cost), value/max_max_cost, 1.-value/max_max_cost );
			glColor3f( 0., value/max_max_cost, 0. );
		}

		GLdouble x,y,z;
		x = n->GetLabelF( GraphAbstractionConstants::kXCoordinate );
		y = n->GetLabelF( GraphAbstractionConstants::kYCoordinate );
		z = n->GetLabelF( GraphAbstractionConstants::kZCoordinate );
		glVertex3f( x, y, z );

		n      = g->GetNode( e->getTo() );
		if( minFirst ) {
			pos[1]  = e->getTo();
			value   = max_cost[ CRHash_MemOptim( pos ) ];
			//glColor3f( value/max_max_cost, 1.-value/max_max_cost, 1.-fabs(0.5-value/max_max_cost) );
			glColor3f( 0., 1.-value/max_max_cost, 0. );
		} else {
			pos[0] = e->getTo();
			value  = min_cost[ CRHash_MemOptim( pos ) ];
			//glColor3f( 1.-fabs(0.5-value/max_max_cost), 1.-value/max_max_cost, value/max_max_cost );
			glColor3f( 0., value/max_max_cost, 0. );
		}

		x = n->GetLabelF( GraphAbstractionConstants::kXCoordinate );
		y = n->GetLabelF( GraphAbstractionConstants::kYCoordinate );
		z = n->GetLabelF( GraphAbstractionConstants::kZCoordinate );
		glVertex3f( x, y, z );

		e = g->edgeIterNext( eit );
	}
	glEnd();

	return;
};

