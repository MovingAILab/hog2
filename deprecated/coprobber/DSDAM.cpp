#include "DSDAM.h"

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

DSDAM::DSDAM( MapAbstraction *_gabs, bool _canPause, unsigned int _cop_speed, bool _useAbstraction ):
	gabs(_gabs), canPause(_canPause), cop_speed(_cop_speed), useAbstraction(_useAbstraction), pra( new praStar() )
{
	// create the minimax objects
	for( unsigned int level = 0; level < gabs->getNumAbstractGraphs(); level++ ) {
		graphmapheuristics.push_back( new MaximumNormAbstractGraphMapHeuristic( gabs->GetAbstractGraph( level ), gabs->GetMap() ) );
		graphenvironments.push_back( new GraphEnvironment( gabs->GetAbstractGraph( level ), graphmapheuristics[level] ) );
		dsminimax.push_back( new DSMinimax<graphState,graphMove>( graphenvironments[level], canPause, cop_speed ) );

		// note that we do NOT use path costs in higher levels, the tree is only generated
		// and the leafs are evaluated, no edge costs are used! And yes, this kind of makes sense ;-)
		dsminimax[level]->useEdgeCosts( false );
	}
	// use edge costs on the lowest level
	dsminimax[0]->useEdgeCosts( true );

};

DSDAM::~DSDAM() {
	while( !dsminimax.empty() ) {
		std::vector<DSMinimax<graphState,graphMove>*>::iterator it = dsminimax.begin();
		DSMinimax<graphState,graphMove> *temp = *it;
		dsminimax.erase( it );
		delete temp;
	}
	while( !graphenvironments.empty() ) {
		std::vector<GraphEnvironment*>::iterator it = graphenvironments.begin();
		GraphEnvironment *temp = *it;
		graphenvironments.erase( it );
		delete temp;
	}
	while( !graphmapheuristics.empty() ) {
		std::vector<MaximumNormAbstractGraphMapHeuristic*>::iterator it = graphmapheuristics.begin();
		MaximumNormAbstractGraphMapHeuristic *temp = *it;
		graphmapheuristics.erase( it );
		delete temp;
	}
	delete pra;
//	delete gabs;
};

void DSDAM::dam( node* pos_robber, node* pos_cop, std::vector<node*> &resultpath, bool minFirst, double depth, double start_level_fraction ) {
	resultpath.clear();
	nodesExpanded = 0;
	nodesTouched = 0;

	// sanity check
	assert( 0. <= start_level_fraction && start_level_fraction <= 1. );

	// find the joint hierarchy
	std::vector<node*> robberChain, copChain;
	gabs->GetNumAbstractGraphs( pos_robber, pos_cop, robberChain, copChain );

	//verbose
	//printf( "robber chain: " );
	//for( std::vector<node*>::iterator it = robberChain.begin(); it != robberChain.end(); it++ ) {
	//	printf( "%d ", (*it)->GetNum() );
	//}
	//printf( "\ncop chain: " );
	//for( std::vector<node*>::iterator it = copChain.begin(); it != copChain.end(); it++ ) {
	//	printf( "%d ", (*it)->GetNum() );
	//}
	//printf( "\n" );

	// determine the level where we start planning
	// make a sanity check (to ensure the start level actually makes sense)
	// if there should not be used any abstraction then set the start level to 0
	unsigned int start = (unsigned int) floor((double)(robberChain.size()-1) * start_level_fraction);
	assert( robberChain.size() <= gabs->getNumAbstractGraphs() );
	assert( robberChain.size() == copChain.size() );
	if( !useAbstraction ) start = 0;

	node* target = NULL;

	// now start planning beginning at the highest level until we find a possible solution
	for( int level = start; level >= 0; level-- ) {
		std::vector<graphState> numberpath;
		graphState probber = (graphState) robberChain[level]->GetNum();
		graphState pcop    = (graphState) copChain[level]->GetNum();
		double temp = dsminimax[level]->minimax( probber, pcop, numberpath, minFirst, depth );
		nodesExpanded += dsminimax[level]->nodesExpanded;
		nodesTouched  += dsminimax[level]->nodesTouched;
		if( fgreater(temp,0.) ) {
			// the robber can escape on this level

			// verbose
			//printf( "found solution on level %d with %g\n", level, temp );

			assert( numberpath.size() > 0 );

			// verbose
			//printf( "path: " );
			//for( unsigned int i = 0; i < numberpath.size(); i++ ) {
			//	printf( "%lu(%d) ", numberpath[i], gabs->GetAbstractGraph(level)->GetNode(numberpath[i])->getUniqueID() );
			//}
			//printf( "\n" );

			// select my last move in the chain of actions
			if( numberpath.size() == 1 || numberpath.size()%2 ) //== 1
				target = gabs->GetAbstractGraph( level )->GetNode( numberpath[numberpath.size()-1] );
			else
				target = gabs->GetAbstractGraph( level )->GetNode( numberpath[numberpath.size()-2] );
			break;
		}
		// else continue to the next level
	}

	if( target != NULL ) {
		// select a target on the lowest level of abstraction
		target = gabs->GetRandomGroundNodeFromNode( target );
		//printf( "going for node %d\n", target->GetNum() );
	} else {
		fprintf( stderr, "Warning: strangely could not select a target\n" );
		fprintf( stderr, "Warning: selecting first node in the map as a target\n" );
		target = gabs->GetAbstractGraph( 0 )->GetNode( 0 );
	}

	path *p, *ptemp;
	if( minFirst ) {
		if( target == pos_cop ) {
			// we did not change our actual position, thus return a stay immediately
			resultpath.push_back( target );
			return;
		} else
			p = pra->GetPath( gabs, pos_cop, target );
	} else {
		if( target == pos_robber ) {
			// we did not change our actual position, thus return a stay immediately
			resultpath.push_back( target );
			return;
		} else
			p = pra->GetPath( gabs, pos_robber, target );
	}

	// get statistics from PRA*
	nodesExpanded += pra->nodesExpanded;
	nodesTouched  += pra->nodesTouched;
	
	// convert p to a vector
	ptemp = p;
	if( minFirst ) {
		// since PRA* gives us back a path we only have to choose
		// the positions that he can get to in cop_speed
		// and the last position he wants to get to
		// note: that we run towards the selected position
		// as fast as possible, this might actually not be optimal
		unsigned int step = 0;
		while( ptemp ) {
			if( step % cop_speed == 0 || ptemp->next == NULL ) {
				resultpath.push_back( ptemp->n );
			}
			ptemp = ptemp->next;
			step++;
		}

	} else {
		// the robber does every step
		while( ptemp ) {
			resultpath.push_back( ptemp->n );
			ptemp = ptemp->next;
		}
	}

	// cleanup
	delete ptemp;
	delete p;

	// the path returned by PRA* has the current position in it
	// this can safely be erased
	resultpath.erase( resultpath.begin() );

	return;
};


node* DSDAM::MakeMove( node* pos_robber, node* pos_cop, bool minFirst, double depth, double start_level_fraction ) {
	std::vector<node*> path;
	dam( pos_robber, pos_cop, path, minFirst, depth, start_level_fraction );
	// sanity check
	assert( path.size() > 0 );
	return( path[0] );
};

graphState DSDAM::MakeMove( graphState pos_robber, graphState pos_cop, unsigned int ) {
	Graph *g = gabs->GetAbstractGraph( 0 );
	node *r  = g->GetNode( pos_robber );
	node *c  = g->GetNode( pos_cop );
	std::vector<node*> path;
	dam( r, c, path, false, 3., 0.5 );
	assert( path.size() > 0 );
	return( path[0]->GetNum() );
};
