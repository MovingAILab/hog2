#include "DSIDAM.h"

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

DSIDAM::DSIDAM( MapAbstraction *_gabs, bool _canPause, unsigned int _cop_speed, bool _useAbstraction ):
	praStar(),
	gabs(_gabs), canPause(_canPause), cop_speed(_cop_speed), useAbstraction(_useAbstraction)
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

	map = _gabs; // map is defined in PRAStar.h
};

DSIDAM::~DSIDAM() {
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
//	delete gabs;
};

void DSIDAM::dam( node* pos_robber, node* pos_cop, std::vector<node*> &resultpath, bool minFirst, double depth, double start_level_fraction ) {
	resultpath.clear();
	myNodesExpanded = 0;
	nodesExpanded   = 0;
	myNodesTouched  = 0;
	nodesTouched    = 0;

	// sanity check
	assert( 0. <= start_level_fraction && start_level_fraction <= 1. );

	if( pos_robber == pos_cop ) return;

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
	unsigned int start = (unsigned int) floor( (double)(robberChain.size()-1) * start_level_fraction );
	assert( robberChain.size() <= gabs->getNumAbstractGraphs() );
	assert( robberChain.size() == copChain.size() );
	if( !useAbstraction ) start = 0;

	std::vector<graphState> numberpath;
	int level = 0;
	// now start planning beginning at the highest level until we find a possible solution
	for( level = start; level >= 0; level-- ) {
		numberpath.clear();
		graphState probber = (graphState) robberChain[level]->GetNum();
		graphState pcop    = (graphState) copChain[level]->GetNum();
		double temp = dsminimax[level]->minimax( probber, pcop, numberpath, minFirst, depth );
		myNodesExpanded += dsminimax[level]->nodesExpanded;
		myNodesTouched  += dsminimax[level]->nodesTouched;
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

			break;
		}
		// else continue to the next level
	}

	// put together the abstract path the robber should go on
	path *p = NULL;
	Graph *g = gabs->GetAbstractGraph( level );
	std::vector<graphState>::reverse_iterator rit = numberpath.rbegin();
	if( !(numberpath.size()%2) ) rit++; // the opponent moves last
	while( rit != numberpath.rend() ) {
		p = new path( g->GetNode( *rit ), p );
		// take every second step
		rit++;
		if( rit != numberpath.rend() ) rit++;
	}
	// put the starting node onto the path
	if( minFirst )
		p = new path( copChain[level], p );
	else
		p = new path( robberChain[level], p );

	// test whether goal position and final position are the same
	// this would cause a problem in the refinement process
	if( level > 0 && p->n == p->tail()->n ) {
		if( minFirst )
			resultpath.push_back( pos_cop );
		else
			resultpath.push_back( pos_robber );
		return;
	}

	// verbose
	//printf( "path on level %d: ", level );
	//for( path *trav = p; trav; trav = trav->next ) {
	//	printf( "%u(%d) ", trav->n->GetNum(), trav->n->getUniqueID() );
	//}
	//printf( "\n" );

	while( level > 0 ) {
		g = gabs->GetAbstractGraph( level );
		// put together the parents vector
		std::vector<unsigned int> eligibleNodeParents;
		for (path *trav = p; trav; trav = trav->next) {
			edge_iterator ei = trav->n->getEdgeIter();
			for (edge *e = trav->n->edgeIterNext(ei); e; e = trav->n->edgeIterNext(ei)) {
				if (e->getFrom() == trav->n->GetNum())
					eligibleNodeParents.push_back(e->getTo());
				else
					eligibleNodeParents.push_back(e->getFrom());
			}
			eligibleNodeParents.push_back(trav->n->GetNum());
		}
		// verbose
		//printf( "eligible parents: \n" );
		//for( std::vector<unsigned int>::iterator testit = eligibleNodeParents.begin();
		//     testit != eligibleNodeParents.end(); testit++ ) {
		//	printf( "%u ", *testit );
		//}
		//printf( "\n" );

		node *target = p->tail()->n;
		node *lower_level_target = gabs->GetNthChild( target, random()%gabs->GetNumChildren(target));

		path *ptemp = p;

		p = getAbstractPath( gabs->GetAbstractGraph(level-1),
			robberChain[level-1]->GetNum(),
			0xFFFFFFFF, eligibleNodeParents,
			GraphAbstractionConstants::kTemporaryLabel,
			lower_level_target->GetNum() );
		// verbose
		//printf( "path on level %d: ", level );
		//for( path *trav = p; trav; trav = trav->next ) {
		//	printf( "%u(%d) ", trav->n->GetNum(), trav->n->getUniqueID() );
		//}
		//printf( "\n" );

		// cleanup
		delete ptemp;

		level--;
	}

	myNodesExpanded += nodesExpanded;
	myNodesTouched  += nodesTouched;

	// convert p to a vector
	path *ptemp = p;
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


node* DSIDAM::MakeMove( node* pos_robber, node* pos_cop, bool minFirst, double depth, double start_level_fraction ) {
	std::vector<node*> path;
	dam( pos_robber, pos_cop, path, minFirst, depth, start_level_fraction );
	// sanity check
	assert( path.size() > 0 );
	return( path[0] );
};
