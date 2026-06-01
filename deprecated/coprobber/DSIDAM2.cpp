#include "DSIDAM2.h"

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

DSIDAM2::DSIDAM2( MapAbstraction *_gabs, bool _canPause, unsigned int _cop_speed, bool _useAbstraction ):
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

DSIDAM2::~DSIDAM2() {
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

void DSIDAM2::dam( node* pos_robber, node* pos_cop, std::vector<node*> &resultpath, bool minFirst, double depth, double start_level_fraction ) {
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
			//printf( "numberpath: " );
			//for( unsigned int i = 0; i < numberpath.size(); i++ ) {
			//	printf( "%lu(%d) ", numberpath[i], gabs->GetAbstractGraph(level)->GetNode(numberpath[i])->getUniqueID() );
			//}
			//printf( "\n" );

			break;
		}
		// else continue to the next level
	}

	if( robberChain[level]->GetNum() == numberpath[0] ) {
		resultpath.push_back( pos_robber );
		return;
	}

	path *p = NULL;
	Graph *g = gabs->GetAbstractGraph( level );
	// put the node on the path that we will visit in two moves
	if( numberpath.size() >= 3 )
		p = new path( g->GetNode( numberpath[2] ), p );
	// put the node on the path that we will visit in the next move
	p = new path( g->GetNode( numberpath[0] ), p );
	// put the current node on the path
	p = new path( robberChain[level], p );

	// verbose
	//printf( "path on level %d: ", level );
	//for( path *trav = p; trav; trav = trav->next ) {
	//	printf( "%u(%d) ", trav->n->GetNum(), trav->n->getUniqueID() );
	//}
	//printf( "\n" );

	bool first_refinement = true;

	while( level > 0 ) {
		g = gabs->GetAbstractGraph( level );

		unsigned int destParent = 0xFFFFFFFF;
		unsigned int dest;
		path* trav = p->next;

		if( first_refinement ) {
			if( trav->next != NULL ) {
				destParent = trav->n->GetNum();
				dest = gabs->GetNthChild( trav->next->n, random()%gabs->GetNumChildren( trav->next->n ) )->GetNum();
				delete trav->next;
				trav->next = NULL;
			} else {
				dest = gabs->GetNthChild( trav->n, random()%gabs->GetNumChildren( trav->n ) )->GetNum();
			}
			first_refinement = false;
		} else {
			node *target = p->tail()->n;
			dest = gabs->GetNthChild( target, random()%gabs->GetNumChildren( target ) )->GetNum();
		}

		// put together the parents vector
		std::vector<unsigned int> eligibleNodeParents;
		for ( trav = p; trav; trav = trav->next ) {
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
		//       testit != eligibleNodeParents.end(); testit++ ) {
		//	printf( "%u ", *testit );
		//}
		//printf( "\n" );

		delete p;

		p = getAbstractPath( gabs->GetAbstractGraph(level-1),
			robberChain[level-1]->GetNum(),
			destParent, eligibleNodeParents,
			GraphAbstractionConstants::kTemporaryLabel, dest );

		level--;
	}

	myNodesExpanded += nodesExpanded;
	myNodesTouched  += nodesTouched;

	// convert p to a vector
	path *ptemp = p->next;
	while( ptemp ) {
		resultpath.push_back( ptemp->n );
		ptemp = ptemp->next;
	}

	// cleanup
	delete ptemp;
	delete p;

	return;
};


node* DSIDAM2::MakeMove( node* pos_robber, node* pos_cop, bool minFirst, double depth, double start_level_fraction ) {
	std::vector<node*> path;
	dam( pos_robber, pos_cop, path, minFirst, depth, start_level_fraction );
	// sanity check
	assert( path.size() > 0 );
	return( path[0] );
};

graphState DSIDAM2::MakeMove( graphState pos_robber, graphState pos_cop, unsigned int ) {
	Graph *g = gabs->GetAbstractGraph( 0 );
	node *r  = g->GetNode( pos_robber );
	node *c  = g->GetNode( pos_cop );
	std::vector<node*> path;
	dam( r, c, path, false, 3., 0.5 );
	assert( path.size() > 0 );
	return( path[0]->GetNum() );
};
