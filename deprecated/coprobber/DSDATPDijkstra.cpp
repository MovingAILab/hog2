#include "DSDATPDijkstra.h"

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

DSDATPDijkstra::DSDATPDijkstra( MapAbstraction *_gabs, unsigned int _cop_speed, bool _useAbstraction ):
	praStar(),
	gabs(_gabs), cop_speed(_cop_speed), useAbstraction(_useAbstraction)
{
	// create the minimax objects
	for( unsigned int level = 0; level < gabs->getNumAbstractGraphs(); level++ ) {
		graphmapheuristics.push_back( new MaximumNormAbstractGraphMapHeuristic( gabs->GetAbstractGraph( level ), gabs->GetMap() ) );
		graphenvironments.push_back( new GraphEnvironment( gabs->GetAbstractGraph( level ), graphmapheuristics[level] ) );
		dstpdijkstra.push_back( new DSTPDijkstra<graphState,graphMove>( graphenvironments[level], cop_speed ) );
	}

	map = _gabs; // map is defined in PRAStar.h
};

DSDATPDijkstra::~DSDATPDijkstra() {
	while( !dstpdijkstra.empty() ) {
		std::vector<DSTPDijkstra<graphState,graphMove>*>::iterator it = dstpdijkstra.begin();
		DSTPDijkstra<graphState,graphMove> *temp = *it;
		dstpdijkstra.erase( it );
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

void DSDATPDijkstra::datpdijkstra( node* pos_robber, node* pos_cop, std::vector<node*> &resultpath, bool minFirst, double min_escape_length ) {
	resultpath.clear();
	myNodesExpanded = 0;
	nodesExpanded   = 0;
	myNodesTouched  = 0;
	nodesTouched    = 0;

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
	unsigned int start = robberChain.size() / 2;
	assert( robberChain.size() <= gabs->getNumAbstractGraphs() );
	assert( robberChain.size() == copChain.size() );
	if( !useAbstraction ) start = 0;

	// now start planning beginning at the highest level until we find a possible solution
	std::vector<graphState> numberpath;
	int level = 0;
	for( level = start; level >= 0; level-- ) {
		numberpath.clear();
		graphState probber = (graphState) robberChain[level]->GetNum();
		graphState pcop    = (graphState) copChain[level]->GetNum();
		double temp = dstpdijkstra[level]->dstpdijkstra( probber, pcop, minFirst, numberpath );
		myNodesExpanded += dstpdijkstra[level]->nodesExpanded;
		myNodesTouched  += dstpdijkstra[level]->nodesTouched;
		if( fgreater(temp,min_escape_length) || level == 0 ) {
			// the robber can escape on this level
			// or we are on the ground level

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

	// if the solution suggests staying in the same spot
	if( robberChain[level]->GetNum() == numberpath[0] ) {
		//printf( "path size is %u\n", numberpath.size() );
		resultpath.assign( numberpath.size(), pos_robber );
		return;
	}

	// transfer the number path into a "normal" path (used in PRA*)
	path *p = NULL;
	Graph *g = gabs->GetAbstractGraph(level);
	for( std::vector<graphState>::reverse_iterator it = numberpath.rbegin();
		it != numberpath.rend(); it++ ) {
		p = new path( g->GetNode( *it ), p );
	}
	// put the starting node onto the path
	p = new path( robberChain[level], p );

	while( level > 0 ) {
		// if we have an abstract path we still have to refine it down
		// to the lowest level
		//
		// this code is mainly copied from PRAStar.h
		g = gabs->GetAbstractGraph(level);

		// add all the nodes around the path to it (window/search radius of 1)
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
		//printf( "eligible node parents:\n" );
		//for( unsigned int i = 0; i < eligibleNodeParents.size(); i++ ) {
		//	printf( "%u ", eligibleNodeParents[i] );
		//}
		//printf( "\n" );

		// now refine
		node *target = p->tail()->n;
		node *lower_level_target = gabs->GetNthChild( target, random()%gabs->GetNumChildren(target));
		//printf( "refining path from %d (%d) to %d (%d)\n", robberChain[level-1]->GetNum(), robberChain[level-1]->getUniqueID(), lower_level_target->GetNum(), lower_level_target->getUniqueID() );

		path *ptemp = p;
		// verbose
		//printf( "trying to plan on abstraction level %d\n", level-1 );
		//printf( "from node %d (abs level %d)\n", robberChain[level-1]->GetNum(), robberChain[level-1]->GetLabelL(GraphAbstractionConstants::kAbstractionLevel) );
		//printf( "to node %d (abs level %d)\n", lower_level_target->GetNum(),
		//	lower_level_target->GetLabelL(GraphAbstractionConstants::kAbstractionLevel) );
		p = getAbstractPath( gabs->GetAbstractGraph(level-1),
			robberChain[level-1]->GetNum(),
			0xFFFFFFFF, eligibleNodeParents,
			GraphAbstractionConstants::kTemporaryLabel,
			lower_level_target->GetNum() );

		// cleanup
		delete ptemp;

		// proceed to the next level
		level--;
	}

	// convert p to a vector 
	// and cut off first node (since that is the start node)
	// (the path returned by PRA* has the current position in it
	// this can safely be erased)
	path *ptemp = p->next;
	while( ptemp ) {
		resultpath.push_back( ptemp->n );
		ptemp = ptemp->next;
	}

	// cleanup
	delete p;

	// get the statistics from PRA*
	myNodesExpanded += nodesExpanded;
	myNodesTouched  += nodesTouched;

	return;
};


node* DSDATPDijkstra::MakeMove( node* pos_robber, node* pos_cop, bool minFirst, double min_escape_length ) {
	std::vector<node*> path;
	datpdijkstra( pos_robber, pos_cop, path, minFirst, min_escape_length );
	// sanity check
	assert( path.size() > 0 );
	return( path[0] );
};

graphState DSDATPDijkstra::MakeMove( graphState pos_robber, graphState pos_cop, unsigned int ) {
	Graph *g = gabs->GetAbstractGraph( 0 );
	node *r = g->GetNode( pos_robber );
	node *c = g->GetNode( pos_cop );
	std::vector<node*> path;
	datpdijkstra( r, c, path, false, 10. );
	assert( path.size() > 0 );
	return( path[0]->GetNum() );
};
