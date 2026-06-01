#include "DSPRAStarCop.h"

DSPRAStarCop::DSPRAStarCop( GraphAbstraction *_graphabs, unsigned int _cop_speed ):
	graphabs(_graphabs), cop_speed(_cop_speed), pathfraction(2), g(graphabs->GetAbstractGraph(0)),
	pra(new praStar())
{ };

DSPRAStarCop::~DSPRAStarCop() {
	delete pra;
};

graphState DSPRAStarCop::MakeMove( graphState &robber, graphState &cop, float &gcost ) {

	nodesExpanded = 0; nodesTouched = 0;
	gcost = 0.;

	if( robber == cop ) return robber;

	if( pathcache.size() == 0 ) {
		// compute new pathcache

		node *r = g->GetNode( robber );
		node *c = g->GetNode( cop );

		path *prapath  = pra->GetPath( graphabs, c, r );
		nodesExpanded  = pra->nodesExpanded;
		nodesTouched   = pra->nodesTouched;
		path *temppath = prapath;

		unsigned int steps = prapath->length() / pathfraction / cop_speed;
		if( steps == 0 ) steps = 1; // make at least one step

		for( unsigned int i = 0; i < steps; i++ ) {
			gcost = 0.;
			for( unsigned int j = 0; j < cop_speed; j++ ) {
				if( temppath->next != NULL ) {
					gcost += graphabs->h(temppath->n,temppath->next->n);
					temppath = temppath->next;
				}
				else
					break;
			}
			gcosts.push_back( gcost );
			pathcache.push_back( temppath->n->GetNum() );
			if( temppath->next == NULL ) break;
		}

		// cleanup
		delete prapath;
		// done computing new pathcache
	}

	if( pathcache.size() > 0 ) {
		gcost = gcosts[0];
		gcosts.erase( gcosts.begin() );
		graphState result = pathcache[0];
		pathcache.erase( pathcache.begin() );
		return result;
	} else {
		// if we cannot make a move, we'll stay where we are
		gcost = 1.;
		return cop;
	}
};
