#include "DSRandomBeacons.h"

DSRandomBeacons::DSRandomBeacons( MapAbstraction *_mabs, bool canPass, unsigned int cop_speed ):
	mabs(_mabs),
	gh( new MaximumNormAbstractGraphMapHeuristic( mabs->GetAbstractGraph(0), mabs->GetMap() ) ),
	env( new GraphEnvironment( mabs->GetAbstractGraph(0), gh ) ),
	dscrenv( new DSCREnvironment<graphState,graphMove>( env, canPass, cop_speed ) ),
	pra( new praStar() )
{
	// initialize the random number generator
	//time_t t; time( &t ); srandom( (unsigned int) t );
};

DSRandomBeacons::~DSRandomBeacons() {
	delete dscrenv;
	delete env;
	delete gh;
	delete pra;
};

void DSRandomBeacons::GetPath( graphState pos_robber, graphState pos_cop, unsigned int num_beacons, std::vector<graphState> &resultpath ) {

	resultpath.clear();
	nodesExpanded = 0; nodesTouched = 0;

	if( pos_robber == pos_cop ) return;

	node *target = NULL;
	double target_h = -DBL_MAX;

	Graph *g = mabs->GetAbstractGraph(0);
	node *probber = g->GetNode( pos_robber );

	for( unsigned int i = 0; i < num_beacons; i++ ) {
		node *r = g->GetRandomNode();
		nodesTouched++;
		graphState rs = r->GetNum();
		double r_h = dscrenv->HCost( rs, pos_cop );
		//printf( "testing node %d: %g\n", r->GetNum(), r_h );
		
		if( mabs->Pathable(probber,r) && (target == NULL || fgreater(r_h, target_h)) ) {
			target = r;
			target_h = r_h;
		}
	}

	if( target == NULL || probber == target ) {
		// if we didn't find a new target at all, i.e. we couldn't find a node
		// to run because all random nodes weren't pathable, stay where you are
		resultpath.push_back( pos_robber );
	} else {
		// generate a PRA* path to this target
		path *p = pra->GetPath( mabs, probber, target );
		nodesExpanded += pra->nodesExpanded;
		nodesTouched  += pra->nodesTouched;
		// transform the path into a vector
		path *trav = p->next;
		while( trav ) {
			resultpath.push_back( trav->n->GetNum() );
			trav = trav->next;
		}
		// cleanup
		delete p;
	}

	return;
};

graphState DSRandomBeacons::MakeMove( graphState pos_robber, graphState pos_cop, unsigned int ) {
	std::vector<graphState> path;
	GetPath( pos_robber, pos_cop, 40, path );
	return( path[0] );
};

