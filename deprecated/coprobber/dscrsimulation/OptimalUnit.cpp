#include "OptimalUnit.h"
#include "GLUtil.h"

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
OptimalUnit::OptimalUnit( AbstractionGraphEnvironment *env, graphState initial_state, unsigned int _cop_speed, bool _I_am_a_robber, DSDijkstra_MemOptim *_dsdijkstra ):
	myenv( env ),
	current_pos(initial_state), copunit(-1), cop_speed(_cop_speed), done(false),
	I_am_a_robber(_I_am_a_robber), movecacheindex(0)
{
	if( _dsdijkstra == NULL ) {
		dsdijkstra = new DSDijkstra_MemOptim( env, _cop_speed );
		fprintf( stdout, "solving entire state space... " );
		fflush( stdout );
		dsdijkstra->dsdijkstra();
		fprintf( stdout, "done\n" );
		fflush( stdout );
		delete_dsdijkstra_when_getting_destroyed = true;
	} else {
		dsdijkstra = _dsdijkstra;
		delete_dsdijkstra_when_getting_destroyed = false;
	}
};

OptimalUnit::OptimalUnit( AbstractionGraphEnvironment *env, graphState initial_state, int _copunit, unsigned int _cop_speed, bool _I_am_a_robber, DSDijkstra_MemOptim *_dsdijkstra ):
	myenv( env ),
	current_pos(initial_state), copunit(_copunit), cop_speed(_cop_speed), done(false),
	I_am_a_robber(_I_am_a_robber), movecacheindex(0)
{
	if( _dsdijkstra == NULL ) {
		dsdijkstra = new DSDijkstra_MemOptim( env, _cop_speed );
		fprintf( stdout, "solving entire state space... " );
		fflush( stdout );
		dsdijkstra->dsdijkstra();
		fprintf( stdout, "done\n" );
		fflush( stdout );
		delete_dsdijkstra_when_getting_destroyed = true;
	} else {
		dsdijkstra = _dsdijkstra;
		delete_dsdijkstra_when_getting_destroyed = false;
	}
};

OptimalUnit::~OptimalUnit() {
	if( delete_dsdijkstra_when_getting_destroyed )
		delete dsdijkstra;
};


const char* OptimalUnit::GetName() {
	char *str; str = (char*) malloc( sizeof(char) * 20 );
	sprintf( str, "OptimalUnit" );
	return str;
};

bool OptimalUnit::MakeMove( AbstractionGraphEnvironment *env, OccupancyInterface<graphState,graphMove> *, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment> *sinfo, graphMove &a ) {

	// if the cop that we have to run away from is not set yet
	if( copunit < 0 || done ) {
		a = env->GetAction( current_pos, current_pos );
		return false;
	}

	if( movecacheindex < movecache.size() ) {
		a = env->GetAction( current_pos, movecache[movecacheindex] );
		movecacheindex++;
		return true;
	}

	movecacheindex = 0;

	DSDijkstra_MemOptim::CRState mypos;
	mypos.assign( 2, 0 );
	// get information from opponent
	PublicUnitInfo<graphState,graphMove,AbstractionGraphEnvironment> pui;
	sinfo->GetPublicUnitInfo( copunit, pui );

	if( I_am_a_robber ) {
		mypos[0] = current_pos;
		mypos[1] = pui.currentState;
		movecache.clear();
		graphState next_state = dsdijkstra->MakeMove( mypos, false );
		movecache.push_back( next_state );
	} else {
		mypos[0] = pui.currentState;
		mypos[1] = current_pos;
		dsdijkstra->MakeSingleStepsCopMove( mypos, movecache );
	}

	a = env->GetAction( current_pos, movecache[movecacheindex] );
	current_pos = movecache[movecacheindex];
	movecacheindex++;
	return true;
};

void OptimalUnit::UpdateLocation( AbstractionGraphEnvironment *, graphState &s, bool, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment> *sinfo ) {
	current_pos = s;
	if( copunit >= 0 ) {
		PublicUnitInfo<graphState,graphMove,AbstractionGraphEnvironment> pui;
		sinfo->GetPublicUnitInfo( copunit, pui );
		if( pui.currentState == current_pos )
			done = true;
	}
};



void OptimalUnit::OpenGLDraw( const AbstractionGraphEnvironment *env, const SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* ) const {
	node *n  = myenv->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = myenv->Scale()/4.;
	x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
	y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
	z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
	if( done )
		glColor3d( 0., 1., 0. ); // turn green when done
	else
		glColor3f( r, g, b );
	DrawSphere( x, y, z, rad );
	return;
};
