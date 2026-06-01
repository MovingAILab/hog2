#include "RobberUnit.h"
#include "GLUtil.h"
#include "gsl/gsl_matrix.h"
#include <time.h>

RobberUnit::RobberUnit( xyLoc _loc, MultilevelCopRobberGame *_game, const char* filename ):
	BaseAbsMapUnit::Unit(),
	loc(_loc),
	game(_game), num_levels(0)
{
	unsigned int player;
	expected_rewards = new double*[1];

	game->GetMarkovGame(0)->ReadExpectedRewardFromDisc( filename, player, expected_rewards[0] );
}

RobberUnit::RobberUnit( xyLoc _loc, MultilevelCopRobberGame *_game, const char* filename, std::vector<unsigned int> _copunits ):
	BaseAbsMapUnit::Unit(),
	loc(_loc),
	game(_game), num_levels(0)
{
	unsigned int player;
	expected_rewards = new double*[1];

	game->GetMarkovGame(0)->ReadExpectedRewardFromDisc( filename, player, expected_rewards[0] );
	SetCopUnits( _copunits );
}

RobberUnit::RobberUnit( xyLoc _loc, MultilevelCopRobberGame *_game,
	unsigned int maxlevel, unsigned int maxiter, double gamma,
	double epsilon, double precision ):
	BaseAbsMapUnit::Unit(),
	loc(_loc),
	game(_game),
	num_levels(maxlevel)
{
	unsigned int *iter;

	game->GetExpectedStateRewards( 0, gamma, epsilon, precision, expected_rewards, iter, maxlevel, maxiter );

	// cleanup right away
	delete[] iter;
}

RobberUnit::~RobberUnit() {
	for( unsigned int i = 0; i < num_levels; i++ )
		delete[] expected_rewards[i];
	delete[] expected_rewards;
}

void RobberUnit::SetCopUnits( std::vector<unsigned int> _copunits ) {
	if( _copunits.size() + 1 == game->GetMarkovGame(0)->GetNumPlayers() )
		copunits = _copunits;
	else {
		fprintf( stderr, "Warning: Number of submitted cop units is not equal to the number of cops in\n" );
		fprintf( stderr, "the submitted game. Cop units will be ignored.\n" );
	}

	return;
}

bool RobberUnit::MakeMove(AbsMapEnvironment *env, OccupancyInterface<xyLoc,tDirection> *, AbsMapSimulationInfo *info, tDirection &a) {

	// if we do not have any cops yet
	if( copunits.size() == 0 ) {
		a = kStay;
		return true;
	}

	CopRobberGame::CRState s;
	xyLoc l;
	MarkovGame<graphState,graphMove> *mgame = game->GetMarkovGame(0);
	unsigned int i;
	CopRobberGame::CRAction my_action;

	assert( copunits.size() + 1 == mgame->GetNumPlayers() );

	PublicUnitInfo<xyLoc,tDirection,AbsMapEnvironment> pui;

	// push my position onto the state
	s.push_back( env->GetMapAbstraction()->GetNodeFromMap( loc.x, loc.y )->GetNum() );
	// get the current position of all the cops
	for( i = 0; i < copunits.size(); i++ ) {
		info->GetPublicUnitInfo( copunits[i], pui );
		l = pui.currentState;
		s.push_back( env->GetMapAbstraction()->GetNodeFromMap( l.x, l.y )->GetNum() );
	}

	// we cannot move anymore, goal reached!!!
	if( mgame->GoalTest( s, s ) ) {
		done = true;
		a = kStay;
		return false;
	}

	// determine when we and the cops move next
	info->GetPublicUnitInfo( GetNum(), pui );
	double myNextMoveTime = pui.nextTime;
	info->GetPublicUnitInfo( copunits[0], pui );
	double copsNextMoveTime = pui.nextTime;
	bool copsmovesimultaneously = true;
	for( i = 1; i < copunits.size(); i++ ) {
		info->GetPublicUnitInfo( copunits[i], pui );
		double temp = pui.nextTime;
		if( copsNextMoveTime != temp )
			copsmovesimultaneously = false;
		if( copsNextMoveTime > temp )
			copsNextMoveTime = temp;
	}

	if( copsNextMoveTime > myNextMoveTime ) {
		// MINIMAX

		my_action = MakeAlternatingMove( s, mgame );

	} else {
		// SIMULATENOUS ACTIONS
		if( !copsmovesimultaneously )
			fprintf( stderr, "Warning: Cops do not move simultaneously but switched to simultaneous action game\n" );

		my_action = MakeSimultaneousMove( s, mgame );
	}


	// transform the action into a tDirection
	if( my_action.noaction ) {
		a = kStay;
	} else {
		int x1, x2, y1, y2;
		env->GetMapAbstraction()->GetTileFromNode( env->GetMapAbstraction()->GetAbstractGraph(0)->GetNode( my_action.a.from ), x1, y1 );
		env->GetMapAbstraction()->GetTileFromNode( env->GetMapAbstraction()->GetAbstractGraph(0)->GetNode( my_action.a.to ), x2, y2 );
		xyLoc l1( x1, y1 ), l2( x2, y2 );
		a = env->GetAction( l1, l2 );
	}

	return true;
}

void RobberUnit::UpdateLocation(AbsMapEnvironment *env, xyLoc &l, bool, AbsMapSimulationInfo *info) {
	loc = l;

/*
	// from this point on we use l as a temporal variable since we updated our location yet

	// check whether we've been captured
	CopRobberGame::CRState s;
	MarkovGame<graphState,graphMove> *mgame = game->GetMarkovGame(0);

	// push my position onto the state
	s.push_back( env->GetMapAbstraction()->GetNodeFromMap( loc.x, loc.y )->GetNum() );
	// get the current position of all the cops
	for( unsigned int i = 0; i < copunits.size(); i++ ) {
		PublicUnitInfo<xyLoc,tDirection,AbsMapEnvironment> pui;
		info->GetPublicUnitInfo( i, pui );
		l = pui.currentState;
		s.push_back( env->GetMapAbstraction()->GetNodeFromMap( l.x, l.y )->GetNum() );
	}

	// we cannot move anymore, goal reached!!!
	if( mgame->GoalTest( s, s ) )
		done = true;
*/

	return;
}



void RobberUnit::OpenGLDraw(int , AbsMapEnvironment *me, AbsMapSimulationInfo *)
{
	Map *map = me->GetMap();
	GLdouble xx, yy, zz, rad;
	if ((loc.x >= map->GetMapWidth()) || (loc.y >= map->GetMapHeight()))
		return;
	map->GetOpenGLCoord(loc.x, loc.y, xx, yy, zz, rad);
	glColor3f(r, g, b);
//	if (getObjectType() == kDisplayOnly)
//		drawTriangle(xx, yy, zz, rad);
//	else
	DrawSphere(xx, yy, zz, rad);
}




CopRobberGame::CRAction RobberUnit::MakeAlternatingMove( CopRobberGame::CRState s, MarkovGame<graphState, graphMove> *mgame ) {

	unsigned int n_my_actions, n_opp_actions, i, j;
	std::vector<CopRobberGame::CRAction> my_actions;
	std::vector<CopRobberGame::CRMove> opp_actions;
	CopRobberGame::CRMove tempmove;
	CopRobberGame::CRState tempstate;
	double max, min;
	unsigned int my_move = 0;

	mgame->GetPossiblePlayerActions( 0, s, my_actions );
	mgame->GetPossibleOpponentActions( 0, s, opp_actions );
	n_my_actions  = my_actions.size();
	n_opp_actions = opp_actions.size();

	if( n_opp_actions == 0 ) return CopRobberGame::CRAction();
	if( n_my_actions  == 0 ) return CopRobberGame::CRAction();

	max = -DBL_MAX;
	for( i = 0; i < n_my_actions; i++ ) {
		min = DBL_MAX;

		for( j = 0; j < n_opp_actions; j++ ) {
			tempmove    = opp_actions[j];
			tempmove[0] = my_actions[i];
			tempstate = s;
			mgame->ApplyAction( tempstate, tempmove );

			if( mgame->GetOccupancyInfo()->CanMove( s, tempstate ) ) {
				if( min > expected_rewards[0][mgame->GetNumberByState( tempstate )] )
					min = expected_rewards[0][mgame->GetNumberByState( tempstate )];
			}
			// ignore the else case
		}

		if( max < min ) {
			max = min;
			my_move = i;
		}
	}

	return my_actions[my_move];
}


CopRobberGame::CRAction RobberUnit::MakeSimultaneousMove( CopRobberGame::CRState s, MarkovGame<graphState, graphMove> *mgame ) {

	unsigned int n_my_actions, n_opp_actions, i, j;
	double rand1, rand2;
	std::vector<CopRobberGame::CRAction> my_actions;
	std::vector<CopRobberGame::CRMove> opp_actions;
	CopRobberGame::CRMove tempmove;
	CopRobberGame::CRState tempstate;
	gsl_matrix *M;
	gsl_vector *p1, *p2;

	mgame->GetPossiblePlayerActions( 0, s, my_actions );
	mgame->GetPossibleOpponentActions( 0, s, opp_actions );
	n_my_actions  = my_actions.size();
	n_opp_actions = opp_actions.size();

/*
	printf( "Robber Actions: " );
	for( i = 0; i < n_my_actions; i++ ) {
		if( my_actions[i].noaction )
			printf( "noaction ");
		else
			printf( "%u=>%u ", my_actions[i].a.from, my_actions[i].a.to );
	}
	printf( "\n" );
	printf( "opp actions: " );
	for( unsigned int h = 0; h < n_opp_actions; h++ ) {
		if( opp_actions[h][1].noaction )
			printf( "noaction " );
		else
			printf( "%u=>%u ", opp_actions[h][1].a.from, opp_actions[h][1].a.to );
	}
	printf( "\n" );
*/

	if( n_opp_actions == 0 ) return CopRobberGame::CRAction();
	if( n_my_actions  == 0 ) return CopRobberGame::CRAction();

	M = gsl_matrix_calloc( n_opp_actions, n_my_actions );

	for( i = 0; i < n_opp_actions; i++ ) {
		tempmove = opp_actions[i];

		for( j = 0; j < n_my_actions; j++ ) {
			tempmove[0] = my_actions[j];
			tempstate = s;
			mgame->ApplyAction( tempstate, tempmove );

			if( mgame->GetOccupancyInfo()->CanMove( s, tempstate ) ) {
				gsl_matrix_set( M, i, j, expected_rewards[0][mgame->GetNumberByState( tempstate )] );
			} else {
				// TODO
				// this is not correct because the move can not be possible because the robber and cop
				// switch position (in which case the robber loses, thus it should be more 0.
				// gsl_matrix_set( M, i, j, expected_rewards[0][mgame->GetNumberByState( s )] );
				// 0. is only(!) correct when used for the calculation of the robber!!!!!
				gsl_matrix_set( M, i, j, 0. );
			}

		}
	}

/*
	printf( "M = \n" );
	for( i = 0; i < M->size1; i++ ) {
		for( j = 0; j < M->size2; j++ ) {
			printf( "%g ", gsl_matrix_get( M, i, j ) );
		}
		printf( "\n" );
	}
*/

	p1 = gsl_vector_alloc( n_opp_actions );
	p2 = gsl_vector_alloc( n_my_actions );

	int k;
	jrobinson( M, ROBBERUNIT_EPSILON, p1, p2, k );

/*	printf( "p2 is:\n" ); gsl_vector_fprintf( stdout, p2, "%g" ); */

	srand( time( NULL ) );
	rand1 = (double)rand() / (double)RAND_MAX;
	rand2 = 0.;
	for( i = 0; i < n_my_actions; i++ ) {
		if( rand2 < rand1 && rand1 < rand2 + gsl_vector_get( p2, i ) )
			break;
		else
			rand2 += gsl_vector_get( p2, i );
	}
/*	printf( "selection is %u\n", i ); */

	gsl_matrix_free( M );
	gsl_vector_free( p1 );
	gsl_vector_free( p2 );

	return my_actions[i];
}
