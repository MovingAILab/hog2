#include <math.h>
#include <vector>
#include "CopRobberGame.h"
#include "MyHash.h"

CopRobberGame::CopRobberGame( GraphEnvironment *_genv, unsigned int _num_cops, bool simultaneous, bool playerscanpass ):
	CopRobberEnvironment<graphState,graphMove>( (SearchEnvironment<graphState,graphMove>*) _genv, playerscanpass ),
	MarkovGame<graphState,graphMove>( simultaneous ),
	num_cops(_num_cops),
	genv(_genv),
	init_with(0),
	num_nodes(genv->GetGraph()->GetNumNodes()),
	dsdijkstra( NULL ), twocopsdijkstra( NULL )
{
}

CopRobberGame::~CopRobberGame() {
	if( dsdijkstra != NULL ) delete dsdijkstra;
	if( twocopsdijkstra != NULL ) delete twocopsdijkstra;
};

unsigned int CopRobberGame::GetNumPlayers() const {
	return num_cops+1;
}

void CopRobberGame::GetPossiblePlayerActions( unsigned int player, CRState s, std::vector<CRAction> &actions ) {

	std::vector<graphMove> orig_actions;

	// cleanup
	actions.clear();

	if( playerscanpass )
		actions.push_back( CRAction() );

	// get all possible actions of the player on the graph without respecting
	// the semantics of the game (CopRobberOccupancy)
	env->GetActions( s[player], orig_actions );

//	if( simultaneous ) {
		// if we are in a simultaneous game, all actions can be considered
		// as possible
		for( std::vector<graphMove>::iterator it = orig_actions.begin(); it != orig_actions.end(); it++ ) {
			actions.push_back( CRAction( *it ) );
		}
/*
	} else {
		// if we are not playing a simultaneous game, we can only consider the
		// actions possible, that are valid when the other agents stand still
		CRMove move;
		CRState temp;
		move.assign( num_cops+1, CRAction() );
		for( unsigned int i = 0; i < orig_actions.size(); i++ ) {
			move[player] = orig_actions[i];
			temp = s;
			ApplyAction( temp, move );
			if( oi->CanMove( s, temp ) )
				actions.push_back( CRAction( orig_actions[i] ) );
		}
	}
*/

	return;
}

void CopRobberGame::GetPossibleOpponentActions( unsigned int excluded_player, CRState s, std::vector<CRMove> &actions ) {

	// number of opponents is the number of all agents excluding exluded_player
	unsigned int num_opps = s.size() - 1;

	unsigned long i, j;
  unsigned int k;
  std::vector<graphMove> p_actions[num_opps+1];
  CRState p_state;
  graphState temp_s;
  // as=agent actions, n__=number of, m_=my
  unsigned int naa[num_opps], aa[num_opps], nmaa = 0, maa;
  // t___=total
  unsigned long tnaa = 0;
	bool found_action;

  actions.clear();


  // get the actions of each individual agent
	j = 0;
  for( i = 0; i < num_opps+1; i++ ) {
		if( i == excluded_player ) {
			if( simultaneous ) {
				env->GetActions( s[i], p_actions[i] );
				nmaa = p_actions[i].size() + (playerscanpass?1:0);
			}
			// if we are not in a simultaneous game, then the excluded player
			// stands still => CRAction()
		} else {
    	env->GetActions( s[i], p_actions[i] );
	    // if an agent can pass his turn then we augment all the numbers
 	   // by one and 0 becomes the pass action
 	   naa[j] = p_actions[i].size() + (playerscanpass?1:0);
 	   if( tnaa ) tnaa *= naa[j];
 	   else tnaa = naa[j];
		 j++;
		}
  }

	j = 0;
	for( i = 0; i < tnaa; i++ ) {
		dehash_permutation( i, num_opps, naa, aa );
		actions.push_back( CRMove() );
		p_state.clear();

		for( k = 0; k < excluded_player; k++ ) {
			if( playerscanpass && aa[k] == 0 ) {
				actions[j].push_back( CRAction() );
				p_state.push_back( s[k] );
			} else {
				actions[j].push_back( CRAction(p_actions[k][aa[k]-(playerscanpass?1:0)]) );
				// apply the action
				temp_s = s[k];
				env->ApplyAction( temp_s, actions[j][k].a );
				p_state.push_back( temp_s );
			}
		}
		// push_back a temporary action for the excluded_player
		actions[j].push_back( CRAction() );
		p_state.push_back( s[excluded_player] );
		// push_back the rest of the player actions
		for( k = excluded_player+1; k < num_opps+1; k++ ) {
			if( playerscanpass && aa[k-1] == 0 ) {
				actions[j].push_back( CRAction() );
				p_state.push_back( s[k] );
			} else {
				actions[j].push_back( CRAction(p_actions[k][aa[k-1]-(playerscanpass?1:0)]) );
				temp_s = s[k];
				env->ApplyAction( temp_s, actions[j][k].a );
				p_state.push_back( temp_s );
			}
		}

		found_action = false;

		if( simultaneous ) {
			// if we are in a simultaneous game, then all possible moves for the
			// opponents are the moves that work with at least one move from
			// excluded_player, thus we test all of them and break if we found one
			for( maa = 0; maa < nmaa; maa++ ) {
				if( !playerscanpass || maa != 0 ) {
					actions[j][excluded_player] = p_actions[excluded_player][maa-(playerscanpass?1:0)];
					temp_s = s[excluded_player];
					env->ApplyAction( temp_s, actions[j][excluded_player].a );
					p_state[excluded_player] = temp_s;
				}

				if( oi->CanMove( s, p_state ) ) {
					found_action = true;
					break;
				}
			}
		} else {
			// if we are not in a simultaneous game, then we just have to test
			// the created move (in which excluded_player does CRAction())
			if( oi->CanMove( s, p_state ) )
				found_action = true;
		}

		if( found_action )
			j++;
		else
			actions.pop_back();
	}

	// cleanup
	for( i = 0; i < num_opps+1; i++ )
		p_actions[i].clear();

	return;
}

double CopRobberGame::GetReward( unsigned int player, CRState s, std::vector<CRAction> act ) {
	// if the game has ended yet
	if( GoalTest( s, s ) ) return 0.;

/* TEMPORARY TODO
	double temp, result = 0.;
	// determine the maximum time the robber cannot be caught
	for( unsigned int i = 0; i < s.size(); i++ ) {
		if( !act[i].noaction ) {
			temp = env->GCost( s[i], act[i].a );
			if( result < temp ) result = temp;
		}
	}

	if( player ) return( -1. * result );
	else return result;
*/


	if( player ) return -1.; // cop
	else return 1.; // robber
}

// this works only for two players (one cop)
double CopRobberGame::InitState( CRState s ) {

	if( GoalTest( s ) ) return 0.;

	switch( init_with ) {
		case 0:
			// initialization with zeros
			return 0.; break;
		case 1: {
			// heuristic initialization
			double h = 0;
			for( unsigned int i = 1; i < s.size(); i++ )
				h = max( h, env->HCost( s[i], s[0] ) );
			return h; break;
		}
		case 2: {
			// cummulative heuristic initialization
			// only makes sense when rewards are 1
			double h = 0;
			for( unsigned int i = 1; i < s.size(); i++ )
				h = max( h, env->HCost( s[i], s[0] ) );
			return( 2*h - 1 ); break;
		}
		case 3: {
			switch( num_cops ) {
				case 1:
					return dsdijkstra->Value( s, true ); break;
				case 2:
					return twocopsdijkstra->Value( s[0], s[1], s[2] );
				default:
					fprintf( stderr, "ERROR: initialization for more than 2 cops is not supported.\n" );
			}
			break;
		}
		default:
			fprintf( stderr, "ERROR: type of initialization not supported\n" );
			exit( 1 );
			break;
	}
	return 0.;
}

void CopRobberGame::Init_With( int with ) {
	init_with = with;

	// solve the game
	if( with == 3 ) {
		switch( num_cops ) {
			case 1:
				dsdijkstra = new DSDijkstra_MemOptim( genv, 1 );
				dsdijkstra->dsdijkstra();
				break;
			case 2:
				twocopsdijkstra = new TwoCopsDijkstra( genv );
				twocopsdijkstra->dijkstra();
				break;
			default:
				// insert some more optimized Dijkstra in here
				fprintf( stderr, "ERROR: more than 2 cops for initialization are currently not supported\n" );
		}
	}

	return;
}



unsigned int CopRobberGame::GetNumStates() const {
	if( num_cops == 1 )
		return( num_nodes * num_nodes );
	else if( num_cops == 2 )
		return( num_nodes * (num_nodes + 1) / 2 * num_nodes );
	std::cerr << "ERROR: more than two cops are currently not supported." << std::endl;
	exit( 1 );
}

// This function is exploiting knowledge about how the GraphEnvironment works
// Unfortunately there is no easy way to avoid this
CopRobberGame::CRState CopRobberGame::GetStateByNumber( unsigned int num ) {
	CRState p_state;
/*
	unsigned int i, nps[num_cops+1], ps[num_cops+1], t;
	t = genv->GetGraph()->GetNumNodes();

	for( i = 0; i < num_cops+1; i++ )
		nps[i] = t;
	dehash_permutation( num, num_cops+1, nps, ps );
	for( i = 0; i < num_cops+1; i++ ) {
		p_state.push_back( ps[i] );
	}
*/

	// new version that uses good caching
	if( num_cops == 1 ) {
		p_state.push_back( (graphState) (num/num_nodes) );
		p_state.push_back( (graphState) (num%num_nodes) );
	} else if( num_cops == 2 ) {
		p_state.push_back( num / (num_nodes*(num_nodes+1)/2) );
		unsigned int h = num % (num_nodes*(num_nodes+1)/2);
		p_state.push_back( (unsigned int) floor( num_nodes + 0.5 - sqrt( (num_nodes+0.5)*(num_nodes+0.5) - 2 * h ) ) );
		p_state.push_back( h - p_state[1]*(p_state[1]+1)/2 - p_state[1]*(num_nodes-p_state[1]-1) );
	} else {
		std::cerr << "ERROR: we do not support more than two cops." << std::endl;
		exit( 1 );
	}

	return p_state;
}

// as GetStateByNumber this exploits knowledge about the GraphEnvironment
unsigned int CopRobberGame::GetNumberByState( CRState s ) {
/*
	unsigned int nps[num_cops+1], ps[num_cops+1], t;

	// security statement
	assert( s.size() == num_cops+1 );

	t = genv->GetGraph()->GetNumNodes();
	for( unsigned int i = 0; i < s.size(); i++ ) {
		nps[i] = t;
		ps[i]  = (unsigned int) s[i];
	}
	return (unsigned int) hash_permutation( num_cops+1, nps, ps );
*/

	// new code with new hashing
	if( num_cops == 1 ) {
		return( s[0] * num_nodes + s[1] );
	} else if( num_cops == 2 ) {
		if( s[1] > s[2] ) { graphState temp = s[1]; s[1] = s[2]; s[2] = temp; };
		return( s[0] * num_nodes*(num_nodes+1)/2 + s[1]*(s[1]+1)/2 + s[1]*(num_nodes-s[1]-1) + s[2] );
	} else {
		std::cerr << "ERROR: we do not support more than two cops yet." << std::endl;
		exit( 1 );
	}

}


void CopRobberGame::WriteExpectedRewardToDisc( const char* filename, unsigned int player, double *V ) {
	FILE *fhandler;
	unsigned int tns = GetNumStates(), tnp = GetNumPlayers(), i, j;
	CRState s;

	fhandler = fopen( filename, "w" );
	fprintf( fhandler, "number of players: %u\n", tnp );
	fprintf( fhandler, "states in space: %u\n", tns );
	fprintf( fhandler, "expected rewards for player: %u\n", player );
	fprintf( fhandler, "\n\n" );
	fprintf( fhandler, "positions:\n" );
	for( i = 0; i < tnp; i++ ) {
		fprintf( fhandler, "player%u ", i );
	}
	fprintf( fhandler, "expected reward\n" );
	for( i = 0; i < tns; i++ ) {
		s = GetStateByNumber( i );
		for( j = 0; j < tnp; j++ ) {
			fprintf( fhandler, "%lu ", s[j] );
		}
		fprintf( fhandler, "%g\n", V[i] );
	}
	fclose( fhandler );
	return;
}

void CopRobberGame::ReadExpectedRewardFromDisc( const char* filename, unsigned int &player, double *&V ) {
	FILE *fhandler;
	unsigned int tnp, tns, i, j;
	graphState ss;
	CRState s;

	fhandler = fopen( filename, "r" );
	if( fhandler == 0 ) {
		fprintf( stderr, "ERROR: File %s not found\n", filename );
		return;
	}

	fscanf( fhandler, "number of players: %u\n", &tnp );
	if( tnp != GetNumPlayers() ) {
		fprintf( stderr, "ERROR: Wrong number of players in %s (should be %u)\n", filename, GetNumPlayers() );
		return;
	}
	fscanf( fhandler, "states in space: %u\n", &tns );
	if( tns != GetNumStates() ) {
		fprintf( stderr, "ERROR: Wrong number of states in space in %s (should be %u)\n", filename, GetNumStates() );
		return;
	}
	fscanf( fhandler, "expected rewards for player: %u\n", &player );
	fscanf( fhandler, "\n\n" );

	fscanf( fhandler, "positions:\n" );
	for( i = 0; i < tnp; i++ ) fscanf( fhandler, "player%*u " );
	fscanf( fhandler, "expected reward\n" );

	// allocate V
	V = new double[tns];

	// first assignment
	for( i = 0; i < tns; i++ )
		V[i] = 0.;

	for( i = 0; i < tns; i++ ) {
		s.clear();
		for( j = 0; j < tnp; j++ ) {
			fscanf( fhandler, "%lu", &ss );
			s.push_back( ss );
		}
		fscanf( fhandler, "%lg\n", &V[GetNumberByState(s)] );
	}
	fclose( fhandler );
}
