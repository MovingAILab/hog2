#include <vector>
#include <values.h>
#include "MultiAgentEnvironment.h"
#include "JRobinson.h"
#include "gsl/gsl_matrix.h"
#include "gsl/gsl_blas.h"
#include "MyHash.h"

#ifndef MARKOVGAME_H
#define MARKOVGAME_H

/*!
	Definition of a Markov Game class that will be used to compute expected
	rewards for all states which can be used to compute optimal policies.
*/
template<class state, class action>
class MarkovGame: public virtual MultiAgentEnvironment<state,action> {
	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState MAState;
	typedef typename MultiAgentEnvironment<state,action>::MAMove MAMove;
	typedef typename MultiAgentEnvironment<state,action>::SAAction SAAction; // single agent action

	MarkovGame( bool _simultaneous = false ): simultaneous(_simultaneous) {};
	virtual ~MarkovGame() {};

	virtual void SetSimultaneous( bool _simultaneous ) { simultaneous = _simultaneous; };

	/*!
		Players are indexed beginning at 0.
	*/
	virtual unsigned int GetNumPlayers() const = 0;

	/*! gives back all possible actions that the player could take */
	virtual void GetPossiblePlayerActions( unsigned int player, MAState s, std::vector<SAAction> &actions ) = 0;
	virtual void GetPossibleOpponentActions( unsigned int excluded_player, MAState s, std::vector<MAMove> &actions ) = 0;
	/*!
		Gives back the reward for the player when all players chose
		to do the moves in act simultaneously. It might even be, that this
		move is not possible in the environment, then set the reward
		to whatever the signification of the game is.
	*/
	virtual double GetReward( unsigned int player, MAState s, std::vector<SAAction> act ) = 0;
	virtual double InitState( MAState s ) = 0;

	virtual unsigned int GetNumStates() const = 0;

	/*!
		This interface adds a method to iterate through the states with the
		help of a number. This is needed to run simultaneous updates on all
		states of the environment. We chose a number instead of an iterator
		concept to avoid hash maps that can be quite expensive in memory.
		In most domains we can number the states right away (which renders
		hash maps useless), in all others you can easily implement this number
		concepts with the help of a hash map (yourself).
	*/
	virtual MAState GetStateByNumber( unsigned int num ) = 0;
	virtual unsigned int GetNumberByState( MAState s ) = 0;

	// make sure iter is set to zero, the number of iterations will be appended
	// if V is allocated, it will be used...
	// maxiter can be set to disregard precision and just use a fixed number of iterations
	virtual void GetExpectedStateRewards( unsigned int player, double gamma, double epsilon, double &precision, double* &V, unsigned int &iter, unsigned int maxiter = 0 );

	virtual void WriteExpectedRewardToDisc( const char* filename, unsigned int player, double *V ) = 0;
	virtual void ReadExpectedRewardFromDisc( const char* filename, unsigned int &player, double *&V ) = 0;

	protected:
	virtual void GetExpectedStateRewardsSimultaneousActionGame( unsigned int player, double gamma, double epsilon, double &precision, double* &V, unsigned int &iter, unsigned int maxiter = 0 );
	virtual void GetExpectedStateRewardsAlternatingActionGame( unsigned int player, double gamma, double epsilon, double &precision, double* &V, unsigned int &iter, unsigned int maxiter = 0 );

	bool simultaneous;

};


/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
| Implementation
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

template<class state, class action>
void MarkovGame<state,action>::GetExpectedStateRewards( unsigned int player, double gamma, double epsilon, double &precision, double* &V, unsigned int &iter, unsigned int maxiter ) {
	if( simultaneous ) {
		GetExpectedStateRewardsSimultaneousActionGame( player, gamma, epsilon, precision, V, iter, maxiter );
	} else {
		GetExpectedStateRewardsAlternatingActionGame(  player, gamma, epsilon, precision, V, iter, maxiter );
	}
	return;
}

template<class state, class action>
void MarkovGame<state,action>::GetExpectedStateRewardsSimultaneousActionGame( unsigned int player, double gamma, double epsilon, double &precision, double* &V, unsigned int &iter, unsigned int maxiter ) {

	unsigned int i,j,k;
	unsigned int tnes; // total number of environment states
	double *Vs[2], *norm_vec, norm = 0.;
	gsl_vector_view nvecview;
	unsigned char v_old = 0;
	MAState p_state, p_tempstate;
	MAMove p_tempaction;
	std::vector<MAMove> opp_actions;
	std::vector<SAAction> my_actions;
	unsigned int n_my_actions, n_opp_actions;
	int iter2, iter3 = 0;
	unsigned int iter4 = 0;
	gsl_matrix *M;
	gsl_vector *p1, *p2;
	double Q;
	gsl_vector **Previous_Strategy_Opponent, **Previous_Strategy_Player;
	double strategy_difference_norm_opponent, strategy_difference_norm_player;

//	iter = 0;
	tnes = GetNumStates();

	// allocate space for the V_new and V_old array
	if( V ) {
		Vs[0] = V;
	} else {
		Vs[0] = new double[tnes];
	}
	Vs[1] = new double[tnes];
	norm_vec = new double[tnes];
	nvecview = gsl_vector_view_array( norm_vec, tnes );
	Previous_Strategy_Opponent = (gsl_vector**) malloc( tnes * sizeof( gsl_vector* ) );
	Previous_Strategy_Player   = (gsl_vector**) malloc( tnes * sizeof( gsl_vector* ) );

	// initialize the values
	for( i = 0; i < tnes; i++ ) {
		if( V == NULL )
			Vs[0][i] = InitState( GetStateByNumber( i ) ); //0.;
		Vs[1][i] = Vs[0][i];
		norm_vec[i] = 0.;
		Previous_Strategy_Opponent[i] = NULL;
		Previous_Strategy_Player[i]   = NULL;
	}

	// make an update to all the values
	do {

		strategy_difference_norm_opponent = 0.;
		strategy_difference_norm_player   = 0.;

		// for all possible states
		for( i = 0; i < tnes; i++ ) {

			p_state = GetStateByNumber( i );
/*
			// VERBOSE PRINTOUT
			printf( "---------\n" );
			printf( "i = %u => %lu %lu\n", i, p_state[0], p_state[1] );
*/

			// if we are at the end of the game, do not update anymore
			if( GoalTest( p_state ) ) continue;

			// get my and the opponent actions
			GetPossiblePlayerActions( player, p_state, my_actions );
			GetPossibleOpponentActions( player, p_state, opp_actions );
			n_my_actions  = my_actions.size();
			n_opp_actions = opp_actions.size();
/*
			// VERBOSE PRINTOUT
			printf( "my actions: " );
			for( unsigned int h = 0; h < n_my_actions; h++ ) {
				if( my_actions[h].noaction )
					printf( "noaction " );
				else20090517-cover_correction/
					printf( "%u=>%u ", my_actions[h].a.from, my_actions[h].a.to );
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

			// if there are no actions then do not update the state
			// but instead continue with the rest
			if( n_opp_actions == 0 ) continue;
			if( n_my_actions == 0 ) continue;


			// allocate the storage for the matrix of our matrix game
			M = gsl_matrix_calloc( n_opp_actions, n_my_actions );

			// for all possible actions
			// fill up the matrix of our matrix game
			for( j = 0; j < n_opp_actions; j++ ) {
				// p_tempaction.clear();
				p_tempaction = opp_actions[j];

				for( k = 0; k < n_my_actions; k++ ) {
					p_tempaction[player] = my_actions[k];
					p_tempstate = p_state;
					ApplyAction( p_tempstate, p_tempaction );

					if( this->GetOccupancyInfo()->CanMove( p_state, p_tempstate ) ) {
						Q = GetReward( player, p_state, p_tempaction ) +
						    gamma * Vs[(size_t)v_old][GetNumberByState(p_tempstate)];
					} else {
						Q = Vs[(size_t)v_old][i];
					}

/*
					// VERBOSE PRINTOUT
					if( Q == 0. ) {
						printf( "--- Q = 0. ---\n" );
						for( unsigned int h = 0; h < p_tempaction.size(); h++ ) {
							if( p_tempaction[h].noaction )
								printf( "noaction " );
							else
								printf( "%u=>%u ", p_tempaction[h].a.from, p_tempaction[h].a.to );
						}
						printf( "\n" );
					}
*/

					gsl_matrix_set( M, j, k, Q );
				}
			}


/*
			// VERBOSE PRINTOUT
			printf( "----\n" );
			for( j = 0; j < M->size1; j++ ) {
				for( k = 0; k < M->size2; k++ ) {
					printf( "%g ", gsl_matrix_get( M, j, k ) );
				}
				printf( "\n" );
			}
*/


			p1 = gsl_vector_alloc( n_opp_actions );
			p2 = gsl_vector_alloc( n_my_actions );

			// compute the randomized optimal strategy for us (=p2)
			jrobinson( M, epsilon, p1, p2, iter2 );
			iter3 += iter2;

			if( iter4 > 0 ) {
				// in every iteration other than the first compare to the previous strategy
				gsl_vector_sub( Previous_Strategy_Opponent[i], p1 );
				gsl_vector_sub( Previous_Strategy_Player[i], p2 );
				strategy_difference_norm_opponent += gsl_vector_max( Previous_Strategy_Opponent[i] );
				strategy_difference_norm_player   += gsl_vector_max( Previous_Strategy_Player[i] );
			} else {
				// initialize the two vectors
				Previous_Strategy_Opponent[i] = gsl_vector_alloc( n_opp_actions );
				Previous_Strategy_Player[i]   = gsl_vector_alloc( n_my_actions );
			}
			// store the values
			gsl_vector_memcpy( Previous_Strategy_Opponent[i], p1 );
			gsl_vector_memcpy( Previous_Strategy_Player[i],   p2 );


/*
			// VERBOSE PRINTOUT
			printf( "p1 = \n" ); gsl_vector_fprintf( stdout, p1, "%g" );
			printf( "p2 = \n" ); gsl_vector_fprintf( stdout, p2, "%g" );
*/

			// compute the value of the game for us!
			// (in general the game should have the same value for us as for our
			// opponents (accumulated together in one opponent). However, this
			// must not be the case if there exist pure strategies...
			// we calculate min M*p2
			gsl_vector_set_zero( p1 );
			gsl_blas_dgemv( CblasNoTrans, 1.0, M, p2, 1.0, p1 );
			Vs[(size_t)(v_old+1)&1][i] = gsl_vector_min( p1 );

			// cleanup matrix game
			gsl_matrix_free( M );
			gsl_vector_free( p1 );
			gsl_vector_free( p2 );

			// update the difference vector
			norm_vec[i] = Vs[(size_t)(v_old+1)&1][i] - Vs[(size_t)v_old][i];
		}
		v_old = (v_old + 1 ) & 1; // (v_old+1)%2
		iter++;
		iter4++;

		norm = gsl_vector_max( &(nvecview.vector) );
		printf( "%g %g %g\n", norm, strategy_difference_norm_opponent/tnes, strategy_difference_norm_player/tnes );
		fflush( stdout );
		//printf( "maximum norm of difference = %g\n", norm );
		//norm = gsl_blas_dnrm2( &(nvecview.vector) );
		//printf( "euclidean norm of difference = %g\n", norm );

/*
		// VERBOSE
		if( iter%20 == 0 ) {
			printf( "iteration %d reached with %d J.Robinson iterations and norm %f\n", iter, iter3, norm );
			fflush( stdout );
		}
*/

	} while ( (maxiter)?(iter4<maxiter):(norm>precision) );

	if( maxiter )
		precision = norm;

	// cleanup
	delete [] Vs[(size_t)(v_old+1)&1];
	delete [] norm_vec;

	for( unsigned int i = 0; i < tnes; i++ ) {
		if( Previous_Strategy_Opponent[i] != NULL ) {
			gsl_vector_free( Previous_Strategy_Opponent[i] );
			gsl_vector_free( Previous_Strategy_Player[i] );
		}
	}
	delete Previous_Strategy_Opponent;
	delete Previous_Strategy_Player;


	V = Vs[(size_t)v_old];
	return;
}


template<class state, class action>
void MarkovGame<state,action>::GetExpectedStateRewardsAlternatingActionGame( unsigned int player, double gamma, double epsilon, double &precision, double* &V, unsigned int &iter, unsigned int maxiter ) {

	unsigned int i,j,k;
	unsigned int tnes; // total number of environment states
	double *Vs[2], *norm_vec, norm = 0.;
	gsl_vector_view nvecview;
	unsigned char v_old = 0;
	MAState p_state, p_tempstate;
	MAMove p_tempaction;
	std::vector<MAMove> opp_actions;
	std::vector<SAAction> my_actions;
	unsigned int n_my_actions, n_opp_actions;
	unsigned int iter4 = 0;
	double max, min;
	double Q;

	tnes = GetNumStates();

	// allocate space for the V_new and V_old array
	if( V ) {
		Vs[0] = V;
	} else {
		Vs[0] = new double[tnes];
	}
	Vs[1] = new double[tnes];
	norm_vec = new double[tnes];
	nvecview = gsl_vector_view_array( norm_vec, tnes );

	// initialize the values
	for( i = 0; i < tnes; i++ ) {
		if( V == NULL )
			Vs[0][i] = InitState( GetStateByNumber( i ) ); //0.;
		Vs[1][i] = Vs[0][i];
		norm_vec[i] = 0.;
	}

	// make an update to all the values
	do {

		// for all possible states
		for( i = 0; i < tnes; i++ ) {

			p_state = GetStateByNumber( i );
/*
			// VERBOSE PRINTOUT
			printf( "---------\n" );
			printf( "i = %u => %lu %lu %lu\n", i, p_state[0], p_state[1], p_state[2] );
*/

			// if we are at the end of the game, do not update anymore
			if( GoalTest( p_state, p_state ) ) continue;

			// get my and the opponent actions
			GetPossiblePlayerActions( player, p_state, my_actions );
			GetPossibleOpponentActions( player, p_state, opp_actions );
			n_my_actions  = my_actions.size();
			n_opp_actions = opp_actions.size();

			// if there are no actions then do not update the state
			// but instead continue with the rest
			if( n_opp_actions == 0 ) continue;
			if( n_my_actions == 0 ) continue;

/*
			// VERBOSE PRINTOUT
			printf( "my actions: " );
			for( unsigned int h = 0; h < n_my_actions; h++ ) {
				if( my_actions[h].noaction )
					printf( "noaction " );
				else
					printf( "%u=>%u ", my_actions[h].a.from, my_actions[h].a.to );
			}
			printf( "\n" );
			printf( "opp actions: " );
			for( unsigned int h = 0; h < n_opp_actions; h++ ) {
				if( opp_actions[h][1].noaction )
					printf( "(noaction " );
				else
					printf( "(%u=>%u ", opp_actions[h][1].a.from, opp_actions[h][1].a.to );
				if( opp_actions[h][2].noaction )
					printf( "noaction) " );
				else
					printf( "%u=>%u) ", opp_actions[h][2].a.from, opp_actions[h][2].a.to );
			}
			printf( "\n" );
*/

			max = -DBL_MAX;

			for( k = 0; k < n_my_actions; k++ ) {
				min = DBL_MAX;

				for( j = 0; j < n_opp_actions; j++ ) {

					p_tempstate = p_state;
					ApplyAction( p_tempstate, opp_actions[j] );

					if( GoalTest( p_tempstate, p_tempstate ) ) {
						Q = GetReward( player, p_state, opp_actions[j] ) +
						    gamma * Vs[(size_t)v_old][GetNumberByState(p_tempstate)];
						if( min > Q ) min = Q;
					} else {

						p_tempaction = opp_actions[j];
						p_tempaction[player] = my_actions[k];

						p_tempstate = p_state;
						ApplyAction( p_tempstate, p_tempaction );

						if( this->GetOccupancyInfo()->CanMove( p_state, p_tempstate ) ) {
							Q = GetReward( player, p_state, p_tempaction ) +
							    gamma * Vs[(size_t)v_old][GetNumberByState(p_tempstate)];

							if( min > Q ) {
								min = Q;
//							printf( "u" );
							}
						}
//						printf( "%g ", Q );
					}
					// we can safely ignore the else case here (not so in SimultaneousActions)

				}
//				printf( "\n" );

				if( max < min ) max = min;
			}

//			printf( "=> %g\n", max );
			Vs[(size_t)(v_old+1)&1][i] = max;

			// update the difference vector
			norm_vec[i] = Vs[(size_t)(v_old+1)&1][i] - Vs[(size_t)v_old][i];
		}
		v_old = (v_old + 1 ) & 1; // (v_old+1)%2
		iter++;
		iter4++;

		norm = gsl_vector_max( &(nvecview.vector) );
		printf( "maximum norm of difference = %g\n", norm );
		//norm = gsl_blas_dnrm2( &(nvecview.vector) );
		//printf( "norm = %g\n", norm );

/*
		if( iter%20 == 0 ) {
			printf( "iteration %d reached with %d J.Robinson iterations and norm %f\n", iter, iter3, norm );
			fflush( stdout );
		}
*/

	} while ( (maxiter)?(iter4<maxiter):(norm>precision) );

	if( maxiter )
		precision = norm;

	// cleanup
	delete [] Vs[(size_t)(v_old+1)&1];
	delete [] norm_vec;

	V = Vs[(size_t)v_old];
	return;
}

#endif
