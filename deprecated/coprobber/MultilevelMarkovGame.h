#include "MultiAgentEnvironment.h"
#include "MarkovGame.h"
#include <sys/times.h>

#ifndef MULTILEVELMARKOVGAME_H
#define MULTILEVELMARKOVGAME_H

/*
	It is possible to compute the expected rewards for a Markov Game.
	This class does the same as MarkovGame::GetExpectedStateRewards
	but in contrast by using the multigrid technique to speed up convergence
*/
template<class state, class action>
class MultilevelMarkovGame {
	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState MAState;
	typedef typename MultiAgentEnvironment<state,action>::MAMove MAMove;
	typedef typename MultiAgentEnvironment<state,action>::SAAction SAAction;

	MultilevelMarkovGame( bool _simultaneous = false ): simultaneous(_simultaneous) {};
	virtual ~MultilevelMarkovGame() {};

	virtual void SetSimultaneous( bool _simultaneous ) { simultaneous = _simultaneous; };

	// defines the number of levels for markov games
	virtual unsigned int NumLevels() = 0;

	virtual MarkovGame<state,action>* GetMarkovGame( unsigned int level ) = 0;
	virtual MAState GetParent( MAState s, unsigned int slevel ) = 0;
	virtual unsigned int GetNumChildren( MAState s, unsigned int slevel ) = 0;
	virtual MAState GetNthChild( MAState s, unsigned int slevel, unsigned int n ) = 0;

	// maxiter here is the number of iterations on each level
	// precision the precision that has to be reached on the lowest level
	virtual void GetExpectedStateRewards( unsigned int player, double gamma, double epsilon, double precision, double** &V, unsigned int* &iter, unsigned int maxlevel, unsigned int maxiter );

	protected:
	virtual void PushDown( double* V_parent, unsigned int parent_level, double* &V_child );
	virtual void PushUp( double* V_children, unsigned int child_level, double* &V_parent );
	
	bool simultaneous;

};

template<class state, class action>
void MultilevelMarkovGame<state,action>::GetExpectedStateRewards( unsigned int player, double gamma, double epsilon, double precision, double** &V, unsigned int* &iter, unsigned int maxlevel, unsigned int maxiter ) {

	// test the maxlevel
	if( maxlevel > NumLevels() ) {
		fprintf( stderr, "Warning: reduced the number of levels in MultilevelMarkovGame::GetExpectedStateRewards from %u to %u\n", maxlevel, NumLevels() );
		maxlevel = NumLevels();
	}

	unsigned int i;
	iter = new unsigned int[maxlevel]; // iterations on each level
	V = new double*[maxlevel];
	double norm = precision;
	MarkovGame<state,action> *game = NULL;

	for( i = 0; i <= maxlevel; i++ ) {
		iter[i] = 0;
		V[i] = NULL;
	}

	// do a first top-down run
	// don't be a fool and try >= 0 instead of >= 1, i is an unsigned int, therefore the first would
	// always be true!
	for( i = maxlevel; i >= 1; i-- ) {
		game = GetMarkovGame( i );

		// VERBOSE
		char filename[250];
		if( i != maxlevel ) {
			sprintf( filename, "er_before_level%u.stat", i );
			game->WriteExpectedRewardToDisc( filename, player, V[i] );
		}

		game->GetExpectedStateRewards( player, gamma, epsilon, norm, V[i], iter[i], maxiter );

		sprintf( filename, "er_after_level%u.stat", i );
		game->WriteExpectedRewardToDisc( filename, player, V[i] );

		//if( i > 0 )
		PushDown( V[i], i, V[i-1] );
		
		printf( "norm after computation on level %u is %g\n", i, norm );
	}

/*
	// V-Cycle!!!
	while( norm > precision ) {
		// do the run up in the V cycle
		for( i = 1; i < maxlevel; i++ ) {
			PushUp( V[i-1], i-1, V[i] );
			game = GetMarkovGame( i );
			game->GetExpectedStateRewards( player, gamma, epsilon, norm, V[i], iter[i], maxiter );

			printf( "norm after computation on level %u is %g\n", i, norm );
		}
		// do the run down
		// don't be a fool and try >= 0 instead of >= 1, i is an unsigned int, therefore the first would
		// always be true!
		for( i = maxlevel-1; i >= 1; i-- ) {
			PushDown( V[i], i, V[i-1] );
			game = GetMarkovGame( i-1 );
			game->GetExpectedStateRewards( player, gamma, epsilon, norm, V[i-1], iter[i-1], maxiter );

			printf( "norm after computation on level %u is %g\n", i-1, norm );
		}
		if( maxlevel == 1 ) {
			game = GetMarkovGame( 0 );
			game->GetExpectedStateRewards( player, gamma, epsilon, norm, V[0], iter[0], maxiter );
			printf( "norm after computation on level %u is %g\n", 0, norm );
		}
	}
*/

	// if the top->down cycle wasn't enough, converge the rest on the lowest level
	//if( norm > precision ) {
		game = GetMarkovGame(0);
		game->GetExpectedStateRewards( player, gamma, epsilon, precision, V[0], iter[0] );

		// VERBOSE
		char filename[250];
		sprintf( filename, "er_solution_level0.stat" );
		game->WriteExpectedRewardToDisc( filename, player, V[0] );
	//}

	return;

}

template<class state, class action>
void MultilevelMarkovGame<state,action>::PushDown( double* V_parent, unsigned int parent_level, double* &V_child ) {

	MarkovGame<state,action> *pgame = GetMarkovGame( parent_level );
	MarkovGame<state,action> *cgame = GetMarkovGame( parent_level-1 );
	unsigned int i, num_cstates = cgame->GetNumStates();
	MAState pstate, cstate;
	bool allocated = false;

	// if V_child has not been allocated, allocate it
	if( V_child == NULL ) {
		V_child = new double[num_cstates];
		allocated = true;
	}

	for( i = 0; i < num_cstates; i++ ) {
		cstate = cgame->GetStateByNumber( i );

		if( cgame->GoalTest( cstate, cstate ) ) V_child[i] = 0.;
		else {

			// get the parent state
			pstate = GetParent( cstate, parent_level-1 );
			if( pstate.size() ) {
				// TODO: make this much better!
				V_child[i] = 2. * V_parent[ pgame->GetNumberByState( pstate ) ];
			} else {
				if( allocated )
					V_child[i] = 0.;
				// else leave it as is
			}
		}
	}

	return;
}

template<class state, class action>
void MultilevelMarkovGame<state,action>::PushUp( double* V_children, unsigned int child_level, double* &V_parent ) {

	MarkovGame<state,action> *pgame = GetMarkovGame( child_level+1 );
	MarkovGame<state,action> *cgame = GetMarkovGame( child_level );
	unsigned int i, j, num_cstates, num_pstates = pgame->GetNumStates();
	MAState pstate, cstate;

	// allocate memory if necessary
	if( V_parent == NULL ) {
		V_parent = new double[num_pstates];
	}

	// for each parent
	for( i = 0; i < num_pstates; i++ ) {
		pstate = pgame->GetStateByNumber( i );

		V_parent[i] = 0.;
		if( !pgame->GoalTest( pstate, pstate ) ) {
			// update the value of the parent as the average of its children
			num_cstates = GetNumChildren( pstate, child_level+1 );
			for( j = 0; j < num_cstates; j++ ) {
				cstate = GetNthChild( pstate, child_level+1, j );
				V_parent[i] += V_children[cgame->GetNumberByState( cstate )];
			}
			V_parent[i] /= (double)num_cstates;
		}
	}

	return;
}

#endif
