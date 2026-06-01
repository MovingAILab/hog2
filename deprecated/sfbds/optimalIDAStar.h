/*
	Performs IDA* while determining the optimal jumping policy

	Notes:
	1. We cannot use BPMX since the pruning might work differently
	   for different jumping policies
	2. Propagating the next cost bound is difficult.
*/

#ifndef OPTIMALIDASTAR
#define OPTIMALIDASTAR

#include <queue>
#include <set>
#include <cstdlib>
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include "PancakePuzzle.h"

template<class state,class action>
class OptimalIDAStar {

	public:

	// Data structures
	typedef std::pair<uint64_t,uint64_t> Node;

	// Constructor
	OptimalIDAStar( SearchEnvironment<state,action> *env );
	~OptimalIDAStar();

	// use max( h(x,y), h(y,x) ) - disabled by default
	void SetUseMaxHeuristic( bool set ) { useMaximumHeuristic = set; };
	// use BPMX - disabled by default
	void SetUseBPMX( bool set ) { useBPMX = set; };

	double IDAStar( state &s, state &g );
	double IDAStarIteration( state &s, state &g, double bound, unsigned &optimalnn );

	unsigned necessaryNodesExpanded;
	unsigned necessaryNodesExpandedInLastIteration;

	protected:
	// Hashing
	uint64_t GetHashFromState( state &s );
	void GetStateFromHash( uint64_t hash, state &s );

	double IDAStarIteration( Node &n, double bound, double h, unsigned &optimalnn );

	private:
	// keeps the search environment
	SearchEnvironment<state,action> *env;
	// keeps the list of visited states from the root to the current node
	std::set<uint64_t> expandedStates;
	// use max( h(x,y), h(y,x) ), default is false
	bool useMaximumHeuristic;
	bool useBPMX;

	// this is a trick so we don't have to store the puzzle sizes etc.
	state goal;

};

/*******************************************************************************
| Implementation
*******************************************************************************/

template<class state,class action>
OptimalIDAStar<state,action>::OptimalIDAStar( SearchEnvironment<state,action> *_env ):
	env(_env), useMaximumHeuristic(false), useBPMX(false) {
	// anything else?
};

template<class state,class action>
OptimalIDAStar<state,action>::~OptimalIDAStar() {
	assert( expandedStates.empty() );
	// anything else?
};

template<class state, class action>
double OptimalIDAStar<state,action>::IDAStar( state &s, state &g ) {
	goal = g;
	Node n( GetHashFromState( s ), GetHashFromState( g ) );
	double bound, result = 0.;
	necessaryNodesExpanded = 0;
	do {
		bound = result;
		// verbose
		//std::cout << "------ setting bound = " << bound << " -----------" << std::endl;
		result = IDAStarIteration( n, bound, 0., necessaryNodesExpandedInLastIteration );
		necessaryNodesExpanded += necessaryNodesExpandedInLastIteration;
		assert( expandedStates.empty() );
	} while( fgreater( result, bound ) );

	return result;
};

template<class state, class action>
double OptimalIDAStar<state,action>::IDAStarIteration( state &s, state &g, double bound, unsigned &optimalnn ) {
	goal = g;
	Node n( GetHashFromState( s ), GetHashFromState( g ) );
	return( IDAStarIteration( n, bound, 0., optimalnn ) );
};


template<class state,class action>
double OptimalIDAStar<state,action>::IDAStarIteration( Node &n, double bound, double h, unsigned &optimalnn ) {

	// verbose
	//state verbosestate = goal;
	//GetStateFromHash( n.first, verbosestate );
	//std::cout << "entering with ( x=" << verbosestate;
	//GetStateFromHash( n.second, verbosestate );
	//std::cout << "| y=" << verbosestate;
	//std::cout << "| b=" << bound;
	//std::cout << "| h=" << h << ")" << std::endl;

	// number of nodes expanded so far is 0
	optimalnn = 0;

	if( expandedStates.find( n.first ) != expandedStates.end() ||
	    expandedStates.find( n.second ) != expandedStates.end() ) {
		// verbose
		//std::cout << "pruning repetition with DBL_MAX" << std::endl;
		return DBL_MAX;
	}

	// get the states
	state x = goal, y = goal;
	GetStateFromHash( n.first, x );
	GetStateFromHash( n.second, y );

	// get the heuristic
	if( useBPMX )
		h = std::max( h, env->HCost( x, y ) );
	else
		h = env->HCost( x, y );
	if( useMaximumHeuristic )
		h = std::max( h, env->HCost( y, x ) );

	// heuristic prune - if bound is exceeded, prune
	if( fgreater( h, bound ) ) {
		// verbose
		//std::cout << "pruning with " << h << std::endl;
		return h;
	}

	// recursion break
	if( n.first == n.second ) {
		// verbose
		//std::cout << "solution => returning 0." << std::endl;
		return 0.;
	}


	// generate the successors
	std::vector<state> successors;
	typename std::vector<state>::iterator it;
	/*************************
	| Expand the left side
	*************************/
	Node child = n;
	env->GetSuccessors( x, successors );
	expandedStates.insert( n.first );
	double result_left = DBL_MAX;
	unsigned optimalnn_left = 1;
	double h_left = h;
	for( it = successors.begin(); it != successors.end(); it++ ) {
		child.first = GetHashFromState( *it );
		double gcost = env->GCost( x, *it );

		unsigned child_optimalnn;
		double child_result = IDAStarIteration( child, bound - gcost, h_left - gcost, child_optimalnn );
		result_left = std::min( result_left, child_result + gcost );
		// BPMX
		if( child_result != DBL_MAX )
			h_left = std::max( h_left, child_result - gcost );
		optimalnn_left += child_optimalnn;

		if( result_left <= bound ) break;
	}

	// cleanup
	successors.clear();
	expandedStates.erase( n.first );
	/*************************
	| Expand the right side
	*************************/
	child = n;
	env->GetSuccessors( y, successors );
	expandedStates.insert( n.second );
	double result_right = DBL_MAX;
	unsigned optimalnn_right = 1;
	double h_right = h;
	for( it = successors.begin(); it != successors.end(); it++ ) {
		child.second = GetHashFromState( *it );
		double gcost = env->GCost( *it, y );

		unsigned child_optimalnn;
		double child_result = IDAStarIteration( child, bound - gcost, h_right - gcost, child_optimalnn );
		result_right = std::min( result_right, child_result + gcost );
		// BPMX
		if( child_result != DBL_MAX )
			h_right      = std::max( h_right, child_result - gcost );
		optimalnn_right += child_optimalnn;

		if( result_right <= bound ) break;
	}

	// cleanup
	successors.clear();
	expandedStates.erase( n.second );

	/*******************************
	| Sanity checks
	*******************************/
	//if( result_left != result_right )
	//	std::cerr << "Warning: left (" << result_left <<
	//		") and right (" << result_right << ") result where not identical." << std::endl;

	// setting the optimal number of expansions
	optimalnn = std::min( optimalnn_left, optimalnn_right );

	// verbose
	//std::cout << "ending computation with " << result_left;
	//if( !fequal( result_left, result_right ) )
	//	std::cout << ", " << result_right;
	//std::cout << " at bound " << bound << std::endl;
	return( std::min( result_left, result_right ) );
};

/***********************************************
| Code that can be found in optimalIDAStar.cpp
***********************************************/
template<>
uint64_t OptimalIDAStar<PancakePuzzleState,unsigned>::GetHashFromState( PancakePuzzleState &s );
template<>
void OptimalIDAStar<PancakePuzzleState,unsigned>::GetStateFromHash( uint64_t hash, PancakePuzzleState &s );


#endif
