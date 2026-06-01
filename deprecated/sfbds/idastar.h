/*
	Performs IDA* while determining the optimal jumping policy

	Notes:
	1. We cannot use BPMX since the pruning might work differently
	   for different jumping policies
	2. Propagating the next cost bound is difficult.
*/

#ifndef IDASTAR_H
#define IDASTAR_H

#include <queue>
#include <set>
#include <cstdlib>
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include "PancakePuzzle.h"

template<class state,class action>
class IDAStar {

	public:

	// Data structures
	typedef std::pair<uint64_t,uint64_t> Node;

	// Constructor
	IDAStar( SearchEnvironment<state,action> *env );
	~IDAStar();

	// use max( h(x,y), h(y,x) ) - disabled by default
	void SetUseMaxHeuristic( bool set ) { useMaximumHeuristic = set; };
	// use BPMX - disabled by default
	void SetUseBPMX( bool set ) { useBPMX = set; };
	// sets the jumping policy
	void SetJumpingPolicy( int set ) { jumpingPolicy = set; };

	double RunIDAStar( state &s, state &g );
	double IDAStarIteration( state &s, state &g, double bound );

	unsigned nodesExpanded;
	unsigned nodesExpandedInLastIteration;
	unsigned numberOfJumps;

	protected:
	// Hashing
	uint64_t GetHashFromState( state &s );
	void GetStateFromHash( uint64_t hash, state &s );

	bool expandheuristic( Node &n, bool last_direction );
	double heuristicLookup( state &s, state &g, bool bothSides );

	double IDAStarIteration( Node &n, double bound, double h, bool last_direction );

	private:
	// keeps the search environment
	SearchEnvironment<state,action> *env;
	// keeps the list of visited states from the root to the current node
	std::set<uint64_t> expandedStates;
	// use max( h(x,y), h(y,x) ), default is false
	bool useMaximumHeuristic;
	bool useBPMX;
	int jumpingPolicy;

	// this is a trick so we don't have to store the puzzle sizes etc.
	state goal;

};

/*******************************************************************************
| Implementation
*******************************************************************************/

template<class state,class action>
IDAStar<state,action>::IDAStar( SearchEnvironment<state,action> *_env ):
	env(_env), useMaximumHeuristic(false), useBPMX(false), jumpingPolicy(0) {
	// anything else?
};

template<class state,class action>
IDAStar<state,action>::~IDAStar() {
	assert( expandedStates.empty() );
	// anything else?
};

template<class state, class action>
double IDAStar<state,action>::RunIDAStar( state &s, state &g ) {
	nodesExpanded = 0;
	numberOfJumps = 0;
	goal = g;
	Node n( GetHashFromState( s ), GetHashFromState( g ) );
	double bound, result = 0.;
	do {
		bound = result;
		nodesExpandedInLastIteration = 0;
		//std::cout << "------ setting bound = " << bound << " -----------" << std::endl;
		result = IDAStarIteration( n, bound, 0., true );
		nodesExpanded += nodesExpandedInLastIteration;
		assert( expandedStates.empty() );
	} while( fgreater( result, bound ) );

	return result;
};


template<class state, class action>
double IDAStar<state,action>::IDAStarIteration( state &s, state &g, double bound ) {
	goal = g;
	Node n( GetHashFromState( s ), GetHashFromState( g ) );
	return( IDAStarIteration( n, bound, 0., true ) );
};


template<class state,class action>
double IDAStar<state,action>::IDAStarIteration( Node &n, double bound, double h, bool last_direction ) {

	// verbose
	//state verbosestate = goal;
	//GetStateFromHash( n.first, verbosestate );
	//std::cout << "entering with ( x=" << verbosestate;
	//GetStateFromHash( n.second, verbosestate );
	//std::cout << "| y=" << verbosestate;
	//std::cout << "| b=" << bound;
	//std::cout << "| h=" << h << ")" << std::endl;

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
		h = std::max( h, heuristicLookup( x, y, useMaximumHeuristic ) );
	else
		h = heuristicLookup( x, y, useMaximumHeuristic );

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
	bool expand = expandheuristic( n, last_direction );
	if( expand ) {
		env->GetSuccessors( x, successors );
		expandedStates.insert( n.first );
	} else {
		env->GetSuccessors( y, successors );
		expandedStates.insert( n.second );
	}

	Node child = n;
	double result = DBL_MAX;
	double gcost;
	nodesExpandedInLastIteration++;
	for( it = successors.begin(); it != successors.end(); it++ ) {
		if( expand ) {
			child.first  = GetHashFromState( *it );
			gcost = env->GCost( x, *it );
		} else {
			child.second = GetHashFromState( *it );
			gcost = env->GCost( *it, y );
		}
		double child_result = IDAStarIteration( child, bound - gcost, h - gcost, expand );
		result = std::min( result, child_result + gcost );
		// BPMX
		if( child_result != DBL_MAX )
			h = std::max( h, child_result - gcost );

		if( result <= bound ) break;
	}

	// cleanup
	successors.clear();
	if( expand )
		expandedStates.erase( n.first );
	else
		expandedStates.erase( n.second );

	return result;
};

/***********************************************
| Heuristic jumping
***********************************************/
template<class state, class action>
bool IDAStar<state,action>::expandheuristic( Node &n, bool last_direction ) {

	if( jumpingPolicy == 0 ) return true;
	else if( jumpingPolicy == 1 ) return false;


	unsigned int num_start, num_goal;
	double h_start = 0., h_goal = 0.;
	long r;
	std::vector<state> successors;
	bool result = true;

	state x = goal, y = goal;
	GetStateFromHash( n.first, x );
	GetStateFromHash( n.second, y );

	switch( jumpingPolicy ) {
		case 2:
			// return the side with smaller branching factor
			env->GetSuccessors( x, successors );
			num_start = successors.size();
			successors.clear();
			env->GetSuccessors( y, successors );
			num_goal  = successors.size();

			if( num_start == num_goal ) return last_direction;

			result = (num_start < num_goal);
			break;

		case 3:
			// return randomly weighted with the branching factor of each side
			env->GetSuccessors( x, successors );
			num_start = successors.size();
			successors.clear();
			env->GetSuccessors( y, successors );
			num_goal = successors.size();
			r = random();

			result = ( (double)r < (double)RAND_MAX/(double)(num_start+num_goal) * (double)num_start );
			break;

		case 4:
			// return the side with the higher average hcost - JIL(1)
			env->GetSuccessors( x, successors );
			for( typename std::vector<state>::iterator it = successors.begin(); it != successors.end(); it++ )
				h_start += heuristicLookup( *it, y, useMaximumHeuristic );
			h_start /= (double)successors.size();
			successors.clear();
			env->GetSuccessors( y, successors );
			for( typename std::vector<state>::iterator it = successors.begin(); it != successors.end(); it++ )
				h_goal += heuristicLookup( *it, x, useMaximumHeuristic );
			h_goal /= (double)successors.size();

			// if both averages are the same do not change direction
			if( fequal( h_start, h_goal ) )
				return last_direction;

			result = (h_start > h_goal);
			break;

		case 7: // jump if larger
			h_start = heuristicLookup( x, y, false );
			h_goal  = heuristicLookup( y, x, false );
			if( fequal( h_start, h_goal ) )
				return last_direction;
			result = (h_start > h_goal);
			break;
	}

	// determine whether we have a jump
	if( last_direction != result )
		numberOfJumps++;

	return result;
};


template<class state,class action>
double IDAStar<state,action>::heuristicLookup( state &s, state &g, bool bothSides ) {
	double result = env->HCost( s, g );
	if( bothSides )
		result = std::max( result, env->HCost( g, s ) );
	return result;
};

/***********************************************
| Code that can be found in optimalIDAStar.cpp
***********************************************/
template<>
uint64_t IDAStar<PancakePuzzleState,unsigned>::GetHashFromState( PancakePuzzleState &s );
template<>
void IDAStar<PancakePuzzleState,unsigned>::GetStateFromHash( uint64_t hash, PancakePuzzleState &s );


#endif
