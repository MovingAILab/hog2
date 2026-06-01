#include <vector>
#include "SearchEnvironment.h"
#include "MultiAgentEnvironment.h"
#include "Map2DEnvironment.h"
#include "MyHash.h"
#include <ext/hash_set>
#include <unordered_map>

#ifndef TIDASTAR_H
#define TIDASTAR_H

/*!
	two players IDA* implementation
	min player (cop) plays first by default (parameter minFirst in tida)
	pos is a tuple of locations of robber (pos[0]) and cop (pos[1])

	This version is more optimized than the version in TIDAStar.h
	and was used to generate our statistics for
	"Optimal Solutions for Moving Target Search" (2008)
*/
template<class state, class action, class environment>
class TIDAStar {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;


	// used for heuristic updates
	struct CRStateHash {
		size_t operator()( const CRState &s ) const {
			return CRHash<state>( s );
		}
	};
	struct CRStateEqual {
		bool operator()( const CRState &c1, const CRState &c2 ) const {
			return ( c1 == c2 );
		}
	};
//	typedef __gnu_cxx::hash_set<CRState, CRStateHash, CRStateEqual> SearchCache;
	typedef std::unordered_map<CRState, double, CRStateHash, CRStateEqual> BoundCache;


	// constructor
	TIDAStar( environment *_env, bool _canPause ):
		env(_env), canPause(_canPause),usePerfectDistanceHeuristic(false) {};
	~TIDAStar();

	void clear_bounds_cache();

	double tida( CRState &pos, bool minFirst = true );

	void set_usePerfectDistanceHeuristic( bool usePerfectDistanceHeuristic );

	unsigned int nodesExpanded, nodesTouched;
	std::vector<unsigned int> iteration_nodesExpanded, iteration_nodesTouched;

	protected:

	double tida_update( CRState &pos, double bound, bool minFirst );

	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest(const  CRState &pos );
	double TerminalCost( CRState &pos );

	environment *env;
	bool canPause;
	bool usePerfectDistanceHeuristic;
	std::vector<std::vector<double> > distance_heuristic;

	// lower bound cache
	BoundCache min_lcache, max_lcache;
	// upper bound cache
	BoundCache min_ucache, max_ucache;

//	SearchCache min_scache, max_scache;
};




/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
// code templates for TIDAStar_optimized.cpp
template<>
double TIDAStar<xyLoc, tDirection, MapEnvironment>::MinHCost( CRState &pos, bool minsTurn );
template<>
double TIDAStar<xyLoc, tDirection, MapEnvironment>::MinGCost( CRState &p1, CRState &p2 );
template<>
void TIDAStar<xyLoc, tDirection, MapEnvironment>::set_usePerfectDistanceHeuristic( bool set );



template<class state, class action, class environment>
TIDAStar<state,action,environment>::~TIDAStar() {
}


// public functions

template<class state, class action, class environment>
void TIDAStar<state,action,environment>::clear_bounds_cache() {
	min_lcache.clear();
	max_lcache.clear();
	min_ucache.clear();
	max_ucache.clear();
	return;
}


template<class state, class action, class environment>
double TIDAStar<state,action,environment>::tida( CRState &pos, bool minFirst ) {
	double b, c = MinHCost( pos, minFirst );

	unsigned int sumNodesTouched = 0, sumNodesExpanded = 0;
	iteration_nodesExpanded.clear();
	iteration_nodesTouched.clear();

	do {
		nodesExpanded = 0; nodesTouched = 0;
		b = c;
		//fprintf( stdout, "set bound to b = %f\n", b );
		c = tida_update( pos, b, minFirst );

		iteration_nodesExpanded.push_back( nodesExpanded );
		iteration_nodesTouched.push_back( nodesTouched );
		sumNodesExpanded += nodesExpanded;
		sumNodesTouched  += nodesTouched;

/*
		// sanity check
		assert( min_scache.empty() );
		assert( max_scache.empty() );
*/

	} while( c > b ); // until c <= b

	nodesExpanded = sumNodesExpanded;
	nodesTouched  = sumNodesTouched;

	assert( c == b );

	// cleanup
	clear_bounds_cache();
	return c;
}


template<class state, class action, class environment>
double TIDAStar<state,action,environment>::tida_update( CRState &pos, double bound, bool minFirst )
{
	double result, temp, c;
	std::vector<state> neighbors;
	CRState neighbor;

	nodesTouched++;

	// verbose output
//	fprintf( stdout, "Considering position (%u,%u) (%u,%u) %d\n", pos[0].x, pos[0].y, pos[1].x, pos[1].y, minFirst );

	if( GoalTest( pos ) ) return TerminalCost( pos );
//	if( pos[0] == pos[1] ) return 0.;

/*
	// lookup in the path from the root to this node
	if( minFirst ) {
		if( min_scache.find( pos ) != min_scache.end() )
			return DBL_MAX;
	} else {
		if( max_scache.find( pos ) != max_scache.end() )
			return DBL_MAX;
	}
*/

	// upper bound cache lookup
	typename BoundCache::iterator hcit;
	BoundCache *current_bcache = minFirst?&min_ucache:&max_ucache;
	hcit = current_bcache->find( pos );
	if( hcit != current_bcache->end() ) {
		if( hcit->second <= bound ) return hcit->second;
	}

	// lower bound cache lookup
	current_bcache = minFirst?&min_lcache:&max_lcache;
	hcit = current_bcache->find( pos );
	if( hcit != current_bcache->end() ) {
		if( bound < hcit->second ) return hcit->second;
	} else {
		// heuristic pruning
		temp = MinHCost( pos, minFirst );
		if( bound < temp ) return temp;
	}

	nodesExpanded++;

	// in case we are the cop/min player
	if( minFirst ) {
//		min_scache.insert( pos );

		env->GetSuccessors( pos[1], neighbors );
		if( canPause ) neighbors.push_back( pos[1] );

		result = DBL_MAX;

		neighbor = pos;
		for( typename std::vector<state>::iterator iti = neighbors.begin(); iti != neighbors.end(); iti++ ) {
			neighbor[1] = *iti;
			c = MinGCost( pos, neighbor );
			temp = c + tida_update( neighbor, bound - c, !minFirst );

			result = min( temp, result );

			// alpha prune
			if( result <= bound ) break;
		}

	// in case we are the robber/max player
	} else {
//		max_scache.insert( pos );

		env->GetSuccessors( pos[0], neighbors );
		if( canPause ) neighbors.push_back( pos[0] );

		result = -DBL_MAX;

		neighbor = pos;
		for( typename std::vector<state>::iterator iti = neighbors.begin(); iti != neighbors.end(); iti++ ) {
			neighbor[0] = *iti;
			c = MinGCost( pos, neighbor );
			temp = c + tida_update( neighbor, bound - c, !minFirst );

			result = max( temp, result );

			// beta pruning
			if( result > bound ) break;
		}

	}

	// update heuristic
	if( bound < result ) {
		current_bcache = minFirst?&min_lcache:&max_lcache;
		(*current_bcache)[pos] = result;
	} else {
		current_bcache = minFirst?&min_ucache:&max_ucache;
		(*current_bcache)[pos] = result;
	}

/*
	if( minFirst )
		min_scache.erase( pos );
	else
		max_scache.erase( pos );
*/

	return result;
}



// protected functions
/*
template<class state, class action, class environment>
double TIDAStar<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return TerminalCost( pos ); 
	if( canPause )
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}


// determines the gcost from one joint state to the next,
// this is only called on adjacent nodes in the alternating move graph
template<class state, class action, class environment>
double TIDAStar<state,action,environment>::MinGCost( CRState &p1, CRState &p2 ) {
	double result = env->GCost( p1[0], p2[0] ) + env->GCost( p1[1], p2[1] );
	if( result == 0. ) return 1.; // cost for sitting out
	return result;
//	return 1.;
}
*/

template<class state, class action, class environment>
bool TIDAStar<state,action,environment>::GoalTest( CRState &pos ) {
	return (pos[0]==pos[1]);
}

template<class state, class action, class environment>
double TIDAStar<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
}

#endif
