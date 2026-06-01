#include <vector>
#include "SearchEnvironment.h"
#include "MultiAgentEnvironment.h"
#include "MyHash.h"
#include <ext/hash_set>
#include <map>

#ifndef TIDASTAR_H
#define TIDASTAR_H

/*!
	two players IDA* implementation
	min player (cop) plays first by default (parameter minFirst in tida)
	pos is a tuple of locations of robber (pos[0]) and cop (pos[1])
*/
template<class state, class action, class environment>
class TIDAStar {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;


	// SearchCache for the path from the root to the current node
	struct CRStateHash {
		size_t operator()( const CRState *s ) const {
			return CRHash<state>( *s );
		}
	};
	struct CRStateEqual {
		bool operator()( const CRState *c1, const CRState *c2 ) const {
			return ( *c1 == *c2 );
		}
	};
	typedef __gnu_cxx::hash_set<CRState*, CRStateHash, CRStateEqual> SearchCache;

	/*
	struct StateHash {
		size_t operator()( const state s ) const {
			return( ((uint64_t)s.x)<<32 | (((uint64_t)s.y)<<32)>>32 );
		}
	};
	struct StateEqual {
		bool operator()( const state s1, const state s2 ) const {
			return (s1==s2);
		}
	};
	typedef __gnu_cxx::hash_set<state,StateHash,StateEqual> NeighborCache;
	*/

	// transposition tables
	class TPEntry {
		public:
		TPEntry( CRState &_pos, double _value = 0. ): pos(_pos), value(_value) {};
		TPEntry( CRState &_pos, std::vector<CRState> &_path, double _value = 0. ): pos(_pos),path(_path),value(_value) {};
		CRState pos; // the node that has to be stored
		std::vector<CRState> path; // path from the leafs to this node
		double value;
	};
	struct TPEntryHash {
		size_t operator()( const TPEntry &e ) const {
			return CRHash<state>( e.pos );
		}
	};
	struct TPEntryEqual {
		bool operator()( const TPEntry &e1, const TPEntry &e2 ) const {
			return( e1.pos == e2.pos );
		}
	};

	typedef __gnu_cxx::hash_set<TPEntry, TPEntryHash, TPEntryEqual> TranspositionTable;
	typedef std::map<double, TranspositionTable> GTTables;
	typedef std::pair<double, TranspositionTable> GTTables_Pair;



	// constructor
	TIDAStar( environment *_env, bool _canPause ):
		maxDepthReached(0),
		env(_env), canPause(_canPause) {};

	double tida( CRState &pos, unsigned int maxDepth, std::vector<CRState> &path, double weight = 1., bool minFirst = true );

	// when TIDA* is run it updates this value
	unsigned int maxDepthReached;

	unsigned int nodesExpanded, nodesTouched;
	std::vector<unsigned int> iteration_nodesExpanded, iteration_nodesTouched;

	protected:

	double tida_update( CRState &pos, double gCost, double bound, unsigned int maxDepth, double weight, bool minFirst, std::vector<CRState> &path );

	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest(const  CRState &pos );
	double TerminalCost( CRState &pos );

	environment *env;
	bool canPause;
	// caching the path from the root to the current node (during the computation)
//	SearchCache min_scache, max_scache;

	// transposition table
	GTTables min_gttables, max_gttables;
};




/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

// public functions

template<class state, class action, class environment>
double TIDAStar<state,action,environment>::tida( CRState &pos, unsigned int maxDepth, std::vector<CRState> &path, double weight, bool minFirst ) {
	double b, c = weight * MinHCost( pos, minFirst );

	maxDepthReached = maxDepth;

	// just to make sure
	/*
	min_scache.clear();
	max_scache.clear();
	*/

	// just to make sure
	min_gttables.clear();
	max_gttables.clear();

	unsigned int sumNodesTouched = 0, sumNodesExpanded = 0;

	do {
		nodesExpanded = 0; nodesTouched = 0;
		b = c;
		fprintf( stdout, "set bound to b = %f\n", b );
		c = tida_update( pos, 0., b, maxDepth, weight, minFirst, path );
		// verbose
//		fprintf( stdout, "solution: " );
//		std::reverse( path.begin(), path.end() );
//		for( unsigned int i = 0; i < path.size(); i++ ) {
//			fprintf( stdout, "(%u,%u)(%u,%u) => ", path[i][0].x, path[i][0].y, path[i][1].x, path[i][1].y );
//		}
//		fprintf( stdout, "\n" );

		// the temporary search cache should be empty
		/*
		assert( min_scache.empty() );
		assert( max_scache.empty() );
		*/

		iteration_nodesExpanded.push_back( nodesExpanded );
		iteration_nodesTouched.push_back( nodesTouched );
		sumNodesExpanded += nodesExpanded;
		sumNodesTouched  += nodesTouched;

//		min_gttables.clear();
//		max_gttables.clear();

	} while( c > b ); // until c <= b

	// turn the path around to have the path from the root to the leafs
	std::reverse( path.begin(), path.end() );

	maxDepthReached = maxDepth - maxDepthReached;

	nodesExpanded = sumNodesExpanded;
	nodesTouched  = sumNodesTouched;

	min_gttables.clear();
	max_gttables.clear();

	return c;
}


template<class state, class action, class environment>
double TIDAStar<state,action,environment>::tida_update( CRState &pos, double gCost, double bound, unsigned int maxDepth, double weight, bool minFirst, std::vector<CRState> &path )
{
	nodesTouched++;

	// keep track of the minimal depth parameter that we encounter
	if( maxDepth < maxDepthReached ) maxDepthReached = maxDepth;
	path.clear();

	// verbose output
//	fprintf( stdout, "Considering position (%u,%u) (%u,%u) %d\n", pos[0].x, pos[0].y, pos[1].x, pos[1].y, minFirst );

	if( GoalTest( pos ) ) {
		path.push_back( pos ); // keep track of our terminal position
		return TerminalCost( pos );
	}
	// determine whether we yet encountered this position on the
	// path from the root to this node
	// technically we need this to make the game tree finite
	// although this is not needed in TIDA* (only in MAB, see there)
	/*
	if( minFirst ) {
		if( min_scache.find( &pos ) != min_scache.end() )
			return DBL_MAX;
	} else {
		if( max_scache.find( &pos ) != max_scache.end() )
			return DBL_MAX;
	}
	*/
	if( bound < weight * MinHCost( pos, minFirst ) + gCost ) {
		return (weight * MinHCost( pos, minFirst ));
	}

	if( maxDepth <= 0 ) {
		return bound - gCost;
	}

	// transposition table lookup
	TPEntry mytpentry( pos );
	typename GTTables::iterator gttit;
	typename TranspositionTable::iterator tit;
	std::pair<typename GTTables::iterator, bool> insert_return;
	GTTables *current_gttables = minFirst?&min_gttables:&max_gttables;
	gttit = current_gttables->find( bound - gCost ); // bound - gCost = rest bound
	if( gttit != current_gttables->end() ) {
		// if a transposition table for this gCost was found
		tit = gttit->second.find( mytpentry );
		if( tit != gttit->second.end() ) {
			path = tit->path;
			return tit->value;
		}
	} else {
		// else create a new transposition table for this gCost
		GTTables_Pair mypair;
		mypair.first = bound - gCost;
		insert_return = current_gttables->insert( mypair );
		// sanity check: do we still correctly create the transposition tables?
		assert( insert_return.second );
		gttit = insert_return.first;
	}

	// push the current node on the path cache
	/*
	if( minFirst )
		min_scache.insert( &pos );
	else
		max_scache.insert( &pos );
	*/


	nodesExpanded++;

	double result, temp, c;
	std::vector<state> neighbors;
	CRState neighbor;
	std::vector<CRState> childpath;

	// get the successor states
	int myid = minFirst?1:0; // am I cop or robber (robber/max is at 0, cop/min at 1)

	env->GetSuccessors( pos[myid], neighbors );
	if( canPause ) neighbors.push_back( pos[myid] );

	// in case we are the cop/min player
	if( minFirst ) {

		result = DBL_MAX;

		for( typename std::vector<state>::iterator iti = neighbors.begin(); iti != neighbors.end(); iti++ ) {
			neighbor = pos;
			neighbor[myid] = *iti;
			c = MinGCost( pos, neighbor );
			temp = c + tida_update( neighbor, gCost + c, bound, maxDepth - 1, weight, !minFirst, childpath );

			// result = min( temp, result )
			if( temp == result && path.empty() && !childpath.empty() ) {
				path = childpath; path.push_back( pos );
			}
			if( temp < result ) {
				result = temp;
				if( childpath.empty() )
					path.clear();
				else {
					path = childpath; path.push_back( pos );
				}
			}

			// alpha pruning
			// in case you do not care about the solution path but only the
			// solution length, you can prune with "<=", otherwise "<"
//			if( result + gCost <= bound )
			if( result + gCost < bound )
				break;
		}
	// in case we are the robber/max player
	} else {

		result = -DBL_MAX;

		for( typename std::vector<state>::iterator iti = neighbors.begin(); iti != neighbors.end(); iti++ ) {
			neighbor = pos;
			neighbor[myid] = *iti;
			c = MinGCost( pos, neighbor );
			temp = c + tida_update( neighbor, gCost + c, bound, maxDepth - 1, weight, !minFirst, childpath );

			// result = max( temp, result )
			if( temp == result && path.empty() && !childpath.empty() ) {
				path = childpath; path.push_back( pos );
			}
			if( temp > result ) {
				result = temp;
				if( childpath.empty() )
					path.clear();
				else {
					path = childpath; path.push_back( pos );
				}
			}

			// beta pruning
			if( result + gCost > bound ) {
				// if you do not want to have any pseudo solutions before
				// the last iteration uncomment this to really prune non
				// solution paths
//				path.clear();
				break;
			}
		}
	}

	// cleanup
	// update the list of moves from the root to this nodes parent
	/*
	if( minFirst )
		min_scache.erase( &pos );
	else
		max_scache.erase( &pos );
	*/

	// store the current value
	mytpentry.path = path;
	mytpentry.value = result;
	gttit->second.insert( mytpentry );

	return result;
}



// protected functions


template<class state, class action, class environment>
double TIDAStar<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	if( canPause )
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}

// specification for state=xyLoc
template<>
double TIDAStar<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	double dist;
	if( abs(pos[1].x - pos[0].x) < abs(pos[1].y - pos[0].y) )
		dist = abs(pos[1].y - pos[0].y);
	else
		dist = abs(pos[1].x - pos[0].x);

	if( canPause )
	// TODO: CHANGE ME BACK!!!!
	//	return( idist + idist%2 - (minsTurn?MinGCost(pos,pos):0.) );
		return( 2. * dist - (minsTurn?MinGCost(pos,pos):0.) );
	else
	//	return( idist/2 );
		return dist;
}


template<class state, class action, class environment>
double TIDAStar<state,action,environment>::MinGCost( CRState&, CRState& ) {
	return 1.;
}

template<class state, class action, class environment>
bool TIDAStar<state,action,environment>::GoalTest( CRState &pos ) {
	return( pos[0] == pos[1] );
}

template<class state, class action, class environment>
double TIDAStar<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
};

#endif
