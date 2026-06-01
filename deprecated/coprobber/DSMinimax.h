#include <ext/hash_set>
#include <map>
#include <vector>
#include <math.h>
#include "MyHash.h"
#include "DSCREnvironment.h"
#include "DSRobberAlgorithm.h"


#ifndef DSMINIMAX_H
#define DSMINIMAX_H


/*!
	two players minimax implementation
	code mostly copied from Minimax.h but adjusted to DSCREnvironment

	note:
		- this code also extracts solution paths
		- this implementation uses heuristic evaluates from DSCREnvironment
		  for the leaf nodes.
*/
template<class state, class action>
class DSMinimax: public DSRobberAlgorithm<state,action> {

	public:

	// constructor
	DSMinimax( SearchEnvironment<state,action> *env, bool canPause = true, unsigned int cop_speed = 1 );
	~DSMinimax();

	// sets whether edge costs should be used within the minimax computation
	// or not. This is absolutely a good idea when computing on the lowest
	// level of abstraction (the original map). When computing on higher levels
	// like in DAM (dynamic abstract minimax) we might not want to include edge costs.
	void useEdgeCosts( bool _use_edge_costs ) { use_edge_costs = _use_edge_costs; };

	double minimax( state &pos_robber, state &pos_cop, std::vector<state> &path, bool minFirst = true, double depth = 5. );

	// compute minimax solution and return first next state
	state MakeMove( state &pos_robber, state &pos_cop, bool minFirst = true, double depth = 5. );

	state MakeMove( state pos_robber, state pos_cop, unsigned int ) {
		return( MakeMove( pos_robber, pos_cop, false, 5. ) );
	};

	unsigned int nodesExpanded, nodesTouched;

	protected:
	
	// the real minimax implementation
	double minimax_help( state &pos_robber, state &pos_cop, bool minFirst, double depth, double alpha, double beta );

	// variables
	DSCREnvironment<state,action> *dscrenv;
	bool use_edge_costs;

	// transposition tables - for one of the two players
	// (thus minFirst does not have to be stored in the entries)
	class TPEntry {
		public:
		TPEntry( state _pos_robber, state _pos_cop, double _value = 0. ):
			pos_robber(_pos_robber),pos_cop(_pos_cop),value(_value) {};

		state pos_robber, pos_cop;
		state next_pos;
		bool no_next_pos;
		double value, alpha, beta;
	};
	struct TPEntryHash {
		size_t operator()( const TPEntry &e ) const {
			return CRHash<state>( e.pos_robber, e.pos_cop );
		}
	};
	struct TPEntryEqual {
		bool operator()( const TPEntry &e1, const TPEntry &e2 ) const {
			return( e1.pos_robber == e2.pos_robber && e1.pos_cop == e2.pos_cop );
		}
	};

	typedef __gnu_cxx::hash_set<TPEntry, TPEntryHash, TPEntryEqual> TranspositionTable;
	// we have one such table for each depth, it would also be possible
	// to include the depth parameter into the TPEntry, but then
	// the hashing function gets more difficult...
	typedef std::map<double, TranspositionTable> GTTables;
	typedef std::pair<double, TranspositionTable> GTTables_Pair;
	// transposition table
	GTTables min_gttables, max_gttables;

};






/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state,class action>
DSMinimax<state,action>::DSMinimax( SearchEnvironment<state,action> *env, bool canPass, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<state,action>( env, canPass, cop_speed ) ),
	use_edge_costs( true )
{ };

template<class state,class action>
DSMinimax<state,action>::~DSMinimax() {
	delete dscrenv;
};


template<class state,class action>
double DSMinimax<state,action>::minimax( state &pos_robber, state &pos_cop, std::vector<state> &path, bool minFirst, double depth ) {

	double result;

	nodesTouched = 0;
	nodesExpanded = 0;

	// just to make sure
	min_gttables.clear();
	max_gttables.clear();

	result = minimax_help( pos_robber, pos_cop, minFirst, depth, -DBL_MAX, DBL_MAX );

	// extract the path from the cache
	path.clear();
	TPEntry mytpentry( pos_robber, pos_cop );
	typename GTTables::iterator gttit;
	typename TranspositionTable::iterator tit;
	GTTables* current_gttables = minFirst?&min_gttables:&max_gttables;
	state tempstate;
	// this loop terminates because we only computed down to a certain depth
	while( true ) {
		gttit = current_gttables->find( depth );
		if( gttit != current_gttables->end() ) {
			tit = gttit->second.find( mytpentry );
			if( tit != gttit->second.end() ) {
				if( !tit->no_next_pos ) {
					path.push_back( tit->next_pos );
					if( minFirst ) {
						tempstate = tit->next_pos;
						depth -= dscrenv->CopGCost( mytpentry.pos_cop, tempstate );
						mytpentry.pos_cop = tempstate;
						current_gttables = &max_gttables;
						minFirst = !minFirst;
					} else {
						tempstate = tit->next_pos;
						depth -= dscrenv->RobberGCost( mytpentry.pos_robber, tempstate );
						mytpentry.pos_robber = tempstate;
						current_gttables = &min_gttables;
						minFirst = !minFirst;
					}
				} else
					break;
			} else
				break;
		} else
			break;
	}
	

	// cleanup
	min_gttables.clear();
	max_gttables.clear();

	return result;
}


// alpha = minimum score that the maximum player is assured of
// beta  = maximum score that the minimum player is assured of
template<class state,class action>
double DSMinimax<state,action>::minimax_help( state &pos_robber, state &pos_cop, bool minFirst, double depth, double alpha, double beta ) {

	// verbose
	//fprintf( stdout, "depth %g: ", depth );
	//fprintf( stdout, "considered position (%lu,%lu) for player ", pos_robber, pos_cop );
	//fprintf( stdout, "%s ", (minFirst?"min":"max") );
	//fprintf( stdout, "and alpha=%g beta=%g\n", alpha, beta );

	nodesTouched++;

	if( dscrenv->GoalTest( pos_robber, pos_cop ) ) {
		return dscrenv->TerminalCost( pos_robber, pos_cop );
	}

	// determine the heuristic cost
	double hcost;
	if( use_edge_costs )
		hcost = dscrenv->AccumulatedHCost( pos_robber, pos_cop, minFirst );
	else
		hcost = dscrenv->HCost( pos_robber, pos_cop );

	// verbose
	//printf( "hcost = %g\n", hcost );

	// in case of an admissible heuristic we can also prune
	// this is because hcost is then a lower bound and beta is supposed to be an upper bound
	// thus beta <= hcost means suboptimality
	if( beta <= hcost ) {
		return hcost;
	}

	// if we reached the bottom of the computation tree
	if( depth <= 0. ) {
		return hcost;
	}

	// transposition table lookup
	TPEntry mytpentry( pos_robber, pos_cop );
	typename GTTables::iterator gttit;
	std::pair<typename GTTables::iterator, bool> insert_return;
	typename TranspositionTable::iterator tit;
	bool old_transposition_available = false;
	bool old_value_upper_bound = true; // true, false = old value is lower bound

	GTTables *current_gttables = minFirst?&min_gttables:&max_gttables;
	gttit = current_gttables->find( depth );
	if( gttit != current_gttables->end() ) {
		// if a transposition table for this gCost was found
		tit = gttit->second.find( mytpentry );
		// check if the position is in the hash table
		if( tit != gttit->second.end() ) {

			old_transposition_available = true;

			// verbose
			//printf( "found transition with (a,b,v) = (%g,%g,%g)\n", tit->alpha,tit->beta,tit->value );
			//if( tit->no_next_pos )
			//	printf( "no next position for transposition entry\n" );
			//else
			//	printf( "next position for transposition entry %lu\n", tit->next_pos );

			// check for usability of the hash table entry
			if( tit->value <= tit->alpha ) {
				old_value_upper_bound = true;
				if( tit->value < beta )
					beta = tit->value;
			}
			if( tit->beta <= tit->value ) {
				old_value_upper_bound = false;
				if( alpha < tit->value )
					alpha = tit->value;
			}
			if( beta <= alpha ) {
				return tit->value;
			}
			if( tit->alpha < tit->value && tit->value < tit->beta ) {
				return tit->value;
			}

		}
	} else {
		// else create a new transposition table for this gCost
		GTTables_Pair mypair;
		mypair.first = depth;
		insert_return = current_gttables->insert( mypair );
		// sanity check: do we still correctly create the transposition tables?
		assert( insert_return.second );
		gttit = insert_return.first;
	}


	// variable declarations
	std::vector<state> next_mystates;
	typename std::vector<state>::iterator it;
	state child, next_pos = minFirst?pos_cop:pos_robber;
	bool no_next_pos = true;
	double child_value, pathcost;
	double result;

	nodesExpanded++;

	// generate the next moves/children
	if( minFirst ) {
		// Min Node
		double decrease_beta = beta;
		result = DBL_MAX;
		dscrenv->GetCopSuccessors( pos_cop, next_mystates );

		for( it = next_mystates.begin(); it != next_mystates.end(); it++ ) {

			child = *it;
			pathcost = dscrenv->CopGCost( pos_cop, child );
			if( use_edge_costs )
				child_value = pathcost +
					minimax_help( pos_robber, child, !minFirst, depth - pathcost, alpha - pathcost, decrease_beta - pathcost );
			else
				child_value = minimax_help( pos_robber, child, !minFirst, depth - pathcost, alpha, decrease_beta );
			// min player updates his score
			if( child_value < result ) {
				result = child_value;
				next_pos = child;
				no_next_pos = false;
			}
			if( child_value < decrease_beta )
				decrease_beta = child_value;

			// beta cutoff
			if( decrease_beta <= alpha ) break;
		}

	} else {
		// Max Node
		double increase_alpha = alpha;
		result = -DBL_MAX;
		dscrenv->GetRobberSuccessors( pos_robber, next_mystates );

		for( it = next_mystates.begin(); it != next_mystates.end(); it++ ) {

			child = *it;
			pathcost = dscrenv->RobberGCost( pos_robber, child );
			if( use_edge_costs )
				child_value = pathcost +
					minimax_help( child, pos_cop, !minFirst, depth - pathcost, increase_alpha - pathcost, beta - pathcost );
			else
				child_value = minimax_help( child, pos_cop, !minFirst, depth - pathcost, increase_alpha, beta );
			// max player updates his score
			if( child_value > result ) {
				result = child_value;
				next_pos = child;
				no_next_pos = false;
			}
			if( child_value > increase_alpha )
				increase_alpha = child_value;

			// alpha cutoff
			if( beta <= increase_alpha ) break;
		}

	}

	// combine the current value with the old hash entry
	if( old_transposition_available ) {
		if( old_value_upper_bound ) {
			// sanity checks
			assert( result <= tit->value );
			assert( beta <= tit->value );
			if( result == tit->value )
				beta = result + 1.;
		} else {
			// sanity checks
			assert( result >= tit->value );
			assert( alpha >= tit->value );
			if( result == tit->value )
				alpha = result - 1.;
		}
		// the old entry is no longer needed
		gttit->second.erase( tit );
	}
	mytpentry.next_pos = next_pos;
	mytpentry.no_next_pos = no_next_pos;
	mytpentry.value = result;
	mytpentry.alpha = alpha;
	mytpentry.beta  = beta;
	gttit->second.insert( mytpentry );

	return result;
}


template<class state, class action>
state DSMinimax<state,action>::MakeMove( state &pos_robber, state &pos_cop, bool minFirst, double depth ) {
	std::vector<state> path;
	minimax( pos_robber, pos_cop, path, minFirst, depth );
	// sanity check
	assert( path.size() > 0 );
	return( path[0] );
}



#endif
