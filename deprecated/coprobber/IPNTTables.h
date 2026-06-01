#include <vector>
#include <ext/hash_set>
#include <map>
#include "SearchEnvironment.h"
#include "MultiAgentEnvironment.h"
#include "MyHash.h"


#ifndef IPNTTABLES_H
#define IPNTTABLES_H

/*
	Implementation of proof number search with (consistent) heuristic
	enhancement and iterative increase of the bound to be proofed
	(inspired by TIDA*).
	This implementation differs from IPNSearch.h in so far that it tries
	to incorporate transposition tables

	Note: This version has been used to generate statistics for
	"Optimal Solutions to Moving Target Search"

	Implementation for one cop and one robber
*/
template<class state, class action, class environment>
class IPNTTables {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;


	// bounds cache
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
	typedef std::unordered_map<CRState, double, CRStateHash, CRStateEqual> BoundCache;


	// transposition tables
	class TPEntry {
		public:
		TPEntry( CRState &_pos ): pos(_pos), last_iteration(0), updated_on_last_iteration(false), iteration_of_update(0) {};
		TPEntry( CRState &_pos, unsigned int pn, unsigned int dn, bool mf = true, double v = 0., double b = 0. ):
			pos(_pos), minFirst(mf), proof_number(pn), disproof_number(dn), value(v), bound(b),
			last_iteration(0), updated_on_last_iteration(false), iteration_of_update(0) {};

		CRState pos;
		bool minFirst;
		unsigned int proof_number, disproof_number;
		double value, bound;
		std::vector<TPEntry*> childs;
		unsigned int last_iteration;
		bool updated_on_last_iteration;
		unsigned int iteration_of_update;
	};
	struct TPEntryHash {
		size_t operator()( const TPEntry *e ) const {
			return CRHash<state>( e->pos );
		}
	};
	struct TPEntryEqual {
		bool operator()( const TPEntry *e1, const TPEntry *e2 ) const {
			return( e1->pos == e2->pos );
		}
	};

	typedef __gnu_cxx::hash_set<TPEntry*, TPEntryHash, TPEntryEqual> TranspositionTable;
	typedef std::map<double, TranspositionTable> GTTables;
	typedef std::pair<double, TranspositionTable> GTTables_Pair;



	// constructor
	IPNTTables( environment *_env, bool _canPause ):
		env(_env), canPause(_canPause) {};

	double ipn( CRState &pos, bool minFirst = true );

	unsigned int nodesExpanded, nodesTouched, nodesUpdated;
	std::vector<unsigned int> iteration_nodesExpanded, iteration_nodesTouched, iteration_nodesUpdated;


	protected:

	// iterates until the root node is proved
	double ipn_update( CRState &pos, bool minFirst, double bound );
	// determines whether a node is prunable or not and prunes it right away
	bool ipn_prunable_node( TPEntry *n );
	// creates a node and registeres it in the transposition tables
	TPEntry* ipn_create_node( CRState &pos, bool minFirst, double bound );
	// updates the values/numbers of a parent due to the values of his childs
	void ipn_update_node( TPEntry *n, unsigned int &iteration );
	// gives back whether in the subtree under n there was an update needed (and done)
	// w.r.t. the expanded node expanded_node in iteration iteration
	bool ipn_update_branch( TPEntry *n, TPEntry* &expanded_node, unsigned int &iteration );
	// the main procedure of ipn
	void ipn_expand( TPEntry *n, unsigned int &iteration, TPEntry* &expanded_node );

	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest(const  CRState &pos );
	double TerminalCost( CRState &pos );

	void clear_bounds_cache();

	environment *env;
	bool canPause;

	// lower bound cache
	BoundCache min_lcache, max_lcache;
	// upper bound cache
	BoundCache min_ucache, max_ucache;

	// transposition tables
	GTTables min_gttables, max_gttables;
};




/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state,class action,class environment>
void IPNTTables<state,action,environment>::clear_bounds_cache() {
	min_gttables.clear();
	max_gttables.clear();
	min_lcache.clear();
	max_lcache.clear();
	min_ucache.clear();
	max_ucache.clear();
	return;
}


template<class state,class action,class environment>
double IPNTTables<state,action,environment>::ipn( CRState &pos, bool minFirst ) {

	double b, c = MinHCost( pos, minFirst );

	unsigned int sumNodesExpanded = 0, sumNodesTouched = 0, sumNodesUpdated = 0;

	do {
		nodesExpanded = 0; nodesTouched = 0; nodesUpdated = 0;
		b = c;
		fprintf( stdout, "bound set to %f\n", b );
		c = ipn_update( pos, minFirst, b );
		iteration_nodesExpanded.push_back( nodesExpanded );
		iteration_nodesTouched.push_back( nodesTouched );
		iteration_nodesUpdated.push_back( nodesUpdated );
		sumNodesExpanded += nodesExpanded;
		sumNodesTouched  += nodesTouched;
		sumNodesUpdated  += nodesUpdated;
	} while( c > b );

	nodesExpanded = sumNodesExpanded;
	nodesTouched  = sumNodesTouched;
	nodesUpdated  = sumNodesUpdated;

	clear_bounds_cache();

	return c;
}

// protected functions

template<class state,class action, class environment>
double IPNTTables<state,action,environment>::ipn_update( CRState &pos, bool minFirst, double bound ) {

	unsigned int iteration = 0;

	// create the root search node
	TPEntry *temptpentry, *root = ipn_create_node( pos, minFirst, bound );

	while( root->proof_number != 0 && root->disproof_number != 0 ) {
		iteration++;
		// verbose
		// fprintf( stdout, "running from the root (to prove: %u, to disprove: %u, value: %f)\n",
		// 	root->proof_number, root->disproof_number, root->value );
		ipn_expand( root, iteration, temptpentry );
		// verbose
		// fprintf( stdout, "expanded node (%u,%u)(%u,%u), player %d, bound %f\n",
		//	temptpentry->pos[0].x, temptpentry->pos[0].y, temptpentry->pos[1].x, temptpentry->pos[1].y,
		//	temptpentry->minFirst, temptpentry->bound );
	}

	// verbose output
	/*
	if( root->proof_number == 0 )
		fprintf( stdout, "Root has been proved with bound %f and value %f\n", bound, root->value );
	else //if( root.disproof_number == 0 )
		fprintf( stdout, "Root has been disproved with bound %f and value %f\n", bound, root->value );
	*/

	double result = root->value;

	typename GTTables::iterator gttit;
	typename TranspositionTable::iterator tit;
	for( gttit = min_gttables.begin(); gttit != min_gttables.end(); gttit++ ) {
		while( !gttit->second.empty() ) {
			tit = gttit->second.begin();
			temptpentry = *tit;
			gttit->second.erase( tit );
			delete temptpentry;
		}
	}
	for( gttit = max_gttables.begin(); gttit != max_gttables.end(); gttit++ ) {
		while( !gttit->second.empty() ) {
			tit = gttit->second.begin();
			temptpentry = *tit;
			gttit->second.erase( tit );
			delete temptpentry;
		}
	}
	min_gttables.clear();
	max_gttables.clear();

	return result;
}



template<class state, class action, class environment>
bool IPNTTables<state,action,environment>::ipn_prunable_node( TPEntry *n ) {

	bool result = false;

	// upper bound cache lookup
	typename BoundCache::iterator hcit;
	BoundCache *current_bcache = (n->minFirst)?&min_ucache:&max_ucache;
	hcit = current_bcache->find( n->pos );
	if( hcit != current_bcache->end() ) {
		if( hcit->second <= n->bound ) {
			// in this case we disproved the node
			n->proof_number    = UINT_MAX;
			n->disproof_number = 0;
			n->value           = hcit->second;
			result = true;
		}
	}

	// lower bound cache lookup
	current_bcache = (n->minFirst)?&min_lcache:&max_lcache;
	hcit = current_bcache->find( n->pos );
	if( hcit != current_bcache->end() ) {
		if( n->bound < hcit->second ) {
			// the node has been proved
			n->proof_number    = 0;
			n->disproof_number = UINT_MAX;
			n->value           = hcit->second;
			result = true;
		}
	} else {
		// perform heuristic pruning step
		double heuristic_value = MinHCost( n->pos, n->minFirst );
		if( n->bound < heuristic_value ) {
			n->proof_number    = 0;
			n->disproof_number = UINT_MAX;
			n->value           = heuristic_value;
			result = true;
		}
	}

	return result;
}




template<class state, class action, class environment>
typename IPNTTables<state,action,environment>::TPEntry* IPNTTables<state,action,environment>::ipn_create_node( CRState &pos, bool minFirst, double bound ) {

//	verbose
//	fprintf( stdout, "creating node for position (%u,%u)(%u,%u) player %d\n", pos[0].x, pos[0].y, pos[1].x, pos[1].y, minFirst );

	nodesTouched++;

	// transposition table lookup

	// allocation
	TPEntry *mytpentry = new TPEntry( pos );
	typename GTTables::iterator gttit;
	typename TranspositionTable::iterator tit;
	GTTables *current_gttable = (minFirst)?&min_gttables:&max_gttables;

	gttit = current_gttable->find( bound );
	if( gttit != current_gttable->end() ) {

		tit = gttit->second.find( mytpentry );
		if( tit != gttit->second.end() ) {
			delete mytpentry; // cleanup
			return (*tit); // return a pointer on the TPEntry
		}
	} else {
		std::pair<typename GTTables::iterator, bool> mypair = current_gttable->insert( typename GTTables::value_type( bound, TranspositionTable() ) );
		gttit = mypair.first;
		assert( mypair.second );
	}

	// if we didn't find the position in the transposition table then
	// create a new entry
	mytpentry->minFirst = minFirst;
	mytpentry->bound    = bound;

	if( pos[0] == pos[1] ) { //GoalTest( pos ) ) {
		// if we are at a terminal state
		if( bound < 0. ) { // TerminalCost( pos ) ) {
			// proof
			mytpentry->proof_number    = 0;
			mytpentry->disproof_number = UINT_MAX;
			mytpentry->value           = 0.; //TerminalCost( pos );
		} else {
			// disproof
			mytpentry->proof_number    = UINT_MAX;
			mytpentry->disproof_number = 0;
			mytpentry->value           = 0.; //TerminalCost( pos );
		}
	} else {

		if( !ipn_prunable_node( mytpentry ) ) {
			// in every other case
			mytpentry->proof_number    = 1;
			mytpentry->disproof_number = 1;
			mytpentry->value           = bound;
		}
	}

	// push the entry onto the transposition tables
	gttit->second.insert( mytpentry );
	return mytpentry;
}





template<class state, class action, class environment>
void IPNTTables<state,action,environment>::ipn_update_node( TPEntry *n, unsigned int &iteration ) {

	nodesUpdated++;

	n->last_iteration = iteration;
	n->updated_on_last_iteration = true;
	n->iteration_of_update = iteration;

	double child_value;
	if( n->minFirst ) {
		// if we are in a min node
		n->proof_number = 0;
		n->disproof_number = UINT_MAX;
		n->value = DBL_MAX;
		for( typename std::vector<TPEntry*>::iterator iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {

			// proof_number = sum( child proof numbers )
			n->proof_number = uintplus( n->proof_number, (*iti)->proof_number );
			// disproof_number = min( child disproof numbers )
			if( n->disproof_number > (*iti)->disproof_number ) n->disproof_number = (*iti)->disproof_number;

			// value = min( childs values )
			child_value = ( n->bound - (*iti)->bound ) + (*iti)->value;
			if( n->value > child_value ) n->value = child_value;
		}

		// update lower and upper bounds
		if( n->proof_number == 0 ) {
			min_lcache[n->pos] = max( min_lcache[n->pos], n->value );
		} else if( n->disproof_number == 0 ) {
			min_ucache[n->pos] = min( min_lcache[n->pos], n->value );
		}


	} else {
		// if we are in a max node
		n->proof_number = UINT_MAX;
		n->disproof_number = 0;
		n->value = -DBL_MAX;
		for( typename std::vector<TPEntry*>::iterator iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
			// proof number = min( child proof numbers )
			if( n->proof_number > (*iti)->proof_number ) n->proof_number = (*iti)->proof_number;
			// disproof number = sum( child disproof numbers )
			n->disproof_number = uintplus( n->disproof_number, (*iti)->disproof_number );

			// value = max( child values )
			child_value = (n->bound - (*iti)->bound) + (*iti)->value;
			if( n->value < child_value ) n->value = child_value;
		}

		if( n->proof_number == 0 ) {
			max_lcache[n->pos] = max( max_lcache[n->pos], n->value );
		} else if( n->disproof_number == 0 ) {
			max_ucache[n->pos] = min( max_ucache[n->pos], n->value );
		}

	}
	return;
}



template<class state, class action, class environment>
bool IPNTTables<state,action,environment>::ipn_update_branch( TPEntry *n, TPEntry* &expanded_node, unsigned int &iteration ) {

	nodesTouched++;

	// h-prune, if we are too far down in the tree and the expanded_node is above us
	// should work fine with non-negative edge costs
	if( n->bound < expanded_node->bound )
		return false;

	// if this node has been updated throughout this iteration
	if( n->last_iteration == iteration ) return n->updated_on_last_iteration;

	// CARE: enabling this can cause bugs when not deleting the subtrees under the nodes.
	// This is because statistics under this node will not be updated. When later on,
	// a node in the subtree is rediscovered over a different path (that hasn't been in the problem
	// within this iteration), then the old stats will be taken from the cache and we have an
	// un-updated problem!!!
	// However, when deleting the subtree, we are probably fine to do this here...
	// if this node is proved yet there is no update needed
//	if( n->proof_number == 0 || n->disproof_number == 0 )
//		return false;

	bool update_needed = false;
	for( typename std::vector<TPEntry*>::iterator iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
		if( ipn_update_branch( *iti, expanded_node, iteration ) ) update_needed = true;
	}

	if( update_needed )
		ipn_update_node( n, iteration );
	else {
		n->last_iteration = iteration;
		n->updated_on_last_iteration = false;
	}

	return update_needed;
}


template<class state, class action, class environment>
void IPNTTables<state,action,environment>::ipn_expand( TPEntry *n, unsigned int &iteration, TPEntry* &expanded_node ) {

	nodesTouched++;

	if( ipn_prunable_node( n ) ) {
		expanded_node = n;
		n->last_iteration = iteration;
		n->updated_on_last_iteration = true;
		n->iteration_of_update = iteration;
		return;
	}

	if( n->childs.empty() ) {
		// if this node doesn't have any children yet then we create them
		nodesExpanded++;

		// Sanity check: do we expand terminal nodes? This should never happen!
		assert( !GoalTest( n->pos ) );

		// get the next positions for the current player
		int myid = n->minFirst?1:0;
		std::vector<state> neighbors;
		env->GetSuccessors( n->pos[myid], neighbors );
		if( canPause ) neighbors.push_back( n->pos[myid] );

		// generate all children
		CRState child_pos;
		for( unsigned int i = 0; i < neighbors.size(); i++ ) {
			// generate child position
			child_pos = n->pos;
			child_pos[myid] = neighbors[i];

			//n->childs.push_back( ipn_create_node( child_pos, !n->minFirst, n->bound - MinGCost( n->pos, child_pos ) ) );
			n->childs.push_back( ipn_create_node( child_pos, !n->minFirst, n->bound - 1. ) );
		}

		// Sanity check: if this fails it would mean we are in a node that has no
		// children but is not a terminal node
		assert( n->childs.size() > 0 );

		// since we have expanded this node, set it to the expanded node
		expanded_node = n;

		// update my own values
		ipn_update_node( n, iteration );

	} else {

		typename std::vector<TPEntry*>::iterator iti, itj;
		if( n->minFirst ) {
			for( iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
				if( (*iti)->disproof_number == n->disproof_number ) break;
			}
		} else {
			for( iti = n->childs.begin(); iti != n->childs.end(); iti++ ) {
				if( (*iti)->proof_number == n->proof_number ) break;
			}
		}

		// expand to the next child
		assert( iti != n->childs.end() );

		/*
		if( iti == n->childs.end() ) {
			if( n->minFirst ) {
				fprintf( stdout, "parent = (%u,%u)(%u,%u) minFirst = %d bound = %f\n",
				         n->pos[0].x, n->pos[0].y, n->pos[1].x, n->pos[1].y, n->minFirst, n->bound );
				fprintf( stdout, "parents disproof number is %u (seen last on %u(%d),updated last on %u)\n",
				         n->disproof_number, n->last_iteration, n->updated_on_last_iteration, n->iteration_of_update );
				for( itj = n->childs.begin(); itj != n->childs.end(); itj++ ) {
					fprintf( stdout, "child (%u,%u)(%u,%u) %d, bound=%f, dpn = %u, li = %u (%d), update = %u\n",
					         (*itj)->pos[0].x, (*itj)->pos[0].y, (*itj)->pos[1].x, (*itj)->pos[1].y,
									 (*itj)->minFirst, (*itj)->bound, (*itj)->disproof_number, (*itj)->last_iteration,
									 (*itj)->updated_on_last_iteration, (*itj)->iteration_of_update );
				}
				fprintf( stdout, "\n" );
			} else {
				fprintf( stdout, "parent = (%u,%u)(%u,%u) minFirst = %d bound = %f\n",
				         n->pos[0].x, n->pos[0].y, n->pos[1].x, n->pos[1].y, n->minFirst, n->bound );
				fprintf( stdout, "parents proof number is %u (seen last on %u(%d), updated last on %u)\n",
				         n->proof_number, n->last_iteration, n->updated_on_last_iteration, n->iteration_of_update );
				for( itj = n->childs.begin(); itj != n->childs.end(); itj++ ) {
					fprintf( stdout, "child (%u,%u)(%u,%u) %d, bound = %f, pn = %u, li = %u (%d), update = %u\n",
					         (*itj)->pos[0].x, (*itj)->pos[0].y, (*itj)->pos[1].x, (*itj)->pos[1].y,
									 (*itj)->minFirst, (*itj)->bound, (*itj)->proof_number, (*itj)->last_iteration,
									 (*itj)->updated_on_last_iteration, (*itj)->iteration_of_update );
				}
				fprintf( stdout, "\n" );
			}
			exit(1);
		}
		*/

		ipn_expand( *iti, iteration, expanded_node );

		// this is the point where the values are coming back from the computed subtree
		// now update all the other branches with respect to the expanded_node
		for( itj = n->childs.begin(); itj != n->childs.end(); itj++ ) {
			if( itj != iti )
				ipn_update_branch( *itj, expanded_node, iteration );
		}

		ipn_update_node( n, iteration );
	}
	return;
}


template<class state, class action, class environment>
double IPNTTables<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	if( canPause )
		//return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?1.:0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}

// specification for state=xyLoc
template<>
double IPNTTables<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	double dist;
	if( abs(pos[1].x - pos[0].x) < abs(pos[1].y - pos[0].y) )
		dist = abs(pos[1].y - pos[0].y);
	else
		dist = abs(pos[1].x - pos[0].x);

	if( canPause )
		//return( 2. * dist - (minsTurn?MinGCost(pos,pos):0.) );
		return( 2. * dist - (minsTurn?1.:0.) );
	else
		return dist;
}


template<class state, class action, class environment>
double IPNTTables<state,action,environment>::MinGCost( CRState&, CRState& ) {
	return 1.;
}

template<class state, class action, class environment>
bool IPNTTables<state,action,environment>::GoalTest( CRState &pos ) {
	return( pos[0] == pos[1] );
}

template<class state, class action, class environment>
double IPNTTables<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
};

#endif
