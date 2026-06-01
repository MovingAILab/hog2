#include <vector>
#include <ext/hash_set>
#include <map>
#include "SearchEnvironment.h"
#include "MultiAgentEnvironment.h"
#include "MyHash.h"
#include "IPNSearch.h" // for function uintplus


#ifndef IPNTTABLES_H
#define IPNTTABLES_H

/*
	The difference to IPNTTables.h is that we use a tree structure here
	to keep the tree in memory and perform the updates on this tree (like
	deletion of subtrees etc).
	IPNTTables.h used a polynomial space structure (keeping only one node
	per state and bound) but cannot perform
	deletion operations due to this structure.

	This version turned out to be slower than IPNTTables.h(!)
	which is due the space complexity

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
	class TPEntry;
	struct TPEntryPointerHash {
		size_t operator() ( const TPEntry* p ) const { return (size_t)p; }
	};
	struct TPEntryPointerEqual {
		bool operator() ( const TPEntry* p1, const TPEntry* p2 ) const { return (p1==p2); }
	};
	typedef __gnu_cxx::hash_set<TPEntry*, TPEntryPointerHash, TPEntryPointerEqual> Transpositions;
	typedef std::unordered_map<CRState, Transpositions, CRStateHash, CRStateEqual> TTable;


	// the game tree nodes
	class TPEntry {
		public:
		TPEntry( CRState &_pos ): pos(_pos), parent(NULL), pointer_to_my_transpositions(NULL) {};
		TPEntry( CRState &_pos, unsigned int pn, unsigned int dn, bool mf = true, double v = 0., double b = 0. ):
		  pos(_pos), minFirst(mf), proof_number(pn), disproof_number(dn), value(v), bound(b), parent(NULL), pointer_to_my_transpositions(NULL) {};
		~TPEntry();

		void delete_children();
		void deregister_myself();

		CRState pos;
		bool minFirst;
		unsigned int proof_number, disproof_number;
		double value, bound;
		std::vector<TPEntry*> children;
		TPEntry* parent;
		Transpositions* pointer_to_my_transpositions;
	};


	// constructor
	IPNTTables( environment *_env, bool _canPause ):
		env(_env), canPause(_canPause) {};
	~IPNTTables();

	void clear_bounds_cache();

	double ipn( CRState &pos, bool minFirst = true );

	unsigned int nodesExpanded, nodesTouched, nodesUpdated;
	std::vector<unsigned int> iteration_nodesExpanded, iteration_nodesTouched, iteration_nodesUpdated;


	protected:

	// iterates until the root node is proved
	double ipn_update( CRState &pos, bool minFirst, double bound );
	// creates a node and registeres it in the transposition tables
	TPEntry* ipn_create_node( CRState &pos, bool minFirst, double bound, TPEntry *parent );
	// determines whether a node is prunable due to lower and upper cache bounds
	bool ipn_prunable_node( TPEntry *n );
	// does the update of a parent due to its child and nothing else
	// gives back whether an update has been done or not (on the proof and disproof numbers, not necessarily on the values)
	bool ipn_update_parent_from_children( TPEntry *n );
	// updates n and all occurences in n plus all their parents and grand parents and grand grand parents and...
	void ipn_update_from_node( TPEntry *n );
	// the main procedure of ipn
	void ipn_expand( TPEntry *n, TPEntry* &expanded_node );

	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest(const  CRState &pos );
	double TerminalCost( CRState &pos );

	environment *env;
	bool canPause;

	// lower bound cache
	BoundCache min_lcache, max_lcache;
	// upper bound cache
	BoundCache min_ucache, max_ucache;

	// transposition tables
	TTable min_ttable, max_ttable;
};


/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state,class action, class environment>
IPNTTables<state,action,environment>::TPEntry::~TPEntry() {
	delete_children();
	deregister_myself();
}

template<class state, class action, class environment>
void IPNTTables<state,action,environment>::TPEntry::deregister_myself() {
	// deregister yourself from the transposition tables
	if( pointer_to_my_transpositions != NULL ) {
		pointer_to_my_transpositions->erase( this );
		pointer_to_my_transpositions = NULL;
	}
	return;
}

template<class state, class action, class environment>
void IPNTTables<state,action,environment>::TPEntry::delete_children() {
	for( typename std::vector<TPEntry*>::iterator it = children.begin(); it != children.end(); it++ ) {
		delete (*it);
	}
	children.clear();
	return;
}


template<class state, class action, class environment>
IPNTTables<state,action,environment>::~IPNTTables() {
	clear_bounds_cache();
}


template<class state, class action, class environment>
void IPNTTables<state,action,environment>::clear_bounds_cache() {
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
		//fprintf( stdout, "bound set to %f\n", b );
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

	return c;
}

// protected functions

template<class state,class action, class environment>
double IPNTTables<state,action,environment>::ipn_update( CRState &pos, bool minFirst, double bound ) {

	// create the root search node
	TPEntry *expanded_node, *root = ipn_create_node( pos, minFirst, bound, NULL );

	while( root->proof_number != 0 && root->disproof_number != 0 ) {
		// verbose
		// fprintf( stdout, "running from the root (to prove: %u, to disprove: %u, value: %f)\n",
		// 	root->proof_number, root->disproof_number, root->value );
		ipn_expand( root, expanded_node );
		// verbose
		// fprintf( stdout, "expanded node (%u,%u)(%u,%u), player %d, bound %f, proof # %d, disproof # %d\n",
		//	expanded_node->pos[0].x, expanded_node->pos[0].y, expanded_node->pos[1].x, expanded_node->pos[1].y,
		//	expanded_node->minFirst, expanded_node->bound, expanded_node->proof_number, expanded_node->disproof_number );
		ipn_update_from_node( expanded_node );
	}

	// verbose output
	if( root->proof_number == 0 )
		fprintf( stdout, "Root has been proved with bound %f and value %f\n", bound, root->value );
	else //if( root.disproof_number == 0 )
		fprintf( stdout, "Root has been disproved with bound %f and value %f\n", bound, root->value );

	double result = root->value;
	delete root;
	return result;
}




template<class state, class action, class environment>
typename IPNTTables<state,action,environment>::TPEntry* IPNTTables<state,action,environment>::ipn_create_node( CRState &pos, bool minFirst, double bound, TPEntry *parent ) {

//	verbose
//	fprintf( stdout, "creating node for position (%u,%u)(%u,%u) player %d\n", pos[0].x, pos[0].y, pos[1].x, pos[1].y, minFirst );
	nodesTouched++;

	TPEntry *mytpentry = new TPEntry( pos );
	mytpentry->minFirst = minFirst;
	mytpentry->bound    = bound;
	mytpentry->parent   = parent;

	if( GoalTest( pos ) ) {
		// if we are at a terminal state
		if( bound < TerminalCost( pos ) ) {
			// proof
			mytpentry->proof_number    = 0;
			mytpentry->disproof_number = UINT_MAX;
			mytpentry->value           = TerminalCost( pos );
		} else {
			// disproof
			mytpentry->proof_number    = UINT_MAX;
			mytpentry->disproof_number = 0;
			mytpentry->value           = TerminalCost( pos );
		}
	} else {

		if( !ipn_prunable_node( mytpentry ) ) {
			// in every other case
			mytpentry->proof_number    = 1;
			mytpentry->disproof_number = 1;
			// the following two work, see the discussion in the paper
			//mytpentry->value           = minFirst?DBL_MAX:-DBL_MAX;
			mytpentry->value           = bound;

			// register the node in the transposition tables
			if( minFirst ) {
				min_ttable[mytpentry->pos].insert( mytpentry );
				mytpentry->pointer_to_my_transpositions = &(min_ttable[mytpentry->pos]);
			} else {
				max_ttable[mytpentry->pos].insert( mytpentry );
				mytpentry->pointer_to_my_transpositions = &(max_ttable[mytpentry->pos]);
			}
		}
	}

	return mytpentry;
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
bool IPNTTables<state,action,environment>::ipn_update_parent_from_children( TPEntry *n ) {
	nodesUpdated++;

	double child_value;
	unsigned int old_proof_number = n->proof_number, old_disproof_number = n->disproof_number;

	if( n->minFirst ) {
		// if we are in a min node
		n->proof_number = 0;
		n->disproof_number = UINT_MAX;
		n->value = DBL_MAX;
		for( typename std::vector<TPEntry*>::iterator iti = n->children.begin(); iti != n->children.end(); iti++ ) {
			// proof_number = sum( child proof numbers )
			n->proof_number = uintplus( n->proof_number, (*iti)->proof_number );
			// disproof_number = min( child disproof numbers )
			if( n->disproof_number > (*iti)->disproof_number ) n->disproof_number = (*iti)->disproof_number;

			// value = min( children values )
			child_value = ( n->bound - (*iti)->bound ) + (*iti)->value;
			if( n->value > child_value ) n->value = child_value;
		}

		// update lower and upper bounds if possible
		if( n->proof_number == 0 ) {
			min_lcache[n->pos] = n->value;
		} else if( n->disproof_number == 0 ) {
			min_ucache[n->pos] = n->value;
		}

	} else {
		// if we are in a max node
		n->proof_number = UINT_MAX;
		n->disproof_number = 0;
		n->value = -DBL_MAX;
		for( typename std::vector<TPEntry*>::iterator iti = n->children.begin(); iti != n->children.end(); iti++ ) {
			// proof number = min( child proof numbers )
			if( n->proof_number > (*iti)->proof_number ) n->proof_number = (*iti)->proof_number;
			// disproof number = sum( child disproof numbers )
			n->disproof_number = uintplus( n->disproof_number, (*iti)->disproof_number );

			// value = max( child values )
			child_value = (n->bound - (*iti)->bound) + (*iti)->value;
			if( n->value < child_value ) n->value = child_value;
		}

		if( n->proof_number == 0 ) {
			max_lcache[n->pos] = n->value;
		} else if( n->disproof_number == 0 ) {
			max_ucache[n->pos] = n->value;
		}

	}

	if( old_proof_number == n->proof_number && old_disproof_number == n->disproof_number )
		return false;

	// prune the unecessary subtree under n
	if( n->proof_number == 0 || n->disproof_number == 0 ) {
		n->delete_children();
		n->deregister_myself();
	}

	return true;
}




template<class state, class action, class environment>
void IPNTTables<state,action,environment>::ipn_update_from_node( TPEntry *n ) {

	typename TTable::iterator ttit;
	bool proceed = false, minFirst = n->minFirst;

	nodesTouched++;
	ipn_update_parent_from_children( n );

	// if we proved or disproved n by updating it then we have to proceed with
	// all the updates
	if( n->proof_number == 0 || n->disproof_number == 0 ) {
		proceed = true;
		ttit = (minFirst)?min_ttable.find(n->pos):max_ttable.find(n->pos);
	}

	// update n's parents first
	TPEntry *m = n->parent;
	while( m != NULL && ipn_update_parent_from_children( m ) ) {
		m = m->parent;
		nodesTouched++;
	}

	if( proceed ) {
		// sanity check, if we can update an element it should be in the transposition tables
		assert( ttit != ((minFirst)?min_ttable.end():max_ttable.end()) );

		std::vector<TPEntry*> my_transpositions;
		// copy all the transitions into a vector that we can iterate over safely
		for( typename Transpositions::iterator tpit = ttit->second.begin(); tpit != ttit->second.end(); tpit++ )
			my_transpositions.push_back( *tpit );

		for( typename std::vector<TPEntry*>::iterator mtit = my_transpositions.begin();
		     mtit != my_transpositions.end(); mtit++ ){

			// in case the element is the old n or does not exist anymore
			if( *mtit == n || ttit->second.find( *mtit ) == ttit->second.end() ) continue;

			m = *mtit;
			nodesTouched++;
			// proceed only if we have some impact on this transition
			if( ipn_prunable_node( m ) ) {
				m->delete_children();
				m->deregister_myself();
				m = m->parent;
				while( m != NULL && ipn_update_parent_from_children( m ) ) {
					m = m->parent;
					nodesTouched++;
				}
			}

		}
	}

	return;
}



template<class state, class action, class environment>
void IPNTTables<state,action,environment>::ipn_expand( TPEntry *n, TPEntry* &expanded_node ) {
	nodesTouched++;

	// prune if possible
	if( ipn_prunable_node( n ) ) {
		// since we prune here we do not expand a node, but the parent of n has
		// still to be updated, hence expanded_node = n->parent
		expanded_node = n->parent;
		n->delete_children();
		n->deregister_myself();
		return;
	}


	if( n->children.empty() ) {
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
		CRState child_pos = n->pos;
		for( typename std::vector<state>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
			// generate child position
			child_pos[myid] = *it;

			n->children.push_back( ipn_create_node( child_pos, !n->minFirst, n->bound - MinGCost( n->pos, child_pos ), n ) );
		}

		// Sanity check: if this fails it would mean we are in a node that has no
		// children but is not a terminal node
		assert( n->children.size() > 0 );

		// the expanded node is obviously n ;-)
		expanded_node = n;

	} else {

		typename std::vector<TPEntry*>::iterator iti, itj;
		if( n->minFirst ) {
			for( iti = n->children.begin(); iti != n->children.end(); iti++ ) {
				if( (*iti)->disproof_number == n->disproof_number ) break;
			}
		} else {
			for( iti = n->children.begin(); iti != n->children.end(); iti++ ) {
				if( (*iti)->proof_number == n->proof_number ) break;
			}
		}

		// expand to the next child
		assert( iti != n->children.end() );
		ipn_expand( *iti, expanded_node );

	}
	return;
}


template<class state, class action, class environment>
double IPNTTables<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	if( canPause )
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}

// specification for state=xyLoc
template<>
double IPNTTables<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	double dist;
	dist = max( abs(pos[1].x - pos[0].x), abs(pos[1].y - pos[0].y) );

	if( canPause )
		return( 2. * dist - (minsTurn?MinGCost(pos,pos):0.) );
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
