#include <vector>
#include "SearchEnvironment.h"
#include "MultiAgentEnvironment.h"
#include "MyHash.h"

#ifndef IPNSEARCH_H
#define IPNSEARCH_H


/*
	Implementation of proof number search with (consistent) heuristic
	enhancement and iterative increase of the bound to be proofed
	(inspired by TIDA*).

	Implementation for one cop and one robber
*/
template<class state, class action, class environment>
class IPNSearch {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// space for transposition tables
	// maybe later, not yet since the update of the proof numbers will get difficult then

	// constructor
	IPNSearch( environment *_env, bool _canPause ):
		env(_env), canPause(_canPause) {};

	// search nodes
	class SearchNode {
		public:
		SearchNode( CRState &pos, bool mf, unsigned int pn, unsigned int dn, double v ):
			position(pos), minFirst(mf), proof_number(pn), disproof_number(dn), value(v) {};

		CRState position;
		bool minFirst;
		unsigned int proof_number, disproof_number;
		double value;
		std::vector<SearchNode> childs;
	};

	double ipn( CRState &pos, bool minFirst = true );

	protected:

	// iterates until the root node is proved
	double ipn_update( CRState &pos, bool minFirst, double bound );
	// creates a child node
	SearchNode ipn_create_node( CRState &pos, bool minFirst, double bound );
	void ipn_expand( SearchNode &n, double bound );

	double MinHCost( CRState &pos, bool minsTurn = true );
	double MinGCost( CRState &pos1, CRState &pos2 );
	bool GoalTest(const  CRState &pos );
	double TerminalCost( CRState &pos );

	environment *env;
	bool canPause;
};




/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

template<class state,class action,class environment>
double IPNSearch<state,action,environment>::ipn( CRState &pos, bool minFirst ) {

	// we have to make sure that we do not submit a terminal node to the algorithm
	if( GoalTest( pos ) ) return TerminalCost( pos );

	double b, c = MinHCost( pos, minFirst );

	do {
		b = c;
//		fprintf( stdout, "bound set to %f\n", b );
		c = ipn_update( pos, minFirst, b );
	} while( c > b );

	return c;
}

// protected functions

template<class state,class action, class environment>
double IPNSearch<state,action,environment>::ipn_update( CRState &pos, bool minFirst, double bound ) {

	// create the root search node
	SearchNode root = ipn_create_node( pos, minFirst, bound );

	while( root.proof_number != 0 && root.disproof_number != 0 ) {
//		fprintf( stdout, "running from the root (to prove: %u, to disprove: %u, value: %f)\n", root.proof_number, root.disproof_number, root.value );
		ipn_expand( root, bound );
	}

	// verbose output
	if( root.proof_number == 0 )
		fprintf( stdout, "Root has been proved with bound %f and value %f\n", bound, root.value );
	else //if( root.disproof_number == 0 )
		fprintf( stdout, "Root has been disproved with bound %f and value %f\n", bound, root.value );

	return root.value;	

}

template<class state, class action, class environment>
typename IPNSearch<state,action,environment>::SearchNode IPNSearch<state,action,environment>::ipn_create_node( CRState &pos, bool minFirst, double bound ) {

//	verbose
//	fprintf( stdout, "creating node for position (%u,%u)(%u,%u) player %d\n", pos[0].x, pos[0].y, pos[1].x, pos[1].y, minFirst );

	if( GoalTest( pos ) ) {
		// if we are at a terminal state
		if( TerminalCost( pos ) > bound )
			// disproved
			return SearchNode( pos, minFirst, UINT_MAX, 0, TerminalCost( pos ) );
		else
			// proved
			return SearchNode( pos, minFirst, 0, UINT_MAX, TerminalCost( pos ) );
	}
	if( bound < MinHCost( pos, minFirst ) ) {
		// if there is no possibility to prove this bound in this node, h-prune
		return SearchNode( pos, minFirst, UINT_MAX, 0, MinHCost( pos, minFirst ) );
	}

	return SearchNode( pos, minFirst, 1, 1, bound );
}

template<class state, class action, class environment>
void IPNSearch<state,action,environment>::ipn_expand( SearchNode &n, double bound ) {

	while( true ) {

		if( n.childs.empty() ) {
			// if this node doesn't have any children yet then we create them

			// Sanity check: do we expand terminal nodes? This should never happen!
			assert( !GoalTest( n.position ) );

			// get the next positions for the current player
			int myid = n.minFirst?1:0;
			std::vector<state> neighbors;
			env->GetSuccessors( n.position[myid], neighbors );
			if( canPause ) neighbors.push_back( n.position[myid] );

			// generate all children
			CRState child_position;
			double path_cost_to_child;
			for( unsigned int i = 0; i < neighbors.size(); i++ ) {

				// generate child position
				child_position = n.position;
				child_position[myid] = neighbors[i];

				path_cost_to_child = MinGCost( n.position, child_position );

				n.childs.push_back( ipn_create_node( child_position, !n.minFirst, bound - path_cost_to_child ) );
			}

			// Sanity check: if this fails it would mean we are in a node that has node
			// children but is not a terminal node
			assert( n.childs.size() > 0 );

		} else {

			unsigned int i = 0;
			if( n.minFirst ) {
				for( i = 0; i < n.childs.size(); i++ )
					if( n.childs[i].proof_number == n.proof_number ) break;
			} else {
				for( i = 0; i < n.childs.size(); i++ )
					if( n.childs[i].disproof_number == n.disproof_number ) break;
			}

			// expand to the next child
			ipn_expand( n.childs[i], bound - MinGCost( n.position, n.childs[i].position ) );
		}


		// copy the old values
		unsigned int old_proof_number = n.proof_number, old_disproof_number = n.disproof_number;

		double child_value;
		// now update my own statistics => this can be made sooo much better by including into the upper cases!!!!
		if( n.minFirst ) {
			// if we are in a min node
			n.proof_number = UINT_MAX;
			n.disproof_number = 0;
			n.value = DBL_MAX;
			for( unsigned int i = 0; i < n.childs.size(); i++ ) {
				// proof_number = min( child proof numbers )
				if( n.proof_number > n.childs[i].proof_number ) n.proof_number = n.childs[i].proof_number;
				// disproof_number = sum( child disproof numbers )
				n.disproof_number = uintplus( n.disproof_number, n.childs[i].disproof_number );

				// value = min( childs values )
				child_value = MinGCost( n.position, n.childs[i].position ) + n.childs[i].value;
				if( n.value > child_value ) n.value = child_value;
			}
		} else {
			// if we are in a max node
			n.proof_number = 0;
			n.disproof_number = UINT_MAX;
			n.value = -DBL_MAX;
			for( unsigned int i = 0; i < n.childs.size(); i++ ) {
				// proof number = sum( child proof numbers )
				n.proof_number = uintplus( n.proof_number, n.childs[i].proof_number );
				// disproof number = min( child disproof numbers )
				if( n.disproof_number > n.childs[i].disproof_number ) n.disproof_number = n.childs[i].disproof_number;

				// value = max( child values )
				child_value = MinGCost( n.position, n.childs[i].position ) + n.childs[i].value;
				if( n.value < child_value ) n.value = child_value;
			}
		}

		// we have to detect whether our own values changed or not
		if( old_proof_number != n.proof_number || old_disproof_number != n.disproof_number )
			break;

	}

	return;
}


template<class state, class action, class environment>
double IPNSearch<state,action,environment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	if( canPause )
		return ( 2. * env->HCost( pos[1], pos[0] ) - (minsTurn?MinGCost(pos,pos):0.) );
	else
		// distance from cop to the robber
		return env->HCost( pos[1], pos[0] );
}

// specification for state=xyLoc
template<>
double IPNSearch<xyLoc,tDirection,MapEnvironment>::MinHCost( CRState &pos, bool minsTurn ) {
	if( GoalTest( pos ) ) return 0.;

	double dist;
	if( abs(pos[1].x - pos[0].x) < abs(pos[1].y - pos[0].y) )
		dist = abs(pos[1].y - pos[0].y);
	else
		dist = abs(pos[1].x - pos[0].x);

	if( canPause )
		return( 2. * dist - (minsTurn?MinGCost(pos,pos):0.) );
	else
		return dist;
}


template<class state, class action, class environment>
double IPNSearch<state,action,environment>::MinGCost( CRState&, CRState& ) {
	return 1.;
}

template<class state, class action, class environment>
bool IPNSearch<state,action,environment>::GoalTest( CRState &pos ) {
	return( pos[0] == pos[1] );
}

template<class state, class action, class environment>
double IPNSearch<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
};

#endif
