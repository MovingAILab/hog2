#ifndef DSTPDIJKSTRA_H
#define DSTPDIJKSTRA_H

#include <vector>
#include <queue>
#include <ext/hash_set>
#include <unordered_map>
#include "MultiAgentEnvironment.h"
#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"
#include "MyHash.h"
#include "DSCREnvironment.h"
#include "DSRobberAlgorithm.h"


/*
	Implementation of two player dijkstra for the cop and robber problem
	added: Implementation now also supports a faster cop
	added: next move generation
	computation has changed (compared to TwoPlayerDijkstra.h) and is thus faster

	note: in this calculation, players are always allowed to pass their turns
*/
template<class state,class action>
class DSTPDijkstra: public DSRobberAlgorithm<state,action> {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// constructor & destructor
	DSTPDijkstra( SearchEnvironment<state,action> *env, unsigned int cop_speed = 1 );
	~DSTPDijkstra();

	// gives back a path that the robber should follow
	// minFirst does only mean whether the cop or the robber are to move
	// first in the situation
	double dstpdijkstra( state pos_robber, state pos_cop, bool minFirst, std::vector<state> &path );

	state MakeMove( state pos_robber, state pos_cop, unsigned int ) {
		if( dscrenv->GoalTest( pos_robber, pos_cop ) )
			return pos_robber;
		std::vector<state> path;
		dstpdijkstra( pos_robber, pos_cop, false, path );
		return path[0];
	};

	unsigned int nodesExpanded, nodesTouched;

	protected:

	DSCREnvironment<state,action> *dscrenv;

	void clear_cache();

	private:

	// queues and hash sets/maps
	class QueueEntry {
		public:
		QueueEntry() {};
		QueueEntry( state &_s, unsigned int _numTurns, state &_parent):
			s(_s),numTurns(_numTurns),parent(_parent) {};
		state s;
		unsigned int numTurns;
		state parent;
	};
	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			return( q1.numTurns > q2.numTurns );
		}
	};

	class RobberClosedListEntry {
		public:
		RobberClosedListEntry(): value(0) {};
		RobberClosedListEntry( unsigned int &_value, state &_parent ):
			value(_value), parent(_parent) {};
		unsigned int value;
		state parent;
	};

	struct MyStateHash {
		size_t operator() ( const state s ) const {
			return StateHash<state>( s );
		}
	};

	typedef std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> MyPriorityQueue;
	typedef __gnu_cxx::hash_set<state, MyStateHash> CopClosedList;
	typedef std::unordered_map<state, RobberClosedListEntry, MyStateHash> RobberClosedList;

	MyPriorityQueue robber_queue, cop_queue;
	CopClosedList cop_closed;
	RobberClosedList robber_closed;

};


/*------------------------------------------------------------------------------
| IMPLEMENTATION
------------------------------------------------------------------------------*/
template<class state, class action>
DSTPDijkstra<state,action>::DSTPDijkstra( SearchEnvironment<state,action> *env, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<state,action>( env, false, cop_speed ) )
{ };

template<class state, class action>
DSTPDijkstra<state,action>::~DSTPDijkstra() {
	delete dscrenv;
};

template<class state, class action>
void DSTPDijkstra<state,action>::clear_cache() {
	robber_queue = MyPriorityQueue();
	cop_queue    = MyPriorityQueue();
	cop_closed.clear();
	robber_closed.clear();
	return;
}

template<class state,class action>
double DSTPDijkstra<state,action>::dstpdijkstra( state pos_robber, state pos_cop, bool minFirst, std::vector<state> &path ) {

	path.clear();
	nodesExpanded = 0; nodesTouched = 0;

	if( pos_robber == pos_cop ) return 0.;

	std::vector<state> neighbors;
	typename std::vector<state>::iterator it;
	QueueEntry qtemp;
	bool last_time_popped_from_robber_queue = true;
	unsigned int robber_positions_caught_by_cop = 0;
	state last_caught_state = pos_robber;
	unsigned int last_cop_turn = 0;

	// push the robber and the cop onto their open queues
	QueueEntry qcop( pos_cop, 0, pos_cop );
	cop_queue.push( qcop );

	QueueEntry qrobber( pos_robber, 0, pos_robber );
	robber_queue.push( qrobber );

	// as long as the cop is in the same connected component his
	// open queue will not run out of nodes faster than the robbers
	while( !robber_queue.empty() ) {

		if( last_time_popped_from_robber_queue )
			qrobber = robber_queue.top();
		else
			qcop    = cop_queue.top();

		nodesTouched++;

		if( minFirst?(qrobber.numTurns<qcop.numTurns):(qrobber.numTurns<=qcop.numTurns) ) {
			robber_queue.pop();
			last_time_popped_from_robber_queue = true;
			//fprintf( stdout, "popped from robber queue: (%u,%u) at turn %u\n", qrobber.s.x, qrobber.s.y, qrobber.numTurns );

			if( robber_closed.find( qrobber.s ) == robber_closed.end() &&
			    cop_closed.find( qrobber.parent ) == cop_closed.end() &&
			    cop_closed.find( qrobber.s ) == cop_closed.end() ) {

				nodesExpanded++;

				robber_closed[qrobber.s] = RobberClosedListEntry( qrobber.numTurns, qrobber.parent );

				dscrenv->GetRobberSuccessors( qrobber.s, neighbors );

				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qtemp.s        = *it;
					qtemp.numTurns = qrobber.numTurns + 1;
					qtemp.parent   = qrobber.s;
					robber_queue.push( qtemp );
				}
			}
		} else {
			cop_queue.pop();
			last_time_popped_from_robber_queue = false;
			//fprintf( stdout, "popped from cop queue: (%u,%u) at turn %u\n", qcop.s.x, qcop.s.y, qcop.numTurns );

			if( cop_closed.find( qcop.s ) == cop_closed.end() ) {

				nodesExpanded++;

				cop_closed.insert( qcop.s );
				last_cop_turn = qcop.numTurns;

				if( robber_closed.find( qcop.s ) != robber_closed.end() ) {
					robber_positions_caught_by_cop++;
					last_caught_state = qcop.s;
					if( robber_positions_caught_by_cop == robber_closed.size() ) // that means we have them all
						break;
				}

				dscrenv->GetCopSuccessors( qcop.s, neighbors );

				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qtemp.s        = *it;
					qtemp.numTurns = qcop.numTurns + 1;
					//qtemp.parent  = qcop.s; // not needed
					cop_queue.push( qtemp );
				}
			}
		}
	}

	// if not all positions of the robber are caught yet
	if( robber_positions_caught_by_cop < robber_closed.size() ) {
		while( !cop_queue.empty() ) {
			qcop = cop_queue.top(); cop_queue.pop();
			if( cop_closed.find( qcop.s ) == cop_closed.end() ) {
				nodesExpanded++;
				cop_closed.insert( qcop.s );
				last_cop_turn = qcop.numTurns;
				if( robber_closed.find( qcop.s ) != robber_closed.end() ) {
					robber_positions_caught_by_cop++;
					last_caught_state = qcop.s;
					if( robber_positions_caught_by_cop == robber_closed.size() )
						break;
				}
				dscrenv->GetCopSuccessors( qcop.s, neighbors );
				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qtemp.s        = *it;
					qtemp.numTurns = qcop.numTurns + 1;
					cop_queue.push( qtemp );
				}
			}
		}
	}

	// path generation
	RobberClosedListEntry rcle = robber_closed[last_caught_state];
	for( unsigned int i = rcle.value + (minFirst?1:0); i < last_cop_turn; i++ )
		path.push_back( last_caught_state );
	while( rcle.value != 0 ) {
		path.push_back( last_caught_state );
		last_caught_state = rcle.parent;
		rcle = robber_closed[last_caught_state];
	}
	std::reverse( path.begin(), path.end() );

	// cleanup
	clear_cache();

	return( (double) (2 * last_cop_turn - (minFirst?1:0) ) );

}

#endif
