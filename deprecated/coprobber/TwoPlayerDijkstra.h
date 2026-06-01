#include <vector>
#include <queue>
#include <ext/hash_set>
#include <unordered_map>
#include "MultiAgentEnvironment.h"
#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"
#include "MyHash.h"

#ifndef TWOPLAYERDIJKSTRA_H
#define TWOPLAYERDIJKSTRA_H

/*
	Implementation of two player dijkstra for the cop and robber problem
*/
template<class state,class action,class environment>
class TwoPlayerDijkstra {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	class QueueEntry {
		public:
		QueueEntry() {};
		QueueEntry( state _s, unsigned int _numTurns, state _parent) : s(_s),numTurns(_numTurns),parent(_parent) {};
		state s;
		unsigned int numTurns;
		state parent;
	};
	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			return( q1.numTurns > q2.numTurns );
		}
	};

	struct MyStateHash {
		size_t operator() ( const state s ) const {
			return StateHash<state>( s );
		}
	};

	typedef std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> MyPriorityQueue;
	typedef __gnu_cxx::hash_set<state, MyStateHash> CopClosedList;
	typedef std::unordered_map<state, unsigned int, MyStateHash> RobberClosedList;

	// constructor
	TwoPlayerDijkstra( environment *_env, bool _canPass ):
		env(_env), canPass(_canPass) {};


	void clear_cache();
	double tpdijkstra( state pos_robber, state pos_cop );

	unsigned int nodesExpanded, nodesTouched;

	protected:

	environment *env;
	bool canPass;

	MyPriorityQueue robber_queue, cop_queue;
	CopClosedList cop_closed;
	RobberClosedList robber_closed;

};


/* IMPLEMENTATION */
template<class state, class action, class environment>
void TwoPlayerDijkstra<state,action,environment>::clear_cache() {
	robber_queue = MyPriorityQueue();
	cop_queue    = MyPriorityQueue();
	cop_closed.clear();
	robber_closed.clear();
	return;
}

template<class state,class action,class environment>
double TwoPlayerDijkstra<state,action,environment>::tpdijkstra( state pos_robber, state pos_cop ) {
	nodesExpanded = 0; nodesTouched = 0;

	// push the robber and the cop onto their open queues
	std::vector<state> neighbors;
	typename std::vector<state>::iterator it;
	QueueEntry qtemp;
	unsigned int last_cop_turn = 0, last_robber_turn = 0;

	QueueEntry qcop( pos_cop, 0, pos_cop );
	cop_queue.push( qcop );

	QueueEntry qrobber( pos_robber, 0, pos_robber );
	robber_queue.push( qrobber );

	while( !robber_queue.empty() ) {
		qrobber = robber_queue.top();
		qcop    = cop_queue.top();

		nodesTouched += 2;

		neighbors.clear();
		if( qrobber.numTurns < qcop.numTurns ) {
			robber_queue.pop();
			//fprintf( stdout, "popped from robber queue: (%u,%u) at turn %u\n", qrobber.s.x, qrobber.s.y, qrobber.numTurns );

			typename RobberClosedList::iterator rclit = robber_closed.find( qrobber.s );
			if( cop_closed.find( qrobber.parent ) == cop_closed.end() &&
			    (rclit == robber_closed.end() || rclit->second < qrobber.numTurns) ) {

				nodesExpanded++;
				last_robber_turn = qrobber.numTurns;

				robber_closed[qrobber.s] = qrobber.numTurns;

				env->GetSuccessors( qrobber.s, neighbors );
				if( canPass ) neighbors.push_back( qrobber.s );

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
			//fprintf( stdout, "popped from cop queue: (%u,%u) at turn %u\n", qcop.s.x, qcop.s.y, qcop.numTurns );

			if( cop_closed.find( qcop.s ) == cop_closed.end() ) {

				nodesExpanded++;
				last_cop_turn = qcop.numTurns;

				cop_closed.insert( qcop.s );

				env->GetSuccessors( qcop.s, neighbors );
				//if( canPass ) neighbors.push_back( qcop.s );

				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					qtemp.s        = *it;
					qtemp.numTurns = qcop.numTurns + 1;
					qtemp.parent   = qcop.s;
					cop_queue.push( qtemp );
				}
			}
		}
	}

	clear_cache();
	return( (double) (last_robber_turn + last_cop_turn) );

}

#endif
