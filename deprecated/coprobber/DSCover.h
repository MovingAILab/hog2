#ifndef DSCOVER_H
#define DSCOVER_H

#include <vector>
//#include <utility>
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
	Implementation of Cover Heuristic for different speed system

	note: this is only for one cop and one robber, the DS interface
		does support one cop only!
	note: TPDijkstra actually computes cover and a little bit more.
	note: in this implementation players are always allowed to pass their turn

	note: although the implementation works for both, cops and robber it should not
	be used for the cop since cover has lots of flaws concerning move determination
*/
template<class state,class action>
class DSCover: public DSRobberAlgorithm<state,action> {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// constructor & destructor
	DSCover( SearchEnvironment<state,action> *env, unsigned int cop_speed = 1 );
	~DSCover();

	// who indicates who the cover was computed for
	// who == false => robber
	// who == true  => cop
	// note: this is done to speed up computation since it is no longer necessary
	// to expand the entire graph
	unsigned int cover( state pos_robber, state pos_cop, bool minFirst, bool &who );

	// this function needs the number of total states in the graph as a parameter
	// this is because the cover routine only computes the cover for one of the two agents
	// the other agent is determined by num_total_states - cover.
	state MakeMove( state pos_robber, state pos_cop, bool minFirst, unsigned int num_total_states );

	state MakeMove( state pos_robber, state pos_cop, unsigned int num_total_states ) {
		return MakeMove( pos_robber, pos_cop, false, num_total_states );
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
		QueueEntry( state &_s, unsigned int _numTurns, state &_parent ):
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

	struct MyStateHash {
		size_t operator() ( const state s ) const {
			return StateHash<state>( s );
		}
	};

	typedef std::priority_queue<QueueEntry,std::vector<QueueEntry>,QueueEntryCompare> MyPriorityQueue;
	typedef __gnu_cxx::hash_set<state,MyStateHash> ClosedList;

	MyPriorityQueue robber_queue, cop_queue;
	ClosedList robber_closed, cop_closed;

};


/*------------------------------------------------------------------------------
| IMPLEMENTATION
------------------------------------------------------------------------------*/

template<class state,class action>
DSCover<state,action>::DSCover( SearchEnvironment<state,action> *env, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<state,action>( env, false, cop_speed ) )
{};

template<class state,class action>
DSCover<state,action>::~DSCover() {
	delete dscrenv;
};

template<class state,class action>
void DSCover<state,action>::clear_cache() {
	robber_queue = MyPriorityQueue();
	cop_queue    = MyPriorityQueue();
	robber_closed.clear();
	cop_closed.clear();
	return;
};

template<class state,class action>
unsigned int DSCover<state,action>::cover( state pos_robber, state pos_cop, bool minFirst, bool &who ) {

	if( pos_robber == pos_cop ) {
		who = false;
		return 0;
	}

	// push the robber and the cop onto their open queues
	QueueEntry qcop( pos_cop, 0, pos_cop );
	cop_queue.push( qcop );
	QueueEntry qrobber( pos_robber, 0, pos_robber );
	robber_queue.push( qrobber );
	nodesTouched += 2;

	// create some variables that will be needed
	std::vector<state> neighbors;
	typename std::vector<state>::iterator it;
	QueueEntry qtemp;
	bool last_time_popped_from_robber_queue = true;

	// the counters for covered states
	unsigned int robber_cover = 0;
	unsigned int cop_cover    = 0;

	// as long as both players still have nodes to expand
	while( !robber_queue.empty() && !cop_queue.empty() ) {

		if( last_time_popped_from_robber_queue )
			qrobber = robber_queue.top();
		else
			qcop    = cop_queue.top();

		nodesTouched++;

		if( minFirst?(qrobber.numTurns<qcop.numTurns):(qrobber.numTurns<=qcop.numTurns) ) {
			// robber makes his move
			robber_queue.pop();
			last_time_popped_from_robber_queue = true;

			if( robber_closed.find( qrobber.s ) == robber_closed.end() &&
			    cop_closed.find( qrobber.parent ) == cop_closed.end() ) {

				// verbose
				//printf( "expanded node %lu for the robber\n", qrobber.s );

				robber_closed.insert( qrobber.s );

				// if we moved into the cop, he thought the node is covered
				// by him but we just stole it
				if( cop_closed.find( qrobber.s ) != cop_closed.end() )
					cop_cover--;
				robber_cover++;

				dscrenv->GetRobberSuccessors( qrobber.s, neighbors );
				nodesExpanded++;

				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;
					if( robber_closed.find( *it ) == robber_closed.end() ) {
						qtemp.s        = *it;
						qtemp.numTurns = qrobber.numTurns + 1;
						qtemp.parent   = qrobber.s;
						robber_queue.push( qtemp );
						//printf( "pushing %lu onto the queue for the robber\n", qtemp.s );
					}
				}
			}

		} else {
			// cop makes his move
			cop_queue.pop();
			last_time_popped_from_robber_queue = false;

			if( cop_closed.find( qcop.s ) == cop_closed.end() ) {

				// verbose
				//printf( "expanded node %lu for the cop\n", qcop.s );

				cop_closed.insert( qcop.s );

				// whether or not this state should be counted towards the cop cover
				// this is the case when
				// 1. the cop is neither capturing the robber
				// (in this case the node is counted towards the robber)
				// 2. the cop is adjacent to the robber and the robber can move into him
				// in his next turn. We count this case as cop_cover here and substract it
				// when the robber moves
				if( robber_closed.find( qcop.s ) == robber_closed.end() )
					cop_cover++;

				dscrenv->GetCopSuccessors( qcop.s, neighbors );
				nodesExpanded++;

				for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
					nodesTouched++;

					if( cop_closed.find( *it ) == cop_closed.end() ) {
						qtemp.s        = *it;
						qtemp.numTurns = qcop.numTurns + 1;
						//qtemp.parent   = qcop.s; // not needed
						cop_queue.push( qtemp );
						//printf( "pushing %lu onto the queue for the cop\n", qtemp.s );
					}
				}

			}
		}
	}

	unsigned int result;
	// determine the return values
	if( robber_queue.empty() ) {
		who    = false;
		result = robber_cover;
	} else {
		who    = true;
		result = cop_cover;
	}

	// cleanup
	clear_cache();

	return result;
};

template<class state,class action>
state DSCover<state,action>::MakeMove( state pos_robber, state pos_cop, bool minFirst, unsigned int num_total_states ) {

	nodesExpanded = 0; nodesTouched = 0;

	unsigned int max_cover = 0, temp;
	state max_cover_state;
	bool who;

	// dscrenv has been initialized with playerscanpass=false for our computations
	dscrenv->SetPlayersCanPass( true );

	std::vector<state> neighbors;
	if( minFirst )
		dscrenv->GetCopSuccessors( pos_cop, neighbors );
	else
		dscrenv->GetRobberSuccessors( pos_robber, neighbors );

	// reset the playerscanpass directive in dscrenv
	dscrenv->SetPlayersCanPass( false );

	// just to make sure that we're not in an isolated node
	assert( neighbors.size() > 0 );
	max_cover_state = neighbors[0];

	// for each successor state compute the cover
	for( typename std::vector<state>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
		if( minFirst )
			temp = cover( pos_robber, *it, false, who ); // !minFirst=false
		else
			temp = cover( *it, pos_cop, true, who ); // !minFirst=true

		if( who != minFirst ) temp = num_total_states - temp;

		if( temp > max_cover ) {
			max_cover = temp;
			max_cover_state = *it;
		}
	}

	return max_cover_state;
};


#endif
