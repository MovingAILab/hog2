#include <vector>
#include <queue>
#include <set>
#include <ext/hash_set>
#include "MultiAgentEnvironment.h"
#include "DSCREnvironment.h"
#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"

#ifndef DSDIJKSTRA_H
#define DSDIJKSTRA_H

/*
	Dijkstra implementation for one cop and one robber
	possibly faster cop (using DSCREnvironment.h)

	Note: there is no support for whether the cop/robber can pass their
	turns or not
*/
template<class state, class action, class environment>
class DSDijkstra {

	public:

	// types
	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;


	// constructor and destructor
	DSDijkstra( environment *env, unsigned int cop_speed = 1 );
	~DSDijkstra();

	void dsdijkstra();

	void WriteValuesToDisk( const char* filename );
	
	// the access to min_cost and max_cost
	// note: it only makes sense to call these functions after dsdijkstra
	double Value( CRState &pos, bool minFirst );
	state MakeMove( CRState &pos, bool minFirst );

	unsigned int nodesExpanded;
	unsigned int nodesTouched;

	protected:

	DSCREnvironment<state,action> *dscrenv;
	environment *env;

	void push_end_states_on_queue();
	double compute_target_value( CRState &s );

	// priority queue
	class QueueEntry {
		public:
		QueueEntry( CRState _pos, bool mf, double v ):
			pos(_pos), minFirst(mf), value(v) {};
		QueueEntry() {};

		CRState pos;
		bool minFirst;
		double value;
	};

	// be aware of q1.value > q2.value, this changes the ordering of our queue s.t.
	// we can top() the one with the lowest value
	struct QueueEntryCompare {
		bool operator() ( const QueueEntry q1, const QueueEntry q2 ) const {
			return( q1.value > q2.value );
		}
	};

	typedef std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> MyPriorityQueue;

	// Priority Queues
	MyPriorityQueue queue;

	struct CRStateHash {
		uint64_t operator() ( const CRState &s ) const {
			return CRHash<state>( s );
		}
	};

	// closed lists
	typedef std::unordered_map<CRState, double, CRStateHash> ClosedList;
	ClosedList min_cost, max_cost;

	// TODO: Make this stable, this will only work with edge costs all set to 1
	// and a dijkstra algorithm where no states are explored multiple times.
	// THIS IS TEMPORARY TESTING CODE!
	//typedef std::set<uint64_t> OpenListIndexSet;
	//OpenListIndexSet min_olis, max_olis;

};

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state, class action, class environment>
DSDijkstra<state,action,environment>::DSDijkstra( environment *_env, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<state,action>( _env, true, cop_speed ) ),
	env(_env)
{ };

template<class state, class action, class environment>
DSDijkstra<state,action,environment>::~DSDijkstra() {
	delete dscrenv;
};


// \see DSDijkstra.cpp
template<>
void DSDijkstra<xyLoc,tDirection,MapEnvironment>::WriteValuesToDisk( const char* filename );
template<>
void DSDijkstra<graphState,graphMove,GraphEnvironment>::WriteValuesToDisk( const char* filename );


// \see DSDijkstra.cpp
template<>
void DSDijkstra<xyLoc,tDirection,MapEnvironment>::push_end_states_on_queue();
template<>
void DSDijkstra<graphState,graphMove,GraphEnvironment>::push_end_states_on_queue();


template<class state, class action, class environment>
double DSDijkstra<state,action,environment>::compute_target_value( CRState &s ) {
	double result = 0.;

	CRState temp = s;
	double tempvalue;
	std::vector<state> myneighbors;
	dscrenv->GetRobberSuccessors( temp, myneighbors );
	//nodesExpanded++;

	// now, for all successor states
	for( typename std::vector<state>::iterator it = myneighbors.begin();
	     it != myneighbors.end(); it++ ) {

		//nodesTouched++;
	
		// build the state
		temp[0] = *it;

		if( min_cost.find( temp ) == min_cost.end() )
			return DBL_MAX;

		tempvalue = dscrenv->RobberGCost( s, temp ) + min_cost[ temp ];
		if( tempvalue > result ) result = tempvalue;
	}
	return result;
};


template<class state, class action, class environment>
void DSDijkstra<state,action,environment>::dsdijkstra() {
	QueueEntry qe, qtemp;
	typename std::vector<state>::iterator it;

	assert( min_cost.empty() );
	assert( max_cost.empty() );
	assert( queue.empty() );

	nodesExpanded = 0;
	nodesTouched  = 0;

	push_end_states_on_queue();

	while( !queue.empty() ) {

		// get the element from the queue
		qe = queue.top(); queue.pop();
		nodesTouched++;

		// verbose
		//fprintf( stdout, "minFirst = %d, pos = (%u,%u)(%u,%u), value = %f\n", qe.minFirst, qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.value );

		if( qe.minFirst ) {

			//min_olis.erase( CRHash<state>( qe.pos ) );

			if( min_cost.find( qe.pos ) == min_cost.end() ) {

				min_cost[qe.pos] = qe.value;

				std::vector<state> myneighbors;
				dscrenv->GetRobberSuccessors( qe.pos, myneighbors, false );
				nodesExpanded++;

				// now, for all successor states
				for( it = myneighbors.begin(); it != myneighbors.end(); it++ ) {
					nodesTouched++;
					// build the state
					qe.pos[0] = *it;

					// check whether its value is not yet set
					if( max_cost.find( qe.pos ) == max_cost.end() ) { //&&
					    //max_olis.find( CRHash<state>( qe.pos ) ) == max_olis.end() ) {
						qe.value = compute_target_value( qe.pos );
						if( qe.value != DBL_MAX ) {
							qe.minFirst = false;
							//printf( "pushed %d (%u,%u)(%u,%u) on queue with %g\n", qe.minFirst,
							//	qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.value );
							queue.push( qe );
							//max_olis.insert( CRHash<state>( qe.pos ) );
						}
					}
				}
			}

		} else {

			//max_olis.erase( CRHash<state>( qe.pos ) );

			if( max_cost.find( qe.pos ) == max_cost.end() ) {

				max_cost[qe.pos] = qe.value;
				qtemp.pos = qe.pos;

				// get neighbors
				std::vector<state> myneighbors;
				dscrenv->GetCopSuccessors( qtemp.pos, myneighbors );
				nodesExpanded++;

				for( it = myneighbors.begin(); it != myneighbors.end(); it++ ) {
					nodesTouched++;
					
					// build the next state
					qtemp.pos[1] = *it;

					// check again whether already set or not
					if( min_cost.find( qtemp.pos ) == min_cost.end() ) { // &&
					    //min_olis.find( CRHash<state>( qtemp.pos ) ) == min_olis.end() ) {
						qtemp.minFirst = true;
						qtemp.value    = qe.value + dscrenv->CopGCost( qtemp.pos, qe.pos );
						//printf( "pushed %d (%u,%u)(%u,%u) on queue with %g\n", qtemp.minFirst,
						//	qtemp.pos[0].x, qtemp.pos[0].y, qtemp.pos[1].x, qtemp.pos[1].y, qtemp.value );
						queue.push( qtemp );
						//min_olis.insert( CRHash<state>( qtemp.pos ) );
					}
				}
			}
		}

	}

}


template<class state,class action,class environment>
double DSDijkstra<state,action,environment>::Value( CRState &pos, bool minFirst ) {
	if( minFirst ) {
		return min_cost[pos];
	} else {
		return max_cost[pos];
	}
};


template<class state,class action, class environment>
state DSDijkstra<state,action,environment>::MakeMove( CRState &pos, bool minFirst ) {

	CRState temppos = pos;
	std::vector<state> neighbors;
	double temp;
	state result = minFirst?temppos[1]:temppos[0];
	typename std::vector<state>::iterator it;

	// get the available moves
	if( minFirst ) {
		double value = DBL_MAX;
		dscrenv->GetCopSuccessors( temppos, neighbors );

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			temppos[1] = *it;
			temp = max_cost[ temppos ];
			if( value >= temp ) {
				value = temp;
				result = *it;
			}
		}
	} else {
		double value = -DBL_MAX;
		dscrenv->GetRobberSuccessors( temppos, neighbors );

		for( it = neighbors.begin(); it != neighbors.end(); it++ ) {
			temppos[0] = *it;
			temp = min_cost[ temppos ];
			if( value <= temp ) {
				value = temp;
				result = *it;
			}
		}
	}

	return result;
};


#endif
