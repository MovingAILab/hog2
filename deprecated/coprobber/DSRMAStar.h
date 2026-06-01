#include <vector>
#include <queue>
#include <unordered_map>
#include "MultiAgentEnvironment.h"
#include "DSCREnvironment.h"
#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"

#ifndef DSRMASTAR_H
#define DSRMASTAR_H

/*
	Implementation for one robber and one cop
	and different speed cop
*/
template<class state,class action,class environment>
class DSRMAStar {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// constructor
	DSRMAStar( environment *_env, unsigned int cop_speed = 1 );
	~DSRMAStar();

	double rmastar( CRState pos, bool minFirst );
	void clear_caches();

	unsigned int nodesExpanded, nodesTouched;

	protected:

	DSCREnvironment<state,action> *dscrenv;
	environment *env;

	void push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst );
	double compute_target_value( CRState &s );


	// priority queue
	class QueueEntry {
		public:
		QueueEntry( CRState &_pos, bool mf, double fv, double gv ):
			pos(_pos), minFirst(mf), fvalue(fv), gvalue(gv) {};
		QueueEntry() {};

		CRState pos;
		bool minFirst;
		double fvalue;
		double gvalue;
	};

	// be aware of q1.fvalue > q2.fvalue, this changes the ordering of our queue s.t.
	// we can top() the one with the lowest value
	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			return( q1.fvalue > q2.fvalue );
		}
	};
	
	typedef std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> MyPriorityQueue;

	// Priority Queues
	MyPriorityQueue queue;

	struct CRStateHash {
		size_t operator() ( const CRState &s ) const {
			return CRHash<state>( s );
		}
	};

	typedef std::unordered_map<CRState, double, CRStateHash> MyClosedList;
	// state => value
//	std::vector<double> min_cost, max_cost;
	MyClosedList min_cost, max_cost;


	// the definition of the heuristic relies on the graph heuristic used
	// in the submitted graph environment (=> use MaximumNormGraphMapHeuristic)
	double HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 );

};

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state, class action, class environment>
DSRMAStar<state,action,environment>::DSRMAStar( environment *_env, unsigned int cop_speed ):
	dscrenv( new DSCREnvironment<state,action>( _env, true, cop_speed ) ),
	env(_env)
{ };


template<class state, class action, class environment>
DSRMAStar<state,action,environment>::~DSRMAStar() {
	delete dscrenv;
};


template<class state, class action, class environment>
void DSRMAStar<state,action,environment>::clear_caches() {
	queue = MyPriorityQueue();
	min_cost.clear();
	max_cost.clear();
	return;
};


// \see DSRMAStar.cpp
template<>
void DSRMAStar<graphState,graphMove,GraphEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst );
template<>
void DSRMAStar<xyLoc,tDirection,MapEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst );

// \see DSRMAStar.cpp
template<>
double DSRMAStar<xyLoc,tDirection,MapEnvironment>::HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 );
// note that we do not really provide an implementation for graphs
// there is a general implementation further down from here that relies
// on the provided environment


template<class state,class action, class environment>
double DSRMAStar<state,action,environment>::rmastar( CRState goal_pos, bool goal_minFirst ) {
	QueueEntry qe, qtemp;
	typename MyClosedList::iterator mclit;
	std::vector<state> myneighbors;

	nodesExpanded = 0; nodesTouched = 0;

	// goal test
	if( dscrenv->GoalTest( goal_pos ) ) return dscrenv->TerminalCost( goal_pos );

	push_end_states_on_queue( goal_pos, goal_minFirst );

	while( !queue.empty() ) {

		// get the element from the queue
		qe = queue.top(); queue.pop();

		nodesTouched++;

		// recursion break
		if( qe.pos == goal_pos && qe.minFirst == goal_minFirst )
			return qe.gvalue;

		// verbose
		//fprintf( stdout, "minFirst = %d, pos = (%u,%u)(%u,%u), fvalue = %f, gvalue = %f\n", qe.minFirst, qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.fvalue, qe.gvalue );

		if( qe.minFirst ) {

			mclit = min_cost.find( qe.pos );

			if( mclit == min_cost.end() || (mclit != min_cost.end() && mclit->second > qe.gvalue) ) {

				min_cost[qe.pos] = qe.gvalue;

				// get the predecessor states
				dscrenv->GetRobberSuccessors( qe.pos, myneighbors, false );
				nodesExpanded++;

				/*
				fprintf( stdout, "neighbors = " );
				for( unsigned int i = 0; i < myneighbors.size(); i++ ) {
					fprintf( stdout, "(%u,%u) ", myneighbors[i].x, myneighbors[i].y );
				}
				fprintf( stdout, "\n" );
				*/

				qtemp.pos = qe.pos;

				// now, for all successor states
				for( typename std::vector<state>::iterator it = myneighbors.begin();
				     it != myneighbors.end(); it++ ) {
					nodesTouched++;
					// build the state
					qtemp.pos[0] = *it;


					qtemp.gvalue = compute_target_value( qtemp.pos );
					mclit = max_cost.find( qtemp.pos );
					if( (mclit == max_cost.end() && qtemp.gvalue != DBL_MAX) ||
					    (mclit != max_cost.end() && mclit->second > qtemp.gvalue ) ) {
						qtemp.minFirst = false;
						qtemp.fvalue = qtemp.gvalue + HCost( goal_pos, goal_minFirst, qtemp.pos, qtemp.minFirst );
						queue.push( qtemp );

						// verbose
						//fprintf( stdout, "pushing up: %d, (%u,%u)(%u,%u), g=%f, f=%f\n", qtemp.minFirst, qtemp.pos[0].x, qtemp.pos[0].y, qtemp.pos[1].x, qtemp.pos[1].y, qtemp.gvalue, qtemp.fvalue );
					}
				}
			}

		} else {

			mclit = max_cost.find( qe.pos );

			if( mclit == max_cost.end() || (mclit!=max_cost.end() && mclit->second > qe.gvalue) ) {

				max_cost[qe.pos] = qe.gvalue;

				dscrenv->GetCopSuccessors( qe.pos, myneighbors );
				nodesExpanded++;

				/*
				fprintf( stdout, "neighbors = " );
				for( unsigned int i = 0; i < myneighbors.size(); i++ ) {
					fprintf( stdout, "(%u,%u) ", myneighbors[i].x, myneighbors[i].y );
				}
				fprintf( stdout, "\n" );
				*/
				
				qtemp.pos = qe.pos;
				for( typename std::vector<state>::iterator it = myneighbors.begin();
				     it != myneighbors.end(); it++ ) {
					nodesTouched++;
					qtemp.pos[1] = *it;

					qtemp.gvalue   = qe.gvalue + dscrenv->CopGCost( qtemp.pos, qe.pos );
					mclit = min_cost.find( qtemp.pos );
					if( mclit == min_cost.end() || (mclit != min_cost.end() && mclit->second > qtemp.gvalue ) ) {
						qtemp.minFirst = true;
						qtemp.fvalue   = qtemp.gvalue + HCost( goal_pos, goal_minFirst, qtemp.pos, qtemp.minFirst );
						queue.push( qtemp );

						// verbose
						//fprintf( stdout, "pushing up: %d, (%u,%u)(%u,%u), g=%f, f=%f\n", qtemp.minFirst, qtemp.pos[0].x, qtemp.pos[0].y, qtemp.pos[1].x, qtemp.pos[1].y, qtemp.gvalue, qtemp.fvalue );
					}
				}
			}
		}

	}

	return DBL_MAX;
}


template<class state,class action,class environment>
double DSRMAStar<state,action,environment>::compute_target_value( CRState &s ) {
	double result = 0.;

	CRState temp = s;
	double tempvalue;
	typename MyClosedList::iterator mclit;
	std::vector<state> myneighbors;

	dscrenv->GetRobberSuccessors( s, myneighbors );
	//nodesExpanded++;

	// now, for all successor states
	for( typename std::vector<state>::iterator it = myneighbors.begin();
	     it != myneighbors.end(); it++ ) {
		//nodesTouched++;
	
		// build the state
		temp[0] = *it;
		mclit = min_cost.find( temp );
		if( mclit != min_cost.end() )
			tempvalue = mclit->second + dscrenv->RobberGCost( s, temp );
		else
			return DBL_MAX;
		if( tempvalue > result ) result = tempvalue;
	}
	return result;
}



// note: this HCost implementation relies on all edge costs 1 and MaximumNormGraphMapHeuristic in the GraphEnvironment
// furthermore, it only makes sense with the above definition of MinGCost===1
template<class state,class action,class environment>
double DSRMAStar<state,action,environment>::HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 ) {
	double hmax = env->HCost( pos1[0], pos2[0] );
	double hmin = ceil( env->HCost( pos1[1], pos2[1] ) / (double)dscrenv->GetCopSpeed() );

	if( minFirst1 == minFirst2 )
		return( 2. * max(hmax,hmin) );

	if( hmax == hmin )
		return( 2. * hmax + 1. );

	if( hmax < hmin ) {
		// robber has less way to go then one of the cops

		if( minFirst1 && !minFirst2 ) // cops starts and ends
			return( 2. * hmin - 1. );
		else // !minFirst1 && minFirst2 // robber starts and ends
			return( 2. * hmin + 1. );
	} else {
		// robber has more way to go then all of the cops
		if( minFirst1 && !minFirst2 )
			return( 2. * hmax + 1. );
		else
			return( 2. * hmax - 1. );
	}

}

#endif
