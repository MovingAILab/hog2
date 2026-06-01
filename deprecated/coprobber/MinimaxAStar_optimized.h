#include <vector>
#include <queue>
#include <unordered_map>
#include "MyHash.h"
#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"
#include "CopRobberGame.h"
#include "FloydWarshall.h"

#ifndef MINIMAXASTAR_H
#define MINIMAXASTAR_H

/*
	Implementation for one robber and one cop

	note: bool canPass is not supported when pushing up the terminal
	states onto the queue
*/
template<class state,class action,class environment>
class MinimaxAStar {

	public:

	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

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

	struct CRStateHash {
		size_t operator() ( const CRState &s ) const {
			return CRHash<state>( s );
		}
	};

	typedef std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> MyPriorityQueue;
	typedef std::unordered_map<CRState, double, CRStateHash> MyClosedList;

	// constructor
	// for usage with state=graphState, action=graphMove, environment=GraphEnvironment
	MinimaxAStar( environment *_env, unsigned int _number_of_cops, bool _canPass );

	double astar( CRState pos, bool minFirst );

	void set_useHeuristic( bool _useHeuristic ) { useHeuristic = _useHeuristic; };
	void set_usePerfectDistanceHeuristic( bool usePerfectDistanceHeuristic );

	unsigned int nodesExpanded, nodesTouched;

	protected:

	void push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst );
	double compute_target_value( CRState &s );

	// we use these functions independent from the environment implementations
	// to be more variable in our own calculations
	double MinGCost( CRState &pos1, CRState &pos2 );
	// the definition of the heuristic relies on the graph heuristic used
	// in the submitted graph environment (=> use MaximumNormGraphMapHeuristic)
	double HCost(CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 );

	bool GoalTest(const  CRState &pos );
	double TerminalCost( CRState &pos );

	environment *env;
	unsigned int number_of_cops;
	bool canPass;

	bool useHeuristic;
	bool usePerfectDistanceHeuristic;

	// Priority Queues
	MyPriorityQueue queue;
	// state => value
//	std::vector<double> min_cost, max_cost;
	MyClosedList min_cost, max_cost;

	void clear_cache();

	//MyClosedList my_minheuristic, my_maxheuristic;
	std::vector<std::vector<double> > distance_heuristic;
	
};


/* Header code for MinimaxAStar_optimized.cpp */
template<>
void MinimaxAStar<graphState,graphMove,GraphEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst );
template<>
void MinimaxAStar<xyLoc,tDirection,MapEnvironment>::push_end_states_on_queue( CRState &goal_pos, bool &goal_minFirst );
template<>
double MinimaxAStar<xyLoc,tDirection,MapEnvironment>::HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 );
template<>
double MinimaxAStar<xyLoc,tDirection,MapEnvironment>::MinGCost( CRState &p1, CRState &p2 );
template<>
void MinimaxAStar<xyLoc,tDirection,MapEnvironment>::set_usePerfectDistanceHeuristic( bool );



/* IMPLEMENTATION */

template<class state,class action,class environment>
MinimaxAStar<state,action,environment>::MinimaxAStar( environment *_env, unsigned int _number_of_cops, bool _canPass ):
	env(_env), number_of_cops(_number_of_cops), canPass(_canPass), useHeuristic(true), usePerfectDistanceHeuristic(false) {
/*
	time_t t; time( &t ); srandom( (unsigned int) t );
	printf( "seed = %u\n", (unsigned int)t );
*/
};

template<class state,class action,class environment>
void MinimaxAStar<state,action,environment>::clear_cache() {
	min_cost.clear();
	max_cost.clear();
	queue = MyPriorityQueue();
	return;
}


template<class state,class action, class environment>
double MinimaxAStar<state,action,environment>::astar( CRState goal_pos, bool goal_minFirst ) {
	QueueEntry qe, qtemp;
	typename MyClosedList::iterator mclit;
	std::vector<state> myneighbors;

	nodesExpanded = 0; nodesTouched = 0;

	if( GoalTest( goal_pos ) ) return TerminalCost( goal_pos );

	push_end_states_on_queue( goal_pos, goal_minFirst );

	while( !queue.empty() ) {

		// get the element from the queue
		qe = queue.top(); queue.pop();

		nodesTouched++;

		// recursion break
		if( qe.pos == goal_pos && qe.minFirst == goal_minFirst ) {
			clear_cache();
			return qe.gvalue;
		}

		// verbose
		//fprintf( stdout, "minFirst = %d, pos = (%u,%u)(%u,%u), fvalue = %f, gvalue = %f\n", qe.minFirst, qe.pos[0].x, qe.pos[0].y, qe.pos[1].x, qe.pos[1].y, qe.fvalue, qe.gvalue );

		if( qe.minFirst ) {

			mclit = min_cost.find( qe.pos );

			if( mclit == min_cost.end() || (mclit != min_cost.end() && mclit->second > qe.gvalue) ) {

				min_cost[qe.pos] = qe.gvalue;

				// the cops moved in this state, thus we have to find the
				// opposite actions of the robber here
				// note: this cannot be implemented with the CopRobberGame
				//   interface since that doesn't allow us to move away from the cops
				//   after capture (CopRobberOccupancy)
				myneighbors.clear();
				env->GetSuccessors( qe.pos[0], myneighbors );
				if( canPass )
					myneighbors.push_back( qe.pos[0] );
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
//						if( qtemp.gvalue != DBL_MAX ) {
							qtemp.minFirst = false;
							qtemp.fvalue = qtemp.gvalue + HCost( goal_pos, goal_minFirst, qtemp.pos, qtemp.minFirst );
							queue.push( qtemp );

							// verbose
							//fprintf( stdout, "pushing up: %d, (%u,%u)(%u,%u), g=%f, f=%f\n", qtemp.minFirst, qtemp.pos[0].x, qtemp.pos[0].y, qtemp.pos[1].x, qtemp.pos[1].y, qtemp.gvalue, qtemp.fvalue );
//						}
					}
				}
			}

		} else {

			mclit = max_cost.find( qe.pos );

			if( mclit == max_cost.end() || (mclit!=max_cost.end() && mclit->second > qe.gvalue) ) {

				max_cost[qe.pos] = qe.gvalue;

				myneighbors.clear();
				env->GetSuccessors( qe.pos[1], myneighbors );
				if( canPass )
					myneighbors.push_back( qe.pos[1] );
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

					qtemp.gvalue   = qe.gvalue + MinGCost( qe.pos, qtemp.pos );
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

	clear_cache();
	return DBL_MAX;
}


template<class state,class action,class environment>
double MinimaxAStar<state,action,environment>::compute_target_value( CRState &s ) {
	double result = 0.;

	CRState temp = s;
	double tempvalue;
	typename MyClosedList::iterator mclit;
	std::vector<state> myneighbors;
	env->GetSuccessors( s[0], myneighbors );
	if( canPass )
		myneighbors.push_back( s[0] );
	//nodesExpanded++;

	// now, for all successor states
	for( typename std::vector<state>::iterator it = myneighbors.begin();
	     it != myneighbors.end(); it++ ) {
		//nodesTouched++;
	
		// build the state
		temp[0] = *it;
		mclit = min_cost.find( temp );
		if( mclit != min_cost.end() )
			tempvalue = mclit->second + MinGCost( temp, s );
		else
			return DBL_MAX;
		if( tempvalue > result ) result = tempvalue;
	}
	return result;
}



template<class state,class action,class environment>
double MinimaxAStar<state,action,environment>::MinGCost( CRState&, CRState& ) {
	return 1.;
};


// note: this HCost implementation relies on all edge costs 1 and MaximumNormGraphMapHeuristic in the GraphEnvironment
// furthermore, it only makes sense with the above definition of MinGCost===1
template<class state,class action,class environment>
double MinimaxAStar<state,action,environment>::HCost( CRState &pos1, bool &minFirst1, CRState &pos2, bool &minFirst2 ) {
	if( !useHeuristic ) return 0.;

	double hmax = env->HCost( pos1[0], pos2[0] );
	double hmin = 0.;
	double h;

	for( unsigned int i = 1; i < pos1.size(); i++ ) {
		h = env->HCost( pos1[i], pos2[i] );
		hmin = max( hmin, h );
	}

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

template<class state, class action, class environment>
bool MinimaxAStar<state,action,environment>::GoalTest( CRState &pos ) {
	return (pos[0]==pos[1]);
}

template<class state, class action, class environment>
double MinimaxAStar<state,action,environment>::TerminalCost( CRState& ) {
	return 0.;
}

#endif
