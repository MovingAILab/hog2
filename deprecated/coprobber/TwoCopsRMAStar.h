#include <vector>
#include <set>
#include <queue>
#include <unordered_map>
#include "GraphEnvironment.h"
#include "OpenClosedList.h"

#ifndef TWOCOPSRMASTAR_H
#define TWOCOPSRMASTAR_H

/*
	RMA* Implementation for one robber and two cops

	note: this implementation is designed for two cops only
	  and does a few optimizations to compute much faster!

	note: agents can always pass their turns

	note: This implementation only works with graphs instead of any
		kind of environments. This is due to the hash functions.
*/
class TwoCopsRMAStar {

	public:

	// constructor
	TwoCopsRMAStar( GraphEnvironment *env );
	//~TwoCopsDijkstra();

	unsigned int rmastar( graphState &r, graphState &c1, graphState &c2, bool minFirst );

	void set_useHeuristic( bool _useHeuristic ) { useHeuristic = _useHeuristic; };
	void set_usePerfectDistanceHeuristic( bool usePerfectDistanceHeuristic );

	unsigned int nodesExpanded, nodesTouched;


	// temporarily moved to public:

	// for internal use within hashing and move generation functions
	typedef uint32_t Position;
	typedef graphState CRState[3];

	// copied from TwoCopsDijkstra
	Position CRHash_MemOptim( CRState &s );
	void MemOptim_Hash_To_CRState( Position &hash, CRState &s );

	unsigned int HCost( Position &pos, bool &minFirst );

	CRState current_goal;
	bool current_goal_minFirst;


	protected:
	
	// variables
	GraphEnvironment *env;
	unsigned int numnodes;

	// neighbor generation for the cops or the robber (depending on minFirst)
	// we're giving back a set here because we want uniqueness of moves
	void GetNeighbors( Position &pos, bool minFirst, std::set<Position> &neighbors );

	// functions for dijkstra computation
	unsigned int compute_target_value( Position &s );
	void push_end_states_on_queue();

	// queue
	class QueueEntry {
		public:
		QueueEntry() {};
		QueueEntry( Position _pos, bool mf, unsigned int gv, unsigned int fv ):
			pos( _pos ), minFirst( mf ), gvalue( gv ), fvalue( fv ) {};

		Position pos;
		bool minFirst;
		unsigned int gvalue, fvalue;
	};
	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			if( q1.fvalue == q2.fvalue ) {
				return( q1.gvalue < q2.gvalue );
			}
			return( q1.fvalue > q2.fvalue );
		}
	};
	struct QueueEntryEqual {
		bool operator() (const QueueEntry &q1, const QueueEntry &q2 ) {
			return( q1.pos == q2.pos && q1.minFirst == q2.minFirst ); }
	};
	struct QueueEntryHash {
		size_t operator() ( const QueueEntry &q ) const {
			return( ((uint64_t)q.minFirst << 32) | q.pos ); // pos is a uint32_t and
			//we hope we're running on a 64 bit machine ;-)
		}
	};
	//typedef OpenClosedList<QueueEntry, QueueEntryHash, QueueEntryEqual, QueueEntryCompare> PQueue;
	//PQueue queue;
	//void AddToOpenList( QueueEntry &qe );

	typedef std::priority_queue<QueueEntry, std::deque<QueueEntry>, QueueEntryCompare> MyPriorityQueue;
	MyPriorityQueue queue;

	// closed lists
	typedef std::unordered_map<Position,unsigned int> MyClosedList;
	MyClosedList min_cost, max_cost;

	void clear_cache();


	bool useHeuristic;
	bool usePerfectDistanceHeuristic;
	std::vector<std::vector<double> > distance_heuristic;

};


#endif
