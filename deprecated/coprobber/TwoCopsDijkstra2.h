#include <vector>
#include <set>
#include <queue>
#include "GraphEnvironment.h"

#ifndef TWOCOPSDIJKSTRA2_H
#define TWOCOPSDIJKSTRA2_H

/*
	Implementation for one robber and two cops

	This is the implementation that was used for the Isaza Cover comparison
	for the two cops same speed game
*/
class TwoCopsDijkstra2 {

	public:

	typedef uint32_t Position;

	// constructor
	TwoCopsDijkstra2( GraphEnvironment *env );
	//~TwoCopsDijkstra2();

	void dijkstra();
	// returns whether the graph is 2-cop-win or not
	bool is_two_cop_win();

	void WriteValuesToDisk( const char* filename );

	float Value( graphState &r, graphState &c1, graphState &c2 );

	unsigned int nodesExpanded, nodesTouched;

	protected:

	// for internal use within hashing and move generation functions
	typedef graphState CRState[3];

	// variables
	GraphEnvironment *env;
	unsigned int numnodes;

	// hash functions for positions
	// since we have two cops, these functions sort the CRState
	Position CRHash_MemOptim( CRState &s );
	void MemOptim_Hash_To_CRState( Position &hash, CRState &s );

	typedef std::pair<Position,float> NeighborPair;
	struct NeighborPairCompare {
		bool operator() ( const NeighborPair &np1, const NeighborPair &np2 ) {
			if( np1.first == np2.first )
				return( np1.second < np2.second );
			return( np1.first < np2.first );
		}
	};
	typedef std::set<NeighborPair,NeighborPairCompare> NeighborSet;


	// neighbor generation for the cops or the robber (depending on minFirst)
	// we're giving back a set here because we want uniqueness of moves
	void GetNeighbors( Position &pos, bool minFirst, NeighborSet &neighbors );

	float GCost(const Position &pos1, const Position &pos2 );

	// functions for dijkstra computation
	float compute_target_value( Position &s );
	void push_end_states_on_queue();

	// queue
	class QueueEntry {
		public:
		QueueEntry() {};
		QueueEntry( Position _pos, bool mf, float v ):
			pos( _pos ), minFirst( mf ), value( v ) {};

		Position pos;
		bool minFirst;
		float value;
	};
	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			return( q1.value > q2.value );
		}
	};
	typedef std::priority_queue<QueueEntry, std::deque<QueueEntry>, QueueEntryCompare> MyQueue;
	MyQueue queue;

	// closed lists
	typedef std::vector<float> ClosedList;
	ClosedList min_cost, max_cost;

};


#endif
