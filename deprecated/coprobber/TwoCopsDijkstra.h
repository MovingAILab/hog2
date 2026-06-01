#include <vector>
#include <set>
#include <queue>
#include "GraphEnvironment.h"

#ifndef TWOCOPSDIJKSTRA_H
#define TWOCOPSDIJKSTRA_H

/*
	Implementation for one robber and two cops

	note: this implementation is designed for two cops only
	  and does a few optimizations to compute much faster!

	note: agents can always pass their turns

	note: edge costs are all set to 1! Therefore, we do not need
	  a priority queue anymore, thus hopefully increase in speed
*/
class TwoCopsDijkstra {

	public:

	typedef uint32_t Position;

	// constructor
	TwoCopsDijkstra( GraphEnvironment *env );
	//~TwoCopsDijkstra();

	void dijkstra();
	// returns whether the graph is 2-cop-win or not
	bool is_two_cop_win();

	void WriteValuesToDisk( const char* filename );

	unsigned int Value( graphState &r, graphState &c1, graphState &c2 );

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
		QueueEntry( Position _pos, bool mf, unsigned int v ):
			pos( _pos ), minFirst( mf ), value( v ) {};

		Position pos;
		bool minFirst;
		unsigned int value;
	};
	typedef std::queue<QueueEntry> MyQueue;
	MyQueue queue;

	// closed lists
	typedef std::vector<unsigned int> ClosedList;
	ClosedList min_cost, max_cost;

};


#endif
