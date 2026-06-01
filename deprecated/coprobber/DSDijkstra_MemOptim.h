#include <vector>
#include <queue>
#include <set>
#include <ext/hash_set>
#include "MultiAgentEnvironment.h"
#include "DSCREnvironment.h"
#include "GraphEnvironment.h"

#ifndef DSDIJKSTRA_MEMOPTIM_H
#define DSDIJKSTRA_MEMOPTIM_H

/*
	Dijkstra implementation for one cop and one robber
	possibly faster cop (using DSCREnvironment.h)

	This implementation has been optimized for memory consumption
	It is basically the exact same implementation as DSDijkstra.h only with
	changes towards memory consumption.
	This is also the reason why we have to instantiate the class
	and why more than 65535 nodes in the graph are not supported!

	Note: there is no support for whether the cop/robber can pass their
	turns or not
*/
class DSDijkstra_MemOptim {

	public:

	// types
	typedef MultiAgentEnvironment<graphState,graphMove>::MAState CRState;


	// constructor and destructor
	DSDijkstra_MemOptim( GraphEnvironment *env, unsigned int cop_speed = 1 );
	~DSDijkstra_MemOptim();

	void dsdijkstra();

	void WriteValuesToDisk( const char* filename );
	void ReadValuesFromDisk( const char* filename );
	
	// the access to min_cost and max_cost
	// note: it only makes sense to call these functions after dsdijkstra
	float Value( CRState &pos, bool minFirst );
	graphState MakeMove( CRState &pos, bool minFirst );
	void MakeSingleStepsCopMove( CRState &pos, std::vector<graphState> &moves );

	void DrawCopRobberEdges( bool minFirst, graphState pos_opponent );

	unsigned int nodesExpanded;
	unsigned int nodesTouched;

	protected:

	DSCREnvironment<graphState,graphMove> *dscrenv;
	GraphEnvironment *env;
	unsigned int numnodes;

	uint32_t CRHash_MemOptim( CRState &s );
	void MemOptim_Hash_To_CRState( uint32_t &hash, CRState &s );
	void push_end_states_on_queue();
	float compute_target_value( CRState &s );

	// priority queue
	class QueueEntry {
		public:
		QueueEntry( uint32_t _pos_hash, bool mf, float v ):
			pos_hash(_pos_hash), minFirst(mf), value(v) {};
		QueueEntry() {};

		uint32_t pos_hash;
		bool minFirst;
		float value;
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

	// closed lists
	typedef std::vector<float> ClosedList;
	ClosedList min_cost, max_cost;
	float max_max_cost;

};

#endif
