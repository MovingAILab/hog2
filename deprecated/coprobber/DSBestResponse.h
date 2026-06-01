#include <vector>
#include <queue>
#include <set>
#include <ext/hash_set>
#include "MultiAgentEnvironment.h"
#include "DSCREnvironment.h"
#include "GraphEnvironment.h"
#include "DSRobberAlgorithm.h"

#ifndef DSBESTRESPONSE_H
#define DSBESTRESPONSE_H

/*
	Computes a best response to a given robber algorithm.
	This robber algorithm cannot keep track of the history
	since we compute the game tree in a bottom-up fashion.

	The computation is basically the Dijkstra algorithm
	described in the thesis, only that we are considering
	the move the robber algorithm would do instead of all
	possible robber moves.
*/
class DSBestResponse {

	public:

	// types
	typedef MultiAgentEnvironment<graphState,graphMove>::MAState CRState;


	// constructor and destructor
	DSBestResponse( GraphEnvironment *env, DSRobberAlgorithm<graphState,graphMove> *ralg, unsigned int cop_speed = 1 );
	~DSBestResponse();

	void compute_best_response();

	void WriteValuesToDisk( const char* filename );
	void ReadValuesFromDisk( const char* filename );
	
	// the access to min_cost and max_cost
	// note: it only makes sense to call these functions after computebestresponse
	float Value( CRState &pos, bool minFirst );
	graphState MakeMove( CRState &pos, bool minFirst );
	void MakeSingleStepsCopMove( CRState &pos, std::vector<graphState> &moves );

	void DrawCopRobberEdges( bool minFirst, graphState pos_opponent );

	unsigned int nodesExpanded;
	unsigned int nodesTouched;
	unsigned int ralgCalls;

	protected:

	DSCREnvironment<graphState,graphMove> *dscrenv;
	GraphEnvironment *env;
	DSRobberAlgorithm<graphState,graphMove> *ralg;
	unsigned int numnodes;

	uint32_t CRHash_MemOptim( CRState &s );
	void MemOptim_Hash_To_CRState( uint32_t &hash, CRState &s );
	void compute_robber_moves();
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

	// moves are from s to ralg_moves[s]
	std::vector<graphState> ralg_moves;

};

#endif
