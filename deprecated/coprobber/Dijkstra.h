#include <vector>
#include <queue>
#include <ext/hash_set>
#include "GraphEnvironment.h"
#include "CopRobberGame.h"

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

/*
	Implementation of Dijkstra algorithm (see thesis of Isaza for description)
	Implementation for one robber and multiple cops
*/
class Dijkstra {

	public:

	typedef CopRobberGame::CRState CRState;
	typedef CopRobberGame::CRMove CRMove;
	typedef CopRobberGame::CRAction CRAction;

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

	// constructor
	Dijkstra( GraphEnvironment *_env, unsigned int _number_of_cops, bool _canPass );
	~Dijkstra();

	void dijkstra();

	void WriteValuesToDisk( const char* filename );

	// state => value (h_p, h_t in description of the algorithm)
	std::vector<double> min_cost, max_cost;
	CopRobberGame *crg;

	protected:

	void push_end_states_on_queue();
	double compute_target_value( CRState &s );

	// we use these functions independent from the environment implementations
	// to be more variable in our own calculations
	double MinGCost( CRState &pos1, CRState &pos2 );

	GraphEnvironment *env;
	unsigned int number_of_cops;
	bool canPass;

	// Priority Queues
	MyPriorityQueue queue;

};


#endif
