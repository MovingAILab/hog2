#ifndef DSCOVERTWO_H
#define DSCOVERTWO_H

#include <vector>
#include <queue>
#include "MyHash.h"
#include "DSCREnvironment.h"
#include "DSRobberAlgorithm.h"

/*
	Alejandro Isaza's code of Cover Heuristic
	adopted to our different speed system

	note: this implementation only supports one cop since it works with the DS interface
	      and hence all multi-cops code has been discarded
*/
template<class state, class action>
class DSCover2: public DSRobberAlgorithm<state,action> {

	public:

	// constructor & destructor
	DSCover2( SearchEnvironment<state,action> *env, unsigned int num_states, unsigned int cop_speed = 1 );
	~DSCover2();


	// return the number of covered and uncovered states
	unsigned int numCovered()   { return m_num_covered; };
	unsigned int numUncovered() { return m_num_uncovered; };

	double value( state &r, state &c, bool minFirst = false, bool time_minFirst = true, double time = 0. );

	state MakeMove( state pos_robber, state pos_cop, bool minFirst = true, double time = 0. );

	state MakeMove( state pos_robber, state pos_cop, unsigned int ) {
		return MakeMove( pos_robber, pos_cop, false, 0. );
	};

	unsigned int nodesExpanded, nodesTouched;

	protected:

	// all the states in the environment will be labeled
	enum CoverState { UNCOVERED, UNKNOWN, COVERED };


	// minFirst determines for who we are calculating the cover and when he is to move
	// we're making the assumption that his opponent starts moving at time 0.0
	void calculateCover( state &r, state &c, bool minFirst, double time );


	// a node for the cover calculation
	struct Node {
		state s;
		double time;
		Node() {};
		Node(state &_s, double _time): s(_s),time(_time) {};
	};

	struct NodeCompare {
		bool operator() ( const Node &n1, const Node &n2 ) const {
			return( n1.time > n2.time );
		}
	};

	typedef std::priority_queue<Node,std::deque<Node>,NodeCompare> MyPriorityQueue;

	// for each state in the environment we know whether it is covered or not
	// (the number of a state is determined by StateHash<state>( s )) (MyHash.h)
	std::vector<CoverState> m_cover;

	// number of covered states
	unsigned int m_num_covered;
	// number of uncovered states
	unsigned int m_num_uncovered;

	// environment variables
	DSCREnvironment<state,action> *dscrenv;
	unsigned int num_states;
	unsigned int cop_speed;

};


/*------------------------------------------------------------------------------
| IMPLEMENTATION
------------------------------------------------------------------------------*/

template<class state,class action>
DSCover2<state,action>::DSCover2( SearchEnvironment<state,action> *env,
                                unsigned int _num_states, unsigned int _cop_speed )
	: dscrenv( new DSCREnvironment<state,action>( env, false, 1 ) ),
	  num_states(_num_states),
	  cop_speed(_cop_speed)
{};

template<class state,class action>
DSCover2<state,action>::~DSCover2() {
	delete dscrenv;
};

template<class state, class action>
double DSCover2<state,action>::value( state &r, state &c, bool minFirst, bool time_minFirst, double time ) {
	// it does not make sense to submit anything below -1.0 or above 1.0
	if( (time < -1. || time > 0.) ) {
		fprintf( stderr, "ERROR: submitted time for cover2 is not supported.\n" );
		exit(1);
	}

	// calculate the cover
	calculateCover( r, c, time_minFirst, time );

	double cover_percent = double(numCovered()) / double(numCovered() + numUncovered());
	// the value shall be between -1 and 1
	double value = 2.0 * cover_percent - 1.0;
	if( value < -1.0 || value > 1.0 ) {
		printf( "ERROR: cover value outside of valid range\n" );
		exit(1);
	}
	if( minFirst )
		return value;
	else
		return -value;
};



template<class state, class action>
void DSCover2<state,action>::calculateCover( state &pos_robber, state &pos_cop, bool minFirst, double mytime ) {

	m_cover.clear();
	m_cover.resize( num_states, UNKNOWN );
	m_num_covered   = 0;
	m_num_uncovered = 0;

	MyPriorityQueue cop_queue, robber_queue;

	// last_move_end = 0.;

	// push both the robber and the cop onto the queues
	Node rnode( pos_robber, 0. );
	Node cnode( pos_cop, 0. );
	if( minFirst ) cnode.time = mytime;
	else           rnode.time = mytime;
	robber_queue.push( rnode );
	cop_queue.push( cnode );
	nodesTouched += 2;

	// as long as there are states to be expanded (by any of the two players)
	while( !robber_queue.empty() || !cop_queue.empty() ) {

		// pop a state from the queues such that the one with smaller time is taken
		// when two states have the same time, the cop goes first
		Node node;
		bool node_minFirst;
		if( robber_queue.empty() ) {
			node_minFirst = true;
			node = cop_queue.top();
			cop_queue.pop();
		}
		else if( cop_queue.empty() ) {
			node_minFirst = false;
			node = robber_queue.top();
			robber_queue.pop();
		} else {
			rnode = robber_queue.top();
			cnode = cop_queue.top();
			if( cnode.time <= rnode.time ) {
				node_minFirst = true;
				node = cnode;
				cop_queue.pop();
			} else {
				node_minFirst = false;
				node = rnode;
				robber_queue.pop();
			}
		}
		nodesTouched++;

		size_t h = StateHash<state>(node.s);
		if( m_cover[h] != UNKNOWN  && (!node_minFirst || node.time > 0.) )
			continue;

		if( m_cover[h] == UNKNOWN ) {
			if( !node_minFirst ) {
				m_cover[h] = UNCOVERED;
				m_num_uncovered += 1;
			} else {
				m_cover[h] = COVERED;
				m_num_covered += 1;
			}
		}

		std::vector<state> neighbors;
		if( node_minFirst )
			dscrenv->GetCopSuccessors( node.s, neighbors );
		else
			dscrenv->GetRobberSuccessors( node.s, neighbors );
		nodesExpanded++;

		double backup_time = node.time;

		for( typename std::vector<state>::iterator it = neighbors.begin();
		     it != neighbors.end(); it++ ) {

			nodesTouched++;
			// update values of the child
			node.s = *it;
			if( node_minFirst ) node.time = backup_time + 1./(double)cop_speed;
			else                node.time = backup_time + 1.;

			if( m_cover[StateHash<state>(node.s)] == UNKNOWN ) {
				if( node_minFirst )
					cop_queue.push( node );
				else
					robber_queue.push( node );
			}
		}

	}

	return;
};







template<class state,class action>
state DSCover2<state,action>::MakeMove( state pos_robber, state pos_cop, bool minFirst, double time ) {

	// enforce -1 <= time <= 0
	if( (time < -1. || time > 0.) ) {
		fprintf( stderr, "ERROR: submitted time for cover2 is not supported.\n" );
		exit(1);
	}

	if( time == 0. ) time = -1.; // see below on why

	if( minFirst ) {
		fprintf( stderr, "ERROR: cover2 is not yet supported for the cop.\n" );
		exit(1);
	};

	nodesExpanded = 0; nodesTouched = 0;

	// the next moves
	std::vector<state> neighbors;
	// dscrenv has been initialized with playerscanpass=false and cop_speed=1 for our computations
	dscrenv->SetPlayersCanPass( true );
	dscrenv->GetRobberSuccessors( pos_robber, neighbors );
	// reset the playerscanpass directive and cop speed in dscrenv
	dscrenv->SetPlayersCanPass( false );

	// just to make sure that we're not in an isolated node
	assert( neighbors.size() > 0 );

	// max_cover is the value of max_cover_state
	double max_cover = -DBL_MAX, temp;
	state max_cover_state;

	max_cover_state = neighbors[0];

	// for each successor state compute the cover
	for( typename std::vector<state>::iterator it = neighbors.begin(); it != neighbors.end(); it++ ) {
		temp = value( *it, pos_cop, false, true, -(time+1) );

		if( temp > max_cover ) {
			max_cover = temp;
			max_cover_state = *it;
		}
	}

	return max_cover_state;
};


#endif
