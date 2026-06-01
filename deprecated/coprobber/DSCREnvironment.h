/*------------------------------------------------------------------------------
| Different Speed Cop Robber Environment
|
| Alternative implementation of the cop robber game
| with a possibly faster cop
------------------------------------------------------------------------------*/

#ifndef DSCRENVIRONMENT_H
#define DSCRENVIRONMENT_H

#include <vector>
#include <queue>
#include <ext/hash_set>
#include "MultiAgentEnvironment.h"
#include "MyHash.h"

/*
	A one cop and one robber implementation, alternating action game.
*/
template<class state, class action>
class DSCREnvironment {

	public:

	// types
	typedef typename MultiAgentEnvironment<state,action>::MAState CRState;

	// constructor and destructor
	DSCREnvironment(
		SearchEnvironment<state,action> *env,
		bool playerscanpass = true, unsigned int cop_speed = 1 );
	virtual ~DSCREnvironment();

	// set the playerscanpass variable
	virtual void SetPlayersCanPass( bool _playerscanpass ) { playerscanpass = _playerscanpass; };
	virtual bool GetPlayersCanpass() { return playerscanpass; };

	// speed management
	virtual void SetCopSpeed( unsigned int speed ) { cop_speed = speed; };
	virtual unsigned int GetCopSpeed() { return cop_speed; };

	// move generation
	virtual void GetRobberSuccessors( CRState &s, std::vector<state> &neighbors, bool take_cop_into_account = true );
	virtual void GetRobberSuccessors( state &s, std::vector<state> &neighbors );
	// if radius == 0 then it takes the currently set cop_speed as radius
	virtual void GetCopSuccessors( CRState &s, std::vector<state> &neighbors, unsigned int radius = 0 );
	virtual void GetCopSuccessors( state &s,   std::vector<state> &neighbors, unsigned int radius = 0 );
	// gives back the real environmental gcosts
	// Warning: copgcost and robbergcost return 1. which is different from this
	virtual void GetCopSuccessors( state &s,   std::vector<state> &neighbors, std::vector<float> &gcosts, unsigned int radius = 0 );

	// termination criteria
	virtual bool GoalTest(const  CRState &s );
	virtual bool GoalTest(const  state &s1, const state &s2 );
	virtual double TerminalCost( CRState &s );
	virtual double TerminalCost( state &s1, state &s2 );

	// transition costs
	virtual double RobberGCost( state &s, state &n );
	virtual double RobberGCost( CRState &s, state &n ) {return RobberGCost(s[1],n);};
	virtual double RobberGCost( CRState &s, CRState &n ) {return RobberGCost(s[0], n[0]);};
	virtual double CopGCost( state &s, state &n );
	virtual double CopGCost( CRState &s, state &n ) {return CopGCost(s[1],n);};
	virtual double CopGCost( CRState &s, CRState &n ) {return CopGCost(s[1],n[1]);};

	// heuristc distance between two states in the graph/map
	// assumes, that all transition costs are 1
	// note: if used with graphs, use MaximumNormGraphMapHeuristic within the GraphEnvironment
	// if used with maps, we implemented a version ourself
	virtual double HCost(const state &s1, const state &s2 );

	virtual double AccumulatedHCost( CRState &s, bool minFirst ) {
		return AccumulatedHCost( s[0], s[1], minFirst );
	};
	virtual double AccumulatedHCost( state &s1, state &s2, bool minFirst ) {
		if( minFirst ) return( 2.*HCost(s1,s2)-1. );
		else return( 2.*HCost(s1,s2) );
	};

	protected:

	// dijkstra algorithm for successor generation for the cops
	// note that we should actually give back trajectories of the cops and then
	// detect collision in the cops trajectories...
	virtual void Dijkstra( state &s, unsigned int &steps, std::vector<state> &neighbors, std::vector<float> &gcosts );

	// variables
	SearchEnvironment<state,action> *env;
	bool playerscanpass;
	unsigned int cop_speed;

	private:
	struct MyStateHash {
		size_t operator() ( const state &s ) const {
			return StateHash<state>( s );
		}
	};

	class QueueEntry {
		public:
		QueueEntry() {};
		QueueEntry( state _pos, unsigned int _distance, float _gcost ):pos(_pos),distance(_distance),gcost(_gcost) {};
		state pos;
		unsigned int distance;
		float gcost;
	};

	struct QueueEntryCompare {
		bool operator() ( const QueueEntry &q1, const QueueEntry &q2 ) const {
			if( fequal( q1.gcost, q2.gcost ) )
				return( q1.distance > q2.distance );
			else
				return( q1.gcost > q2.gcost );
		}
	};

	typedef std::priority_queue< QueueEntry, std::vector<QueueEntry>, QueueEntryCompare > OpenList;
	typedef __gnu_cxx::hash_set< state, MyStateHash > ClosedList;
	OpenList openlist;
	ClosedList closedlist;

};

/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state, class action>
DSCREnvironment<state,action>::DSCREnvironment( SearchEnvironment<state,action> *_env, bool _playerscanpass, unsigned int _cop_speed ):
	env(_env),playerscanpass(_playerscanpass),cop_speed(_cop_speed)
{
};

template<class state, class action>
DSCREnvironment<state,action>::~DSCREnvironment() {};

template<class state, class action>
void DSCREnvironment<state,action>::GetRobberSuccessors( CRState &s, std::vector<state> &neighbors, bool take_cop_into_account ) {
	neighbors.clear();
	if( take_cop_into_account ) {
		if( s[0] == s[1] ) return; // if the cop is on the robbers position, the robber cannot move
	}
	// otherwise generate all successors for s
	GetRobberSuccessors( s[0], neighbors );
	return;
};

template<class state, class action>
void DSCREnvironment<state,action>::GetRobberSuccessors( state &s, std::vector<state> &neighbors ) {
	neighbors.clear();
	env->GetSuccessors( s, neighbors );
	if( playerscanpass ) neighbors.push_back( s );
	return;
};

template<class state, class action>
void DSCREnvironment<state,action>::GetCopSuccessors( CRState &s, std::vector<state> &neighbors, unsigned int radius ) {
	GetCopSuccessors( s[1], neighbors, radius );
	return;
};

template<class state, class action>
void DSCREnvironment<state,action>::GetCopSuccessors( state &s, std::vector<state> &neighbors, unsigned int radius ) {
	std::vector<float> gcosts;
	if( radius == 0 ) radius = cop_speed;
	Dijkstra( s, radius, neighbors, gcosts );
	// we reverse the order to expand nodes first where the cop is moving full speed and the "stay" move last
	std::reverse( neighbors.begin(), neighbors.end() );
	return;
};

template<class state, class action>
void DSCREnvironment<state,action>::GetCopSuccessors( state &s, std::vector<state> &neighbors, std::vector<float> &gcosts, unsigned int radius ) {
	if( radius == 0 ) radius = cop_speed;
	Dijkstra( s, radius, neighbors, gcosts );
	// we reverse the order to expand nodes first where the cop is moving full speed and the "stay" move last
	std::reverse( neighbors.begin(), neighbors.end() );
	std::reverse( gcosts.begin(), gcosts.end() );
	return;
};

template<class state, class action>
bool DSCREnvironment<state,action>::GoalTest( CRState &s ) {
	return( s[0] == s[1] );
};

template<class state, class action>
bool DSCREnvironment<state,action>::GoalTest( state &s1, state &s2 ) {
	return( s1 == s2 );
};

template<class state, class action>
double DSCREnvironment<state,action>::TerminalCost( CRState& ) {
	return 0.;
};

template<class state, class action>
double DSCREnvironment<state,action>::TerminalCost( state &s1, state &s2 ) {
	return 0.;
};

template<class state, class action>
double DSCREnvironment<state,action>::RobberGCost( state&, state& ) {
	return 1.;
};

template<class state, class action>
double DSCREnvironment<state,action>::CopGCost( state&, state& ) {
	return 1.;
};

// \see DSCREnvironment.h
template<>
double DSCREnvironment<xyLoc,tDirection>::HCost(const xyLoc &s1, const xyLoc &s2 );
template<>
double DSCREnvironment<graphState,graphMove>::HCost(const graphState &s1, const graphState &s2 );

template<class state, class action>
void DSCREnvironment<state,action>::Dijkstra( state &s, unsigned int &steps, std::vector<state> &neighbors, std::vector<float> &gcosts ) {
	neighbors.clear();
	openlist.push( QueueEntry( s, 0, 0. ) );

	std::vector<state> tempneighbors;
	typename std::vector<state>::iterator it;
	QueueEntry qe;
	while( !openlist.empty() ) {

		qe = openlist.top();
		openlist.pop();

		if( closedlist.find( qe.pos ) == closedlist.end() ) {
			if( playerscanpass || !(qe.pos == s) ) {
				neighbors.push_back( qe.pos );
				gcosts.push_back( qe.gcost );
			}
			closedlist.insert( qe.pos );

			// if there can be neighbors from this node
			if( qe.distance < steps ) {
				env->GetSuccessors( qe.pos, tempneighbors );
				// for all neighbors
				for( it = tempneighbors.begin(); it != tempneighbors.end(); it++ ) {
					state new_state = *it;
					if( closedlist.find( new_state ) == closedlist.end() ) {
						// if not yet in list push it on the open queue
						openlist.push( QueueEntry( new_state, qe.distance + 1, qe.gcost + env->GCost(qe.pos, new_state) ) );
					}
				}
				tempneighbors.clear();
			}
		}
	}

	// cleanup
	// openlist = OpenList(); // shouldn't be neccessary
	closedlist.clear();

	return;
}

#endif
