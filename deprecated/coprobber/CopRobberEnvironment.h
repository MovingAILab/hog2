#ifndef COPROBBERENVIRONMENT_H
#define COPROBBERENVIRONMENT_H

#include <stdint.h>
#include <math.h>
#include <vector>
#include "MultiAgentEnvironment.h"
#include "OccupancyInterface.h"
#include "MyHash.h"

/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
| Definitions
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
| Occupancy Interface
------------------------------------------------------------------------------*/
/*!
	CopRobberOccupancy can be used for playing the cop and robber game
	on a planar graph where nodes are connected by single bidirected edges only.
*/
// Make sure the state has an operator == defined!
template<class state, class action>
class CopRobberOccupancy: public MultiAgentOccupancy<state, action> {

	public:

	typedef typename MultiAgentOccupancy<state,action>::MAState CRState;

	CopRobberOccupancy() {};
	~CopRobberOccupancy();

	// please avoid using the next three functions as they are not intended
	// to be used extensively! They are just there to fit the class definitions
	void SetStateOccupied( CRState &s, bool o );
	bool GetStateOccupied( CRState &s );
	void MoveUnitOccupancy( CRState &s1, CRState &s2 );

	// this is the main function of this class!
	bool CanMove( CRState &s1, CRState &s2 );

	protected:
	std::vector<CRState> states;
};

/*------------------------------------------------------------------------------
| Environment
------------------------------------------------------------------------------*/
/*!
	Models the game of n cops vs. one robber on
	a planar graph where nodes are connected by single bidirected edges only.

	Players are indexed from 0 to n where 0 is the robber and 1\ldots n
	are the cops.
	During the game cops can be added and deleted as you wish, simply omit them
	from the current state. There is no differentiation between the cops.
*/
template<class state, class action>
class CopRobberEnvironment : public virtual MultiAgentEnvironment<state,action> {

	public:

	typedef typename MultiAgentEnvironment<state, action>::MAState CRState;
	typedef typename MultiAgentEnvironment<state, action>::SAAction CRAction; // single player action
	typedef typename MultiAgentEnvironment<state, action>::MAMove CRMove;

	/*!
		\param playerscanpass switch whether the submitted search environment
		returns actions that do not alter the current state or not.
		In case it does set playerscanpass to false otherwise to true.
	*/
	CopRobberEnvironment( SearchEnvironment<state,action> *env, bool playerscanpass = false );
	virtual ~CopRobberEnvironment();

	virtual void GetSuccessors(const  CRState &nodeID, std::vector<CRState> &neighbors ) const;
	virtual void GetActions(const  CRState &nodeID, std::vector<CRMove> &actions ) const;

	virtual CRMove GetAction(CRState &s1, CRState &s2) const;
	virtual void ApplyAction(CRState &s, CRMove a) const;
  
//	virtual void GetNextState(CRState &currents, action dir, CRState &news){};

	virtual bool InvertAction(CRMove &a) const;

	virtual double HCost(const CRState &node1, const CRState &node2);
	virtual double GCost(const CRState &node1, const CRState &node2);
	virtual double GCost(const CRState &node, const CRMove &act);
	virtual bool GoalTest(const CRState &node, const CRState& );
	virtual bool GoalTest(const CRState &node);

	// I would recommend testing these functions extensively before using them
	// because they won't work well with a lot of agents!
	virtual uint64_t GetStateHash(const CRState &node) const;
	virtual uint64_t GetActionHash(CRMove act) const;

	virtual OccupancyInterface<CRState,CRMove> *GetOccupancyInfo() { return oi; };

	virtual void OpenGLDraw() const {};
	virtual void OpenGLDraw(const CRState&) const {};
	virtual void OpenGLDraw(const CRState&, const CRMove&) const {};


	// additional stuff for the new SearchEnvironment interface
	virtual void StoreGoal(CRState &node) {};
	virtual void ClearGoal() {};
	virtual bool IsGoalStored() { return false; };
	virtual double HCost(const CRState &node1) { fprintf( stderr, "ERROR: HCost of one state is not implemented!\n" ); exit(1); return 0.; };


	protected:
	bool playerscanpass;
	SearchEnvironment<state,action> *env;
	CopRobberOccupancy<state,action> *oi;

};








/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
| Implementation
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
| Occupancy
------------------------------------------------------------------------------*/
template<class state, class action>
CopRobberOccupancy<state,action>::~CopRobberOccupancy() {
	states.clear();
};

// horribly unperformant, hopefully nobody is going to use this code extensively
template<class state, class action>
void CopRobberOccupancy<state,action>::SetStateOccupied( CRState &s, bool o ) {
	for( typename std::vector<CRState>::iterator it = states.begin(); it != states.end(); it++ ) {
		if( *it == s ) {
			if( !o ) states.erase( it );
			return;
		}
	}
	if( o )
		states.push_back( s );
	return;
};

template<class state, class action>
bool CopRobberOccupancy<state,action>::GetStateOccupied( CRState &s ) {
	for( typename std::vector<CRState>::iterator it = states.begin(); it != states.end(); it++ ) {
		if( *it == s ) return true;
	}
	return false;
}

template<class state, class action>
void CopRobberOccupancy<state,action>::MoveUnitOccupancy( CRState &s1, CRState &s2 ) {
	SetStateOccupied( s1, false );
	states.push_back( s2 );
}

template<class state, class action>
bool CopRobberOccupancy<state,action>::CanMove( CRState &s1, CRState &s2 ) {
	assert( s1.size() == s2.size() );
	assert( s1.size() > 1 );
	// does the robber change its position?
	bool robber_moves = !( s1[0] == s2[0] );
	// we start from 1 because 0 is the robber,
	// thus we check for the cops
	for( unsigned int i = 1; i < s1.size(); i++ ) {
		// check whether the robber attempts to change position while a cop
		// stays seated on him
		if( robber_moves && s1[0] == s1[i] && s1[i] == s2[i] )
			return false;

		// the robber cannot trade places with a cop (but he is of course allowed
		// to run into one)
		if( robber_moves && s2[0] == s1[i] && s2[i] == s1[0] )
			return false;

		for( unsigned int j = 1; j < i; j++ ) {
			// check that no two cops are moving to the same state
			// TODO: Change me back!!!
//			if( s2[i] == s2[j] ) return false;

			// two cops change position, which means they just collidated by
			// walking over each other
			if( s2[i] == s1[j] && s2[j] == s1[i] ) return false;

		}
	}
	return true;
}


/*------------------------------------------------------------------------------
| Environment
------------------------------------------------------------------------------*/
template<class state, class action>
CopRobberEnvironment<state,action>::CopRobberEnvironment( SearchEnvironment<state,action> *_env, bool _playerscanpass ):
	playerscanpass(_playerscanpass), env(_env)
{
	oi = new CopRobberOccupancy<state,action>();
}

template<class state, class action>
CopRobberEnvironment<state,action>::~CopRobberEnvironment() {
	delete oi;
	// delete env; ???
}

template<class state, class action>
void CopRobberEnvironment<state,action>::GetSuccessors(const  CRState &nodeID, std::vector<CRState> &neighbors ) const {

	unsigned int num_players = nodeID.size();

	unsigned long i, j;
  unsigned int k;
  CRState p_states[num_players];
  // as=agent states, n__=number of
  unsigned int nas[num_players], as[num_players];
  // t___=total
  unsigned long tnas = 0;

  neighbors.clear();

  // get all the successor states for each agent/player
  for( i = 0; i < num_players; i++ ) {
    env->GetSuccessors( nodeID[i], p_states[i] );
    if( playerscanpass ) {
       p_states[i].push_back( nodeID[i] );
    }
    nas[i] = p_states[i].size();
		if( tnas ) tnas *= nas[i];
    else tnas = nas[i];
  }

  j = 0;
  for( i = 0; i < tnas; i++ ) {
    dehash_permutation( i, num_players, nas, as );

    neighbors.push_back( CRState() );

    for( k = 0; k < num_players; k++ ) {
      neighbors[j].push_back( p_states[k][as[k]] );
		}

		// test whether we can move this way or not
		if( !oi->CanMove( nodeID, neighbors[j] ) )
      neighbors.pop_back();
		else
      j++;
  }

  // cleanup
  for( i = 0; i < num_players; i++ )
    p_states[i].clear();

  return;
}

template<class state, class action>
void CopRobberEnvironment<state,action>::GetActions(const  CRState &nodeID, std::vector<CRMove> &actions ) const {

	unsigned int num_players = nodeID.size();

	unsigned long i, j;
  unsigned int k;
  std::vector<action> p_actions[num_players];
  CRState p_state;
  state s;
  // as=agent actions, n__=number of
  unsigned int naa[num_players], aa[num_players];
  // t___=total
  unsigned long tnaa = 0;

  actions.clear();

  // get the actions of each individual agent
  for( i = 0; i < num_players; i++ ) {
    env->GetActions( nodeID[i], p_actions[i] );
    // if an agent can pass his turn then we augment all the numbers
    // by one and 0 becomes the pass action
    naa[i] = p_actions[i].size() + (playerscanpass?1:0);
    if( tnaa ) tnaa *= naa[i];
    else tnaa = naa[i];
  }

  j = 0;
  for( i = 0; i < tnaa; i++ ) {
    dehash_permutation( i, num_players, naa, aa );
		actions.push_back( CRMove() );
    p_state.clear();

    for( k = 0; k < num_players; k++ ) {
      if( playerscanpass && aa[k] == 0 ) {
        actions[j].push_back( CRAction() );
        p_state.push_back( nodeID[k] );
      } else {
        actions[j].push_back( CRAction(p_actions[k][aa[k]-(playerscanpass?1:0)]) );
        // apply the action
        s = nodeID[k];
        env->ApplyAction( s, actions[j][k].a );
        p_state.push_back( s );
      }
    }

		if( !oi->CanMove( nodeID, p_state ) )
      actions.pop_back();
    else
      j++;
  }

	// cleanup
  for( i = 0; i < num_players; i++ )
    p_actions[i].clear();

  return;

}

// I'd like to be able to do something here like
// CRMove CopRobberEnvironment<state,action>::GetAction(CRState &s1, CRState &s2 );
template<class state, class action>
typename CopRobberEnvironment<state,action>::CRMove CopRobberEnvironment<state,action>::GetAction(CRState &s1, CRState &s2) const {
	CRMove result;
	// safety statement
	assert( s1.size() == s2.size() );

	for( unsigned int i = 0; i < s1.size(); i++ ) {
		if( s1[i] == s2[i] )
			result.push_back( CRAction() );
		else
			result.push_back( env->GetAction( s1[i], s2[i] ) );
	}
	return result;
}

template<class state, class action>
void CopRobberEnvironment<state,action>::ApplyAction(CRState &s, CRMove a) const {
	assert( s.size() == a.size() );
	for( unsigned int i = 0; i < s.size(); i++ ) {
		if( ! a[i].noaction ) {
			env->ApplyAction( s[i], a[i].a );
		}
	}
	return;
}

template<class state, class action>
bool CopRobberEnvironment<state,action>::InvertAction(CRMove &a) const {
	bool invertible;
	CRMove old = a;

	for( typename CRMove::iterator it = a.begin(); it != a.end(); it++ ) {
		if( ! it->noaction ) {
			invertible = env->InvertAction( it->a );
			if( !invertible ) {
				a = old;
				return false;
			}
		}
	}
	return true;
}

// accumulated HCost of all the state differences
template<class state, class action>
double CopRobberEnvironment<state,action>::HCost(const CRState &node1, const CRState &node2) {
	double h = 0.;
	assert( node1.size() == node2.size() );
	for( unsigned int i = 0; i < node1.size(); i++ ) {
		h += env->HCost( node1[i], node2[i] );
	}
	return h;
}

// accumulated GCost of all the state differences
template<class state, class action>
double CopRobberEnvironment<state,action>::GCost(const CRState &node1, const CRState &node2) {
	double g = 0.;
	assert( node1.size() == node2.size() );
	for( unsigned int i = 0; i < node1.size(); i++ ) {
		g += env->GCost( node1[i], node2[i] );
	}
	return g;
}

template<class state, class action>
double CopRobberEnvironment<state,action>::GCost(const CRState &node, const CRMove &act) {
	double g = 0.;
	assert( node.size() == act.size() );
	for( unsigned int i = 0; i < node.size(); i++ ) {
		if( !act[i].noaction ) {
			g += env->GCost( node[i], act[i].a );
		}
	}
	return g;
}

// test's whether the robber has been caught or not
template<class state, class action>
bool CopRobberEnvironment<state,action>::GoalTest(CRState &node, CRState& ) {
	return GoalTest( node );
};

template<class state, class action>
bool CopRobberEnvironment<state,action>::GoalTest(CRState &node) {
	for( unsigned int i = 1; i < node.size(); i++ ) {
		if( node[0] == node[i] ) return true;
	}
	return false;
}

// we'll try to hash as good as possible...
// this will not work very well with a lot of agents
// and is buggy with more than 64 agents
template<class state, class action>
uint64_t CopRobberEnvironment<state,action>::GetStateHash(const CRState &node) const {
	uint64_t hash = 0, t;
	double psize = 64./(double)node.size();
	double count = 0.;
	unsigned int i;
	unsigned char j;

	for( i = 0; i < node.size(); i++ ) {
		t = env->GetStateHash( node[i] );
		for( j = 0; j < floor(count+psize)-floor(count); j++ ) {
			hash |= (t & (1<<j))<<(unsigned int)floor(count);
		}
		count += psize;
	}
	return hash;
}

template<class state, class action>
uint64_t CopRobberEnvironment<state,action>::GetActionHash(CRMove act) const {
	uint64_t hash = 0, t;
	double psize = 64./(double)act.size();
	double count = 0.;
	unsigned int i;
	unsigned char j;

	for( i = 0; i < act.size(); i++ ) {
		if( !act[i].noaction ) {
			t = env->GetActionHash( act[i].a );
			for( j = 0; j < floor(count+psize)-floor(count); j++ ) {
				hash |= (t & (1<<j))<<(unsigned int)floor(count);
			}
		}
		count += psize;
	}
	return hash;
}



#endif
