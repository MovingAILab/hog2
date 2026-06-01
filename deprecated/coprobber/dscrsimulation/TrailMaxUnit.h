#include "Unit.h"
#include "Map2DEnvironment.h"
#include "GraphEnvironment.h"
#include "../DSTPDijkstra.h"

#ifndef TRAILMAXUNIT_H
#define TRAILMAXUNIT_H

/*!
	Provides a unit that simulates TrailMax(k)
*/
template<class state,class action,class environment>
class TrailMaxUnit: public Unit<state,action,environment> {

	public:
	TrailMaxUnit( state initial_state, unsigned int k, unsigned int cop_speed );
	TrailMaxUnit( state initial_state, unsigned int k, unsigned int copunit, unsigned int cop_speed );
	//~TrailMaxUnit();

	// sets the unit number of the cop we want to run away from
	void SetCopUnit( unsigned int _copunit ) { copunit = _copunit; };

	// returns TrailMaxUnit(k)
	const char *GetName();

	bool MakeMove( environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *sinfo, action &a );
	void UpdateLocation( environment *env, state &s, bool success, SimulationInfo<state,action,environment> *sinfo );
	void GetLocation( state &s ) { s = current_pos; };
	void OpenGLDraw( int window, environment *env, SimulationInfo<state,action,environment>* );
	void GetGoal( state &s ) { s = current_pos; };
	bool Done() { return done; };

	void SetColor( GLfloat _r, GLfloat _g, GLfloat _b ) {
		r = _r; g = _g; b = _b;
	};

	protected:

	GLfloat r, g, b;
	state current_pos;
	unsigned int k, copunit, cop_speed;
	bool done;

	private:

	std::vector<state> pathcache;
};


/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state,class action,class environment>
TrailMaxUnit<state,action,environment>::TrailMaxUnit( state initial_state, unsigned int _k, unsigned int _cop_speed ):
	current_pos(initial_state), k(_k), copunit(0), cop_speed(_cop_speed), done(false)
{ };

template<class state,class action,class environment>
TrailMaxUnit<state,action,environment>::TrailMaxUnit( state initial_state, unsigned int _k, unsigned int _copunit, unsigned int _cop_speed ):
	current_pos(initial_state), k(_k), copunit(_copunit), cop_speed(_cop_speed), done(false)
{ };


template<class state,class action,class environment>
const char* TrailMaxUnit<state,action,environment>::GetName() {
	char *str; str = (char*) malloc( sizeof(char) * 20 );
	sprintf( str, "TrailMaxUnit(%u)", k );
	return str;
};

template<class state,class action,class environment>
bool TrailMaxUnit<state,action,environment>::MakeMove( environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *sinfo, action &a ) {

	// if the cop that we have to run away from is not set yet
	if( copunit == 0 || done ) {
		a = env->GetAction( current_pos, current_pos );
		return false;
	}

	// if we have no more move in the cache, then recompute
	if( pathcache.size() == 0 ) {
		// generate new TrailMax object
		DSTPDijkstra<state,action> *dstp = new DSTPDijkstra<state,action>( env, cop_speed );
		// get the position of the cop
		state cop_pos = sinfo->GetPublicUnitInfo( copunit )->currentState;
		// run TrailMax
		dstp->dstpdijkstra( current_pos, cop_pos, false, pathcache );
		// truncate pathcache to the maximum size of k successors
		if( pathcache.size() > k ) pathcache.resize( k );
		// cleanup
		delete dstp;
	}

	if( pathcache.size() > 0 ) {
		a = env->GetAction( current_pos, pathcache[0] );
		pathcache.erase( pathcache.begin() );
		return true;
	} else {
		a = env->GetAction( current_pos, current_pos );
		return false;
	}
};

template<class state,class action,class environment>
void TrailMaxUnit<state,action,environment>::UpdateLocation( environment *, state &s, bool, SimulationInfo<state,action,environment> *sinfo ) {
	current_pos = s;
	if( copunit > 0 && sinfo->GetPublicUnitInfo( copunit )->currentState == current_pos )
		done = true;
};


// \see TrailMaxUnit.cpp for implementation
template<>
void TrailMaxUnit<xyLoc,tDirection,MapEnvironment>::OpenGLDraw( int window, MapEnvironment *env, SimulationInfo<xyLoc,tDirection,MapEnvironment>* );
template<>
void TrailMaxUnit<xyLoc,tDirection,AbsMapEnvironment>::OpenGLDraw( int window, AbsMapEnvironment *env, SimulationInfo<xyLoc,tDirection,AbsMapEnvironment>* );
template<>
void TrailMaxUnit<graphState,graphMove,GraphEnvironment>::OpenGLDraw( int window, GraphEnvironment *env, SimulationInfo<graphState,graphMove,GraphEnvironment>* );
template<>
void TrailMaxUnit<graphState,graphMove,AbstractionGraphEnvironment>::OpenGLDraw( int window, AbstractionGraphEnvironment *env, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* );


#endif
