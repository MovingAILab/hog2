#include "Unit.h"
#include "Map2DEnvironment.h"
#include "GraphEnvironment.h"
#include "MapAbstraction.h"
#include "PRAStar.h"

#ifndef PRASTARMAPUNIT_H
#define PRASTARMAPUNIT_H

/*!
	Provides a unit that does PRA* and follows the path
*/
template<class state,class action,class environment>
class PraStarMapUnit: public Unit<state,action,environment> {

	public:
	PraStarMapUnit( MapAbstraction *abs, state initial_state, unsigned int cop_speed );
	PraStarMapUnit( MapAbstraction *abs, state initial_state, unsigned int robberunit, unsigned int cop_speed );
	~PraStarMapUnit();

	// sets the unit number of the robber that we want to run to
	void SetRobberUnit( unsigned int _robberunit ) { robberunit = _robberunit; };

	// returns PraStarMapUnit
	const char *GetName();

	bool MakeMove( environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *sinfo, action &a );
	void UpdateLocation( environment *env, state &s, bool success, SimulationInfo<state,action,environment> *sinfo );
	void GetLocation( state &s ) { s = current_pos; };
	void OpenGLDraw( const environment *env, const SimulationInfo<state,action,environment>* ) const;
	void GetGoal( state &s ) { s = current_pos; };
	bool Done() { return done; };

	void SetColor( GLfloat _r, GLfloat _g, GLfloat _b ) {
		r = _r; g = _g; b = _b;
	};

	protected:

	// gives back kStay for MapEnvironment and (current_pos,current_pos) for GraphEnvironment
	action nomove();

	void GetPraStarPath( environment *env, state robber_pos );

	GLfloat r, g, b;
	MapAbstraction *abs;
	state current_pos;
	unsigned int robberunit, cop_speed;
	bool done;
	praStar *pra;

	private:

	std::vector<state> pathcache;
};


/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state,class action,class environment>
PraStarMapUnit<state,action,environment>::PraStarMapUnit( MapAbstraction *_abs, state initial_state, unsigned int _cop_speed ):
	abs(_abs), current_pos(initial_state), robberunit(0), cop_speed(_cop_speed),
	done(false), pra( new praStar() )
{ };

template<class state,class action,class environment>
PraStarMapUnit<state,action,environment>::PraStarMapUnit( MapAbstraction *_abs, state initial_state,
	unsigned int _copunit, unsigned int _cop_speed ):
	abs(_abs), current_pos(initial_state), robberunit(_copunit), cop_speed(_cop_speed),
	done(false), pra( new praStar() )
{ };

template<class state,class action,class environment>
PraStarMapUnit<state,action,environment>::~PraStarMapUnit() {
	delete pra;
};


template<class state,class action,class environment>
const char* PraStarMapUnit<state,action,environment>::GetName() {
	return "PraStarMapUnit";
};

template<class state,class action,class environment>
bool PraStarMapUnit<state,action,environment>::MakeMove( environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *sinfo, action &a ) {

	state robber_pos = sinfo->GetPublicUnitInfo( robberunit )->currentState;

	if( robber_pos == current_pos ) {
		done = true;
		a = nomove();
		return false;
	}

	if( pathcache.size() == 0 )
		GetPraStarPath( env, robber_pos );

	if( pathcache.size() > 0 ) {
		a = env->GetAction( current_pos, pathcache[0] );
		current_pos = pathcache[0]; // update my current position for me
		pathcache.erase( pathcache.begin() );

		if( robber_pos == current_pos ) done = true;
		return true;
	} else {
		a = nomove();
		return false;
	}
};

template<class state,class action,class environment>
void PraStarMapUnit<state,action,environment>::UpdateLocation( environment *, state &s, bool, SimulationInfo<state,action,environment> *sinfo ) {
	current_pos = s;
};


// \see PraStarMapUnit.cpp for implementation
template<>
void PraStarMapUnit<xyLoc,tDirection,MapEnvironment>::OpenGLDraw( const MapEnvironment *env, const SimulationInfo<xyLoc,tDirection,MapEnvironment>* ) const;
template<>
void PraStarMapUnit<xyLoc,tDirection,AbsMapEnvironment>::OpenGLDraw( const AbsMapEnvironment *env, const SimulationInfo<xyLoc,tDirection,AbsMapEnvironment>* ) const;
template<>
void PraStarMapUnit<graphState,graphMove,GraphEnvironment>::OpenGLDraw( const GraphEnvironment *env, const SimulationInfo<graphState,graphMove,GraphEnvironment>* ) const;
template<>
void PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment>::OpenGLDraw( const AbstractionGraphEnvironment *env, const SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* ) const;
template<>
tDirection PraStarMapUnit<xyLoc,tDirection,MapEnvironment>::nomove();
template<>
tDirection PraStarMapUnit<xyLoc,tDirection,AbsMapEnvironment>::nomove();
template<>
graphMove PraStarMapUnit<graphState,graphMove,GraphEnvironment>::nomove();
template<>
graphMove PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment>::nomove();
template<>
void PraStarMapUnit<xyLoc,tDirection,MapEnvironment>::GetPraStarPath( MapEnvironment *env, xyLoc robber_pos );
template<>
void PraStarMapUnit<xyLoc,tDirection,AbsMapEnvironment>::GetPraStarPath( AbsMapEnvironment *env, xyLoc robber_pos );
template<>
void PraStarMapUnit<graphState,graphMove,GraphEnvironment>::GetPraStarPath( GraphEnvironment *env, graphState robber_pos );
template<>
void PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment>::GetPraStarPath( AbstractionGraphEnvironment *env, graphState robber_pos );


#endif
