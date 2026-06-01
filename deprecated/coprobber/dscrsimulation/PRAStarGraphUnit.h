#include "Unit.h"
#include "GraphEnvironment.h"
#include "GraphAbstraction.h"
#include "PRAStar.h"

#ifndef PRASTARGRAPHUNIT_H
#define PRASTARGRAPHUNIT_H

/*!
	Provides a unit that does PRA* and follows the path
*/
template<class state,class action,class environment>
class PraStarGraphUnit: public Unit<state,action,environment> {

	public:
	PraStarGraphUnit( GraphAbstraction *abs, state initial_state, unsigned int cop_speed );
	PraStarGraphUnit( GraphAbstraction *abs, state initial_state, unsigned int robberunit, unsigned int cop_speed );
	~PraStarGraphUnit();

	// sets the unit number of the robber that we want to run to
	void SetRobberUnit( unsigned int _robberunit ) { robberunit = _robberunit; };

	// returns PraStarGraphUnit
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
	GraphAbstraction *abs;
	state current_pos;
	int robberunit;
	unsigned int cop_speed;
	bool done;
	praStar *pra;

	private:

	std::vector<state> pathcache;
};


/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/
template<class state,class action,class environment>
PraStarGraphUnit<state,action,environment>::PraStarGraphUnit( GraphAbstraction *_abs, state initial_state, unsigned int _cop_speed ):
	abs(_abs), current_pos(initial_state), robberunit(-1), cop_speed(_cop_speed),
	done(false), pra( new praStar() )
{ };

template<class state,class action,class environment>
PraStarGraphUnit<state,action,environment>::PraStarGraphUnit( GraphAbstraction *_abs, state initial_state,
	unsigned int _robberunit, unsigned int _cop_speed ):
	abs(_abs), current_pos(initial_state), robberunit(_robberunit), cop_speed(_cop_speed),
	done(false), pra( new praStar() )
{ };

template<class state,class action,class environment>
PraStarGraphUnit<state,action,environment>::~PraStarGraphUnit() {
	delete pra;
};


template<class state,class action,class environment>
const char* PraStarGraphUnit<state,action,environment>::GetName() {
	return "PraStarGraphUnit";
};

template<class state,class action,class environment>
bool PraStarGraphUnit<state,action,environment>::MakeMove( environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *sinfo, action &a ) {

	// if robber unit is not set yet
	if( robberunit < 0 ) {
		a = nomove();
		return false;
	}

	PublicUnitInfo<state,action,environment> pui;
	sinfo->GetPublicUnitInfo( robberunit, pui );
	state robber_pos = pui.currentState;

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
void PraStarGraphUnit<state,action,environment>::UpdateLocation( environment *, state &s, bool, SimulationInfo<state,action,environment> *sinfo ) {
	current_pos = s;
};


// \see PraStarGraphUnit.cpp for implementation
template<>
void PraStarGraphUnit<graphState,graphMove,GraphEnvironment>::OpenGLDraw( const GraphEnvironment *env, const SimulationInfo<graphState,graphMove,GraphEnvironment>* ) const;
template<>
void PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>::OpenGLDraw( const AbstractionGraphEnvironment *env, const SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* ) const;
template<>
graphMove PraStarGraphUnit<graphState,graphMove,GraphEnvironment>::nomove();
template<>
graphMove PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>::nomove();
template<>
void PraStarGraphUnit<graphState,graphMove,GraphEnvironment>::GetPraStarPath( GraphEnvironment *env, graphState robber_pos );
template<>
void PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>::GetPraStarPath( AbstractionGraphEnvironment *env, graphState robber_pos );


#endif
