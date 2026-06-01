
// Most of this code has been copied from CRSimulation.h
// note: this is customization to DSCREnvironment

#ifndef DSCRSIMULATION_H
#define DSCRSIMULATION_H

#include <vector>
#include <queue>
#include "Unit.h"
#include "Timer.h"
#include "FPUtil.h"
#include "StatCollection.h"
#include "SearchEnvironment.h"
#include "OccupancyInterface.h"
#include "SimulationInfo.h"
#include "../DSCREnvironment.h"


template <class state, class action, class environment>
class DSCRUnitInfo {
public:
	DSCRUnitInfo() {};
	Unit<state, action, environment> *agent;
	action lastMove;
	state startState;
	state currentState;
	state lastState;
	double nextTime;
	double lastTime;
	double totalDistance;
	double totalThinking;
};


/**
 * A simulation of the game on cops and robber
 * Note: Please add a robber before you use anything else of the simulation (this simulation
 * is only designed for one robber)
 */
template<class state, class action, class environment>
class DSCRSimulation: public SimulationInfo<state,action,environment> {
public:

	typedef typename DSCREnvironment<state,action>::CRState CRState;

	/*! creates the simulation by specifying the environment */
	DSCRSimulation(environment *env, bool playerscanpause = false, unsigned int cop_speed = 1, bool simultaneous = false );
	~DSCRSimulation();

	/*! adds the robber to the simulation */
	unsigned int AddRobber( Unit<state,action,environment> *unit, double timeOffset = 0. );
	/*! add cops to the simulation */
	unsigned int AddCop( Unit<state, action, environment> *unit, double timeOffset = 0. );

	unsigned int GetNumUnits() const;
	unsigned int GetNumUnitGroups() const;

	/*!
		Use this functionality if you want to have the copgroup make moves and such.
		This can be used to coordinate the cops and to spread information amongst them
		(as to where the actual current position of each cop is etc etc)
	*/
	void AddCopGroup( UnitGroup<state,action,environment> *copgroup );

	Unit<state, action, environment> *GetUnit( unsigned int which );
	UnitGroup<state,action,environment> *GetCopGroup() { return copgroup; };

	/*! deletes all the cops of the simulation (and it's copgroup) */
	void ClearAllUnits();

	environment *GetEnvironment() { return env; }

	void StepTime(double);
	double GetSimulationTime() const { return currTime; }
	double GetTimeToNextStep() const;
	void DoNextStep() { StepTime( GetTimeToNextStep() ); };

	void SetPaused(bool val) { paused = val; }
	bool GetPaused() { return paused; }

	/*! simulation is over when either all the agents are done
	 * or the robber is caught */
	bool Done();

	void OpenGLDraw() const;

	SimulationInfo<state,action,environment>* GetSimulationInfo() { return this; };
	void GetPublicUnitInfo(unsigned int which, PublicUnitInfo<state,action,environment> &info) const;
	unsigned int GetCurrentUnit() const { return currentActor; }

protected:
	void StepUnitTime( unsigned int index, double timeStep);
	bool MakeUnitMove( unsigned int index, std::vector<action> &next_actions, double &moveCost);

	// if index == -1 then update all information, otherwise just that particular unit
	void UpdatePublicInfo( int index = -1 );

	// variables
	environment *env;
	bool simultaneous; // whether the simulation is simultaneous or alternating
	double currTime;
	bool paused;
	UnitGroup<state,action,environment> *copgroup;
	std::vector<DSCRUnitInfo<state, action, environment> *> units;
	DSCREnvironment<state,action> *dscrenvironment;
	CRState worldstate;
	mutable unsigned int currentActor;
	//SimulationInfo<state,action,environment> sinfo;
	std::vector<PublicUnitInfo<state,action,environment> > unitinfos;
};



typedef DSCRSimulation<xyLoc, tDirection, AbsMapEnvironment> DSCRAbsMapSimulation;







/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

template<class state, class action, class environment>
DSCRSimulation<state, action, environment>::DSCRSimulation(environment *_env,
	bool playerscanpass, unsigned int cop_speed, bool _simultaneous ):
	env(_env), simultaneous(_simultaneous), currTime(0.), paused(false), copgroup(NULL),
	dscrenvironment( new DSCREnvironment<state,action>(env,playerscanpass,cop_speed) ),
	currentActor(0)
{ };

template<class state, class action, class environment>
DSCRSimulation<state, action, environment>::~DSCRSimulation() {
	// deletes all the cops
	ClearAllUnits();
	// deletes the Cop Robber environment
	delete dscrenvironment;
	// deletes the submitted environment
	//delete env;
}


template<class state,class action,class environment>
unsigned int DSCRSimulation<state,action,environment>::AddRobber( Unit<state,action,environment> *unit, double timeOffset ) {
	DSCRUnitInfo<state,action,environment> *ui = new DSCRUnitInfo<state,action,environment>();
	ui->agent = unit;
	unit->GetLocation( ui->startState );
	ui->currentState = ui->startState;

	ui->nextTime = currTime + timeOffset;
	ui->totalThinking = 0.;
	ui->totalDistance = 0.;

	units.insert( units.begin(), ui ); // put the robber on top of the vector
	worldstate.insert( worldstate.begin(), ui->currentState );
	ui->agent->SetNum( 0 );

	UpdatePublicInfo();

	//SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
	ui->agent->UpdateLocation( env, ui->currentState, true, this );
	//delete s;

	return 0;
};


template<class state, class action, class environment>
unsigned int DSCRSimulation<state,action,environment>::AddCop( Unit<state,action,environment> *unit, double timeOffset ) {
	DSCRUnitInfo<state,action,environment> *ui = new DSCRUnitInfo<state,action,environment>();
	ui->agent = unit;
	unit->GetLocation( ui->startState );
	ui->currentState = ui->startState;

	ui->nextTime = currTime + timeOffset;
	ui->totalThinking = 0.;
	ui->totalDistance = 0.;

	units.push_back( ui );
	worldstate.push_back( ui->currentState );
	ui->agent->SetNum( units.size() - 1 );

	UpdatePublicInfo();

	//SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
	if( copgroup ) {
		// if there is a copgroup set we'll update its position through the copgroup
		copgroup->UpdateLocation( ui->agent, env, ui->currentState, true, this );
	} else {
		ui->agent->UpdateLocation( env, ui->currentState, true, this );
	}
	//delete s;

	return( units.size()-1 );
}


template<class state,class action, class environment>
void DSCRSimulation<state,action,environment>::AddCopGroup( UnitGroup<state,action,environment> *_copgroup ) {
	if( copgroup )
		fprintf( stderr, "Warning: Added a group for the cops twice.\n" );
	copgroup = _copgroup;
	UpdatePublicInfo();
	return;
}


template<class state,class action,class environment>
unsigned int DSCRSimulation<state,action,environment>::GetNumUnits() const {
	return units.size();
}

template<class state,class action,class environment>
unsigned int DSCRSimulation<state,action,environment>::GetNumUnitGroups() const {
	if( copgroup != NULL ) return 1;
	else return 0;
}


template<class state, class action, class environment>
Unit<state,action,environment>* DSCRSimulation<state,action,environment>::GetUnit( unsigned int which ) {
	assert( which < units.size() );
	return units[which]->agent;
}


template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::ClearAllUnits()
{
	while( units.size() > 0 ) {
		DSCRUnitInfo<state,action,environment> *ui = units.back();
		units.pop_back();
		delete ui->agent;
		delete ui;
	}

	currTime = 0.; // reset simulation time
	currentActor = 0;
	UpdatePublicInfo();
}


template<class state, class action, class environment>
bool DSCRSimulation<state,action,environment>::Done()
{

	// check if robber is caught
	if( dscrenvironment->GoalTest( worldstate ) )
		return true;

	// check if robber is done
	if( !units[0]->agent->Done() )
		return false;

	// check if cops are done
	if( copgroup )
		return( copgroup->Done() );
	else {
		for( unsigned int i = 1; i < units.size(); i++ ) {
			if( !units[i]->agent->Done() )
				return false;
		}
	}

	return true;
}



template<class state, class action, class environment>
double DSCRSimulation<state,action,environment>::GetTimeToNextStep() const {
	double minimum = DBL_MAX;

	for( unsigned int i = 0; i < units.size(); i++ ) {
		if( minimum > units[i]->nextTime - currTime )
			minimum = units[i]->nextTime - currTime;
	}
	if( minimum < 1./30. )
		fprintf( stderr, "Warning: GetTimeToNextStep sank under 1./30. (%g).\n", minimum );
	return minimum;
}



template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::StepTime(double timeStep)
{
	if (paused) return;
	if( Done() ) {
		fprintf( stderr, "Warning: simulation is over but StepTime was requested.\n" );
		return;
	}

	currTime += timeStep;
	for( unsigned int i = 1; i < units.size(); i++ ) {
		currentActor = i;
		StepUnitTime( i, timeStep );
		if( !simultaneous ) UpdatePublicInfo( i ); // update our public information record
	}
	if( !Done() ) {
		currentActor = 0;
		StepUnitTime( 0, timeStep );
		if( !simultaneous ) UpdatePublicInfo( 0 );
	} else {
		// tell the robber that he's done
		//SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
		units[0]->agent->UpdateLocation( env, units[0]->currentState, false, this );
		//delete s;
	}
	if( simultaneous ) UpdatePublicInfo();
	
	return;
}



template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::StepUnitTime( unsigned int index, double timeStep)
{
	DSCRUnitInfo<state,action,environment> *theUnit = units[index];

	// if this unit is not scheduled to move, it doesn't
	if (currTime < theUnit->nextTime) return;

	action next_action;
	std::vector<action> next_actions;
	Unit<state, action, environment>* u = theUnit->agent;


	// make a copy of the current simulation info
	// this is done for security reasons
	//SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>(&sinfo);


	// call the agent to make it's move
	// note: cops are called multiple times to make their subsequent moves (according to cop_speed)
	if( index > 0 && copgroup ) {
		// cop in a cop group
		for( unsigned int i = 0; i < dscrenvironment->GetCopSpeed(); i++ ) {
			if( ! copgroup->MakeMove( u, env, this, next_action ) ) break;
			next_actions.push_back( next_action );
		}
	} else if( index > 0 ) {
		// seperate cop
		for( unsigned int i = 0; i < dscrenvironment->GetCopSpeed(); i++ ) {
			if( ! u->MakeMove( env, env->GetOccupancyInfo(), this, next_action ) ) break;
			next_actions.push_back( next_action );
		}
	}	else {
		// robber
		if( u->MakeMove( env, env->GetOccupancyInfo(), this, next_action ) )
			next_actions.push_back( next_action );
	}


	// check whether there is actually anything to do
	bool success = true;
	double moveTime = 1.;
	if( next_actions.size() > 0 ) {
		success = MakeUnitMove( index, next_actions, moveTime);
	}
	// notice the agent of his location update
	if( index && copgroup ) {
		copgroup->UpdateLocation( u, env, theUnit->currentState, success, this );
	} else {
		u->UpdateLocation( env, theUnit->currentState, success, this );
	}

	// delete the copy of public unit information
	//delete s;

	// next time the agent is to move again
	// this can be made arbitrarily complex
	theUnit->nextTime += moveTime;

	return;
}

template<class state, class action, class environment>
bool DSCRSimulation<state, action, environment>::MakeUnitMove( unsigned int index, std::vector<action> &next_actions, double &moveCost)
{
	DSCRUnitInfo<state,action,environment> *theUnit = units[index];

	bool success = true;
	state current_state = worldstate[index];
	action last_move = next_actions[0];
	theUnit->lastState = current_state;
	theUnit->lastTime  = theUnit->nextTime;

	// compute the result state from the actions
	// i.e. execute his chosen path
	for( typename std::vector<action>::iterator it = next_actions.begin(); it != next_actions.end(); it++ ) {
		state temp_state = current_state;
		env->ApplyAction( temp_state, *it );

		if( index > 0 ) {
			// if the agent is a cop, check whether he intersects with another cop
			for( unsigned int i = 1; i < worldstate.size(); i++ ) {
				if( i != index && worldstate[i] == temp_state ) {
					success = false;
					break;
				}
			}
			if( !success ) break;
		}
		current_state = temp_state;
		last_move = *it;
	}

	if( index > 0 ) {
		moveCost = dscrenvironment->CopGCost( worldstate[index], current_state );
	} else {
		moveCost = dscrenvironment->RobberGCost( worldstate[index], current_state );
	}

	// update the world state
	worldstate[index] = current_state;
	theUnit->currentState = current_state;
	theUnit->lastMove = last_move;

	return success;
}

/* ATTENTION!!!! This function gives a way the current real information,
 * this is a security issue but for performance we still did it!
*/
template<class state, class action, class environment>
void DSCRSimulation<state, action, environment>::OpenGLDraw() const
{
	env->OpenGLDraw();

//	UpdatePublicInfo();
	//SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );

	for (unsigned int i = 0; i < units.size(); i++)
		units[i]->agent->OpenGLDraw( env, this );

	if( copgroup )
		copgroup->OpenGLDraw( env, this );

	//delete s;
	return;
}


template<class state, class action, class environment>
void DSCRSimulation<state,action,environment>::UpdatePublicInfo( int index ) {
	unsigned int i;

	if( index >= 0 && (unsigned int)index < units.size() ) {

		unitinfos[index].init(
			units[index]->startState, worldstate[index], units[index]->lastState,
			units[index]->lastMove, units[index]->lastTime, units[index]->nextTime );

	} else {
		// update all information
		unitinfos.clear();
		for( i = 0; i < units.size(); i++ ) {
			unitinfos.push_back( PublicUnitInfo<state,action,environment>(
				units[i]->startState, worldstate[i], units[i]->lastState,
				units[i]->lastMove, units[i]->lastTime, units[i]->nextTime ) );
		}

		/*
		sinfo.unitsingroups.clear();
		if( copgroup ) {
			std::vector<Unit<state,action,environment> *> mems = copgroup->GetMembers();
			std::vector<unsigned int> isingroup;
			for( i = 0; i < mems.size(); i++ ) {
				isingroup.push_back( mems[i]->GetNum() );
			}
			sinfo.unitsingroups.push_back( isingroup );
		}
		*/
	}

	//sinfo.currentTime = currTime;
	return;
}

template<class state,class action,class environment>
void DSCRSimulation<state,action,environment>::GetPublicUnitInfo( unsigned int unitnum, PublicUnitInfo<state,action,environment> &info ) const {

	state ss = unitinfos[unitnum].startState;
	state cs = unitinfos[unitnum].currentState;
	state ls = unitinfos[unitnum].lastState;
	action lm = unitinfos[unitnum].lastMove;
	double lt = unitinfos[unitnum].lastTime;
	double nt = unitinfos[unitnum].nextTime;

	info.init( ss, cs, ls, lm, lt, nt );
	return;
}


#endif
