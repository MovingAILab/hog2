
// Most of this code has been copied from UnitSimulation.h
// except that it has been customized for Robber-Cop Simulations

#ifndef CRSIMULATION_H
#define CRSIMULATION_H

#include <vector>
#include <queue>
#include "Unit.h"
#include "Timer.h"
#include "FPUtil.h"
#include "StatCollection.h"
#include "SearchEnvironment.h"
#include "OccupancyInterface.h"
#include "SimulationInfo.h"

template <class state, class action, class environment>
class CRUnitInfo {
public:
	CRUnitInfo() {};
	Unit<state, action, environment> *agent;
	action lastMove;
	state startState;
	state currentState;
	double nextTime;
	double totalDistance;
	double totalThinking;
};


// kLockStep - each unit goes exactly to the next time
// kRealTime - each unit goes up thinkingTime*thinkingPenalty + movement time*speed
// kMinTime - each unit goes up at least step time, but can go longer
//            that is, max(next time, thinkingTime*thinkingPenalty + movement time*speed)
//enum tTimestep {
//	kLockStep, kRealTime, kMinTime, kUniTime
//};

/**
 * A simulation of the game on cops and robber
 * Note: Please add a robber before you use anything else of the simulation
 */
template<class state, class action, class environment>
class CRSimulation {
public:

	typedef typename CopRobberEnvironment<state,action>::CRState CRState;
	typedef typename CopRobberEnvironment<state,action>::CRMove CRMove;
	typedef typename CopRobberEnvironment<state,action>::CRAction CRAction;

	/*! creates the simulation by specifying the environment,
	 * the robber we are working with, when it is first scheduled to move,
	 * and whether players can pass when it's their turn or not */
	CRSimulation(environment *se, Unit<state,action,environment> *robber, double robberTimeOffset = 0., bool playerscanpause = false );
	~CRSimulation();

	/*! add cops to the simulation */
	unsigned int AddUnit( Unit<state, action, environment> *unit, double timeOffset = 0. );

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
	double GetSimulationTime() { return currTime; }
	double GetTimeToNextStep();
	void DoNextStep() { StepTime( GetTimeToNextStep() ); };

	void SetStepType(tTimestep step) { stepType = step; }
	tTimestep GetStepType() { return stepType; }

	void SetPaused(bool val) { paused = val; }
	bool GetPaused() { return paused; }

	/*! simulation is over when either all the agents are done
	 * or the robber is caught */
	bool Done();

	/** setPenalty for thinking. Sets the multiplier used to penalize thinking time. */
	void SetThinkingPenalty(double pen) { penalty = pen; }
	/** getPenalty for thinking. Gets the multiplier used to penalize thinking time. */
	double GetThinkingPenalty() { return penalty; }

	void OpenGLDraw();

	SimulationInfo<state,action,environment>* GetSimulationInfo();

protected:
	void StepUnitTime( unsigned int index, double timeStep);
	bool MakeUnitMove( unsigned int index, action where, double &moveCost);

	void DoPreTimestepCalc();
	void DoTimestepCalc(double amount);
	void DoPostTimestepCalc();
	
	void UpdatePublicInfo();

	// variables
	double penalty;
	bool paused;
	double currTime;
	tTimestep stepType;
	environment *env;
	std::vector<CRUnitInfo<state, action, environment> *> units;
	UnitGroup<state,action,environment> *copgroup;
	CopRobberEnvironment<state,action> *crenvironment;
	CRState worldstate;
	CRMove joint_move;
	SimulationInfo<state,action,environment> sinfo;
};



typedef CRSimulation<xyLoc, tDirection, AbsMapEnvironment> CRAbsMapSimulation;






/*------------------------------------------------------------------------------
| Implementation
------------------------------------------------------------------------------*/

template<class state, class action, class environment>
CRSimulation<state, action, environment>::CRSimulation(environment *se, Unit<state,action,environment> *robber, double robberTimeOffset, bool playerscanpass )
{
	env = se;
	stepType = kRealTime;
	penalty = 0.;
	currTime = 0.;
	paused = false;
	copgroup = NULL;

	crenvironment = new CopRobberEnvironment<state,action>( env, playerscanpass );

	// add the robber to the simulation
	CRUnitInfo<state,action,environment> *ui = new CRUnitInfo<state,action,environment>();
	ui->agent = robber;
	robber->GetLocation( ui->startState );
	ui->currentState = ui->startState;

	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
	ui->agent->UpdateLocation( env, ui->currentState, true, s );
	delete s;

	ui->nextTime = currTime + robberTimeOffset;
	ui->totalThinking = 0.;
	ui->totalDistance = 0.;
	units.push_back( ui );
	worldstate.push_back( ui->currentState );
	ui->agent->SetNum( 0 );

}

template<class state, class action, class environment>
CRSimulation<state, action, environment>::~CRSimulation() {
	// deletes all the cops
	ClearAllUnits();
	// deletes the robber
	delete units[0]->agent;
	units.clear();
	// deletes the Cop Robber environment
	delete crenvironment;
	// deletes the submitted environment
	delete env;
}

template<class state, class action, class environment>
unsigned int CRSimulation<state,action,environment>::AddUnit( Unit<state,action,environment> *unit, double timeOffset ) {
	CRUnitInfo<state,action,environment> *ui = new CRUnitInfo<state,action,environment>();
	ui->agent = unit;
	unit->GetLocation( ui->startState );
	ui->currentState = ui->startState;

	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );
	if( copgroup ) {
		// if there is a copgroup set we'll update its position through the copgroup
		copgroup->UpdateLocation( ui->agent, env, ui->currentState, true, s );
	} else {
		ui->agent->UpdateLocation( env, ui->currentState, true, s );
	}
	delete s;

	ui->nextTime = currTime + timeOffset;
	ui->totalThinking = 0.;
	ui->totalDistance = 0.;

	units.push_back( ui );
	worldstate.push_back( ui->currentState );
	ui->agent->SetNum( units.size() - 1 );
	return( units.size()-1 );
}

template<class state,class action, class environment>
void CRSimulation<state,action,environment>::AddCopGroup( UnitGroup<state,action,environment> *_copgroup ) {
	if( copgroup )
		fprintf( stderr, "Warning: Added a group for the cops twice.\n" );
	copgroup = _copgroup;
	return;
}

template<class state, class action, class environment>
Unit<state,action,environment>* CRSimulation<state,action,environment>::GetUnit( unsigned int which ) {
	assert( which < units.size() );
	return units[which];
}

template<class state, class action, class environment>
void CRSimulation<state, action, environment>::ClearAllUnits()
{
	while( units.size() > 1 ) {
		CRUnitInfo<state,action,environment> *ui = units.back();
		units.pop_back();
		delete ui->agent;
		delete ui;
	}

	UpdatePublicInfo();
//	currTime = 0.; // we cannot reset the simulation time because the robber stays in the simulation
}

template<class state, class action, class environment>
bool CRSimulation<state,action,environment>::Done()
{

	// check if robber is caught
	if( crenvironment->GoalTest( worldstate, worldstate ) )
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
double CRSimulation<state,action,environment>::GetTimeToNextStep() {
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
void CRSimulation<state, action, environment>::StepTime(double timeStep)
{
	if (paused) return;
	if( Done() ) {
		fprintf( stderr, "Warning: simulation is over but StepTime was requested.\n" );
		return;
	}

	DoPreTimestepCalc();
	currTime += timeStep;
	DoTimestepCalc(timeStep);
	DoPostTimestepCalc();

	return;
}

template<class state, class action, class environment>
void CRSimulation<state, action, environment>::DoPreTimestepCalc()
{
	UpdatePublicInfo();
	return;
}

template<class state, class action, class environment>
void CRSimulation<state, action, environment>::DoTimestepCalc(double timeStep)
{
	unsigned int i;
	// reset the joint move
	// this move will be set together in StepUnitTime (or more precisely in MakeUnitMove)
	joint_move.clear();
	for( i = 0; i < units.size(); i++ ) {
		joint_move.push_back( CRAction() );
	}

	// it is important that we move the robber before the cops!
	// otherwise he could get caught although he isn't really
	// (because we compute this sequentially)
	for( i = 0; i < units.size(); i++ )
		StepUnitTime( i, timeStep );

	// update the worldstate
	crenvironment->ApplyAction( worldstate, joint_move );

	return;
}

template<class state, class action, class environment>
void CRSimulation<state, action, environment>::DoPostTimestepCalc()
{
}

template<class state, class action, class environment>
void CRSimulation<state, action, environment>::StepUnitTime( unsigned int index, double timeStep)
{
	CRUnitInfo<state,action,environment> *theUnit = units[index];

	// if this unit is not scheduled to move, it doesn't
	if (currTime < theUnit->nextTime) return;

	double moveThinking, locThinking, moveTime;
	action where;
	Timer t;
	Unit<state, action, environment>* u = theUnit->agent;
	
	// need to do if/then check - makemove ok or not? need to stay where you are? 
	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>(&sinfo);

	bool moving;
	if( index && copgroup ) {
		t.StartTimer();
		moving = copgroup->MakeMove( u, env, s, where );
		moveThinking = t.EndTimer();
	}	else {
		t.StartTimer();
		//moving = u->MakeMove( env, NULL, s, where ); would probably work the same way since we're not working with
		// any OccupancyInterfaces here...
		moving = u->MakeMove( env, env->GetOccupancyInfo(), s, where );
		moveThinking = t.EndTimer();
	}

	if( moving ) {
		theUnit->totalThinking += moveThinking;
	
		bool success = MakeUnitMove( index, where, moveTime);

		if( success )
			theUnit->lastMove = where;

		if( index && copgroup ) {
			t.StartTimer();
			copgroup->UpdateLocation( u, env, theUnit->currentState, success, s );
			locThinking = t.EndTimer();
		} else {
			t.StartTimer();
			u->UpdateLocation( env, theUnit->currentState, success, s );
			locThinking = t.EndTimer();
		}

		theUnit->totalThinking += locThinking;

		switch( stepType ) {
			case kLockStep:
				theUnit->nextTime += timeStep;
				break;
			case kUniTime:
				theUnit->nextTime += 1.;
				break;
			case kRealTime:
				theUnit->nextTime += (locThinking+moveThinking)*penalty + moveTime;
				break;
			case kMinTime:
				theUnit->nextTime += min((locThinking+moveThinking)*penalty + moveTime,	 timeStep);
			break;
		}
	}
	else // stay where you are
	{
		switch( stepType ) {
			case kLockStep:
				theUnit->nextTime += timeStep;
				break;
			case kUniTime:
				theUnit->nextTime += 1.;
				break;
			default:
				theUnit->nextTime += u->GetSpeed();
		}

		if( index && copgroup )
			copgroup->UpdateLocation( u, env, theUnit->currentState, true, s);
		else
			u->UpdateLocation( env, theUnit->currentState, true, s );
	}
	delete s;

	return;
}

template<class state, class action, class environment>
bool CRSimulation<state, action, environment>::MakeUnitMove( unsigned int index, action where, double &moveCost)
{
	CRUnitInfo<state,action,environment> *theUnit = units[index];

	bool success = false;
	moveCost = 0;
	CRState worldstatecopy = worldstate;

	joint_move[index] = CRAction( where );
	crenvironment->ApplyAction( worldstatecopy, joint_move );

	OccupancyInterface<CRState,CRMove> *crocc = crenvironment->GetOccupancyInfo();

	// if there is no OccupancyInterface we did the move and that's it
	if( !crocc ) return true;

	if ( crocc->CanMove( worldstate, worldstatecopy ) )
	{	
		success = true;
		theUnit->currentState = worldstatecopy[index];
		moveCost = env->GCost( worldstate[index], theUnit->currentState )*theUnit->agent->GetSpeed();
		theUnit->totalDistance += env->GCost( worldstate[index], theUnit->currentState );
	} else {
		success = false;
		joint_move[index] = CRAction();
	}
	
	return success;
}

/* ATTENTION!!!! This function gives a way the current real information,
 * this is a security issue but for performance we still did it!
*/
template<class state, class action, class environment>
void CRSimulation<state, action, environment>::OpenGLDraw()
{
	env->OpenGLDraw();

	UpdatePublicInfo();
	SimulationInfo<state,action,environment> *s = new SimulationInfo<state,action,environment>( &sinfo );

	for (unsigned int i = 0; i < units.size(); i++)
		units[i]->agent->OpenGLDraw( env, s );

	if( copgroup )
		copgroup->OpenGLDraw( env, s );

	delete s;
	return;
}


template<class state, class action, class environment>
void CRSimulation<state,action,environment>::UpdatePublicInfo() {
	unsigned int i;

	sinfo.unitinfos.clear();
	state goal;
	for( i = 0; i < units.size(); i++ ) {
		units[i]->agent->GetGoal( goal );

		sinfo.unitinfos.push_back( PublicUnitInfo<state,action,environment>(
			units[i]->agent->GetName(),
			units[i]->startState, units[i]->currentState,
			units[i]->lastMove, units[i]->nextTime, units[i]->agent->GetSpeed(),
			units[i]->agent->Done(), goal
			) );
	}

	sinfo.unitsingroups.clear();
	if( copgroup ) {
		std::vector<Unit<state,action,environment> *> mems = copgroup->GetMembers();
		std::vector<unsigned int> isingroup;
		for( i = 0; i < mems.size(); i++ ) {
			isingroup.push_back( mems[i]->GetNum() );
		}
		sinfo.unitsingroups.push_back( isingroup );
	}

	sinfo.currentTime = currTime;
	return;
}


#endif
