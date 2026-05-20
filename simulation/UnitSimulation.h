/*
 *  $Id: unitSimulation.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/30/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef UNITSIMULATION_H
#define UNITSIMULATION_H

#include <vector>
#include <queue>
#include "Unit.h"
#include "Timer.h"
#include "FPUtil.h"
#include "StatCollection.h"
#include "SearchEnvironment.h"
#include "OccupancyInterface.h"
#include "SimulationInfo.h"

/**
 * Private per-unit unitSimulation data.
 */

template <class state, class action>
class TimeStep {
public:
	TimeStep(state s, double time)
	:m_state(s), startTime(time) {}
	TimeStep() { startTime = 0; }
  state m_state;
  double startTime;
};

template <class state, class action, class environment>
class Unit;

template <class state, class action, class environment>
class UnitGroup;


template <class state, class action, class environment>
class UnitInfo {
public:
	UnitInfo() :stateHistory(0), converged(false) {}
	Unit<state, action, environment> *agent;
	int GetPriority() { return agent->GetPriority(); }
	action lastMove;
	state startState;
	state lastState;
	state currentState;
//	double thinkTime, moveDist;
	double lastTime;
	double nextTime;
	unsigned int historyIndex;
	double totalDistance;
	double totalThinking;
	bool converged;
	std::vector<TimeStep<state, action> > stateHistory;
};

//template<class state, class action>
//class UnitInfoCompare {
//public:
//	bool operator()(const UnitInfo<state, action> *u1, const UnitInfo<state, action> *u2);
//};


// kLockStep - each unit goes exactly to the next time
// kRealTime - each unit goes up thinkingTime*thinkingPenalty + movement time*speed
// kMinTime - each unit goes up at least step time, but can go longer
//            that is, max(next time, thinkingTime*thinkingPenalty + movement time*speed)
enum tTimestep {
	kLockStep, kRealTime, kMinTime, kUniTime
};

/**
 * The basic simulation class for the world.
 */
template<class state, class action, class environment>
class UnitSimulation : public SimulationInfo<state, action, environment> {
public:
	UnitSimulation(environment *se);
	virtual ~UnitSimulation();

	/*! \param timeOffset is a setting to let the unit start a little bit later */
	virtual int AddUnit(Unit<state, action, environment> *u, double timeOffset = 0.);
	unsigned int GetNumUnits() const;
	Unit<state, action, environment> *GetUnit(unsigned int which);
	unsigned int GetNumUnitGroups() const;

	int AddUnitGroup(UnitGroup<state, action, environment> *ug);
	UnitGroup<state, action, environment> *GetUnitGroup(unsigned int which);
	virtual void ClearAllUnits();
	
	virtual void GetPublicUnitInfo(unsigned int which, PublicUnitInfo<state,action,environment> &info) const;

	environment *GetEnvironment() { return env; }
		
	void StepTime(double);
	double GetSimulationTime() const { return currTime; }
	double GetTimeToNextStep() const;
	void SetStepType(tTimestep step) { stepType = step; }
	tTimestep GetStepType() const { return stepType; }

	void SetPaused(bool val) { paused = val; }
	bool GetPaused() { return paused; }
	
	bool Done();

	/** setPenalty for thinking. Sets the multiplier used to penalize thinking time. */
	void SetThinkingPenalty(double pen) { penalty = pen; }
	/** getPenalty for thinking. Gets the multiplier used to penalize thinking time. */
	double GetThinkingPenalty() { return penalty; }

//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(unsigned int whichUnit) const;
	
	void SetLogStats(bool val) { logStats = val; }
	bool GetLogStats() { return logStats; }
	StatCollection* GetStats() { return &stats; }
	

	virtual SimulationInfo<state,action,environment>* GetSimulationInfo() { return this; }
	virtual unsigned int GetCurrentUnit() const { return currentActor; }

protected:
	void StepUnitTime(UnitInfo<state, action, environment> *ui, double timeStep);
	bool MakeUnitMove(UnitInfo<state, action, environment> *theUnit, action where, double &moveCost);

	virtual void DoPreTimestepCalc();
	virtual void DoTimestepCalc(double amount);
	virtual void DoPostTimestepCalc();
	
	double penalty;
	bool paused, logStats;
	std::vector<UnitInfo<state, action, environment> *> units;
	std::vector<UnitGroup<state, action, environment> *> unitGroups;
	environment *env;
	double currTime;
	tTimestep stepType;
	StatCollection stats;
	mutable unsigned int currentActor;
//	SimulationInfo<state,action,environment> sinfo;
};

template<class state, class action, class environment>
UnitSimulation<state, action, environment>::UnitSimulation(environment *se)
{
	env = se;
	stepType = kRealTime;
	penalty = 0.;
	currTime = 0.;
	paused = false;
	logStats = true;
	unitGroups.push_back(new UnitGroup<state, action, environment>);
	currentActor = 0;
	// allocate default unit group!(?)
}

template<class state, class action, class environment>
UnitSimulation<state, action, environment>::~UnitSimulation() {
	ClearAllUnits();
	unitGroups.clear();
}

template<class state, class action, class environment>
int UnitSimulation<state, action, environment>::AddUnit(Unit<state, action, environment> *u, double timeOffset)
{
	UnitInfo<state, action, environment> *ui = new UnitInfo<state, action, environment>();
	ui->agent = u;
	u->GetLocation(ui->startState);
	ui->currentState = ui->startState;
	ui->lastState = ui->startState;
	if (ui->agent->GetUnitGroup() == 0)
	{
		ui->agent->SetUnitGroup(unitGroups[0]);
	}
	ui->agent->GetUnitGroup()->UpdateLocation(ui->agent, env, ui->currentState, true, this );
	ui->nextTime = currTime + timeOffset;
	ui->totalThinking = 0.0;
	ui->totalDistance = 0.0;
	ui->historyIndex = 0;
	
	if (0)//(keepHistory)
	{
		TimeStep<state, action> ts(ui->startState, ui->nextTime);
		ui->stateHistory.push_back(ts);		
	}
	units.push_back(ui);
	ui->agent->SetNum( units.size() - 1 );
	return units.size()-1;
}

template<class state, class action, class environment>
unsigned int UnitSimulation<state, action, environment>::GetNumUnits() const
{
	return units.size();
}

template<class state, class action, class environment>
Unit<state, action, environment> *UnitSimulation<state, action, environment>::GetUnit(unsigned int which)
{
	if (which < units.size())
		return units[which]->agent;
	return 0;
}

template<class state, class action, class environment>
int UnitSimulation<state, action, environment>::AddUnitGroup(UnitGroup<state, action, environment> *ug)
{
	unitGroups.push_back(ug);
	return unitGroups.size()-1;
}

template<class state, class action, class environment>
unsigned int UnitSimulation<state, action, environment>::GetNumUnitGroups() const
{
	return unitGroups.size();
}

template<class state, class action, class environment>
UnitGroup<state, action, environment> *UnitSimulation<state, action, environment>::GetUnitGroup(unsigned int which)
{
	if (which < unitGroups.size())
		return units[which];
	return 0;
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::GetPublicUnitInfo(unsigned int which, PublicUnitInfo<state,action,environment> &info) const
{
	info.init(units[which]->startState, units[which]->currentState,
			  units[which]->lastState, units[which]->lastMove,
			  units[which]->lastTime, units[which]->nextTime);
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::ClearAllUnits()
{
	while (units.size() > 0)
	{
		UnitInfo<state, action, environment> *ui = units.back();
		if (logStats)
			ui->agent->LogFinalStats(&stats);
		OccupancyInterface<state, action> *envInfo = env->GetOccupancyInfo();
		if (envInfo)
			envInfo->SetStateOccupied(ui->currentState, false);
		units.pop_back();
		delete ui->agent;
		delete ui;
	}
//	while (unitGroups.size() > 0)
//	{
//		//unitGroups.back()->LogFinalStats(&stats);
//		delete unitGroups.back();
//		unitGroups.pop_back();
//	}
	//unitGroups.push_back(new unitGroup(this));
	//sinfo = SimulationInfo<state,action,environment>();
	currTime = 0.;
	currentActor = 0;
}

template<class state, class action, class environment>
bool UnitSimulation<state,action,environment>::Done()
{
	// assumes that all units belong to a unit group
	for (unsigned int i=0; i<unitGroups.size(); i++)
	{
		if (!unitGroups[i]->Done())
			return false;
	}
	return true;
}

//template<class state, class action, class environment>
//double UnitSimulation<state, action, environment>::SetUnitLocation(UnitInfo<state, action, environment> *ui,
//																																	 bool success, bool usetimer)
//{
//	Timer t;
//	if (usetimer)
//	{
//		t.StartTimer();
//	}
//	if (ui->agent->getUnitGroup() == 0)
//	{
//		ui->agent->UpdateLocation(ui->currentState, success);
//	}
//	else {
//		u->agent->getUnitGroup()->updateLocation(ui->agent, ui->currentState, success);
//	}
//	if (usetimer)
//	{
//		return t.EndTimer();
//	}
//	return 0;
//}

template<class state, class action, class environment>
double UnitSimulation<state,action,environment>::GetTimeToNextStep() const {
	double minimum = DBL_MAX;

	for ( unsigned int i = 0; i < units.size(); i++ ) {
		if ( minimum > units[i]->nextTime - currTime )
			minimum = units[i]->nextTime - currTime;
	}
	if ( minimum < 1./30. )
		fprintf( stderr, "Warning: GetTimeToNextStep sank under 1./30. (%g).\n", minimum );
	return minimum;
}


template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::StepTime(double timeStep)
{
	//std::cout<<"StepTime\n";
	if (paused)
	{
		return;
	}
	DoPreTimestepCalc();
	if (logStats)
		stats.AddStat("simulationTime", "UnitSimulation", currTime);
	currTime += timeStep;
	DoTimestepCalc(timeStep);
	DoPostTimestepCalc();
	
	//std::cout<<"currTime "<<currTime<<std::endl;
	//if (Done()) std::cout<<"DONE!!!\n";
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::DoPreTimestepCalc()
{
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::DoTimestepCalc(double timeStep)
{
	for (unsigned int x = 0; x < units.size(); x++)
	{
		currentActor = x;
		StepUnitTime(units[x], timeStep);
		
		if (stepType == kRealTime)
		{
			while (currTime > units[x]->nextTime)
				StepUnitTime(units[x], 0);
		}
	}
}

template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::DoPostTimestepCalc()
{
}

/**
* step time for a single unit.
 *
 * This function takes care of all the simulation details for moving a
 * single unit, doing timing, etc. When overloading advanceTime, this
 * function can be called for each unit that moves.
 */
template<class state, class action, class environment>
void UnitSimulation<state, action, environment>::StepUnitTime(UnitInfo<state, action, environment> *theUnit, double timeStep)
{
	if (currTime < theUnit->nextTime) return;

	double moveThinking=0, locThinking=0, moveTime=0;
	action where;
	Timer t;
	Unit<state, action, environment>* u = theUnit->agent;
	
	t.StartTimer();
	// need to do if/then check - makemove ok or not? need to stay where you are? 
	if (u->GetUnitGroup()->MakeMove(u, env, this, where))
	{
		moveThinking = t.EndTimer();
		theUnit->totalThinking += moveThinking;
		theUnit->lastMove = where;
		theUnit->lastTime = theUnit->nextTime;
		theUnit->lastState = theUnit->currentState;
		if (logStats)
			stats.SumStat("MakeMoveThinkingTime", u->GetName(), moveThinking);
	
		bool success = MakeUnitMove(theUnit, where, moveTime);
		//printf("Updating last state\n");
		
		t.StartTimer();
		u->GetUnitGroup()->UpdateLocation(theUnit->agent, env, theUnit->currentState, success, this);
		locThinking = t.EndTimer();
		theUnit->totalThinking += locThinking;
		if (logStats)
			stats.SumStat("UpdateLocationThinkingTime", u->GetName(), locThinking);

		switch (stepType)
		{
			case kLockStep:
				theUnit->nextTime = currTime + timeStep;
				break;
			case kUniTime:
				theUnit->nextTime = currTime + 1.;
				break;
			case kRealTime:
				//printf("Incrementing time from %f to %f\n", theUnit->nextTime, theUnit->nextTime+moveTime);
				theUnit->nextTime += (locThinking+moveThinking)*penalty + moveTime;
				break;
			case kMinTime:
				theUnit->nextTime += max((locThinking+moveThinking)*penalty + moveTime,	 timeStep);
			break;
		}
		//u->GetUnitGroup()->LogStats(&stats);
		u->LogStats(&stats);
	}
	else // stay where you are
	{
		theUnit->lastTime = theUnit->nextTime;
		theUnit->lastState = theUnit->currentState;
		
		if ( stepType == kUniTime )
			theUnit->nextTime = currTime + 1.;
		else
			theUnit->nextTime += theUnit->agent->GetSpeed();
		theUnit->agent->GetUnitGroup()->UpdateLocation(theUnit->agent, env, theUnit->currentState, true, this);	
	}

}

template<class state, class action, class environment>
bool UnitSimulation<state, action, environment>::MakeUnitMove(UnitInfo<state, action, environment> *theUnit, action where, double &moveCost)
{
	bool success = false;
	moveCost = 0;
	state oldState = theUnit->currentState;
	env->ApplyAction(theUnit->currentState, where);
	OccupancyInterface<state, action> *envInfo = env->GetOccupancyInfo();
	
	// TODO: Perhaps we need a legal action test here to handle the
	//       cases where we can't generate all possible actions.
	bool legal = false;

	std::vector<action> succ;
	env->GetActions(oldState, succ);
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		if (succ[x] == where)
		{
			legal = true;
			break;
		}
	}
	
	if (legal && 
		(!envInfo || (envInfo && envInfo->CanMove(oldState, theUnit->currentState))))
	{	
		success = true;
		moveCost = env->GCost(oldState, theUnit->currentState)*theUnit->agent->GetSpeed();
		theUnit->totalDistance += env->GCost(oldState, theUnit->currentState);
		
		if (envInfo)
		{
			envInfo->MoveUnitOccupancy(oldState, theUnit->currentState);
		}
	}
	else {
//		if (!envInfo)
//			success = true;
//		else
		success = false;
			
		theUnit->currentState = oldState;
	}
	
	return success;
}

/* ATTENTION!!!! This function gives away the current real information,
 * this is a security issue but for performance we still did it!
*/
//template<class state, class action, class environment>
//void UnitSimulation<state, action, environment>::OpenGLDraw() const
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		currentActor = x;
//		units[x]->agent->OpenGLDraw(env, this);
//	}
//	
//	for (unsigned int x = 0; x <unitGroups.size(); x++)
//		unitGroups[x]->OpenGLDraw(env, this);
//}
//
//template<class state, class action, class environment>
//void UnitSimulation<state, action, environment>::OpenGLDraw(unsigned int whichUnit) const
//{
//	if (whichUnit >= units.size())
//		return;
//
//	currentActor = whichUnit;
//	units[whichUnit]->agent->OpenGLDraw(env, this);
//}


#endif
