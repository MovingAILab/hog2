/*
 *  $Id: unit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef UNITS_H
#define UNITS_H

#include "UnitGroup.h"
#include "UnitSimulation.h"
#include "OccupancyInterface.h"

template<class state, class action, class environment>
class SimulationInfo;

template <class state, class action, class environment>
class Unit {
public:
	//	Unit(state s, Unit<state, action, env> *target);
	Unit() :speed(0), group(0) { SetColor(1.0, 0.0, 0.0); priority = 999; }
	virtual ~Unit() { SetUnitGroup(0); }
	virtual const char *GetName() = 0;
	virtual bool MakeMove(environment *, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *, action& a) = 0;
	virtual void UpdateLocation(environment *, state &, bool success, SimulationInfo<state,action,environment> *) = 0;
	virtual void GetLocation(state &) = 0;
//	virtual void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *) const = 0;
	virtual void GetGoal(state &s) {};
	virtual bool Done() { return true;} 

	virtual double GetSpeed() { return speed; }
	void SetSpeed(double s) { speed = s; }

	/** log an stats that may have been computed during the last run */
	virtual void LogStats(StatCollection *) {}
	/** log any final one-time stats before a simulation is ended */
	virtual void LogFinalStats(StatCollection *) {}
	
	virtual void SetColor(float _r, float _g, float _b) { r=_r; g=_g; b=_b; }
	virtual void GetColor(float& _r, float& _g, float& _b) const { _r=r; _g=g; _b=b; }
	
	UnitGroup<state, action, environment> *GetUnitGroup() { return group; }
	void SetUnitGroup(UnitGroup<state, action, environment> *_group)
		{
			// If we're already set to the given group then do nothing
			if (_group == group)
				return;
			
			UnitGroup<state, action, environment> *tmp = group;
			
			group = 0; 
			if (tmp)
				tmp->RemoveUnit(this);
			
			// Set the back pointer
			group = _group;
			
			if (_group != 0)
				_group->AddUnit(this);
			
			// OLD CODE
/*			// If we had a group before then move
			if (tmp != 0)
			{
				tmp->RemoveUnit(this);
				if (_group)
					_group->AddUnit(this);
			} */ 
		}

	virtual unsigned int GetNum() { return unitid; }
	virtual void SetNum( unsigned int num ) { unitid = num; return; }
	virtual void StartNewTrial(StatCollection *) {}

	virtual int GetPriority() { return priority; }
	virtual void SetPriority(int val) { priority = val; }
private:
	double speed;
	int priority;
	UnitGroup<state, action, environment> *group;
	float r, g, b;
	unsigned int unitid;
};


#endif
