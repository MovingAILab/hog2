/*
 *  RandomUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/14/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "Unit.h"

#ifndef RANDOMUNIT_H
#define RANDOMUNIT_H

template <class state, class action, class environment>
class RandomUnit : public Unit<state, action, environment>
{
public:
	RandomUnit(state startLoc) :loc(startLoc) {}
	virtual ~RandomUnit() {}
	virtual const char *GetName() { return "Random Unit"; }
	virtual bool MakeMove(environment *theEnv, OccupancyInterface<state, action> *, SimulationInfo<state,action,environment> *, action& a)
	{
		std::vector<action> acts;
		theEnv->GetActions(loc, acts);
		a = acts[random()%acts.size()];
		return true;
	}
	virtual void UpdateLocation(environment *, state &newloc, bool success, SimulationInfo<state,action,environment> *)
	{
		if (success)
			loc = newloc;
	}
	virtual void GetLocation(state &l)
	{ l = loc; }
//	virtual void OpenGLDraw(const environment *theEnv, const SimulationInfo<state,action,environment> *si) const
//	{
//		PublicUnitInfo<state, action, environment> i;
//		si->GetPublicUnitInfo(si->GetCurrentUnit(), i);
//		printf("(%f-%f)/(%f-%f)\n", 
//			   si->GetSimulationTime(), i.lastTime, i.nextTime, i.lastTime);
//		theEnv->OpenGLDraw(i.lastState, i.currentState,
//						(si->GetSimulationTime()-i.lastTime)/(i.nextTime-i.lastTime));
//	}
	virtual void GetGoal(state &s)
	{ s = loc; }
private:
	state loc;
};

#endif

