/*
 *  $Id: unitGroup.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/9/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */


#ifndef UNITGROUP_H
#define UNITGROUP_H

#include "BitVector.h"
#include "Map.h"
//#include "MapAbstraction.h"
#include "ReservationProvider.h"
//#include "MapProvider.h"
#include "UnitSimulation.h"
#include "StatCollection.h"

template <class state, class action, class environment>
class Unit;

template<class state, class action, class environment>
class SimulationInfo;

template <class state, class action, class environment>
class UnitGroup {
public:
	virtual ~UnitGroup() {}
	
	virtual const char *GetName() { return "defaultUnitGroup"; }
	
	virtual bool MakeMove(Unit<state, action, environment> *u, environment *e, SimulationInfo<state,action,environment> *si, action& a)
	{
		return (u->MakeMove(e, e->GetOccupancyInfo(), si,a));
	}

	virtual void UpdateLocation(Unit<state, action, environment> *u, environment *e, state &loc, bool success, SimulationInfo<state,action,environment> *si)
	{
		u->UpdateLocation(e, loc, success, si);
	}
	
	void AddUnit(Unit<state, action, environment> *u)
	{
		// Check if we already have this unit
		for (unsigned int x = 0; x < members.size(); x++)
			if (members[x] == u)
				return;
				
		// If not then add the unit to the group and set the unit's group
		members.push_back(u);
		u->SetUnitGroup(this);
	}
	
	virtual bool Done()
	{
		for (unsigned int x = 0; x < members.size(); x++)
      	if (!members[x]->Done())
      	{
         	return false;
         }
      return true;
   }


	void RemoveUnit(Unit<state, action, environment> *u)
	{
		for (unsigned int x = 0; x < members.size(); x++)
		{
			if (members[x] == u)
			{
				u->SetUnitGroup(0);
				members[x] = members[members.size()-1];
				members.pop_back();
			}
		}
	}

	virtual void StartNewTrial(StatCollection *c)
	{
		for (unsigned int x = 0; x < members.size(); x++)
		{
			members[x]->StartNewTrial(c);
		}
	}
	
//	virtual void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *)  const { }

	virtual std::vector<Unit<state,action,environment> *> GetMembers() {return members;}
	unsigned int GetNumMembers() { return members.size(); }
	Unit<state,action,environment> *GetMember(int which) const { return members[which]; }
	
private:
	std::vector<Unit<state, action, environment> *> members;
};


//class unit;
//class SimulationInfo;
//
///**
// * A unitGroup provides shared memory and computation for all units within the group.
// */
//
//class unitGroup {
//public:
//	unitGroup(MapProvider *);
//	virtual ~unitGroup() {}
//	virtual tDirection makeMove(unit *u, MapProvider *, reservationProvider *, SimulationInfo *simInfo);
//	/** gives the unit group time to think on a regular basis. In a synchronous simluation
//		this will be called once at the beginning of each timestep. */
//	virtual void think(MapProvider *) { }
//	virtual const char *GetName() { return "UnitGroupx"; }
//
//	virtual void OpenGLDraw(MapProvider *, SimulationInfo *);
//	virtual void addUnit(unit *);
//	virtual void removeUnit(unit *);
//	virtual MapAbstraction *GetMapAbstraction();
//	
//	/** Inform the given unit of its current/new location */
//	virtual void updateLocation(unit *, MapProvider *, int _x, int _y, bool, SimulationInfo *);
//	/** Is the group done with racing? */
//	virtual bool done();
//	/** Lets the unit group do what it needs to reset a trial */
//	virtual void StartNewTrial(StatCollection *stats);
//	// print stats; if we don't know any, get the unit to do it
//	virtual void LogStats(StatCollection *stats);
//	virtual void LogFinalStats(StatCollection *) {}
//	//virtual void printRoundStats(unit *u, FILE *f);
//
//	/** lets the user cycle through the display mode by hitting Shift-Tab */
//	virtual void cycleDisplayMode(void) { }
//	
//	/** lets the user cycle through the display mode by hitting [ and ] */
//	virtual void increaseDisplayALevel(void) {  }
//	virtual void decreaseDisplayALevel(void) {  }
//	
//	/** allows you to iterate through units in the group */
//	unit *getUnit(unsigned int which);
//	int getGroupID() { return id; }
//
//protected:
//	//friend void unitSimulation::addUnitGroup(unitGroup *);
//	///** This is only for the unit simulation to set when the unitGroup is added. */
//	//virtual void setUnitSimulation(unitSimulation *_us, Map *m);
//	//Map *map;
//	std::vector<unit *> myUnits;
//	//unitSimulation *us;
//	int id;
//	static int groupID;
//};
//
//#include "Unit.h"

#endif

