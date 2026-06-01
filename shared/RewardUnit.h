/*
 *  $Id: rewardUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 2/16/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */


#include "Unit.h"
#include "AStar.h"

#ifndef REWARDUNIT_H
#define REWARDUNIT_H

class rewardUnit : public unit {
public:
	rewardUnit(int x, int y);
	virtual const char *GetName() { return "rewardUnit"; }
	virtual double sendReward() { return 1.0; }
	virtual void receiveReward(double) {}
	void OpenGLDraw(const MapProvider *, const SimulationInfo *) const;
};

class rewardSeekingUnit : public rewardUnit {
public:
	rewardSeekingUnit(int x, int y);
	virtual const char *GetName() { return "rewardUnit"; }
	virtual void receiveReward(double);
	virtual tDirection makeMove(MapProvider *, reservationProvider *, SimulationInfo *simInfo);
	void OpenGLDraw(const MapProvider *, const SimulationInfo *) const;
	void addRewardLocation(rewardUnit *);
private:
	double goToRewardLoc(MapAbstraction *aMap, int which);
	void addPathToCache(path *p);
	std::vector<tDirection> moves;
	std::vector<rewardUnit *> rewardLocs;
	aStar a;
};

#endif
