/*
 *  MountainCar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/29/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#ifndef MOUNTAINCAR_H
#define MOUNTAINCAR_H

#include "SearchEnvironment.h"

class MountainCarState {
public:
	MountainCarState()
	:loc(-0.5), speed(0)
	{}
	double loc;
	double speed;
};

static bool operator==(const MountainCarState &l1, const MountainCarState &l2) {
	return fequal(l1.loc, l2.loc) && fequal(l1.speed, l2.speed);
}

typedef int MountainCarAction;

class MountainCarEnvironment : public SearchEnvironment<MountainCarState, MountainCarAction>
{
public:
	MountainCarEnvironment();
	virtual void GetSuccessors(const MountainCarState &nodeID, std::vector<MountainCarState> &neighbors) const;
	virtual void GetActions(const MountainCarState &nodeID, std::vector<MountainCarAction> &actions) const;
	//virtual int GetNumSuccessors(const MountainCarState &stateID) const;
	virtual MountainCarAction GetAction(const MountainCarState &s1, const MountainCarState &s2) const;
	virtual void ApplyAction(MountainCarState &s, MountainCarAction a) const;
	
	virtual void GetNextState(const MountainCarState &, MountainCarAction , MountainCarState &) const;
	
	virtual bool InvertAction(MountainCarAction &a) const;	
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const MountainCarState &node1, const MountainCarState &node2) const;
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const MountainCarState &node) const
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }
	
	virtual double GCost(const MountainCarState &node1, const MountainCarState &node2) const;
	virtual double GCost(const MountainCarState &node, const MountainCarAction &act) const;
	virtual bool GoalTest(const MountainCarState &node, const MountainCarState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const MountainCarState &node) const
	{ return bValidSearchGoal&&(node == searchGoal); }
	
	virtual uint64_t GetStateHash(const MountainCarState &node) const;
	virtual uint64_t GetActionHash(MountainCarAction act) const;
	
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const MountainCarState&) const;
//	virtual void OpenGLDraw(const MountainCarState&, const MountainCarState&, float) const;
//	virtual void OpenGLDraw(const MountainCarState&, const MountainCarAction&) const;
private:
	double GetHeightAtPosition(double queryPosition) const;
	double GetSlope(double queryPosition) const;
	
    double minPosition;
    double maxPosition;
    double minVelocity;
    double maxVelocity;
    double goalPosition;
    double accelerationFactor;
    double gravityFactor;
    double hillPeakFrequency;
};

#endif
