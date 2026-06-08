/*
 *  SteeringEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/12/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#ifndef STEERINGENVIRONMENT_H
#define STEERINGENVIRONMENT_H

#include "SearchEnvironment.h"
#include "FPUtil.h"

struct steeringState {
public:
	float x, y;
	float v;
	float heading;
};

static bool operator==(const steeringState &l1, const steeringState &l2)
{
	return ((fequal(l1.x, l2.x)) &&
			(fequal(l1.y, l2.y)) &&
			(fequal(l1.v, l2.v)) &&
			(fequal(l1.heading, l2.heading)));
}

//struct steeringAction {
//public:
//	steeringAction(float xVel=0.0f, float yVel=0.0f, float hVel=0.0f) :dx(xVel), dy(yVel), dh(hVel) {}
//	float dx, dy, dh;
//};
// state

struct steeringAction {
public:
	steeringAction(float d_v=0.0f, float d_h=0.0f) :dv(d_v), dh(d_h) {}
	float dv, dh;
};

static bool operator==(const steeringAction &l1, const steeringAction &l2)
{
	return ((fequal(l1.dv, l2.dv)) &&
			(fequal(l1.dh, l2.dh)));
}

extern float maxSpeed;
extern float worldRadius;

class SteeringEnvironment : public SearchEnvironment<steeringState, steeringAction>
{
public:
	SteeringEnvironment() {  }
	virtual void GetSuccessors(const steeringState &nodeID, std::vector<steeringState> &neighbors) const;
	virtual void GetActions(const steeringState &nodeID, std::vector<steeringAction> &actions) const;
	//virtual int GetNumSuccessors(const steeringState &stateID) const;
	virtual steeringAction GetAction(const steeringState &s1, const steeringState &s2) const;
	virtual void ApplyAction(steeringState &s, steeringAction a) const;
	
	virtual void GetNextState(const steeringState &, steeringAction , steeringState &) const;
	
	virtual bool InvertAction(steeringAction &a) const;	
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const steeringState &node1, const steeringState &node2) const;
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const steeringState &node) const
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }
	
	virtual double GCost(const steeringState &node1, const steeringState &node2) const;
	virtual double GCost(const steeringState &node, const steeringAction &act) const;
	virtual bool GoalTest(const steeringState &node, const steeringState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const steeringState &node) const
	{ return bValidSearchGoal&&(node == searchGoal); }
	
	virtual uint64_t GetStateHash(const steeringState &node) const;
	virtual uint64_t GetActionHash(steeringAction act) const;
	
	//virtual double GetPathLength(std::vector<steeringState> &neighbors);
	
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const steeringState&) const;
//	/** Draw the transition at some percentage 0...1 between two states */
//	virtual void OpenGLDraw(const steeringState&, const steeringState&, float) const;
//	virtual void OpenGLDraw(const steeringState&, const steeringAction&) const;
private:
};


#endif
