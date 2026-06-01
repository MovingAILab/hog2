//
//  MotionCaptureMovement.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 11/9/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__MotionCaptureMovement__
#define __hog2_glut__MotionCaptureMovement__

#include <iostream>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "FPUtil.h"

struct mcMovementState {
public:
	float x, y;
	float heading;
};

static bool operator==(const mcMovementState &l1, const mcMovementState &l2)
{
	return ((abs(l1.x-l2.x)<0.01) &&
			(abs(l1.y-l2.y)<0.01) &&
			((int)l1.heading==(int)l2.heading));

}

static std::ostream& operator <<(std::ostream & out, const mcMovementState &loc)
{
	out << "{ (" << loc.x << ", " << loc.y << ") - " << loc.heading << "}";
	return out;
}

typedef int mcMovementAction;

class MCEnvironment : public SearchEnvironment<mcMovementState, mcMovementAction>
{
public:
	MCEnvironment() {  }
	virtual void GetSuccessors(const mcMovementState &nodeID, std::vector<mcMovementState> &neighbors) const;
	virtual void GetActions(const mcMovementState &nodeID, std::vector<mcMovementAction> &actions) const;
	//virtual int GetNumSuccessors(const mcMovementState &stateID) const;
	virtual mcMovementAction GetAction(const mcMovementState &s1, const mcMovementState &s2) const;
	virtual void ApplyAction(mcMovementState &s, mcMovementAction a) const;
	
	virtual void GetNextState(const mcMovementState &, mcMovementAction , mcMovementState &) const;
	
	virtual bool InvertAction(mcMovementAction &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const mcMovementState &node1, const mcMovementState &node2) const;
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const mcMovementState &node) const
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }
	
	virtual double GCost(const mcMovementState &node1, const mcMovementState &node2) const;
	virtual double GCost(const mcMovementState &node, const mcMovementAction &act) const;
	virtual bool GoalTest(const mcMovementState &node, const mcMovementState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const mcMovementState &node) const
	{ return bValidSearchGoal&&(node == searchGoal); }
	
	virtual uint64_t GetStateHash(const mcMovementState &node) const;
	virtual uint64_t GetActionHash(mcMovementAction act) const;
	
	//virtual double GetPathLength(std::vector<mcMovementState> &neighbors);
	
	virtual void OpenGLDraw() const {}
	virtual void OpenGLDraw(const mcMovementState&) const;
	/** Draw the transition at some percentage 0...1 between two states */
	virtual void OpenGLDraw(const mcMovementState&, const mcMovementState&, float) const;
	virtual void OpenGLDraw(const mcMovementState&, const mcMovementAction&) const;
	void GLDrawLine(const mcMovementState &a, const mcMovementState &b) const;
private:
	double distance(const mcMovementState &n1, const mcMovementState &n2) const;
	bool GetOpenGLCoord(float x_, float y_, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const;

};


#endif /* defined(__hog2_glut__MotionCaptureMovement__) */
