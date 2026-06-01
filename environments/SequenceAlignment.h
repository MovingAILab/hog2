/*
 *  SequenceAlignment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/20/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */


#include "SearchEnvironment.h"
#include <vector>

#ifndef SequenceAlignment_H
#define SequenceAlignment_H

typedef uint64_t SequenceAlignmentState;
typedef int SequenceAlignmentAction;

class SequenceAlignment : public SearchEnvironment<SequenceAlignmentState, SequenceAlignmentAction>
{
public:
	SequenceAlignment(int length) :d(length) {  }
	virtual void GetSuccessors(const SequenceAlignmentState &nodeID, std::vector<SequenceAlignmentState> &neighbors) const;
	virtual void GetActions(const SequenceAlignmentState &nodeID, std::vector<SequenceAlignmentAction> &actions) const;
	//virtual int GetNumSuccessors(const SequenceAlignmentState &stateID) const;
	virtual SequenceAlignmentAction GetAction(const SequenceAlignmentState &s1, const SequenceAlignmentState &s2) const;
	virtual void ApplyAction(SequenceAlignmentState &s, SequenceAlignmentAction a) const;
	
	virtual void GetNextState(const SequenceAlignmentState &, SequenceAlignmentAction , SequenceAlignmentState &) const;
	
	virtual bool InvertAction(SequenceAlignmentAction &a) const;	
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const SequenceAlignmentState &node1, const SequenceAlignmentState &node2) const;
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const SequenceAlignmentState &node) const
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }
	
	virtual double GCost(const SequenceAlignmentState &node1, const SequenceAlignmentState &node2) const;
	virtual double GCost(const SequenceAlignmentState &node, const SequenceAlignmentAction &act) const;
	virtual bool GoalTest(const SequenceAlignmentState &node, const SequenceAlignmentState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const SequenceAlignmentState &node) const
	{ return bValidSearchGoal&&(node == searchGoal); }
	
	virtual uint64_t GetStateHash(const SequenceAlignmentState &node) const;
	virtual uint64_t GetActionHash(SequenceAlignmentAction act) const;
	
//	virtual void OpenGLDraw() const { }
//	virtual void OpenGLDraw(const SequenceAlignmentState&) const { }
//	virtual void OpenGLDraw(const SequenceAlignmentState&, const SequenceAlignmentState&, float) const { }
//	virtual void OpenGLDraw(const SequenceAlignmentState&, const SequenceAlignmentAction&) const { }
private:
	int d;
};

#endif
