/*
 *  SFBDPancakePuzzle.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/14/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#ifndef SFBDPANCAKEPUZZLE_H
#define SFBDPANCAKEPUZZLE_H

#include "PancakePuzzle.h"
#include "BurnedPancakePuzzle.h"

class pancakeStatePair {
public:
	pancakeStatePair()
	{
	}
	pancakeStatePair(BurnedPancakePuzzleState s, BurnedPancakePuzzleState g)
	:start(s), goal(g)
	{ }
	BurnedPancakePuzzleState start;
	BurnedPancakePuzzleState goal;
};

class pancakeMovePair {
public:
	pancakeMovePair(){}
	pancakeMovePair(uint16_t act, bool start):theAction(act), applyToStart(start) {}
	uint16_t theAction;
	bool applyToStart;
};

static std::ostream& operator <<(std::ostream & out, const pancakeStatePair &loc)
{
	out << "[" << loc.start << ", " << loc.goal << "]";
	return out;
}

static bool operator==(const pancakeMovePair &l1, const pancakeMovePair &l2)
{
	return (l1.theAction == l2.theAction)&&(l1.applyToStart==l2.applyToStart);
}

static bool operator==(const pancakeStatePair &l1, const pancakeStatePair &l2)
{
	return (l1.start == l2.start)&&(l1.goal==l2.goal);
}


class SFBDPancakeEnvironment : public SearchEnvironment<pancakeStatePair, pancakeMovePair> {
public:
	SFBDPancakeEnvironment(int theSize);
	virtual ~SFBDPancakeEnvironment();
	virtual void GetSuccessors(const pancakeStatePair &stateID, std::vector<pancakeStatePair> &neighbors) const;
	virtual void GetActions(const pancakeStatePair &stateID, std::vector<pancakeMovePair> &actions) const;
	virtual pancakeMovePair GetAction(const pancakeStatePair &s1, const pancakeStatePair &s2) const;
	virtual void ApplyAction(pancakeStatePair &s, pancakeMovePair a) const;
	virtual bool InvertAction(pancakeMovePair &a) const;
//	void SetUseBidirectional(bool use) { swap = use; }
	
	OccupancyInterface<pancakeStatePair, pancakeMovePair> *GetOccupancyInfo() { return 0; }
	virtual double HCost(const pancakeStatePair &state1, const pancakeStatePair &state2) const;
	virtual double GCost(const pancakeStatePair &state1, const pancakeStatePair &state2) const;
	virtual double GCost(const pancakeStatePair &state1, const pancakeMovePair &state2) const;
	virtual bool GoalTest(const pancakeStatePair &state, const pancakeStatePair &goal) const;
	virtual uint64_t GetStateHash(const pancakeStatePair &state) const;
	virtual uint64_t GetActionHash(pancakeMovePair act) const;
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const pancakeStatePair &s) const;
//	virtual void OpenGLDraw(const pancakeStatePair &s, const pancakeMovePair &gm) const;
//	virtual void OpenGLDraw(const pancakeStatePair &s, const pancakeStatePair&, float) const { OpenGLDraw(s); }
	
	virtual void StoreGoal(pancakeStatePair &) {}
	virtual void ClearGoal() {}
	virtual bool IsGoalStored() const {return false;}
	
	virtual double HCost(const pancakeStatePair &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for SFBDPancakeEnvironment\n");
		exit(1); return -1.0;}
	
	virtual bool GoalTest(const pancakeStatePair &) const {
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for SFBDPancakeEnvironment\n");
		exit(1); return false;
	}
	
protected:
	int MeasureLocalHeuristic(const BurnedPancakePuzzleState &a, const BurnedPancakePuzzleState &b) const;
	BurnedPancakePuzzle *pan;
};


#endif
