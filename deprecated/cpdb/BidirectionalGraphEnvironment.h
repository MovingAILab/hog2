/*
 *  BidirectionalGraphEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/13/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include <stdint.h>
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include "GraphCanonicalHeuristic.h"

class graphStatePair {
public:
	graphStatePair()
	{
		previousStart = -1;
		previousGoal = -1;
	}
	graphStatePair(uint32_t s, uint32_t g, int32_t prev_s=-1, int32_t prev_g=-1)
	:start(s), goal(g), previousStart(prev_s), previousGoal(prev_g)
	{ }
	uint32_t start;
	uint32_t goal;

	int32_t previousStart;
	int32_t previousGoal;
};

class graphMovePair {
public:
	graphMovePair() :from(UINT16_MAX), to(UINT16_MAX) {}
	graphMovePair(uint16_t f, uint16_t t) :from(f), to(t) {}
	uint16_t from, to;
};

static std::ostream& operator <<(std::ostream & out, const graphStatePair &loc)
{
	out << "[" << loc.start << ", " << loc.goal << "]";
	return out;
}

static bool operator==(const graphStatePair &l1, const graphStatePair &l2)
{
	return (l1.start == l2.start)&&(l1.goal==l2.goal);
}


class BidirectionalGraphEnvironment : public SearchEnvironment<graphStatePair, graphMovePair> {
public:
	BidirectionalGraphEnvironment(GraphCanonicalHeuristic *gh);
	BidirectionalGraphEnvironment(Graph *graph, GraphHeuristic *h = 0);
	virtual ~BidirectionalGraphEnvironment();
	virtual void GetSuccessors(const graphStatePair &stateID, std::vector<graphStatePair> &neighbors) const;
	virtual void GetActions(const graphStatePair &stateID, std::vector<graphMovePair> &actions) const;
	virtual graphMovePair GetAction(const graphStatePair &s1, const graphStatePair &s2) const;
	virtual void ApplyAction(graphStatePair &s, graphMovePair a) const;
	virtual bool InvertAction(graphMovePair &a) const;
	void SetUseBidirectional(bool use) { swap = use; }
	
	void SetDirected(bool b) { directed = b; }
	
	OccupancyInterface<graphStatePair, graphMovePair> *GetOccupancyInfo() { return 0; }
	virtual double HCost(const graphStatePair &state1, const graphStatePair &state2) const;
	virtual double GCost(const graphStatePair &state1, const graphStatePair &state2) const;
	virtual double GCost(const graphStatePair &state1, const graphMovePair &state2) const;
	virtual bool GoalTest(const graphStatePair &state, const graphStatePair &goal) const;
	virtual uint64_t GetStateHash(const graphStatePair &state) const;
	virtual uint64_t GetActionHash(graphMovePair act) const;
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const graphStatePair &s) const;
	virtual void OpenGLDraw(const graphStatePair &s, const graphMovePair &gm) const;
	virtual void OpenGLDraw(const graphStatePair &s, const graphStatePair&, float) const { OpenGLDraw(s); }
	
	Graph *GetGraph() { return g; };
	
	virtual void StoreGoal(graphStatePair &) {}
	virtual void ClearGoal() {}
	virtual bool IsGoalStored() {return false;}
	
	virtual double HCost(const graphStatePair &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for BidirectionalGraphEnvironment\n");
		exit(1); return -1.0;}
	
	virtual bool GoalTest(const graphStatePair &) const {
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for BidirectionalGraphEnvironment\n");
		exit(1); return false;
	}
	
protected:
	bool directed;
	bool swap;
	Map *m;
	Graph *g;
	GraphCanonicalHeuristic *gh;
	GraphHeuristic *h;
};
