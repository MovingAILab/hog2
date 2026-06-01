//
//  CanonicalGraphEnvironment.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/11/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef CanonicalGraphEnvironment_h
#define CanonicalGraphEnvironment_h

#include <stdio.h>
#include <unordered_map>
#include "TemplateAStar.h"
#include "Graph.h"
#include "GraphEnvironment.h"

struct canGraphState {
	graphState parent;
	graphState s;
};

bool operator==(const canGraphState &s1, const canGraphState &s2);
std::ostream &operator<<(std::ostream &out, const canGraphState &s1);

namespace std
{
	template<> struct hash<std::pair<graphState, graphState>>
	{
		inline size_t operator()(const pair<graphState, graphState> & v) const
		{
			return (v.first<<32)|v.second;
		}
	};

	template<> struct hash<canGraphState>
	{
		inline size_t operator()(const canGraphState & v) const
		{
			return (v.parent<<32)|v.s;
		}
	};
}

class CanonicalGraphEnvironment : public SearchEnvironment<canGraphState, graphMove> {
public:
	CanonicalGraphEnvironment(Graph *g);
	~CanonicalGraphEnvironment();
	void GetSuccessors(const canGraphState &stateID, std::vector<canGraphState> &neighbors) const;
//	int GetNumSuccessors(const canGraphState &stateID) const;
	void GetActions(const canGraphState &stateID, std::vector<graphMove> &actions) const;
	graphMove GetAction(const canGraphState &s1, const canGraphState &s2) const;
	void ApplyAction(canGraphState &s, graphMove a) const;
	bool InvertAction(graphMove &a) const;
	
	void SetDirected(bool b) { directed = b; }
	
	double HCost(const canGraphState &state1, const canGraphState &state2) const;
	double GCost(const canGraphState &state1, const canGraphState &state2) const;
	double GCost(const canGraphState &state1, const graphMove &state2) const;
	bool GoalTest(const canGraphState &state, const canGraphState &goal) const;
	uint64_t GetMaxHash() const;
	uint64_t GetStateHash(const canGraphState &state) const;
	uint64_t GetActionHash(graphMove act) const;
//	void OpenGLDraw() const;
//	void OpenGLDraw(const canGraphState &s) const;
//	void OpenGLDraw(const canGraphState &s, const graphMove &gm) const;
//	void OpenGLDraw(const canGraphState &s, const canGraphState&, float) const { OpenGLDraw(s); }
//	void GLDrawLine(const canGraphState &x, const canGraphState &y) const;
	
	Graph *GetGraph() { return g; };
	
	void StoreGoal(canGraphState &) {}
	void ClearGoal() {}
	bool IsGoalStored() const {return false;}
	
	double HCost(const canGraphState &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for CanonicalGraphEnvironment\n");
		exit(1); return -1.0;}
	
	bool GoalTest(const canGraphState &) const {
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for CanonicalGraphEnvironment\n");
		exit(1); return false;
	}
	void SetDrawEdgeCosts(bool val) { drawEdgeCosts = val; }
	void SetDrawNodeLabels(bool val) { drawNodeLabels = val; }
	void SetNodeScale(double v) { nodeScale = v; }
	void ComputeOrdering();
protected:
	std::unordered_map<canGraphState, uint8_t> canonicalOrdering;
	bool directed;
	Graph *g;
	bool drawEdgeCosts;
	bool drawNodeLabels;
	double nodeScale;
};


#endif /* CanonicalGraphEnvironment_hpp */
