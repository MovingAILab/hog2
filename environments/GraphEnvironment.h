/*
 *  GraphEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/29/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef GRAPHENVIRONMENT_H
#define GRAPHENVIRONMENT_H


#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "Graph.h"
//#include "GraphAbstraction.h"
#include "Constants.h"

#include <iostream>
#include <stdint.h>
#include <unordered_map>
#include <functional>

#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif
#ifndef UINT16_MAX
#define UINT16_MAX 65535
#endif

typedef unsigned long graphState;

class graphMove {
public:
	graphMove() :from(UINT32_MAX), to(UINT32_MAX) {}
	graphMove(uint32_t f, uint32_t t) :from(f), to(t) {}
	uint32_t from, to;
};


static bool operator==(const graphMove &l1, const graphMove &l2)
{
	return (l1.from == l2.from)&&(l1.to==l2.to);
}

namespace GraphSearchConstants
{
	enum {
		kHCost = 0, // this is relative to a single goal
		kXCoordinate = 5,
		kYCoordinate = 6,
		kZCoordinate = 7,
		kTemporaryLabel = 8,
		kMapX = 9,
		kMapY = 10,
		kFirstData = 11
	};

	const double kStraightEdgeCost = 1.0;
	const double kDiagonalEdgeCost = ROOT_TWO;

	Graph *GetEightConnectedGraph(Map *m, bool directed = true);
	Graph *GetFourConnectedGraph(Map *m, bool directed = true);
	Graph *GetGraph(Map *m);
	Graph *GetUndirectedGraph(Map *m);
	void AddNodesToGraph(Map *m, Graph *g);
	void AddEdges(Map *m, Graph *g, int x, int y,
				  bool directed = true,
				  double straigtEdgeCost = 1.0,
				  double diagEdgeCost = ROOT_TWO,
				  int straightEdgeProb = 100,
				  int diagEdgeProb = 100);
}

// pure virtual class
class GraphHeuristic {
public:
	virtual ~GraphHeuristic() { }
	virtual Graph *GetGraph() = 0;
	virtual double HCost(const graphState &state1, const graphState &state2) const = 0;
	// if one is better as the start or goal state, this can swap for you.
	virtual void ChooseStartGoal(graphState &/*start*/, graphState &/*goal*/) {}
//	virtual void OpenGLDraw() const {}
private:
};

class GraphHeuristicContainer : public GraphHeuristic {
public:
	GraphHeuristicContainer(Graph *gg) {g = gg;}
	~GraphHeuristicContainer() {}
	virtual Graph *GetGraph() { return g; }
	void AddHeuristic(GraphHeuristic *h) { heuristics.push_back(h); }
	virtual double HCost(const graphState &state1, const graphState &state2) const
	{
		double cost = 0;
		for (unsigned int x = 0; x < heuristics.size(); x++)
			cost = max(cost, heuristics[x]->HCost(state1, state2));
		return cost;
	}
private:
	std::vector<GraphHeuristic*> heuristics;
	Graph *g;
};

// this class uses the label on a graph for a heuristic
// but the heuristic is only available for a single goal node
class GraphLabelHeuristic : public GraphHeuristic {
public:
	GraphLabelHeuristic(Graph *graph, graphState target)
	{ g = graph; goal = target; }
	Graph *GetGraph() { return g; }
	double HCost(const graphState &state1, const graphState &state2) const
	{
		if (state2 == goal)
			return g->GetNode(state1)->GetLabelF(GraphSearchConstants::kHCost);
		return 0;
	}
private:
	graphState goal;
	Graph *g;
};

class GraphMapHeuristic : public GraphHeuristic {
public:
	GraphMapHeuristic(Map *map, Graph *graph)
	:m(map), g(graph) {}
	Graph *GetGraph() { return g; }
	double HCost(const graphState &state1, const graphState &state2) const
	{
		int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
		int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
		int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
		int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);

		double a = ((x1>x2)?(x1-x2):(x2-x1));
		double b = ((y1>y2)?(y1-y2):(y2-y1));
		return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
	}
private:
	Map *m;
	Graph *g;
};

/*
class GraphAbstractionHeuristic : public GraphHeuristic {
public:
	GraphAbstractionHeuristic(MapAbstraction *mabs, int lev)
	:mAbs(mabs), level(lev) { }
	Graph *GetGraph() { return mAbs->GetAbstractGraph(level); }
	double HCost(const graphState &state1, const graphState &state2) const
	{
		return mAbs->h(mAbs->GetAbstractGraph(level)->GetNode(state1),
					   mAbs->GetAbstractGraph(level)->GetNode(state2));
	}
private:
	MapAbstraction *mAbs;
	int level;
};
*/
class GraphMapPerfectHeuristic : public GraphHeuristic {
public:
	GraphMapPerfectHeuristic(Map *map, Graph *graph):m(map), g(graph)
	{
		prob = 0.5;
		fillProbTable();
	}
	Graph *GetGraph() { return g; }
	void SetProbability(double p) { prob = p; }
	double HCost(const graphState &state1, const graphState &state2) const
	{ // warning: in this implementation HCost(s1,s2) != HCost(s2,s1)

		if (probTable[int(state1)]) {
			int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
			int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
			int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
			int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
			return GetOctileDistance(x1-x2, y1-y2);
		}
		else
			return 0;
	}
	~GraphMapPerfectHeuristic()
	{
		delete probTable;
	}
	double prob;
private:
	double GetOctileDistance(double dx, double dy) const
	{
		dx = fabs(dx);
		dy = fabs(dy);

		if (dx > dy)
			return dx-dy + sqrt(2)*dy;
		else
			return dy-dx + sqrt(2)*dx;
	}
	void fillProbTable()
	{
		int size = m->GetMapWidth() * m->GetMapHeight();
		probTable = (bool*)malloc( size );
		for (int i=0;i<size;i++) {
#pragma message("drand was removed here. New code hasn't been tested.")
			if ((random()%10000)/10000.0 < prob)
				probTable[i] = 1;
			else
				probTable[i] = 0;
		}
	}
	Map *m;
	Graph *g;
	bool* probTable;
};

enum placementScheme {
	kFarPlacement,
	kRandomPlacement,
	kAvoidPlacement
};

class GraphDistanceHeuristic : public GraphHeuristic {
public:
	GraphDistanceHeuristic(Graph *graph) :g(graph) { placement = kRandomPlacement; }
	~GraphDistanceHeuristic() {}
	virtual double HCost(const graphState &state1, const graphState &state2) const;
	void AddHeuristic(node *n = 0);
	int GetNumHeuristics() { return heuristics.size(); }
	void SetPlacement(placementScheme s) { placement = s; }
	Graph *GetGraph() { return g; }
	void ChooseStartGoal(graphState &start, graphState &goal);
//	virtual void OpenGLDraw() const;
protected:
	void GetOptimalDistances(node *n, std::vector<double> &values);
	void AddHeuristic(std::vector<double> &values, graphState location);
	node *FindFarNode(node *n);
	node *FindAvoidNode(node *n);
	node *FindBestChild(int best, std::vector<double> &dist,
						std::vector<double> &weight);
	void ComputeSizes(node *n, std::vector<double> &dist,
					  std::vector<double> &weight, std::vector<double> &sizes);
		
	placementScheme placement;
	Graph *g;
	std::vector<std::vector<double> > heuristics;
	std::vector<graphState> locations;

	// for avoid node computation
	std::vector<double> dist;
	std::vector<double> weight;
	std::vector<double> sizes;	
};

enum tHeuristicCombination
{
	kIgnore = 0, // don't combine with database
	kRandom = 1, // combine random selection of databases
	kMax = 2,    // take max of all heuristics
	kGridMax = 3, // 0 on non-grid points
	kCompressed = 4 // simulates compression down to O(N) memory
};


class GraphMapInconsistentHeuristic : public GraphDistanceHeuristic {
public:
	GraphMapInconsistentHeuristic(Map *map, Graph *graph);
	double HCost(const graphState &state1, const graphState &state2) const;
	void SetMode(tHeuristicCombination mode)
	{ hmode = mode; if (compressed) hmode = kCompressed; }
	tHeuristicCombination GetMode() { return hmode; }
	void SetNumUsedHeuristics(int count)
	{ numHeuristics = count; if (numHeuristics > GetNumHeuristics()) numHeuristics = GetNumHeuristics(); }
	int GetNumUsedHeuristics()
	{ return numHeuristics; }	
	void Compress();
//	virtual void OpenGLDraw() const;
	
	void IncreaseDisplayHeuristic()
	{ displayHeuristic = (displayHeuristic+1)%(heuristics.size()+1); }
private:
	void FillInCache(std::vector<double> &vals,
					 std::vector<double> &errors,
					 graphState state2) const;
	tHeuristicCombination hmode;
	int numHeuristics;
	int displayHeuristic;
	bool compressed;
	Map *m;
};

class GraphEnvironment : public SearchEnvironment<graphState, graphMove> {
public:
	GraphEnvironment(Graph *g, GraphHeuristic *gh = 0);
	GraphEnvironment(Map *m, Graph *g, GraphHeuristic *gh = 0);
	virtual ~GraphEnvironment();
	virtual void GetSuccessors(const graphState &stateID, std::vector<graphState> &neighbors) const;
	virtual int GetNumSuccessors(const graphState &stateID) const;
	virtual void GetActions(const graphState &stateID, std::vector<graphMove> &actions) const;
	virtual graphMove GetAction(const graphState &s1, const graphState &s2) const;
	virtual void ApplyAction(graphState &s, graphMove a) const;
	virtual bool InvertAction(graphMove &a) const;

	void SetDirected(bool b) { directed = b; }

	OccupancyInterface<graphState, graphMove> *GetOccupancyInfo() { return 0; }
	virtual double HCost(const graphState &state1, const graphState &state2) const;
	virtual double GCost(const graphState &state1, const graphState &state2) const;
	virtual double GCost(const graphState &state1, const graphMove &state2) const;
	virtual bool GoalTest(const graphState &state, const graphState &goal) const;
	virtual uint64_t GetMaxHash() const { return g->GetNumNodes(); }
	virtual uint64_t GetStateHash(const graphState &state) const;
	virtual void GetStateFromHash(uint64_t parent, graphState &s) const;
	virtual uint64_t GetActionHash(graphMove act) const;
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const graphState &s) const;
//	virtual void OpenGLDraw(const graphState &s, const graphMove &gm) const;
//	virtual void OpenGLDraw(const graphState &s, const graphState&, float) const { OpenGLDraw(s); }
//	virtual void GLDrawLine(const graphState &x, const graphState &y) const;
//	virtual void GLLabelState(const graphState&, const char *) const;

	virtual void Draw(Graphics::Display &disp) const;
	void DrawLERP(Graphics::Display &disp, Graph *a, Graph *b, float mix) const;
	void DrawLERP(Graphics::Display &disp, Graph *a, Graph *b, float mix, std::function<float(float, float, float)> l1, std::function<float(float, float, float)> l2) const;
	void DrawLERP(Graphics::Display &disp, Graph *a, Graph *b, graphState sa, graphState sb, float mix,
				  std::function<float(float, float, float)> l1,
				  std::function<float(float, float, float)> l2) const;

	virtual void Draw(Graphics::Display &disp, const graphState &l) const;
	virtual void DrawStateLabel(Graphics::Display &disp, const graphState &l1, const char *txt) const;
	virtual void DrawLine(Graphics::Display &disp, const graphState &x, const graphState &y, double width = 1.0) const;
	virtual void DrawLine(Graphics::Display &disp, float x1, float y1, float x2, float y2, double width = 1.0) const;
	Graphics::point GetLocation(const graphState &s) const;
	Graph *GetGraph() { return g; };

	virtual void StoreGoal(graphState &) {}
	virtual void ClearGoal() {}
	virtual bool IsGoalStored() const {return false;}

	virtual double HCost(const graphState &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for GraphEnvironment\n");
		exit(1); return -1.0;}

	virtual bool GoalTest(const graphState &) const {
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for GraphEnvironment\n");
		exit(1); return false;
	}
	void SetIntegerEdgeCosts(bool val) { integerEdgeCosts = val; }
	void SetDrawEdgeCosts(bool val) { drawEdgeCosts = val; }
	void SetDrawNodeLabels(bool val) { drawNodeLabels = val; }
	void SetNodeScale(float v) { nodeScale = v; }
	float GetNodeScale() { return nodeScale; }
protected:
	bool directed;
	Map *m;
	Graph *g;
	GraphHeuristic *h;
	bool drawEdgeCosts;
	bool integerEdgeCosts;
	bool drawNodeLabels;
	float nodeScale;
};
/*
class AbstractionGraphEnvironment: public GraphEnvironment {
	public:
	AbstractionGraphEnvironment( GraphAbstraction *gabs, unsigned int level, GraphHeuristic *gh );
	~AbstractionGraphEnvironment();

	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const graphState &s) const { GraphEnvironment::OpenGLDraw(s); }
	virtual void OpenGLDraw(const graphState &s, const graphMove &gm) const { GraphEnvironment::OpenGLDraw(s, gm); }
	virtual void OpenGLDraw(const graphState &s, const graphState&, float) const { OpenGLDraw(s); }
	
	double Scale() { return graphscale; };

protected:
	GraphAbstraction *gabs;
	double graphscale;
};
*/
typedef UnitSimulation<graphState, graphMove, GraphEnvironment> GraphSimulation;

#endif
