/*
 *  GraphCanonicalHeuristic.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/9/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef GRAPHCANONICALHEURISTIC_H
#define GRAPHCANONICALHEURISTIC_H

#include "GraphEnvironment.h"
#include "MapSectorAbstraction.h"
#include "Map2DEnvironment.h"
#include "MapOverlay.h"
#include <vector>

// pure virtual class
class GraphCanonicalHeuristic : public GraphHeuristic {
public:
	GraphCanonicalHeuristic(Map *m, int numClosest, int sizeFactor = 10);
	GraphCanonicalHeuristic(Graph *g, GraphHeuristic *gh, int numClosest, int sizeFactor = 10);
	GraphCanonicalHeuristic(char *file);
	~GraphCanonicalHeuristic();
	Graph *GetGraph();
	Map *GetMap();
	void Load(char *file);
	void Save(char *file);
	double HCost(const graphState &state1, const graphState &state2) const;
	void ChooseStartGoal(graphState &start, graphState &goal);
	double GetDistToCanonicalState(graphState &which);
	graphState GetCanonicalStateID(graphState &which);

	void OpenGLDraw() const;
	int GetNumEntries() {return centerDist.size() + lengths.size() * (lengths.size()); }
private:
	void BuildPDB();
	void GetCenters();
	void GetPDBValues();
	void ComputerCenterDistances();
	
	std::vector<std::vector<float> > lengths;
	std::vector<int> centers;
	std::vector<std::vector<float> > centerDist;
	std::vector<std::vector<int> > whichPDB;
	//MapSectorAbstraction *msa;
	Map *map;
	Graph *g;
	GraphEnvironment *ga;
	MapOverlay *mo;
	int m_SizeFactor;
	int m_NumClosest;
	int m_NumCanonicals;
};

#endif
