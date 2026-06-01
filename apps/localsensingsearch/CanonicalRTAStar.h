//
//  CanonicalRTAStar.hpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/21/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef CanonicalRTAStar_hpp
#define CanonicalRTAStar_hpp

#include <stdio.h>


#include "LearningAlgorithm.h"
#include "FPUtil.h"
#include <deque>
#include <vector>
#include <unordered_map>
#include "Map2DEnvironment.h"
#include "CanonicalGrid.h"
#include <unordered_map>
#include <cstdint>

struct canonicalRTAData {
	CanonicalGrid::xyLoc theState;
	CanonicalGrid::tDirection parent;
	double theHeuristic;
};

//CanonicalGrid::CanonicalGrid
class CanonicalRTAStar : public LearningAlgorithm<xyLoc, tDirection, MapEnvironment> {
public:
	CanonicalRTAStar()
	{ fAmountLearned = 0.0f; localEnv = 0; }
	virtual ~CanonicalRTAStar(void) { }
	
	void GetPath(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath);
	void GetPath(MapEnvironment *, const xyLoc& , const xyLoc& , std::vector<tDirection> & ) { assert(false); };
	virtual const char *GetName() { return "CanonicalRTAStar"; }
	void SetHCost(const CanonicalGrid::xyLoc &where, double val)
	{
		xyLoc l(where.x, where.y);
		SetHCost(l, val);
	}
	void SetHCost(const xyLoc &where, double val)
	{
		auto s = heur.find(where);
		if (s == heur.end())
			printf("Error - setting h-cost for uninitialized state\n");
		s->second.theHeuristic = val;
	}
	double HCost(const CanonicalGrid::xyLoc &from) const
	{
		xyLoc l(from.x, from.y);
		HCost(l);
	}
	double HCost(const xyLoc &from) const
	{
		auto s = heur.find(from);
		if (s == heur.end())
		{
			printf("Error - getting h-cost for uninitialized state\n");
			return 0;
		}
		return s->second.theHeuristic;
	}
	void AddToTable(const xyLoc &from, tDirection parentAction, CanonicalGrid::tDirection canonicalParent)
	{
		if (heur.find(from) == heur.end())
		{
			heur[from].theHeuristic = externalEnv->HCost(from, goal);
//			heur[from].parent = parentAction;
			CanonicalGrid::xyLoc l;
			l.x = from.x;
			l.y = from.y;
			l.parent = canonicalParent;
			heur[from].theState = l;
		}
	}
	virtual uint64_t GetNodesExpanded() const { return nodesExpanded; }
	virtual uint64_t GetNodesTouched() const { return nodesTouched; }
	virtual void LogFinalStats(StatCollection *s)
	{
		s->AddStat("TotalLearning", GetName(),fAmountLearned);
	}
	
	double GetAmountLearned() { return fAmountLearned; }
	void OpenGLDraw() const {}
	void OpenGLDraw(const MapEnvironment *env) const;
private:
	typedef std::unordered_map<xyLoc, canonicalRTAData, xyLocHash> LearnedHeuristic;
	
	CanonicalGrid::CanonicalGrid *localEnv;
	MapEnvironment *externalEnv;
	LearnedHeuristic heur;
	xyLoc goal;
	double fAmountLearned;
	uint64_t nodesExpanded, nodesTouched;
};


#endif /* CanonicalRTAStar_hpp */
