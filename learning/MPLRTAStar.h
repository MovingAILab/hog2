//
//  MPLRTAStar.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/28/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef hog2_glut_MPLRTAStar_h
#define hog2_glut_MPLRTAStar_h

#include "LearningAlgorithm.h"
#include "FPUtil.h"
#include <deque>
#include <vector>
//#include <unordered_map>
#include <unordered_map>
#include "Map2DEnvironment.h"

namespace MPLRTA {
	
	struct learnedData {
		learnedData() {dead = false; }
		xyLoc theState;
		bool dead;
		double theHeuristic;
	};
	
	// This class defines the LRTA* algorithm
	class MPLRTAStar : public LearningAlgorithm<xyLoc,tDirection,MapEnvironment> {
	public:
		MPLRTAStar(bool kill = true)
		{ fAmountLearned = 0.0f; setStart = false; doKilling = kill; }
		virtual ~MPLRTAStar(void) { }
		
		void GetPath(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath);
		void GetPath(MapEnvironment *, const xyLoc& , const xyLoc& , std::vector<tDirection> & ) { assert(false); };
		virtual const char *GetName() { if (doKilling) return "MPLRTAStar[kill]"; return "MPLRTAStar[]"; }
		void SetHCost(MapEnvironment *env, const xyLoc &where, const xyLoc &to, double val)
		{
			heur[env->GetStateHash(where)].theHeuristic = val-env->HCost(where, to);
			heur[env->GetStateHash(where)].theState = where;
		}
		double HCost(MapEnvironment *env, const xyLoc &from, const xyLoc &to) const
		{
			auto val = heur.find(env->GetStateHash(from));
			if (val != heur.end())
			{
				return val->second.theHeuristic+env->HCost(from, to);
			}
			return env->HCost(from, to);
		}
		void Kill(MapEnvironment *env, const xyLoc &s)
		{
			heur[env->GetStateHash(s)].theState = s;
			heur[env->GetStateHash(s)].dead = true;
		}
		bool IsDead(MapEnvironment *env, const xyLoc &from)
		{
			if (!doKilling) return false;
			if (heur.find(env->GetStateHash(from)) != heur.end())
				return heur[env->GetStateHash(from)].dead;
			return false;
		}
				  
		virtual uint64_t GetNodesExpanded() const { return nodesExpanded; }
		virtual uint64_t GetNodesTouched() const { return nodesTouched; }
		virtual void LogFinalStats(StatCollection *s)
		{
			s->AddStat("TotalLearning", GetName(),fAmountLearned);
		}
		
		double GetAmountLearned() { return fAmountLearned; }
//		void OpenGLDraw() const {}
//		void OpenGLDraw(const MapEnvironment *env) const;
	private:
		typedef std::unordered_map<uint64_t, learnedData, Hash64 > LearnedHeuristic;
		
		LearnedHeuristic heur;
		xyLoc goal, startState;
		bool setStart, doKilling;
		double fAmountLearned;
		uint64_t nodesExpanded, nodesTouched;
	};
	
	/** The core routine of MPLRTAStar -- computes at most one-move path */
	void MPLRTAStar::GetPath(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath)
	{
		if (setStart == false)
		{
			setStart = true;
			startState = from;
		}
		goal = to;
		thePath.resize(0);
		if (from==to)
			return;
		
		// Initialization -----------------------------------------------------
		// Use the benefit rule (i.e., never decrease h[]) ?
		static const bool benefitRule = true;
		
		// Initialize the variables
		double minF = DBL_MAX;
		double minG = 0;
		int minC = -1; 
		
		nodesExpanded = 0;
		nodesTouched = 0;
		nodesTouched++;
		
		// Lookahead --------------------------------------------------------------------
		// get the current node off the open List
		nodesExpanded++;
		
		// Iterate through all edges coming out of *n
		std::vector<xyLoc> neighbors;
		env->GetSuccessors(from, neighbors);
		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			// Generate a child
			nodesTouched++;

			if (IsDead(env, neighbors[x]))
				continue;
			
//			// Check if the child's tile is occupied
//			if (env->GetOccupancyInfo() && !env->GetOccupancyInfo()->CanMove(from, neighbors[x]))
//				continue;
			
			// Compute the g values
			double g = env->GCost(from, neighbors[x]);// e->getWeight();
			
			// See if the min on this level needs to be updated
			double h = HCost(env, neighbors[x], to);
			double f = g + h;
			
			if (fless(f, minF))
			{
				minF = f;
				minG = g;
				minC = (int)x;
			}
			// tie break the same as other algorithms
			if (fequal(f, minF) && fgreater(g, minG))
			{
				minF = f;
				minG = g;
				minC = (int)x;
			}
		}
		
		// If there were no legitimate children (e.g., we are blocked out)
		// then return an empty path
		if (minC == -1)
			return;
		
		// Update h ---------------------------------------------------------------------
		
		// Compute the new h
		double newH = minF;
		
		// Compute the amount of learning
		double oldH = HCost(env, from, to);
		double deltaH = newH - oldH;
		
		// The benefit rule disallows us to make updates or account for decreases in h
		if (benefitRule && fless(deltaH, 0.0))
			deltaH = 0.0;
		
		deltaH = fabs(deltaH);			// decreasing h is also learning
		
		// update h[from,to]
		if (fgreater(deltaH,0.0))
			SetHCost(env, from, to, newH);
		
		// Update the amount learned on this trial
		// We do this with an if to avoid accumulating floating point errors
		if (fgreater(deltaH,0.0))
			fAmountLearned += deltaH;
		
		// test for deadness
		tDirection dirs[] = {kN, kNW, kW, kSW, kS, kSE, kE, kNE};
		xyLoc curr = from;
		int changes = 0;
		int passable = 0;
		env->ApplyAction(curr, dirs[7]);
		bool currPass = env->GetMap()->CanStep(curr.x, curr.y, from.x, from.y) && !IsDead(env, curr);
		//if (currPass) passable++;
		for (int x = 0; x < 8; x++)
		{
			curr = from;
			env->ApplyAction(curr, dirs[x]);
			bool p = env->GetMap()->CanStep(curr.x, curr.y, from.x, from.y) && !IsDead(env, curr);
			if (p) passable++;
			if (p == currPass)
				continue;
			currPass = p;
			changes++;
		}
		if (passable <= 4 && changes <= 2 && !(from == startState))
			Kill(env, from);
		
		// Move -------------------------------------------------------------------------
		thePath.push_back(neighbors[minC]);
		thePath.push_back(from);
		return;
	}
	
//	void MPLRTAStar::OpenGLDraw(const MapEnvironment *e) const
//	{
//		double learned = 0;
//		for (LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//		{
//			double thisState = (*it).second.theHeuristic;
//			if (learned < thisState)
//				learned = thisState;
//		}
//		for (LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//		{
//			double r = (*it).second.theHeuristic;
//			if ((*it).second.dead)
//			{
//				e->SetColor(0,0,0);
//				e->OpenGLDraw((*it).second.theState);
//			}
//			else if (r > 0)
//			{
//				e->SetColor(0.5+0.5*r/learned, 0, 0, 0.1+0.8*r/learned);
//				e->OpenGLDraw((*it).second.theState);
//			}
//		}
//	}

}

#endif
