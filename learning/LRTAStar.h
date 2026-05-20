/*
 *  $Id: LRTAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/04/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef LRTASTAR_H
#define LRTASTAR_H

#include "LearningAlgorithm.h"
#include "FPUtil.h"
#include <deque>
#include <vector>
#include <unordered_map>

template <class state>
struct learnedData {
	state theState;
	double theHeuristic;
};

// This class defines the LRTA* algorithm
template <class state, class action, class environment>
class LRTAStar : public LearningAlgorithm<state,action,environment> {
public:
	LRTAStar()
	{ fAmountLearned = 0.0f; }
	virtual ~LRTAStar(void) { }

	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	virtual const char *GetName() { return "LRTAStar"; }
	void SetHCost(environment *env, const state &where, const state &to, double val)
	{
		heur[env->GetStateHash(where)].theHeuristic = val-env->HCost(where, to);
		heur[env->GetStateHash(where)].theState = where;
	}
	double HCost(environment *env, const state &from, const state &to) const
	{
		auto val = heur.find(env->GetStateHash(from));
		if (val != heur.end())
		{
			return val->second.theHeuristic+env->HCost(from, to);
		}
		return env->HCost(from, to);
//		if (heur.find(env->GetStateHash(from)) != heur.end())
//			return heur[env->GetStateHash(from)].theHeuristic+env->HCost(from, to);
//		return env->HCost(from, to);
	}
	
	virtual uint64_t GetNodesExpanded() const { return nodesExpanded; }
	virtual uint64_t GetNodesTouched() const { return nodesTouched; }
	virtual void LogFinalStats(StatCollection *s)
	{
		s->AddStat("TotalLearning", GetName(),fAmountLearned);
	}

	double GetAmountLearned() { return fAmountLearned; }
//	void OpenGLDraw() const {}
//	void OpenGLDraw(const environment *env) const;
private:
	typedef std::unordered_map<uint64_t, learnedData<state>, Hash64 > LearnedHeuristic;

	LearnedHeuristic heur;
	state goal;
	double fAmountLearned;
	uint64_t nodesExpanded, nodesTouched;
};

/** The core routine of LRTAStar -- computes at most one-move path */
template <class state, class action, class environment>
void LRTAStar<state, action, environment>::GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath)
{
	goal = to;
	thePath.resize(0);
	if (from==to)
		return;

	// Initialization -----------------------------------------------------
	// Use the benefit rule (i.e., never decrease h[]) ?
	static const bool benefitRule = true;
	
	// Initialize the variables
	double minF = DBL_MAX;
	double minF_D = DBL_MAX;
	double minG = 0;
	int minC = -1; 
	int minD = -1; // depression avoidance
	nodesExpanded = 0;
	nodesTouched = 0;
	nodesTouched++;
	
	// Lookahead --------------------------------------------------------------------
	// get the current node off the open List
	nodesExpanded++;
	
	// Iterate through all edges coming out of *n
	std::vector<state> neighbors;
	env->GetSuccessors(from, neighbors);
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		// Generate a child
		nodesTouched++;
		
		// Check if the child's tile is occupied
		if (env->GetOccupancyInfo() && !env->GetOccupancyInfo()->CanMove(from, neighbors[x]))
			continue;
		
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
		if (1)// da
		{
			if (fless((h-env->HCost(neighbors[x], to))*10
					  +g+h, minF_D))
			{
				minF_D = (h-env->HCost(neighbors[x], to))*10+g+h;
				minD = (int)x;
			}
			//[h(s')-h0(s')]*LARGE + [cost(s,s')+h(s')]
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
	
	// Move -------------------------------------------------------------------------
	if (1) // daLRTA*
		thePath.push_back(neighbors[minD]);
	else
		thePath.push_back(neighbors[minC]);
	thePath.push_back(from);
	return;
}

//template <class state, class action, class environment>
//void LRTAStar<state, action, environment>::OpenGLDraw(const environment *e) const
//{
//	double learned = 0;
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double thisState = (*it).second.theHeuristic;
//		if (learned < thisState)
//			learned = thisState;
//	}
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double r = (*it).second.theHeuristic;
//		if (r > 0)
//		{
//			e->SetColor(0.5+0.5*r/learned, 0, 0, 0.1+0.8*r/learned);
//			e->OpenGLDraw((*it).second.theState);
//		}
//	}
//}

#endif
