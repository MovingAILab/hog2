/**
 *
 * HLRTA*
 *
 * 2/23/12
 *
 * Tansel Uras, Carlos Ulloa & Nathan Sturtevant
 *
 */

#ifndef HLRTASTAR_H
#define HLRTASTAR_H

#include "LearningAlgorithm.h"
#include "FPUtil.h"
#include <deque>
#include <vector>
#include <unordered_map>

namespace HLRTA{

template <class state>
struct learnedData {
	learnedData()
	{
		validLastMove = false;	
	}
	state theState;	//only for drawing
	state lastMove;
	bool validLastMove;
	double h1;
	double h2;
};

// This class defines the LRTA* algorithm
template <class state, class action, class environment>
class HLRTAStar : public LearningAlgorithm<state,action,environment> {
public:
	HLRTAStar()
	{ fAmountLearned = 0.0f; }
	virtual ~HLRTAStar(void) { }

	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	virtual const char *GetName() { return "HLRTAStar"; }
	void SetH1Cost(environment *env, const state &where, const state &to, double val)
	{	
		fAmountLearned += val-(heur[env->GetStateHash(where)].h1+env->HCost(where, to));
		heur[env->GetStateHash(where)].h1 = val-env->HCost(where, to);
		heur[env->GetStateHash(where)].theState = where;
	}
	void SetH2Cost(environment *env, const state &where, const state &to, double val)
	{
		heur[env->GetStateHash(where)].h2 = val-env->HCost(where, to);
		heur[env->GetStateHash(where)].theState = where;
	}
	void SetLastMove(environment *env, const state &where, const state &to)
	{
		heur[env->GetStateHash(where)].lastMove = to;
		heur[env->GetStateHash(where)].validLastMove = true;
	}
	bool GetLastMove(environment *env, const state &where, state &to)
	{
		if (heur[env->GetStateHash(where)].validLastMove)
			to = heur[env->GetStateHash(where)].lastMove;

		return heur[env->GetStateHash(where)].validLastMove;
	}
	
	double LocalFCost(environment *env, const state &from, const state &neighbor, const state &to)
	{
		double myFCost = env->GCost(from,neighbor);
		state stateLast;
		if (GetLastMove(env, neighbor, stateLast) && stateLast == from)
			myFCost += HCost2(env,neighbor,to);
		else
			myFCost += HCost1(env,neighbor,to);
			
		return myFCost;
	}
	
	double HCost1(environment *env, const state &from, const state &to)
	{
		if (heur.find(env->GetStateHash(from)) != heur.end())
			return heur[env->GetStateHash(from)].h1+env->HCost(from, to);
		return env->HCost(from, to);
	}

	double HCost2(environment *env, const state &from, const state &to)
	{
		if (heur.find(env->GetStateHash(from)) != heur.end())
			return heur[env->GetStateHash(from)].h2+env->HCost(from, to);
		return env->HCost(from, to);
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
void HLRTAStar<state, action, environment>::GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath)
{
	goal = to;
	thePath.resize(0);
	if (from==to)
		return;

	
	nodesExpanded = 0;
	nodesTouched = 0;
	nodesExpanded++;
	
	// Iterate through all edges coming out of *n
	std::vector<state> neighbors;
	env->GetSuccessors(from, neighbors);

	int best = -1;
	int secondBest = -1;

	//double bestFCost, secondBestFCost;

	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		// Generate a child
		nodesTouched++;

		if (best == -1){
			best = x;		
		}
		else{
			double myFCost = LocalFCost(env, from, neighbors[x], to);
			double bestFCost = LocalFCost(env, from, neighbors[best], to);
			
			if (fless(myFCost,bestFCost))
			{
				secondBest = best;
				best = x;
				
			}
			
			else {
				if (secondBest == -1)
				{
					secondBest = x;
				}
				else {
					double secondBestFCost = LocalFCost(env, from, neighbors[secondBest], to);						
					if (fless(myFCost,secondBestFCost))
					{
						secondBest = x;
					}
				}
			}
		}		
	}
	
	SetH1Cost(env, from, to, LocalFCost(env, from, neighbors[best], to));
	
	if (secondBest == -1)
		SetH2Cost(env, from, to, 1000);
	else
		SetH2Cost(env, from, to, LocalFCost(env, from, neighbors[secondBest], to));
	SetLastMove(env, from, neighbors[best]);
	
//	std::cout<<neighbors[best]<<" "<<from<<std::endl;

	thePath.push_back(neighbors[best]);
	thePath.push_back(from);
	return;
}

//template <class state, class action, class environment>
//void HLRTAStar<state, action, environment>::OpenGLDraw(const environment *e) const
//{
//	double learned = 0;
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double thisState = (*it).second.h1;
//		if (learned < thisState)
//			learned = thisState;
//	}
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double r = (*it).second.h1;
//		if (r > 0)
//		{
//			e->SetColor(0.5+0.5*r/learned, 0, 0.5, 0.1+0.8*r/learned);
//			e->OpenGLDraw((*it).second.theState);
//		}
//	}
//}

}	//HLRTA namespace
#endif
