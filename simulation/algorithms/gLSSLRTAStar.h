//
//  gLSSLRTAStar.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 11/10/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef gLSSLRTAStar_h
#define gLSSLRTAStar_h

#include "LearningAlgorithm.h"
#include "FPUtil.h"
#include <deque>
#include <vector>
#include <unordered_map>
#include "TemplateAStar.h"
#include "Timer.h"
#include <queue>
#include <iostream>

//static bool verbose = false;

//template <class state>
//class borderData {
//public:
//	borderData(const state &s, const double h) :theState(s), heuristic(h) {}
//	state theState;
//	double heuristic;
//};
//
//template <class state>
//class compareBorderData
//{
//public:
//	bool operator() (const borderData<state> &lhs, const borderData<state> &rhs) const
//	{
//		return (lhs.heuristic > rhs.heuristic);
//	}
//};
//

template <class state>
struct glssLearnedData {
	glssLearnedData() { dead = false; gCost = DBL_MAX; }
	state theState;
	bool dead;
	double theHeuristic;
	double gCost;
};

template <class state, class action, class environment>
class gLSSLRTAStar : public LearningAlgorithm<state,action,environment>, public Heuristic<state> {
public:
	gLSSLRTAStar(int nodeLimit = 8)
	{
		fAmountLearned = 0.0f;
		nodeExpansionLimit = nodeLimit;
		avoidLearning = false;
		minInitialHeuristic = DBL_MAX;
		maxLaterHeuristic = 0;
		initialHeuristic = true;
		firstRun = true;
		initialHeuristicWeight = 1.0;
	}
	virtual ~gLSSLRTAStar(void) { }
	
	void SetAvoidLearning(bool val) { avoidLearning = val; }
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	virtual const char *GetName()
	{ static char name[255];
		sprintf(name, "gLSSLRTAStar(%d)", nodeExpansionLimit); return name; }
	void SetHCost(environment *env, const state &where, const state &to, double val)
	{
		auto loc = heur.find(m_pEnv->GetStateHash(where));
		if (loc != heur.end())
		{
			heur[env->GetStateHash(where)].theHeuristic = val-BaseHCost(env, where, to);
		}
		else {
			heur[env->GetStateHash(where)].theHeuristic = val-BaseHCost(env, where, to);
			heur[env->GetStateHash(where)].theState = where;
			heur[env->GetStateHash(where)].gCost = DBL_MAX;
		}
	}
	double HCostLearned(const state &from)
	{
		if (heur.find(m_pEnv->GetStateHash(from)) != heur.end())
			return heur[m_pEnv->GetStateHash(from)].theHeuristic;
		return 0;
	}
	void SetGCost(const state &s, double cost)
	{
		if (verbose) std::cout << "Setting g-cost of " << s << " to " << cost << "\n";
		auto val = heur.find(m_pEnv->GetStateHash(s));
		if (val != heur.end())
		{
			heur[m_pEnv->GetStateHash(s)].gCost = cost;
		}
		else {
			// initialize
			heur[m_pEnv->GetStateHash(s)].theHeuristic = 0;
			heur[m_pEnv->GetStateHash(s)].theState = s;
			heur[m_pEnv->GetStateHash(s)].gCost = cost;
		}
	}
	double GCost(const state &s)
	{
		auto val = heur.find(m_pEnv->GetStateHash(s));
		if (val != heur.end())
		{
			return val->second.gCost;
		}
		return DBL_MAX;
	}
	double HCost(environment *env, const state &from, const state &to) const
	{
		auto val = heur.find(env->GetStateHash(from));
		if (val != heur.end())
		{
			return val->second.theHeuristic+env->HCost(from, to);
		}
		return BaseHCost(env, from, to);//env->HCost(from, to);
		//		if (heur.find(env->GetStateHash(from)) != heur.end())
		//			return heur[env->GetStateHash(from)].theHeuristic+BaseHCost(env, from, to);
		//		return BaseHCost(env, from, to);
	}
	double BaseHCost(environment *env, const state &from, const state &to) const
	{ return initialHeuristicWeight*env->HCost(from, to);
	}
	double HCost(const state &from, const state &to) const
	{ return HCost(m_pEnv, from, to); }
	
	double GetMaxStateLearning()
	{
		double learned = 0;
		for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
		{
			double thisState = (*it).second.theHeuristic;
			if (learned < thisState)
				learned = thisState;
		}
		return learned;
	}
	void SetInititialHeuristicWeight(double val)
	{ initialHeuristicWeight = val; }
	
	virtual uint64_t GetNodesExpanded() const { return nodesExpanded; }
	virtual uint64_t GetNodesTouched() const { return nodesTouched; }
	virtual void LogFinalStats(StatCollection *s)
	{
		s->AddStat("TotalLearning", GetName(),fAmountLearned);
		s->AddStat("MinInitial", GetName(),minInitialHeuristic);
		s->AddStat("MaxLater", GetName(),maxLaterHeuristic);
		s->AddStat("MaxStateLearning", GetName(), GetMaxStateLearning());
	}
	
	double GetAmountLearned() { return fAmountLearned; }
//	void OpenGLDraw() const {}
//	void OpenGLDraw(const environment *env) const;
private:
	typedef std::unordered_map<uint64_t, glssLearnedData<state>, Hash64 > LearnedHeuristic;
	typedef std::unordered_map<uint64_t, bool, Hash64 > ClosedList;
	
	environment *m_pEnv;
	LearnedHeuristic heur;
	double fAmountLearned;
	double initialHeuristicWeight;
	uint64_t nodesExpanded, nodesTouched;
	int nodeExpansionLimit;
	bool avoidLearning;
	TemplateAStar<state, action, environment> astar;
	std::vector<state> succ;
	bool initialHeuristic;
	bool firstRun;
	double minInitialHeuristic;
	double maxLaterHeuristic;
};

/** The core routine of LSSLRTAStar -- computes at most one-move path */
template <class state, class action, class environment>
void gLSSLRTAStar<state, action, environment>::GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath)
{
	m_pEnv = env;

	// only run the first time
	if (firstRun)
	{
		SetGCost(from, 0);
		firstRun = false;
	}

	// This code measures the size of the first heuristic minima that the agent passes over
	if (initialHeuristic)
	{
		double tmp;
		if ((tmp = BaseHCost(env, from, to)) > minInitialHeuristic)
		{
			initialHeuristic = false;
			maxLaterHeuristic = tmp;
		}
		else {
			minInitialHeuristic = tmp;
		}
	}
	else {
		maxLaterHeuristic = std::max(BaseHCost(env, from, to), maxLaterHeuristic);
	}
	// Done measurement
	
	Timer t;
	t.StartTimer();
	thePath.resize(0);
	if (from==to)
		return;
	
	astar.SetHeuristic(this);
	astar.InitializeSearch(env, from, to, thePath);
	astar.SetUseBPMX(1);
	for (int x = 0; x < nodeExpansionLimit; x++)
	{
		const state &tmp = astar.CheckNextNode();
		// update gcosts
		{
			env->GetSuccessors(tmp, succ);
			for (const auto &s : succ)
				SetGCost(s, std::min(GCost(s), GCost(tmp)+m_pEnv->GCost(tmp, s)));
		}
		if (tmp == to)
			break;
		
		astar.DoSingleSearchStep(thePath);
		//		if (astar.DoSingleSearchStep(thePath))
		//		{
		//			// return reversed path
		//			std::reverse(thePath.begin(), thePath.end());
		//			break;
		//		}
	}
	nodesExpanded = astar.GetNodesExpanded();
	nodesTouched = astar.GetNodesTouched();
	//	std::cout << GetName() << " " << nodesExpanded << " expanded by A* with expansion limit " << nodeExpansionLimit << std::endl;
	
	if (thePath.size() != 0)
		return;
	
	// 1. put all open nodes in pqueue
	typedef std::priority_queue<borderData<state>,std::vector<borderData<state> >,compareBorderData<state> > pQueue;
	pQueue q;
	
	double bestF = -1;
	double bestLearning = -1;
	state first;
	
	unsigned int openSize = astar.GetNumOpenItems();
	for (unsigned int x = 0; x < openSize; x++)
	{
		const auto data = astar.GetOpenItem(x);
		double currLearning = HCostLearned(data.data);
		if (avoidLearning)
		{
			if ((bestF == -1) ||
				(currLearning == 0 && bestLearning != 0) ||
				(currLearning == 0 && bestLearning == 0 && (fless(data.g+data.h, bestF))) ||
				(currLearning != 0 && bestLearning != 0 && (fless(data.g+data.h, bestF))))
			{
				bestLearning = currLearning;
				bestF = data.g+data.h;
				first = data.data;
			}
		}
		else {
			if ((bestF == -1) || (fless(data.g+data.h, bestF)))
			{
				bestF = data.g+data.h;
				first = data.data;
			}
			if (fequal(data.g+data.h, bestF))
			{
				if (GCost(data.data) > GCost(first))
				{
					first = data.data;
				}
			}
		}
		q.push(borderData<state>(data.data, data.h));
		if (verbose) std::cout << "Preparing border state: " << data.data << " h: " << data.h << std::endl;
	}
	
	std::vector<state> succ;
	ClosedList c;
	while (q.size() > 0)
	{
		nodesExpanded++;
		nodesTouched++;
		state s = q.top().theState;
		if (verbose) std::cout << "Starting with " << s << " h: " << q.top().heuristic << "/" << HCost(s, to) << std::endl;
		q.pop();
		//			std::cout << s << " " << learnData[env->GetStateHash(s)].learnedHeuristic << std::endl;
		env->GetSuccessors(s, succ);
		double hCost = HCost(s, to);
		for (unsigned int x = 0; x < succ.size(); x++)
		{
			nodesTouched++;
			double succHCost;
			if (!astar.GetClosedListGCost(succ[x], succHCost))
			{
				if (verbose) std::cout << succ[x] << " not in closed\n";
				continue;
			}
			double edgeCost = env->GCost(s, succ[x]);
			if (verbose) std::cout << s << " to " << succ[x] << " " << edgeCost << " ";
			succHCost = HCost(env, succ[x], to);
			if (c[env->GetStateHash(succ[x])]) // in closed list, but seen before, update if smaller
			{
				if (verbose) std::cout << succ[x] << " updated before ";
				if (fless(hCost + edgeCost, succHCost))
				{
					if (verbose) std::cout << "lowering cost to " << hCost + edgeCost << " from " << succHCost << std::endl;
					fAmountLearned = fAmountLearned - (succHCost - (hCost+edgeCost));
					if (verbose) std::cout << " learning now " << fAmountLearned;
					SetHCost(env, succ[x], to, hCost + edgeCost);
					q.push(borderData<state>(succ[x], hCost + edgeCost));
				}
				if (verbose) std::cout << std::endl;
			}
			else { // not expanded before and in closed list, always update
				// 2. expand successors, checking if they are in closed list and adding to queue
				//				if (fless(succHCost, hCost - edgeCost))
				if (verbose) std::cout << succ[x] << " NOT updated before ";
				//if (fgreater(hCost + edgeCost, succHCost))
				{
					if (verbose) std::cout << "setting cost to " << hCost + edgeCost << " over " << succHCost;
					fAmountLearned += (edgeCost + hCost) - succHCost;
					if (verbose) std::cout << " learning now " << fAmountLearned;
					SetHCost(env, succ[x], to, hCost + edgeCost);
					q.push(borderData<state>(succ[x], hCost + edgeCost));
					c[env->GetStateHash(succ[x])] = true;
				}
				if (verbose) std::cout << std::endl;
			}
		}
	}
	//std::cout << GetName() << " " << nodesExpanded-nodeExpansionLimit << " expanded during learning" << std::endl;
	
	//	if (thePath.size() != 0)
	//		return;
	
	// 3. construct best path
	astar.ExtractPathToStart(first, thePath);
	if (verbose) std::cout << "LSS-LRTA* heading towards " << thePath[0] << "with h-value " << HCost(env, thePath[0], to) << std::endl;
	t.EndTimer();
	//std::cout << GetName() << "\t" << nodesExpanded << "\t" << t.GetElapsedTime() << "\t" << nodesExpanded/t.GetElapsedTime() << std::endl;
}

//template <class state, class action, class environment>
//void gLSSLRTAStar<state, action, environment>::OpenGLDraw(const environment *e) const
//{
//	astar.OpenGLDraw();
//	
//	double learned = 0;
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double thisState = (*it).second.theHeuristic;
////		double thisState = (*it).second.gCost;
//		if (learned < thisState)
//			learned = thisState;
//	}
//	for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
//	{
//		double r = (*it).second.theHeuristic;
//		//double r = (*it).second.gCost;
//		if (r > 0)
//		{
//			e->SetColor(0.5+0.5*r/learned, 0, 0, 0.1+0.8*r/learned);
//			e->OpenGLDraw((*it).second.theState);
//		}
//	}
//}

#endif /* gLSSLRTAStar_h */
