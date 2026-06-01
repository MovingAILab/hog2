//
//  FLRTAStar2.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 12/19/11.
//  Copyright (c) 2011 University of Denver. All rights reserved.
//

#ifndef hog2_glut_FLRTAStar2_h
#define hog2_glut_FLRTAStar2_h

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
namespace FLRTA2 {
	
	static bool verbose = false;
	
	template <class state>
	class borderData {
	public:
		borderData(const state &s, const double h) :theState(s), heuristic(h) {}
		state theState;
		double heuristic;
	};
	
	template <class state>
	class compareBorderData
	{
	public:
		bool operator() (const borderData<state> &lhs, const borderData<state> &rhs) const
		{
			return (lhs.heuristic > rhs.heuristic);
		}
	};
	
	template <class state>
	struct lssLearnedData {
		lssLearnedData() { theHeuristic[0] = theHeuristic[1] = 0; }
#ifndef NO_OPENGL
		state theState;
#endif
		double theHeuristic[2];
	};

	template <class state, class action, class environment>
	class FLRTAStar2 : public LearningAlgorithm<state,action,environment>, public Heuristic<state> {
	public:
		FLRTAStar2(int nodeLimit = 8)
		{ fAmountLearned = 0.0f; nodeExpansionLimit = nodeLimit; m_pEnv = 0; }
		virtual ~FLRTAStar2(void) { }
		
		void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
		
		void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
		virtual const char *GetName()
		{ 
			static char name[255];
			sprintf(name, "FLRTAStar2(%d)", nodeExpansionLimit); return name;
		}
		void SetHCost(environment *env, const state &where, const state &to, double val)
		{
			int ID = GetGoalID(to);
			heur[env->GetStateHash(where)].theHeuristic[ID] = val-env->HCost(where, to);
#ifndef NO_OPENGL
			heur[env->GetStateHash(where)].theState = where;
#endif
		}
		double HCost(environment *env, const state &from, const state &to) const
		{
			int ID = GetGoalID(to);
			auto val = heur.find(env->GetStateHash(from));
			if (val != heur.end())
			{
				return val->second.theHeuristic[ID]+env->HCost(from, to);
			}
			return env->HCost(from, to);

			
//			int ID = GetGoalID(to);
//			if (heur.find(env->GetStateHash(from)) != heur.end())
//				return heur[env->GetStateHash(from)].theHeuristic[ID]+
//				env->HCost(from, to);
//			return env->HCost(from, to);
		}
		double HCost(const state &from, const state &to) const
		{
			assert(m_pEnv != 0);
			return HCost(m_pEnv, from, to);
		}
		
		virtual uint64_t GetNodesExpanded() const { return nodesExpanded; }
		virtual uint64_t GetNodesTouched() const { return nodesTouched; }
		virtual void LogFinalStats(StatCollection *s)
		{ s->AddStat("TotalLearning", GetName(),fAmountLearned); }
		
		double GetAmountLearned() { return fAmountLearned; }
		void OpenGLDraw() const {}
		void OpenGLDraw(const environment *env) const;
	private:
		typedef std::priority_queue<borderData<state>,std::vector<borderData<state> >,compareBorderData<state> > pQueue;
		typedef std::unordered_map<uint64_t, lssLearnedData<state>, Hash64 > LearnedHeuristic;
		typedef std::unordered_map<uint64_t, bool, Hash64 > ClosedList;
		

		bool ExpandLSS(environment *env, const state &from, const state &to, std::vector<state> &thePath);
		void BuildLSSQ(environment *env, pQueue &q, state &best, const state &target);
		void LearnHeuristic(environment *env, pQueue &q, const state &to);
		int GetGoalID(const state &which) const
		{
			for (unsigned int x = 0; x < goals.size(); x++)
				if (goals[x] == which)
					return x;
			assert(false);
		}
		
		environment *m_pEnv;
		LearnedHeuristic heur;
		double fAmountLearned;
		uint64_t nodesExpanded, nodesTouched;
		int nodeExpansionLimit;
		TemplateAStar<state, action, environment> astar;

		std::vector<state> goals;
	};
	
	const int kFrom = 1;
	const int kTo = 0;

	/** The core routine of FLRTAStar2 -- computes at most one-move path */
	template <class state, class action, class environment>
	void FLRTAStar2<state, action, environment>::GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath)
	{
		if (goals.size() == 0)
		{
			goals.push_back(to);
			goals.push_back(from);
		}
		Timer t;
		t.StartTimer();
		m_pEnv = env;
		thePath.resize(0);
		if (from==to)
			return;
		
		if (thePath.size() != 0)
			return;
		
		pQueue q1, q2;
		state first;
		bool learng = ExpandLSS(env, from, to, thePath);
		
		BuildLSSQ(env, q1, first, goals[kTo]);
		LearnHeuristic(env, q1, goals[kTo]);

		if (learng)
		{
			BuildLSSQ(env, q2, first, goals[kFrom]);
			LearnHeuristic(env, q2, goals[kFrom]);
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
	
	template <class state, class action, class environment>
	bool FLRTAStar2<state, action, environment>::ExpandLSS(environment *env, const state &from, const state &to, std::vector<state> &thePath)
	{
		bool expandedStart = false;
		astar.InitializeSearch(env, from, to, thePath);
		astar.SetHeuristic(this);
		astar.SetUseBPMX(1);
		for (int x = 0; x < nodeExpansionLimit; x++)
		{
			if (astar.CheckNextNode() == to) // never expand goal
				break;
			if (astar.CheckNextNode() == goals[kFrom])
				expandedStart = true;
			astar.DoSingleSearchStep(thePath);
		}
		nodesExpanded = astar.GetNodesExpanded();
		nodesTouched = astar.GetNodesTouched();
		return !expandedStart;
	}
	
	template <class state, class action, class environment>
	void FLRTAStar2<state, action, environment>::BuildLSSQ(environment *env, pQueue &q, state &first, const state &target)
	{
		// 1. put all open nodes in pqueue
		double bestF = -1;
		
		unsigned int openSize = astar.GetNumOpenItems();
		for (unsigned int x = 0; x < openSize; x++)
		{
			const auto data = astar.GetOpenItem(x);
			if ((bestF == -1) || (fless(data.g+data.h, bestF)))
			{
				bestF = data.g+data.h;
				first = data.data;
			}
			double h = HCost(env, data.data, target);
			q.push(borderData<state>(data.data, h));
			if (verbose) std::cout << "Preparing border state: " << data.data << " h: " << h << std::endl;
		}
		double hCost;
		if (astar.GetClosedListGCost(target, hCost))
		{
			q.push(borderData<state>(target, 0));
		}
	}
	
	template <class state, class action, class environment>
	void FLRTAStar2<state, action, environment>::LearnHeuristic(environment *env, pQueue &q, const state &to)
	{
		std::vector<state> succ;
		ClosedList c;
		while (q.size() > 0)
		{
			nodesExpanded++;
			nodesTouched++;
			state s = q.top().theState;
			if (verbose) std::cout << "Starting with " << s << " h: " << q.top().heuristic << "/" << HCost(env, s, to) << std::endl;
			q.pop();
			//			std::cout << s << " " << learnData[env->GetStateHash(s)].learnedHeuristic << std::endl;
			env->GetSuccessors(s, succ);
			double hCost = HCost(env, s, to);
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
	}

	
	template <class state, class action, class environment>
	void FLRTAStar2<state, action, environment>::OpenGLDraw(const environment *e) const
	{
		astar.OpenGLDraw();
		
		double learned[2] = {1, 1};
		for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
		{
			for (int x = 0; x < 2; x++)
			{
				double thisState = (*it).second.theHeuristic[x];
				if (learned[x] < thisState)
					learned[x] = thisState;
			}
		}
		//printf("Max learning: %f/%f\n", learned[0], learned[1]);
		for (typename LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
		{
			double r = (*it).second.theHeuristic[0];
			double b = (*it).second.theHeuristic[1];
			if (r > 0 || b > 0)
			{
				//e->SetColor(0.5+0.5*r/learned[0], 0, 0.5+0.5*b/learned[1], 0.5+0.25*r/learned[0]+0.25*b/learned[1]);
				e->SetColor(0.5+0.5*r/learned[0], 0, 0.0+1.0*b/learned[1], 0.2+0.4*r/learned[0]+0.4*b/learned[1]);
				//e->SetColor(0.0, 0.0, 0.0+1.0*b/learned[1], 1.0*b/learned[1]);
#ifndef NO_OPENGL
				e->OpenGLDraw((*it).second.theState);
#endif
			}
		}
	}
	
}

#endif
