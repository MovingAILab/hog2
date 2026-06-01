//
//  GridLSSLRTAStar.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/6/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef hog2_glut_GridLSSLRTAStar_h
#define hog2_glut_GridLSSLRTAStar_h

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
namespace GridLRTA {
	
	static bool verbose = false;
	
	class borderData {
	public:
		borderData(const xyLoc &s, const double h) :theState(s), heuristic(h) {}
		xyLoc theState;
		double heuristic;
	};
	
	class compareBorderData
	{
	public:
		bool operator() (const borderData &lhs, const borderData &rhs) const
		{
			return (lhs.heuristic > rhs.heuristic);
		}
	};
	
	struct lssLearnedData {
		lssLearnedData() { theHeuristic = 0; dead = false; }
#ifndef NO_OPENGL
		xyLoc theState;
#endif
		double theHeuristic;
		bool dead;
	};
	
	class GridLRTAStar : public LearningAlgorithm<xyLoc,tDirection,MapEnvironment>, public Heuristic<xyLoc> {
	public:
		GridLRTAStar(int nodeLimit = 8)
		{ fAmountLearned = 0.0f; nodeExpansionLimit = nodeLimit; m_pEnv = 0; }
		virtual ~GridLRTAStar(void) { }
		
		void GetPath(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath);
		
		void GetPath(MapEnvironment *, const xyLoc& , const xyLoc& , std::vector<tDirection> & ) { assert(false); };
		virtual const char *GetName()
		{ 
			static char name[255];
			sprintf(name, "GridLRTAStar(%d)", nodeExpansionLimit); return name;
		}
		void SetHCost(MapEnvironment *env, const xyLoc &where, const xyLoc &to, double val)
		{
			heur[env->GetStateHash(where)].theHeuristic = val-env->HCost(where, to);
#ifndef NO_OPENGL
			heur[env->GetStateHash(where)].theState = where;
#endif
		}
		double HCost(MapEnvironment *env, const xyLoc &from, const xyLoc &to) const
		{
			auto val = heur.find(env->GetStateHash(from));
			if (val != heur.end())
			{
				return val->second.theHeuristic+env->HCost(from, to);
			}
			return env->HCost(from, to);
//			if (heur.find(env->GetStateHash(from)) != heur.end())
//				return heur[env->GetStateHash(from)].theHeuristic+
//				env->HCost(from, to);
//			return env->HCost(from, to);
		}
		double HCost(const xyLoc &from, const xyLoc &to) const
		{
			assert(m_pEnv != 0);
			return HCost(m_pEnv, from, to);
		}
		bool IsDead(const MapEnvironment *env, const xyLoc &from) const
		{
			if (heur.find(env->GetStateHash(from)) != heur.end())
				return (heur.find(env->GetStateHash(from)))->second.dead;
			//return heur[env->GetStateHash(from)].dead;
			return false;
		}
		void Kill(MapEnvironment *env, const xyLoc &s)
		{
			heur[env->GetStateHash(s)].dead = true;
#ifndef NO_OPENGL
			heur[env->GetStateHash(s)].theState = s;
#endif
		}
		
		virtual uint64_t GetNodesExpanded() const { return nodesExpanded; }
		virtual uint64_t GetNodesTouched() const { return nodesTouched; }
		virtual void LogFinalStats(StatCollection *s)
		{ s->AddStat("TotalLearning", GetName(),fAmountLearned); }
		
		double GetAmountLearned() { return fAmountLearned; }
	  //void OpenGLDraw() const {}
	  //void OpenGLDraw(const MapEnvironment *env) const;
	private:
		typedef std::priority_queue<borderData,std::vector<borderData >,compareBorderData > pQueue;
		typedef std::unordered_map<uint64_t, lssLearnedData, Hash64 > LearnedHeuristic;
		typedef std::unordered_map<uint64_t, bool, Hash64 > ClosedList;
		
		
		void ExpandLSS(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &thePath);
		void BuildLSSQ(MapEnvironment *env, pQueue &q, xyLoc &best, const xyLoc &target);
		void LearnHeuristic(MapEnvironment *env, pQueue &q, const xyLoc &to);
		void TestKilling(MapEnvironment *env, const xyLoc &curr);
		bool TryKill(MapEnvironment *env, const xyLoc &from, const xyLoc &myLoc);


		MapEnvironment *m_pEnv;
		LearnedHeuristic heur;
		double fAmountLearned;
		uint64_t nodesExpanded, nodesTouched;
		int nodeExpansionLimit;
		//TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
		AStarOpenClosed<xyLoc, AStarCompare<xyLoc> > aoc;

		xyLoc startState, goal;
	};
	
	/** The core routine of GridLRTAStar -- computes at most one-move path */
	void GridLRTAStar::GetPath(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath)
	{
		nodesExpanded = 0;
		nodesTouched = 0;
		goal = to;
		if (m_pEnv == 0)
			startState = from;
		Timer t;
		t.StartTimer();
		if (m_pEnv == 0)
		{
			m_pEnv = env; //new MapDeadEnvironment(env, this);
		}
		thePath.resize(0);
		if (from==to)
			return;
		
		if (thePath.size() != 0)
			return;
		
		pQueue q1, q2;
		xyLoc first;
		ExpandLSS(env, from, to, thePath);
		
		BuildLSSQ(env, q1, first, goal);
		LearnHeuristic(env, q1, goal);

		// 2. learn dead states
		TestKilling(env, from);

		// re-find best
		BuildLSSQ(env, q2, first, goal);
		
		// 3. construct best path
		uint64_t node;
		aoc.Lookup(env->GetStateHash(first), node);
		do {
			thePath.push_back(aoc.Lookup(node).data);
			node = aoc.Lookup(node).parentID;
		} while (aoc.Lookup(node).parentID != node);
		thePath.push_back(aoc.Lookup(node).data);

		//astar.ExtractPathToStart(first, thePath);
		if (verbose) std::cout << "LSS-LRTA* heading towards " << thePath[0] << "with h-value " << HCost(env, thePath[0], to) << std::endl;
		t.EndTimer();
		//std::cout << GetName() << "\t" << nodesExpanded << "\t" << t.GetElapsedTime() << "\t" << nodesExpanded/t.GetElapsedTime() << std::endl;
	}
	
	void GridLRTAStar::ExpandLSS(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &thePath)
	{
		aoc.Reset();
		aoc.AddOpenNode(from, m_pEnv->GetStateHash(from), 0.0, 0.0);
		
		for (int x = 0; x < nodeExpansionLimit; x++)//nodeLearningLimit
		{
			if (aoc.OpenSize() == 0)
				break;
			if (aoc.Lookat(aoc.Peek()).data == to)
				break;
			// 1. choose next node to expand
			uint64_t next = aoc.Close();
			if (verbose) std::cout << "Next in LSS: " << aoc.Lookat(next).data << std::endl;
			
			// 2. propagate g-costs to neighbors and beyond if this node isn't dead
			if (IsDead(env, aoc.Lookat(next).data))// && x != 0)
			{
				assert(false); // should never have dead nodes on list
				x--;
				if (verbose) std::cout << aoc.Lookat(next).data << " is dead; left on closed (LSS)" << std::endl;
				continue;
			}
			else {
				// cannot pass data from AOC by reference, because when data structures are
				// resized it can become invalidated
				xyLoc val = aoc.Lookat(next).data;
				std::vector<xyLoc> neighbors;
				
				m_pEnv->GetSuccessors(val, neighbors);
				nodesExpanded++;
				uint64_t parentKey;
				dataLocation pLoc = aoc.Lookup(m_pEnv->GetStateHash(val), parentKey);
				assert(pLoc == kClosedList);
				
				for (unsigned int x = 0; x < neighbors.size(); x++)
				{
					nodesTouched++;
					if (IsDead(env, neighbors[x]))
						continue;

					double edgeCost = env->GCost(val, neighbors[x]);
					uint64_t childKey;
					//std::cout << "Hashing state:10 " << std::endl << neighbors[x] << std::endl;
					dataLocation cLoc = aoc.Lookup(env->GetStateHash(neighbors[x]), childKey);
					
					if (cLoc == kNotFound)
					{
						if (verbose) std::cout << "Adding " << neighbors[x] << " to open with f:" << 
							aoc.Lookat(parentKey).g+edgeCost + HCost(neighbors[x], to) << std::endl;
						aoc.AddOpenNode(neighbors[x], env->GetStateHash(neighbors[x]),
										aoc.Lookat(parentKey).g+edgeCost,
										//cost,
										HCost(neighbors[x], to), parentKey);
					}
					else if (cLoc == kOpenList)
					{
						if (fless(aoc.Lookup(parentKey).g+edgeCost, aoc.Lookup(childKey).g))
						{
							if (verbose) std::cout << "Updating " << neighbors[x] << " on open" << std::endl;
							aoc.Lookup(childKey).parentID = parentKey;
							aoc.Lookup(childKey).g = aoc.Lookup(parentKey).g+edgeCost;
							aoc.KeyChanged(childKey);
						}
					}
					else if (cLoc == kClosedList)
					{
						continue;
						//oassert(false);
					}
				}
			}
		}
		
//		astar.InitializeSearch(env, from, to, thePath);
//		astar.SetHeuristic(this);
//		astar.SetUseBPMX(1);
//		for (int x = 0; x < nodeExpansionLimit; x++)
//		{
//			if (astar.CheckNextNode() == to) // never expand goal
//				break;
//			astar.DoSingleSearchStep(thePath);
//		}
//		nodesExpanded = astar.GetNodesExpanded();
//		nodesTouched = astar.GetNodesTouched();
	}
	
	void GridLRTAStar::BuildLSSQ(MapEnvironment *env, pQueue &q, xyLoc &first, const xyLoc &target)
	{
		// 1. put all open nodes in pqueue
		double bestF = -1;
		
		unsigned long openSize = aoc.OpenSize();
		for (unsigned int x = 0; x < openSize; x++)
		{
			const AStarOpenClosedData<xyLoc> data = aoc.Lookup(aoc.GetOpenItem(x));
			if (IsDead(env, data.data))
				continue;
			if ((bestF == -1) || (fless(data.g+data.h, bestF)))
			{
				bestF = data.g+data.h;
				first = data.data;
			}
			double h = HCost(env, data.data, target);
			q.push(borderData(data.data, h));
			if (verbose) std::cout << "Preparing border xyLoc: " << data.data << " h: " << h << std::endl;
		}
	}
	
	void GridLRTAStar::LearnHeuristic(MapEnvironment *env, pQueue &q, const xyLoc &to)
	{
		std::vector<xyLoc> succ;
		ClosedList c;
		while (q.size() > 0)
		{
			nodesExpanded++;
			nodesTouched++;
			xyLoc s = q.top().theState;
			if (verbose) std::cout << "Starting with " << s << " h: " << q.top().heuristic << "/" << HCost(env, s, to) << std::endl;
			q.pop();
			//			std::cout << s << " " << learnData[env->GetStateHash(s)].learnedHeuristic << std::endl;
			env->GetSuccessors(s, succ);
			double hCost = HCost(env, s, to);
			for (unsigned int x = 0; x < succ.size(); x++)
			{
				nodesTouched++;
				double succHCost;
				uint64_t key;
				if (aoc.Lookup(env->GetStateHash(succ[x]), key) == kNotFound)
					continue;
				if (aoc.Lookat(key).where != kClosedList)
					continue;
				
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
						q.push(borderData(succ[x], hCost + edgeCost));
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
						q.push(borderData(succ[x], hCost + edgeCost));
						c[env->GetStateHash(succ[x])] = true;
					}
					if (verbose) std::cout << std::endl;
				}
			}
		}
	}
	
	void GridLRTAStar::TestKilling(MapEnvironment *env, const xyLoc &myLoc)
	{
		int cnt = aoc.size();
		std::vector<xyLoc> killed;
		// try to kill states
		for (int x = 0; x < aoc.size(); x++)
		{
			const AStarOpenClosedData<xyLoc> data = aoc.Lookat(x);
			if (TryKill(env, data.data, myLoc) == true)
			{
				cnt--;
				killed.push_back(data.data);
			}
			if (cnt == 1)
				return;
		}
		std::vector<xyLoc> succ;
		while (killed.size() > 0)
		{
			xyLoc next = killed.back();
			killed.pop_back();
			env->GetSuccessors(next, succ);
			nodesExpanded++;
			for (unsigned int x = 0; x < succ.size(); x++)
			{
				if (cnt == 1)
					return;

				uint64_t key;
				// if (1) in aoc (2) not already dead
				if ((aoc.Lookup(env->GetStateHash(succ[x]), key) != kNotFound) &&
					!IsDead(env, succ[x]))
				{
					if (TryKill(env, succ[x], myLoc))
					{
						killed.push_back(succ[x]);
						cnt--;
					}
				}
			}
		}
	}
	
	bool GridLRTAStar::TryKill(MapEnvironment *env, const xyLoc &from, const xyLoc &myLoc)
	{
		nodesExpanded++;
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
		//if (passable <= 4 && changes <= 2 && !(from == startState) && !(from == myLoc))
		if (passable <= 4 && changes <= 2 && !(from == startState) && !(from == goal))
		{
			Kill(env, from);
			return true;
		}
		return false;
	}
  /*	
	void GridLRTAStar::OpenGLDraw(const MapEnvironment *e) const
	{
#ifndef NO_OPENGL
		//astar.OpenGLDraw();
		for (int x = 0; x < aoc.size(); x++)
		{
			const AStarOpenClosedData<xyLoc> data = aoc.Lookat(x);
			if (IsDead(e, data.data))
			{
				e->SetColor(1.0, 0.0, 1.0);
				//e->OpenGLDraw(data.data);
			}
			else if (data.where == kOpenList)
			{
				e->SetColor(0.0, 1.0, 0.0);
				//e->OpenGLDraw(data.data);
			}
			else if (data.where == kClosedList)
			{
				e->SetColor(0.0, 0.0, 1.0);
				//e->OpenGLDraw(data.data);
			}
			else {
				e->SetColor(1.0, 1.0, 0.0);
				//e->OpenGLDraw(data.data);
			}
		}
		//return;
		
		double learned = 1;
		// typename was here; not sure if older compilers still need it
		for (LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
		{
			double thisState = (*it).second.theHeuristic;
			if (learned < thisState)
			{
				learned = thisState;
			}
		}
		//printf("Max learning: %f/%f\n", learned[0], learned[1]);
		for (LearnedHeuristic::const_iterator it = heur.begin(); it != heur.end(); it++)
		{
			double r = (*it).second.theHeuristic;
			if ((*it).second.dead)
			{
				e->SetColor(0.0, 0.0, 0.0);
				//e->OpenGLDraw((*it).second.theState);
			}
			else if (r > 0)
			{
				e->SetColor(0.5+0.5*r/learned, 0.0, 0.0, 0.2+0.5*r/learned);
				//e->OpenGLDraw((*it).second.theState);
			}
		}
#endif
	}
  */	
}


#endif
