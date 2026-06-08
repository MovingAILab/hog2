/*
 *  HeuristicLearningMeasure.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/5/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#ifndef HEURISTICLEARNINGMEASURE_H
#define HEURISTICLEARNINGMEASURE_H

#include "SearchEnvironment.h"
#include <unordered_map>
#include "TemplateAStar.h"
#include <iostream>
#include <queue>

template <class state>
class stateData {
public:
	stateData() :initialHeuristic(-1), learnedHeuristic(-1) {}
	state theState;
	double initialHeuristic;
	double learnedHeuristic;
};

template <class state>
class compareStateData
{
public:
	bool operator() (const stateData<state> &lhs, const stateData<state> &rhs) const
	{
		return (lhs.learnedHeuristic < rhs.learnedHeuristic);
	}
};

struct HashDouble { // 2 digits accuracy
	size_t operator()(const double &x) const
	{ return (size_t)(x*100); }
};

class sVal {
	public:
	sVal() :count(0) {}
	sVal(double v) :diff(v), count(0) {}
	void increase() {count++;}
	double diff; int count;
};

template <class state, class action, class environment>
class HeuristicLearningMeasure {
public:
	double MeasureDifficultly(environment *env, const state &start, const state &goal)
	{
		learnData.clear();
		queue.clear();
		BuildExactDistances(env, start, goal);
		ComputeRequiredLearning(env, start, goal);
		std::cout << "Intermediate learning: " << SumLearningRequired() <<
		" over " << learnData.size() << " states." << std::endl;
		ComputeConsistencyLearning(env, goal);
		std::cout << "Final learning: " << SumLearningRequired() << std::endl;
		return SumLearningRequired();
	}
	
	void ShowHistogram()
	{
		typedef std::unordered_map<double, sVal, HashDouble> HistogramData;
		HistogramData histogram;
		for (typename EnvironmentData::const_iterator it = learnData.begin(); it != learnData.end(); it++)
		{
			double diff = (*it).second.learnedHeuristic - (*it).second.initialHeuristic;
			histogram[diff].increase();
			histogram[diff].diff = diff;
		}
		for (typename HistogramData::const_iterator it = histogram.begin(); it != histogram.end(); it++)
		{
			printf("Histogram:\t%f\t%d\n", (*it).second.diff, (*it).second.count);
		}
	}
	
//	void OpenGLDraw(environment *env) const
//	{
//		double maxLearning = 0;
//		for (typename EnvironmentData::const_iterator it = learnData.begin(); it != learnData.end(); it++)
//		{
//			double cnt = (*it).second.learnedHeuristic - (*it).second.initialHeuristic;
//			if (cnt > maxLearning)
//				maxLearning = cnt;
//		}
//		
//		for (typename EnvironmentData::const_iterator it = learnData.begin(); it != learnData.end(); it++)
//		{
//			double r = (*it).second.learnedHeuristic - (*it).second.initialHeuristic;
//			if (r > 0)
//			{
//				env->SetColor(0.5+0.5*r/maxLearning, 0, 0, 0.1+0.8*r/maxLearning);
//				env->OpenGLDraw((*it).second.theState);
//			}
//		}
//		
//	}
	
private:
	typedef std::unordered_map<uint64_t, stateData<state>, Hash64 > EnvironmentData;

	double SumLearningRequired()
	{
		double sum = 0;
		for (typename EnvironmentData::const_iterator it = learnData.begin(); it != learnData.end(); it++)
		{
//			std::cout << "State " << (*it).second.theState << " initial heuristic: " << (*it).second.initialHeuristic <<
//			" learned: " << (*it).second.learnedHeuristic <<
//			" diff: " << (*it).second.learnedHeuristic - (*it).second.initialHeuristic << std::endl;
			sum += (*it).second.learnedHeuristic - (*it).second.initialHeuristic;
		}
		return sum;
	}

	void BuildExactDistances(environment *env, const state &start, const state &goal)
	{
		std::vector<state> thePath;

		astarStart.SetStopAfterGoal(false);
		astarStart.InitializeSearch(env, start, goal, thePath);
		//astarStart.GetPath(env, start, goal, path);

		astarGoal.SetStopAfterGoal(false);
		astarGoal.InitializeSearch(env, goal, start, thePath);
		//astarGoal.GetPath(env, goal, start, path);
	}

	double LookupGCost(state &s)
	{
		std::vector<state> thePath;
		double cost;
		while (astarStart.GetClosedListGCost(s, cost) == false)
		{
			for (unsigned int x = 0; x < 20; x++)
				astarStart.DoSingleSearchStep(thePath);
		}
		return cost;
	}

	double LookupHCost(const state &s)
	{
		std::vector<state> thePath;
		double cost;
		while (astarGoal.GetClosedListGCost(s, cost) == false)
		{
			for (unsigned int x = 0; x < 20; x++)
				astarGoal.DoSingleSearchStep(thePath);
		}
		return cost;
	}
	
	void ComputeRequiredLearning(environment *env, const state &start, const state &goal)
	{
		// 1. find the optimal cost
		// 2. find all nodes with optimal cost, update & put in list
		// 3. do BPMX stuff?

		double optimalCost = LookupHCost(start);
		
		std::vector<state> tempQueue;
		std::vector<state> succ;
		tempQueue.push_back(start);

//		learnData[env->GetStateHash(start)].theState = start;
//		learnData[env->GetStateHash(start)].learnedHeuristic = optimalCost;
//		learnData[env->GetStateHash(start)].initialHeuristic = env->HCost(start, goal);
		
		// add optimal path and neighbors to data structure
		while (tempQueue.size() > 0)
		{
			state s = tempQueue.back();
			tempQueue.pop_back();

			double gCost;
			double hCost;

			gCost = LookupGCost(s);
			hCost = LookupHCost(s);
//			std::cout << s << " has g: " << gCost << " h:" << hCost << std::endl;
//			if (!astarGoal.GetClosedListGCost(s, hCost))
//				std::cout << "Didn't find: " << s << std::endl;
//			if (!astarStart.GetClosedListGCost(s, gCost))
//				std::cout << "Didn't find: " << s << std::endl;

			// seen before
			if (!fequal(learnData[env->GetStateHash(s)].learnedHeuristic, -1))
				continue;

			learnData[env->GetStateHash(s)].theState = s;
			learnData[env->GetStateHash(s)].learnedHeuristic = hCost;
			learnData[env->GetStateHash(s)].initialHeuristic = env->HCost(s, goal);
			
			if (fequal(hCost+gCost, optimalCost))
			{
				env->GetSuccessors(s, succ);
				for (unsigned int y = 0; y < succ.size(); y++)
					tempQueue.push_back(succ[y]);
				//std::cout << "*";
			}
			//std::cout << s << " has distance cost " << hCost << " and h-cost " << env->HCost(s, goal) << std::endl;
		}

		// clear out memory used by A* searches
		TemplateAStar<state, action, environment> clear;
		astarGoal = clear;
		astarStart = clear;
	}

	void ComputeConsistencyLearning(environment *env, const state &goal)
	{
		typedef std::priority_queue<stateData<state>,std::vector<stateData<state> >,compareStateData<state> > pQueue;
		
		pQueue q;
		
		for (typename EnvironmentData::const_iterator it = learnData.begin(); it != learnData.end(); it++)
		{
			q.push((*it).second);
		}

		std::vector<state> succ;
		double learning = 0;
//		int cnt = 0;
		while (q.size() > 0)
		{
//			if (((cnt++)%1000) == 0)
//				std::cout << cnt << "\t" << q.size() << "\t" << learning << std::endl;
			state s = q.top().theState;
			q.pop();
//			std::cout << s << " " << learnData[env->GetStateHash(s)].learnedHeuristic << std::endl;
			env->GetSuccessors(s, succ);
			for (unsigned int x = 0; x < succ.size(); x++)
			{
				double edgeCost = env->GCost(s, succ[x]);
				
				if (fgreater(learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost,
							 env->HCost(succ[x], goal)))
				{
					// only store if we can update the default heuristic.
					if (learnData[env->GetStateHash(succ[x])].learnedHeuristic == -1)
					{
						learnData[env->GetStateHash(succ[x])].initialHeuristic = env->HCost(succ[x], goal);
						learnData[env->GetStateHash(succ[x])].learnedHeuristic = env->HCost(succ[x], goal);
						learnData[env->GetStateHash(succ[x])].theState = succ[x];
					}
					
					if (learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost-learnData[env->GetStateHash(succ[x])].learnedHeuristic > 0)
					{
						learning += learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost-learnData[env->GetStateHash(succ[x])].learnedHeuristic;
						learnData[env->GetStateHash(succ[x])].learnedHeuristic = learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost;
						q.push(learnData[env->GetStateHash(succ[x])]);
					}
				}
			}
		}
		
//		std::vector<state> succ;
//		double learning = 1;
//		while (learning > 0)
//		{
//			learning = 0;
//			for (typename EnvironmentData::const_iterator it = learnData.begin(); it != learnData.end(); it++)
//			{
//				state s = (*it).second.theState;
//				env->GetSuccessors(s, succ);
////				std::cout << "Generating successors of " << s << std::endl;
//				for (unsigned int x = 0; x < succ.size(); x++)
//				{
////					std::cout << "Successor " << x << " is " << succ[x] << std::endl;
//					
//					double edgeCost = env->GCost(s, succ[x]);
//					
//					if (fgreater(learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost,
//								 env->HCost(succ[x], goal)))
//					{
//						// only store if we can update the default heuristic.
//						if (learnData[env->GetStateHash(succ[x])].learnedHeuristic == -1)
//						{
//							learnData[env->GetStateHash(succ[x])].initialHeuristic = env->HCost(succ[x], goal);
//							learnData[env->GetStateHash(succ[x])].learnedHeuristic = env->HCost(succ[x], goal);
//							learnData[env->GetStateHash(succ[x])].theState = succ[x];
//						}
////						std::cout << s << " has heuristic " << learnData[env->GetStateHash(s)].learnedHeuristic << std::endl;
////						std::cout << succ[x] << " has heuristic " << learnData[env->GetStateHash(succ[x])].learnedHeuristic << std::endl;
////						std::cout << "Updating to " << learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost << std::endl;
//						if (learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost-learnData[env->GetStateHash(succ[x])].learnedHeuristic > 0)
//						{
//							learning += learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost-learnData[env->GetStateHash(succ[x])].learnedHeuristic;
//							learnData[env->GetStateHash(succ[x])].learnedHeuristic = learnData[env->GetStateHash(s)].learnedHeuristic-edgeCost;
//						}
//					}
//				}
//			}
//			std::cout << "Learned " << learning << " over " << learnData.size() << " states." << std::endl;
//		}
		std::cout << "Learned on " << learnData.size() << " states." << std::endl;
	}
		
	
	EnvironmentData learnData;
	TemplateAStar<state, action, environment> astarStart;
	TemplateAStar<state, action, environment> astarGoal;
	std::vector<state> queue;
	// hash table for all states containing initial heuristic, learned heuristic & optimal distance
	// 
};

#endif
