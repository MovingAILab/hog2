/*
 *  GraphCheck.h
 *  hog2
 *
 *  Created by Zhifu Zhang on 7/7/07.
 *
 */

#ifndef GRAPHCHECK_H
#define GRAPHCHECK_H

#include <stdint.h>
#include <vector>
#include <unordered_map>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "Graph.h"

template <typename T>
class SimpleNode {
public:
	SimpleNode() 
	{
		depth = (T)0;
		me = (T)0;
		parent = (T)0; 
	}
	SimpleNode(T m, T p, int d) 
	{
		depth = d;
		me = m;
		parent = p;
	}

	T parent;
	T me;
	int depth;
};


// Hash64 struct has been moved to SearchEnvironment.h


template <typename state, typename action>
class GraphCheck {
public:
	static void NumNodesWithinRadius(SearchEnvironment<state, action> &env, state from, int depth, int &inner_count, int &leaf_count);
	static void PathCountWithinRadius(SearchEnvironment<state, action> &env, state from, int depth, std::unordered_map<uint64_t, int, Hash64> &counts, std::unordered_map<uint64_t, double, Hash64> &aveCosts );

private:
	static void DFSVisit(SearchEnvironment<state, action> &env, std::vector<SimpleNode<state> > &thePath, int depth, std::unordered_map<uint64_t, int, Hash64> &counts, std::unordered_map<uint64_t, double, Hash64> &aveCosts, double gval);

};

template <typename state, typename action>
void GraphCheck<state, action>::NumNodesWithinRadius(SearchEnvironment<state, action> &env, state from, int depth, int &inner_count, int &leaf_count)
{
	// using BFS
	inner_count = 0;
	leaf_count = 0;
	std::queue<SimpleNode<state> > myqueue;
	std::unordered_map<uint64_t, SimpleNode<state>, Hash64> closedlist;

	std::vector<state> neighbors;

	SimpleNode<state> n0(from, from, 0);
	myqueue.push(n0);

	while(! myqueue.empty()) 
	{
		SimpleNode<state> frontN = myqueue.front();
		uint64_t frontID = env.GetStateHash(frontN.me);
		myqueue.pop();
	
		if (frontN.depth >= depth) 
		{
			leaf_count++;
			continue;
		}
		else 
		{
			inner_count++;
		}

		env.GetSuccessors(frontN.me, neighbors);

		for (unsigned int x = 0; x<neighbors.size(); x++)
		{
			state neighbor = neighbors[x];
			uint64_t neighborID = env.GetStateHash(neighbor);
			if (closedlist.find(neighborID) == closedlist.end())
			{
				//count++;

				SimpleNode<state> newNode(neighborID,frontID, frontN.depth+1);
				myqueue.push(newNode);
			}
		}

		closedlist[frontID] = frontN;
	}

	//return count;
}

template <typename state, typename action>
void GraphCheck<state, action>::PathCountWithinRadius(SearchEnvironment<state, action> &env, state from, int depth, std::unordered_map<uint64_t, int, Hash64> &counts, std::unordered_map<uint64_t, double, Hash64> &aveCosts )
{
	// using recursive version of DFS
	std::vector<SimpleNode<state> > thePath;

	SimpleNode<state> n0(from,from,0);
	counts[env.GetStateHash(from)]++;
	thePath.push_back(n0);

	DFSVisit(env, thePath,depth,counts,aveCosts,0);

	for (std::unordered_map<uint64_t,int, Hash64> ::iterator it = counts.begin(); it != counts.end(); it++)
	{
		if (it->second > 0)
			aveCosts[it->first] /= it->second;
	}
}

template <typename state, typename action>
void GraphCheck<state, action>::DFSVisit(SearchEnvironment<state, action> &env, std::vector<SimpleNode<state> > &thePath, int depth, std::unordered_map<uint64_t, int, Hash64> &counts, std::unordered_map<uint64_t, double, Hash64> &aveCosts, double gval)
{
	std::vector<state> neighbors;

	SimpleNode<state> current = thePath.back();
	if (current.depth >= depth)
		return;

	env.GetSuccessors(current.me, neighbors);

	for (unsigned int x = 0; x<neighbors.size(); x++)
	{
		state neighbor = neighbors[x];
		if (neighbor == current.parent)
			continue;

		bool flag = false;
		std::vector< SimpleNode<state> >::iterator iter;

		for (iter = thePath.begin(); iter != thePath.end(); iter++) 
		{
			if (neighbor == iter->me) 
			{
				flag = true; // this neighbor is in the path
				break;
			}
		}

		// this check is important! 
		if (flag)
			continue;

		uint64_t uniqueID = env.GetStateHash(neighbor);

		counts[uniqueID]++;
		aveCosts[uniqueID] += gval + GCost(current.me,neighbor);

		SimpleNode<state> sn(neighbor, current.me, current.depth+1);
		thePath.push_back(sn);
		DFSVisit(env, thePath, depth, counts, aveCosts, aveCosts[uniqueID]);  // recursion
		thePath.pop_back();
	}
}



#endif
