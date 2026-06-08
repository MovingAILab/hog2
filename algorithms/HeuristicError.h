//
//  HeuristicError.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/24/18.
//  Copyright © 2018 University of Denver. All rights reserved.
//

#ifndef HeuristicError_h
#define HeuristicError_h

#include "Heuristic.h"

// This function counts the number of states that match the functional criteria on the heuristic when
// regressing the given distance and then doing a forward search the forward distance.
// For instance, HR2 could be tested by regressing 3, forward 2, and testing for 0 heuristics.
// Returns the percentage of states at the regression distance that have a child that meet the criteria
template <class environment, class state, typename func>
float MeasureHeuristicErrors(environment *e, state s, Heuristic<state> *h, int regressionDistance, int forwardDistance, func f)
{
//	std::cout << "Processing " << s << "\n";
	// Store states and their depth
	std::unordered_map<state, int> table;
	std::vector<state> succ;
	table[s] = 0;
	for (int x = 0; x < regressionDistance; x++)
	{
		for (auto iter = table.begin(); iter != table.end(); iter++)
		{
			// Expands states at current depth
			if (iter->second == x)
			{
				e->GetSuccessors(iter->first, succ);
				for (auto &itm : succ)
				{
					if (table.find(itm) == table.end())
					{
//						std::cout << "Adding (" << itm << ") at depth " << x+1 << "\n";
						table[itm] = x+1;
					}
				}
			}
		}
	}

	std::function<int (const state&, int)> DFS;
	DFS = [h,&s,&f,e,&DFS,&table](const state &i, int depth)->int {
		if (depth == 0)
			return f(h->HCost(i, s))?1:0;
		std::vector<state> neighbors;
		e->GetSuccessors(i, neighbors);
		int count = 0;
		for (const state &succ : neighbors)
		{
//			if (table.find(succ) != table.end())
//				continue;
			count += DFS(succ, depth-1);
		}
		return count;
	};

	float total = 0;
	float pass = 0;
	float children = 0;
	for (auto iter = table.begin(); iter != table.end(); iter++)
	{
		// Expands states at current depth
		if (iter->second == regressionDistance)
		{
			total++;
			int res = DFS(iter->first, forwardDistance);
			if (res > 0)
				pass++;
			children += res;
		}
	}
	printf("%d of %d match condition. Avg %1.2f per state\n", (int)pass, (int)total, children/total);
	return pass/total;
}

#endif /* HeuristicError_h */
