//
//  Permutations.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/8/21.
//  Copyright © 2021 NS Software. All rights reserved.
//

#ifndef Permutations_h
#define Permutations_h

#include "Combinations.h"

// Standard N^2 algorithm for a permutation of N elements
template <int N>
class Permutations {
public:
	uint64_t MaxRank() const { return ConstFactorial(N); }
	uint64_t Rank(const int *items) const
	{
		uint64_t hashVal = 0;
		// local copy that can be modified
		int values[N];
		for (unsigned int x = 0; x < N; x++)
		{
			values[x] = items[x];
		}

		for (unsigned int x = 0; x < N; x++)
		{
			hashVal += values[x]*ConstFactorial(N-1-x);
			for (unsigned y = x; y < N; y++)
			{
				if (values[y] > values[x])
					values[y]--;
			}
		}
		return hashVal;
	}
	
	void Unrank(uint64_t hash, int *items) const
	{
		for (int x = 0; x < N; x++)
		{
			items[x] = hash/ConstFactorial(N-1-x);
			hash = hash%ConstFactorial(N-1-x);
		}
		for (int x = N-2; x >= 0; x--)
		{
			for (int y = x+1; y < N; y++)
			{
				if (items[y] >= items[x])
					items[y]++;
			}
		}

	}
};


#endif /* Permutations_h */
