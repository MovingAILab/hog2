//
//  MR1Permutation.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 6/19/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include "MR1Permutation.h"
#include <assert.h>
#include <string.h>

template <typename T>
inline void swap(T &a, T &b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

uint64_t MR1KPermutation::Rank(int *locs, int *dual, int distinctSize, int puzzleSize) const
{
	uint64_t result2 = 0;
	uint64_t multiplier = 1;
	for (int i = 0; i < distinctSize; i++)
	{
		int tmp = dual[i];
		unsigned int tmp2 = locs[i];
		
		result2 += (tmp-i)*multiplier;
		multiplier *= (puzzleSize-i);
		
		if (tmp2 < puzzleSize)
		{
			swap(locs[i], locs[dual[i]]);
			swap(dual[tmp2], dual[i]);
		}
	}
	
	return result2;
}

uint64_t MR1KPermutation::Rank(uint8_t *locs, uint8_t *dual, int distinctSize, int puzzleSize) const
{
	uint64_t result2 = 0;
	uint64_t multiplier = 1;
	for (int i = 0; i < distinctSize; i++)
	{
		int tmp = dual[i];
		unsigned int tmp2 = locs[i];
		
		result2 += (tmp-i)*multiplier;
		multiplier *= (puzzleSize-i);
		
		if (tmp2 < puzzleSize)
		{
			swap(locs[i], locs[dual[i]]);
			swap(dual[tmp2], dual[i]);
		}
	}
	
	return result2;
}

/**
 * Given the hash returns the state and its dual
 */
void MR1KPermutation::Unrank(uint64_t hash, int *puzzle, int *dual, int distinctSize, int puzzleSize) const
{
//	std::vector<int> &dual = dualCache[threadID];
//	dual.resize(puzzleSize); // vector for distinct item locations
	for (int x = 0; x < puzzleSize; x++)
		dual[x] = x;

	int last = (puzzleSize-distinctSize);
	memset(puzzle, 0xFF, puzzleSize*sizeof(puzzle[0]));

	for (int i = 0; i < distinctSize; i++)
	{
		swap(dual[i+hash%(puzzleSize-i)], dual[i]);
		hash = hash/(puzzleSize-i);
		puzzle[dual[i]] = i;//distinct[i-last-1];
		//printf("Setting %d to be %d\n", dual[i-1], i-last-1);
	}
}

//MR1Permutation::MR1Permutation(std::vector<int> distincts, int permSize, int maxNumThreads)
//:distinct(distincts), puzzleSize(permSize), distinctSize(distincts.size()),
// dualCache(maxNumThreads), locsCache(maxNumThreads), valueStack(maxNumThreads)
//{
//	maxRank = 1;
//	for (int x = (int)puzzleSize; x > puzzleSize-distincts.size(); x--)
//	{
//		maxRank *= x;
//	}
//}
//
//uint64_t MR1Permutation::GetMaxRank() const
//{
//	return maxRank;
//}
//
//inline void swap(int &a, int &b)
//{
//	int tmp = a;
//	a = b;
//	b = tmp;
//}
//
//uint64_t MR1Permutation::GetRank(const std::vector<int> &puzzle, int threadID) const
//{
//	std::vector<int> &locs = locsCache[threadID];
//	std::vector<int> &dual = dualCache[threadID];
//	std::vector<int> &values = valueStack[threadID];
//	locs.resize(puzzleSize); // vector for distinct item locations
//	dual.resize(puzzleSize); // vector for distinct item locations
//	values.resize(0);
//	memset(&locs[0], 0xFF, locs.size()*sizeof(locs[0]));
//	memset(&dual[0], 0xFF, dual.size()*sizeof(dual[0]));
//	
//	// find current duals
//	for (unsigned int x = 0; x < puzzleSize; x++)
//	{
//		if (puzzle[x] != -1)
//			dual[puzzle[x]] = x;
//	}
//	// get locs by converting from the distinct array
//	for (int x = 0; x < distinctSize; x++)
//	{
//		locs[puzzleSize-x-1] = dual[distinct[distinctSize-x-1]];
//		//dual[distinct[distinctSize-x-1]] = -1;
//	}
//	memset(&dual[0], 0xFF, dual.size()*sizeof(dual[0]));
//	// get new duals for the actual locs (after conversion)
//	for (int x = puzzleSize-distinctSize; x < puzzleSize; x++)
//	{
//		dual[locs[x]] = x;
//	}
//	
//	size_t last = puzzleSize-distinctSize;
//	for (size_t i = puzzleSize; i > last; i--)
//	{
//		values.push_back(locs[i-1]); //val = locs[i-1];//get(perm, i-1);
//		
//		if (dual[i-1] != -1)
//		{
//			swap(locs[i-1], locs[dual[i-1]]);
//			swap(dual[values.back()], dual[i-1]);
//		}
//	}
//	uint64_t result = 0;
//	int cnt = last+1;
//	while (values.size() > 0)
//	{
//		result *= cnt;
//		result += values.back();
//		values.pop_back();
//		cnt++;
//	}
//	return result;
//}
//
//void MR1Permutation::Unrank(uint64_t hash, std::vector<int> &puzzle, int threadID) const
//{
//	puzzle.resize(puzzleSize);
//	std::vector<int> &dual = dualCache[threadID];
//	dual.resize(puzzleSize); // vector for distinct item locations
//	for (int x = 0; x < dual.size(); x++)
//		dual[x] = x;
//	
//	size_t last = (puzzleSize-distinctSize);
//	memset(&puzzle[0], 0xFF, puzzleSize*sizeof(puzzle[0]));
//	
//	for (size_t i = puzzleSize; i > last; i--)
//	{
//		swap(dual[hash%i], dual[i-1]);
//		hash = hash/i;
//		puzzle[dual[i-1]] = distinct[i-last-1];
//	}
//}
