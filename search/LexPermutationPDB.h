//
//  LexPermutationPDB.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/10/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef LexPermutationPDB_h
#define LexPermutationPDB_h

#include "PermutationPDB.h"

/**
 * This class does the basic permutation calculation with a regular N^2 permutation
 * computation.
 */
template <class state, class action, class environment, int bits = 8>
class LexPermutationPDB : public PermutationPDB<state, action, environment, bits> {
public:
	LexPermutationPDB(environment *e, const state &s, const std::vector<int> &distincts);
	LexPermutationPDB(environment *e, const std::vector<state> &s, const std::vector<int> &distincts);
	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const { return GetPDBHash(s); }
	virtual state GetStateFromAbstractState(state &s) const { return s; }
	
	std::string GetFileName(const char *prefix);
protected:
	using PermutationPDB<state, action, environment, bits>::example;
	using PermutationPDB<state, action, environment, bits>::distinct;
	using PermutationPDB<state, action, environment, bits>::puzzleSize;

	uint64_t Factorial(int val) const;
	uint64_t FactorialUpperK(int n, int k) const;
	
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dualCache;
	mutable std::vector<std::vector<int> > locsCache;
};

template <class state, class action, class environment, int bits>
LexPermutationPDB<state, action, environment, bits>::LexPermutationPDB(environment *e, const state &s, const std::vector<int> &distincts)
:PermutationPDB<state, action, environment, bits>(e, s, distincts), dualCache(maxThreads), locsCache(maxThreads)
{
	this->SetGoal(s);
}

template <class state, class action, class environment, int bits>
LexPermutationPDB<state, action, environment, bits>::LexPermutationPDB(environment *e, const std::vector<state> &s, const std::vector<int> &distincts)
:PermutationPDB<state, action, environment, bits>(e, s[0], distincts), dualCache(maxThreads), locsCache(maxThreads)
{
	this->SetGoal(s);
}

template <class state, class action, class environment, int bits>
uint64_t LexPermutationPDB<state, action, environment, bits>::GetPDBHash(const state &s, int threadID) const
{
	std::vector<int> &locs = locsCache[threadID];
	std::vector<int> &dual = dualCache[threadID];
	// TODO: test definition
	locs.resize(distinct.size()); // vector for distinct item locations
	dual.resize(s.size()); // vector for distinct item locations
	
	// find item locations
	for (unsigned int x = 0; x < s.size(); x++)
	{
		if (s.puzzle[x] != -1)
			dual[s.puzzle[x]] = x;
	}
	for (size_t x = 0; x < distinct.size(); x++)
	{
		locs[x] = dual[distinct[x]];
	}
	
	uint64_t hashVal = 0;
	int numEntriesLeft = (int)s.size();
	
	for (unsigned int x = 0; x < locs.size(); x++)
	{
		hashVal += locs[x]*FactorialUpperK(numEntriesLeft-1, s.size()-distinct.size());
		numEntriesLeft--;
		
		// decrement locations of remaining items
		for (unsigned y = x; y < locs.size(); y++)
		{
			if (locs[y] > locs[x])
				locs[y]--;
		}
	}
	return hashVal;
}

template <class state, class action, class environment, int bits>
void LexPermutationPDB<state, action, environment, bits>::GetStateFromPDBHash(uint64_t hash, state &s, int threadID) const
{
	uint64_t hashVal = hash;
	std::vector<int> &dual = dualCache[threadID];
	
	dual.resize(distinct.size());
	
	int numEntriesLeft = puzzleSize-(int)(distinct.size())+1;
	for (int x = (int)distinct.size()-1; x >= 0; x--)
	{
		dual[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (size_t y = x+1; y < distinct.size(); y++)
		{
			if (dual[y] >= dual[x])
				dual[y]++;
		}
	}
	//	s.puzzle.resize(puzzleSize);
	std::fill(&s.puzzle[0], &s.puzzle[s.size()-1]+1, -1);
//	std::fill(s.puzzle.begin(), s.puzzle.end(), -1);
	for (int x = 0; x < dual.size(); x++)
		s.puzzle[dual[x]] = distinct[x];
	s.FinishUnranking();
}

//void GetStateFromHash(uint64_t hash, int *pieces, int count)
//{
//	int numEntriesLeft = 1;
//	for (int x = count-1; x >= 0; x--)
//	{
//		pieces[x] = hash%numEntriesLeft;
//		hash /= numEntriesLeft;
//		numEntriesLeft++;
//		for (int y = x+1; y < count; y++)
//		{
//			if (pieces[y] >= pieces[x])
//				pieces[y]++;
//		}
//	}
//}


template <class state, class action, class environment, int bits>
uint64_t LexPermutationPDB<state, action, environment, bits>::Factorial(int val) const
{
	static uint64_t table[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	if (val > 20)
		return (uint64_t)-1;
	return table[val];
}

template <class state, class action, class environment, int bits>
std::string LexPermutationPDB<state, action, environment, bits>::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += prefix;
	// For unix systems, the prefix should always end in a trailing slash
	if (fileName.back() != '/' && prefix[0] != 0)
		fileName+='/';
	fileName += PermutationPDB<state, action, environment, bits>::GetFileName("");
	fileName += "-lex.pdb";
	
	return fileName;
}

template <class state, class action, class environment, int bits>
uint64_t LexPermutationPDB<state, action, environment, bits>::FactorialUpperK(int n, int k) const
{
	uint64_t value = 1;
	assert(n >= 0 && k >= 0);
	
	for (int i = n; i > k; i--)
	{
		value *= i;
	}
	
	return value;
}


#endif /* LexPermutation_h */
