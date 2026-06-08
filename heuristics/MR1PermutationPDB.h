//
//  MR1PermutationPDB.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/14/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef MR1PermutationPDB_h
#define MR1PermutationPDB_h

#include "PermutationPDB.h"

/**
 * This class uses the first of two Myrvold-Russkey ranking functions
 * which run in linear time, but does not produce a lexicographical ranking.
 * Note that it produces the dual of their result, as this is necessary for
 * the partial ranking.
 *
 * For more details see:
 * http://webhome.cs.uvic.ca/~ruskey/Publications/RankPerm/MyrvoldRuskey.pdf
 */

template <class state, class action, class environment, int bits = 8>
class MR1PermutationPDB : public PermutationPDB<state, action, environment, bits> {
public:
	MR1PermutationPDB(environment *e, const state &s, const std::vector<int> &distincts);
	
	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const { return GetPDBHash(s); }
	virtual state GetStateFromAbstractState(state &s) const { return s; }

	std::string GetFileName(const char *prefix);
private:
	using PermutationPDB<state, action, environment, bits>::example;
	using PermutationPDB<state, action, environment, bits>::distinct;

	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dualCache;
	mutable std::vector<std::vector<int> > locsCache;
	mutable std::vector<std::vector<int> > valueStack;

};

template <class state, class action, class environment, int bits>
MR1PermutationPDB<state, action, environment, bits>::MR1PermutationPDB(environment *e, const state &s, const std::vector<int> & distincts)
:PermutationPDB<state, action, environment, bits>(e, s, distincts), dualCache(maxThreads), locsCache(maxThreads), valueStack(maxThreads)
{
	this->SetGoal(s);
}

inline void swap(int &a, int &b)
{
	int tmp = a;
	a = b;
	b = tmp;
}

template <class state, class action, class environment, int bits>
uint64_t MR1PermutationPDB<state, action, environment, bits>::GetPDBHash(const state &s, int threadID) const
{
	std::vector<int> &locs = locsCache[threadID];
	std::vector<int> &dual = dualCache[threadID];
	std::vector<int> &values = valueStack[threadID];
	locs.resize(example.size()); // vector for distinct item locations
	dual.resize(example.size()); // vector for distinct item locations
	values.resize(0);
	memset(&locs[0], 0xFF, locs.size()*sizeof(locs[0]));
	memset(&dual[0], 0xFF, dual.size()*sizeof(dual[0]));
	int puzzleSize = (int)example.size();

	// find current duals
	for (unsigned int x = 0; x < puzzleSize; x++)
	{
		if (s.puzzle[x] != -1)
			dual[s.puzzle[x]] = x;
	}
	// get locs by converting from the distinct array
	for (size_t x = 0; x < distinct.size(); x++)
	{
		locs[puzzleSize-x-1] = dual[distinct[distinct.size()-x-1]];
		//dual[distinct[distinct.size()-x-1]] = -1;
	}
	memset(&dual[0], 0xFF, dual.size()*sizeof(dual[0]));
	// get new duals for the actual locs (after conversion)
	for (int x = puzzleSize-distinct.size(); x < puzzleSize; x++)
	{
		dual[locs[x]] = x;
	}
	
	size_t last = puzzleSize-distinct.size();
	for (size_t i = puzzleSize; i > last; i--)
	{
		values.push_back(locs[i-1]); //val = locs[i-1];//get(perm, i-1);
		
		if (dual[i-1] != -1)
		{
			swap(locs[i-1], locs[dual[i-1]]);
			swap(dual[values.back()], dual[i-1]);
		}
		
		//		swap(perm, i-1, get(dual, i-1));
		//		swap(dual, s, i-1);
	}
	uint64_t result = 0;
	int cnt = last+1;
	while (values.size() > 0)
	{
		result *= cnt;
		result += values.back();
		values.pop_back();
		cnt++;
	}
	return result;
}

template <class state, class action, class environment, int bits>
void MR1PermutationPDB<state, action, environment, bits>::GetStateFromPDBHash(uint64_t hash, state &s, int threadID) const
{
	int puzzleSize = (int)example.size();
	//s.puzzle.resize(puzzleSize);
	std::vector<int> &dual = dualCache[threadID];
	dual.resize(puzzleSize); // vector for distinct item locations
	for (size_t x = 0; x < dual.size(); x++)
		dual[x] = x;
	
	size_t last = (puzzleSize-distinct.size());
	memset(&s.puzzle[0], 0xFF, puzzleSize*sizeof(s.puzzle[0]));
	
	for (size_t i = puzzleSize; i > last; i--)
	{
		swap(dual[hash%i], dual[i-1]);
		hash = hash/i;
		s.puzzle[dual[i-1]] = distinct[i-last-1];
	}
	s.FinishUnranking();
}

template <class state, class action, class environment, int bits>
std::string MR1PermutationPDB<state, action, environment, bits>::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += prefix;
	// For unix systems, the prefix should always end in a trailing slash
	if (fileName.back() != '/' && prefix[0] != 0)
		fileName+='/';
	fileName += PermutationPDB<state, action, environment, bits>::GetFileName("");
	fileName += "-MR1.pdb";
	
	return fileName;
}


#endif /* MR1PermutationPDB_h */
