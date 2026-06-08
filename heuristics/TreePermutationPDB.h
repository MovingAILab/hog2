//
//  TreePermutationPDB.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/19/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef TreePermutationPDB_h
#define TreePermutationPDB_h

#include "PermutationPDB.h"

/**
 * This class does the basic permutation calculation in lexicographical order.
 * It uses the tree permutation scheme from Bonet
 * http://aaaipress.org/Papers/Workshops/2008/WS-08-10/WS08-10-004.pdf
 * to do this in n log n time instead of n^2 time.
 */
template <class state, class action, class environment, int bits = 8>
class TreePermutationPDB : public PermutationPDB<state, action, environment, bits> {
public:
	TreePermutationPDB(environment *e, const state &s, const std::vector<int> &distincts);
	
	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const { return GetPDBHash(s); }
	virtual state GetStateFromAbstractState(state &s) { return s; }

	std::string GetFileName(const char *prefix);
private:
	using PermutationPDB<state, action, environment, bits>::example;
	using PermutationPDB<state, action, environment, bits>::distinct;

	uint64_t Factorial(int val) const;
	uint64_t FactorialUpperK(int n, int k) const;
	
	const int maxThreads = 32;

	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dualCache;
	mutable std::vector<std::vector<int> > locsCache;
	mutable std::vector<std::vector<int> > tempCache;

	state example;
};

inline int mylog2(int val)
{
	switch (val)
	{
		case 1: return 0;
		case 2: return 1;
		case 3: return 2;
		case 4: return 2;
		case 5: return 3;
		case 6: return 3;
		case 7: return 3;
		case 8: return 3;
		case 9: return 4;
		case 10:return 4;
		case 11:return 4;
		case 12:return 4;
		case 13:return 4;
		case 14:return 4;
		case 15:return 4;
		case 16:return 4;
		case 17:return 5;
		case 18:return 5;
		case 19:return 5;
		case 20:return 5;
		case 21:return 5;
		case 22:return 5;
		case 23:return 5;
		case 24:return 5;
		case 25:return 5;
		case 26:return 5;
		case 27:return 5;
		case 28:return 5;
		case 29:return 5;
		case 30:return 5;
		case 31:return 5;
		case 32:return 5;
		default:return 6;
	}
}


template <class state, class action, class environment, int bits>
TreePermutationPDB<state, action, environment, bits>::TreePermutationPDB(environment *e, const state &s, const std::vector<int> &distincts)
:PermutationPDB<state, action, environment, bits>(e, s, distincts),
dualCache(maxThreads), locsCache(maxThreads), tempCache(maxThreads)
{
	this->SetGoal(s);
}

template <class state, class action, class environment, int bits>
uint64_t TreePermutationPDB<state, action, environment, bits>::GetPDBHash(const state &s, int threadID) const
{
	std::vector<int> &values = locsCache[threadID];
	std::vector<int> &dual = dualCache[threadID];
	// TODO: test definition
	values.resize(distinct.size()); // vector for distinct item locations
	dual.resize(s.puzzle.size()); // vector for distinct item locations
	
	// find item locations
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		if (s.puzzle[x] != -1)
			dual[s.puzzle[x]] = x;
	}
	for (int x = 0; x < distinct.size(); x++)
	{
		values[x] = dual[distinct[x]];
	}
	
	uint64_t rank = 0;
	int k = mylog2(s.puzzle.size());
	//static std::vector<int> temp;
	std::vector<int> &temp = tempCache[threadID];

	temp.resize((1<<(1+k))-1);
	std::fill(temp.begin(), temp.end(), 0);
	for (int x = 0; x < values.size(); x++)
	{
		int counter = values[x];
		int node = (1<<k)-1+values[x];
		for (int y = 0; y < k; y++)
		{
			int isEven = (1-(node&1));
			counter -= isEven*(temp[(node-1)>>1] - temp[node]);
			//			if (0 == node%2) // right node
			//				counter -= (temp[(node-1)>>1] - temp[node]);
			temp[node]++;
			node = (node-1)>>1;
		}
		temp[node]++;
		rank = rank*(s.puzzle.size()-x)+counter;
	}
	
	return rank;
}

template <class state, class action, class environment, int bits>
void TreePermutationPDB<state, action, environment, bits>::GetStateFromPDBHash(uint64_t hash, state &s, int threadID) const
{
	size_t count = example.puzzle.size();
	s.puzzle.resize(count);
	int k = mylog2(count);
	std::vector<int> &temp = tempCache[threadID];
	temp.resize((1<<(1+k))-1);
	for (int x = 0; x < temp.size(); x++)
		temp[x] = (1<<(k-mylog2(x+2)+1));
	
	std::vector<int> &values = locsCache[threadID];
	values.resize(distinct.size());
	int numEntriesLeft = s.puzzle.size()-distinct.size()+1;
	for (int x = (int)values.size()-1; x >= 0; x--)
	{
		values[x] = hash%numEntriesLeft;
		hash /= numEntriesLeft;
		numEntriesLeft++;
	}
	memset(&s.puzzle[0], 0xFF, s.puzzle.size()*sizeof(s.puzzle[0]));
	for (int x = 0; x < values.size(); x++)
	{
		int digit = values[x];
		int node = 0;
		for (int y = 0; y < k; y++)
		{
			temp[node]--;
			node = ((node+1)<<1)-1;
			uint32_t diff = digit-temp[node];
			diff = (~diff)>>31;
			digit -= diff*temp[node];
			node += diff;
			//			if (digit >= temp[node])
			//			{
			//				digit -= temp[node];
			//				node++;
			//			}
		}
		temp[node] = 0;
		values[x] = node - (1<<k) + 1;
		s.puzzle[values[x]] = distinct[x];
	}
	s.FinishUnranking();
}

template <class state, class action, class environment, int bits>
uint64_t TreePermutationPDB<state, action, environment, bits>::Factorial(int val) const
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
std::string TreePermutationPDB<state, action, environment, bits>::GetFileName(const char *prefix)
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
uint64_t TreePermutationPDB<state, action, environment, bits>::FactorialUpperK(int n, int k) const
{
	uint64_t value = 1;
	assert(n >= 0 && k >= 0);
	
	for (int i = n; i > k; i--)
	{
		value *= i;
	}
	
	return value;
}


#endif /* TreePermutationPDB_h */
