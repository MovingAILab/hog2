//
//  PermutationPDB.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/27/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_PermutationPDB_h
#define hog2_glut_PermutationPDB_h

#include "PDBHeuristic.h"

/**
 * This class does the basic permutation calculation with a regular N^2 permutation
 * computation.
 */
template <class state, class action, class environment, int bits = 8>
class PermutationPDB : public PDBHeuristic<state, action, environment, state, bits> {
public:
	PermutationPDB(environment *e, const state &s, const std::vector<int> &distincts);

	virtual uint64_t GetPDBSize() const;

	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const = 0;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const = 0;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const = 0;
	virtual state GetStateFromAbstractState(state &s) const = 0;

	bool Load(FILE *f);
	void Save(FILE *f);
	bool Load(const char *prefix);
	void Save(const char *prefix);
	virtual std::string GetFileName(const char *prefix);
private:
	uint64_t Factorial(int val) const;
	uint64_t FactorialUpperK(int n, int k) const;

protected:
	std::vector<int> distinct;
	size_t puzzleSize;
	uint64_t pdbSize;
	state example;
};

template <class state, class action, class environment, int bits>
PermutationPDB<state, action, environment, bits>::PermutationPDB(environment *e, const state &s, const std::vector<int> &distincts)
:PDBHeuristic<state, action, environment, state, bits>(e), distinct(distincts), puzzleSize(s.size()), example(s)
{
	pdbSize = 1;
	for (int x = (int)s.size(); x > s.size()-distincts.size(); x--)
	{
		pdbSize *= x;
	}
}

template <class state, class action, class environment, int bits>
uint64_t PermutationPDB<state, action, environment, bits>::GetPDBSize() const
{
	return pdbSize;
}


template <class state, class action, class environment, int bits>
std::string PermutationPDB<state, action, environment, bits>::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += prefix;
	// For unix systems, the prefix should always end in a trailing slash
	if (fileName.back() != '/' && prefix[0] != 0)
		fileName+='/';
	fileName += PDBHeuristic<state, action, environment, state, bits>::env->GetName();
	fileName += "-";
	for (size_t x = 0; x < PDBHeuristic<state, action, environment, state, bits>::goalState.size(); x++)
	{
		fileName += std::to_string(PDBHeuristic<state, action, environment, state, bits>::goalState[0].puzzle[x]);
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
	fileName += "-";
	for (size_t x = 0; x < distinct.size(); x++)
	{
		fileName += std::to_string(distinct[x]);
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
	fileName += "-";
	fileName += std::to_string(bits);
	fileName += "bpe";
	return fileName;
}

template <class state, class action, class environment, int bits>
bool PermutationPDB<state, action, environment, bits>::Load(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "r+");
	if (f == 0)
	{
		printf("Unable to open for loading '%s'\n", GetFileName(prefix).c_str());
		return false;
	}
	bool result = Load(f);
	fclose(f);
	return result;
}

template <class state, class action, class environment, int bits>
void PermutationPDB<state, action, environment, bits>::Save(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
	if (f == 0)
	{
		fprintf(stderr, "Error saving");
		return;
	}
	Save(f);
	fclose(f);
}

template <class state, class action, class environment, int bits>
bool PermutationPDB<state, action, environment, bits>::Load(FILE *f)
{
	if (PDBHeuristic<state, action, environment, state, bits>::Load(f) != true)
	{
		return false;
	}
	if (fread(&puzzleSize, sizeof(puzzleSize), 1, f) != 1)
		return false;
	if (fread(&pdbSize, sizeof(pdbSize), 1, f) != 1)
		return false;
	if (fread(&example, sizeof(example), 1, f) != 1)
		return false;
	size_t distinctSize = distinct.size();
	if (fread(&distinctSize, sizeof(distinctSize), 1, f) != 1)
		return false;
	distinct.resize(distinctSize);
	if (fread(&distinct[0], sizeof(distinct[0]), distinct.size(), f) != distinctSize)
		return false;
	return true;
}

template <class state, class action, class environment, int bits>
void PermutationPDB<state, action, environment, bits>::Save(FILE *f)
{
	PDBHeuristic<state, action, environment, state, bits>::Save(f);
	fwrite(&puzzleSize, sizeof(puzzleSize), 1, f);
	fwrite(&pdbSize, sizeof(pdbSize), 1, f);
	fwrite(&example, sizeof(example), 1, f);
	size_t distinctSize = distinct.size();
	fwrite(&distinctSize, sizeof(distinctSize), 1, f);
	fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f);
}

template <class state, class action, class environment, int bits>
uint64_t PermutationPDB<state, action, environment, bits>::FactorialUpperK(int n, int k) const
{
	uint64_t value = 1;
	assert(n >= 0 && k >= 0);
	
	for (int i = n; i > k; i--)
	{
		value *= i;
	}
	
	return value;
}

#endif
