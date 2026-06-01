//
//  MR1Permutation.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 6/19/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef MR1Permutation_h
#define MR1Permutation_h

#include <stdio.h>
#include <vector>
#include <stdint.h>

class MR1KPermutation {
public:
	uint64_t Rank(int *items, int *dual, int k, int N) const;
	void Unrank(uint64_t hash, int *items, int *dual, int k, int N) const;
	uint64_t Rank(uint8_t *items, uint8_t *dual, int k, int N) const;
};


//class MR1Permutation {
//public:
//	MR1Permutation(std::vector<int> distincts, int permSize, int maxNumThreads);
//	uint64_t GetMaxRank() const;
//	uint64_t GetRank(const std::vector<int> &items, int threadID = 0) const;
//	uint64_t GetRank(const std::vector<int> &items, int threadID = 0);
//	void Unrank(uint64_t hash, std::vector<int> &items, int threadID = 0) const;
//private:
//	std::vector<int> distinct;
//	size_t puzzleSize;
//	size_t distinctSize;
//	uint64_t maxRank;
//
//	// cache for computing ranking/unranking
//	mutable std::vector<std::vector<int> > dualCache;
//	mutable std::vector<std::vector<int> > locsCache;
//	mutable std::vector<std::vector<int> > valueStack;
//};

#endif /* MR1Permutation_h */
