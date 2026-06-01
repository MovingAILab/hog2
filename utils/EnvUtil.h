//
//  EnvUtil.h
//  Rubik
//
//  Created by Nathan Sturtevant on 4/7/13.
//  Copyright (c) 2013 Nathan Sturtevant. All rights reserved.
//

#ifndef __Rubik__EnvUtil__
#define __Rubik__EnvUtil__

#include <iostream>
#include <vector>
#include <cassert>
#include <stdint.h>
#include <cstdio>
#include <cinttypes>

const int openSize = 256;
const int numBuckets = 2;

struct bucketInfo {
	bool unused;
	int bucketID;
	int64_t numEntries;
	int64_t bucketOffset;
};

struct bucketData {
	int64_t theSize;
#ifndef DISK
	std::vector<uint8_t> data;
#endif
};

struct bucketChanges {
	bucketChanges() { 	pthread_mutex_init(&lock, NULL); }
	bool updated;
	int currDepthWritten;
	int lastDepthWritten;
	std::vector<bool> changes;
	//	std::vector<bool> roundChanges;
	std::vector<bool> nextChanges;
	pthread_mutex_t lock;
};

template <class Environment, class State>
void InitTwoPieceData(std::vector<bucketInfo> &data,
					  uint64_t maxBucketSize);

template <class Environment, class State>
void InitBucketSize(std::vector<bucketData> &buckets,
					 uint64_t maxBucketSize);

template <class Environment, class State>
void InitChanges(std::vector<bucketData> &buckets,
				 std::vector<bucketChanges> &twoPieceChanges,
				 uint64_t maxBucketSize);

template <class Environment, class State>
uint64_t GetMaxBucketSize(bool print);

//-// template defintions //-//

template <class Environment, class State>
void InitTwoPieceData(std::vector<bucketInfo> &data, uint64_t maxBucketSize)
{
	Environment cc;
	State s;
	
	int64_t maxVal2 = cc.getMaxSinglePlayerRank2();
	int64_t maxRank = cc.getMaxSinglePlayerRank();
	
	data.resize(maxVal2);
	
	uint64_t bucketSize = 0;
	
	int currBucket = 0;
	
	// build data sets
	for (int64_t x = 0; x < maxRank; x++)
	{
		cc.unrankPlayer(x, s, 0);
		
		int64_t r1;
		cc.rankPlayerFirstTwo(s, 0, r1);
		
		int64_t maxValOther = cc.getMaxSinglePlayerRank2(r1);
		x+=maxValOther-1;
		
		if (bucketSize > maxBucketSize)
		{
			bucketSize = 0;
			currBucket++;
		}
		data[r1].unused = false;
		data[r1].bucketID = currBucket;
		data[r1].numEntries = maxValOther;
		data[r1].bucketOffset = bucketSize;
		bucketSize += maxValOther;
		// 64-bit align data in buckets
		while (bucketSize%openSize != 0)
			bucketSize++;
	}
}

template <class Environment, class State>
void InitBucketSize(std::vector<bucketData> &buckets,
					uint64_t maxBucketSize)
{
	Environment cc;
	State s;
	
	int64_t maxRank = cc.getMaxSinglePlayerRank();
		
	//	uint64_t bucketSize = 0;
	buckets.resize(1);
	buckets[0].theSize = 0;
	
	int currBucket = 0;
	
	// build data sets
	for (int64_t x = 0; x < maxRank; x++)
	{
		cc.unrankPlayer(x, s, 0);
		
		int64_t r1, r2, r3, r4;
		cc.rankPlayer(s, 0, r1, r2);
		cc.rankPlayerFirstTwo(s, 0, r3);
		cc.rankPlayerRemaining(s, 0, r4);
		assert(r1 == r3);
		assert(r2 == r4);
		
		int64_t maxValOther = cc.getMaxSinglePlayerRank2(r1);
		x+=maxValOther-1;
		
		if (buckets[currBucket].theSize > maxBucketSize)
		{
			currBucket++;
			buckets.resize(buckets.size()+1);
			buckets[currBucket].theSize = 0;
		}
		buckets[currBucket].theSize += maxValOther;

		// 64-bit align data in buckets
		while (buckets[currBucket].theSize%openSize != 0)
			buckets[currBucket].theSize++;
	}
}

template <class Environment, class State>
void InitChanges(std::vector<bucketData> &buckets,
				 std::vector<bucketChanges> &twoPieceChanges,
				 uint64_t maxBucketSize)
{
	Environment cc;
	State s;
	
	int64_t maxVal2 = cc.getMaxSinglePlayerRank2();
	int64_t maxRank = cc.getMaxSinglePlayerRank();
	
	twoPieceChanges.resize(maxVal2);
	
	//	uint64_t bucketSize = 0;
	buckets.resize(1);
	buckets[0].theSize = 0;
	
	int currBucket = 0;
	
	// build data sets
	for (int64_t x = 0; x < maxRank; x++)
	{
		cc.unrankPlayer(x, s, 0);
		
		int64_t r1, r2, r3, r4;
		cc.rankPlayer(s, 0, r1, r2);
		cc.rankPlayerFirstTwo(s, 0, r3);
		cc.rankPlayerRemaining(s, 0, r4);
		assert(r1 == r3);
		assert(r2 == r4);
		
		int64_t maxValOther = cc.getMaxSinglePlayerRank2(r1);
		x+=maxValOther-1;
		
		if (buckets[currBucket].theSize > maxBucketSize)
		{
			currBucket++;
			buckets.resize(buckets.size()+1);
			buckets[currBucket].theSize = 0;
		}
		twoPieceChanges[r1].lastDepthWritten = -1;
		twoPieceChanges[r1].currDepthWritten = -1;
		twoPieceChanges[r1].updated = false;
		twoPieceChanges[r1].changes.resize((maxValOther+openSize-1)/openSize);
		//		twoPieceChanges[r1].roundChanges.resize((maxValOther+openSize-1)/openSize);
		twoPieceChanges[r1].nextChanges.resize((maxValOther+openSize-1)/openSize);
		buckets[currBucket].theSize += maxValOther;
		// 64-bit align data in buckets
		while (buckets[currBucket].theSize%openSize != 0)
			buckets[currBucket].theSize++;
		//numStatesLeft += maxValOther;
	}
}

template <class Environment, class State>
uint64_t GetMaxBucketSize(bool print)
{
	uint64_t maxBucketSize;
	State s;
	Environment cc;
	int64_t maxRank = cc.getMaxSinglePlayerRank();
	//int64_t maxVal2 = cc.getMaxSinglePlayerRank2();
	uint64_t statesLeft = 0;
	
	// count total states left to solve
	for (int64_t x = 0; x < maxRank; x++)
	{
		cc.unrankPlayer(x, s, 0);
		
		int64_t r1;
		cc.rankPlayerFirstTwo(s, 0, r1);
		
		int64_t maxValOther = cc.getMaxSinglePlayerRank2(r1);
		x+=maxValOther-1;
		
		statesLeft += maxValOther;
	}
	
	maxBucketSize = (statesLeft-2)/numBuckets; // leave space for extra storage needed to align things
	//	maxBucketSize *= 3;
	if (print)
	{
		printf("%" PRId64 " total states. Using 2 buckets, each with about %" PRId64 " entries\n",
			   statesLeft, maxBucketSize);
	}
	return maxBucketSize;
}

#endif /* defined(__Rubik__EnvUtil__) */
