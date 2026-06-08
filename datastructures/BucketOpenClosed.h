//
//  BucketOpenClosed.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/16/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef hog2_glut_BucketOpenClosed_h
#define hog2_glut_BucketOpenClosed_h

#include "AStarOpenClosed.h"
#include <list>

template<typename state, typename CmpKey, class dataStructure = AStarOpenClosedData<state> >
class BucketOpenClosed {
public:
	BucketOpenClosed();
	~BucketOpenClosed();
	void Reset();
	uint64_t AddOpenNode(const state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddClosedNode(state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	void KeyChanged(uint64_t objKey);
	//void IncreaseKey(uint64_t objKey);
	dataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	uint64_t Peek() const;
	uint64_t Close();
	void Reopen(uint64_t objKey);
	
	uint64_t GetOpenItem(unsigned int which);
	size_t OpenSize() const { return openCount; }
	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }
	//	void verifyData();
	void Print();
private:
	void FindNewMin();
	uint64_t Add(uint64_t key, uint64_t fCost);
	void Remove(uint64_t key, uint64_t fCost);
	int openCount;
	int minBucket, minSubBucket;
	struct qData {
		qData() { fCost = -1; }
		std::vector<uint64_t> entries;
		int fCost;
	};
	std::vector<qData> pQueue;
	// storing the element id; looking up with...hash?
	typedef std::unordered_map<uint64_t, uint64_t, AHash64> IndexTable;
	IndexTable table;
	std::vector<dataStructure > elements;
};

template<typename state, typename CmpKey, class dataStructure>
BucketOpenClosed<state, CmpKey, dataStructure>::BucketOpenClosed()
{
	openCount = 0;
	minBucket = -1;
}

template<typename state, typename CmpKey, class dataStructure>
BucketOpenClosed<state, CmpKey, dataStructure>::~BucketOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey, class dataStructure>
void BucketOpenClosed<state, CmpKey, dataStructure>::Reset()
{
	openCount = 0;
	minBucket = -1;
	table.clear();
	elements.clear();
	pQueue.resize(0);
	//pQueue.resize(3);
}

//inline uint64_t compactGH(uint64_t g, uint64_t h)
//{
//	return (g<<32)|h;
//}
//
//inline uint64_t extractG(uint64_t value)
//{
//	return value>>32;
//}
//
//inline uint64_t extractH(uint64_t value)
//{
//	return (value)&0xFFFFFFFF;
//}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t BucketOpenClosed<state, CmpKey, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double f, double g, double h, uint64_t parent)
{
	// should do lookup here...
	if (table.find(hash) != table.end())
	{
		//return -1; // TODO: find correct id and return
		assert(false);
	}
	openCount++;
//	uint64_t newg = g;
//	uint64_t newh = h;
	uint64_t loc = Add(elements.size(), f);
	elements.push_back(dataStructure(val, f, g, h, parent, loc, kOpenList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	FindNewMin();
	//printf("Added node to buckets %" PRId64 "/%" PRId64 "\n", newg+newh, newh);
	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t BucketOpenClosed<state, CmpKey, dataStructure>::AddClosedNode(state &val, uint64_t hash, double f, double g, double h, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, f, g, h, parent, 0, kClosedList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size(); // hashing to element list location
	return elements.size()-1;
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey, class dataStructure>
void BucketOpenClosed<state, CmpKey, dataStructure>::KeyChanged(uint64_t val)
{
//	uint64_t g, h;
//	g = extractG(elements[val].openLocation);
//	h = extractH(elements[val].openLocation);
	//printf("-----\nChanging key of [%" PRId64 "][%" PRId64 "] to [%1.0f][%1.0f]\n", g+h, h, elements[val].g+elements[val].h, elements[val].h);
	//Print();
	Remove(val, elements[val].openLocation);
//	int len = pQueue[g+h][0].size();
//	pQueue[g+h][0].remove(val);
//	assert(pQueue[g+h][0].size() == len-1);
//	int len = pQueue[g+h][h].size();
//	pQueue[g+h][h].remove(val);
//	assert(pQueue[g+h][h].size() == len-1);
	
//	g = elements[val].g;
//	h = elements[val].h;
//	elements[val].openLocation = compactGH(g, h);
//	if (pQueue.size() < (g+h+1))
//		pQueue.resize((int)(g+h+1));
//	if (pQueue[g+h].size() < /*h+*/1)
//		pQueue[g+h].resize((int)/*h+*/1);
//	pQueue[g+h][0].push_back(val);
	elements[val].openLocation = Add(val, elements[val].g+elements[val].h);
	//pQueue[g+h][h].push_back(val);
	//printf("Removed and re-added node to buckets %" PRId64 "/%" PRId64 "\n", g+h, h);

//	FindNewMin();
	//Print();
}

/**
 * Returns location of object as well as object key.
 */
template<typename state, typename CmpKey, class dataStructure>
dataLocation BucketOpenClosed<state, CmpKey, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	typename IndexTable::const_iterator it;
	it = table.find(hashKey);
	if (it != table.end())
	{
		objKey = (*it).second;
		return elements[objKey].where;
	}
	return kNotFound;
}


/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t BucketOpenClosed<state, CmpKey, dataStructure>::Peek() const
{
	assert(OpenSize() != 0);

	assert(pQueue[minBucket].entries.size() > 0);
	return pQueue[minBucket].entries.back();
//	assert(pQueue[minBucket][minSubBucket].size() > 0);
//	return pQueue[minBucket][minSubBucket].front();
//	return theHeap[0];
}

/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t BucketOpenClosed<state, CmpKey, dataStructure>::Close()
{
	assert(OpenSize() != 0);
	assert(pQueue[minBucket].entries.size() > 0);
	openCount--;
	
	uint64_t ans = pQueue[minBucket].entries.back();
	pQueue[minBucket].entries.pop_back();
	if (pQueue[minBucket].entries.size() == 0)
		pQueue[minBucket].fCost = -1;
	elements[ans].where = kClosedList;
	//printf("Removed item from %" PRId64 "/%" PRId64 "\n", minBucket, minSubBucket);
	FindNewMin();
	
	return ans;
}

/**
 * Move item off the closed list and back onto the open list.
 */
template<typename state, typename CmpKey, class dataStructure>
void BucketOpenClosed<state, CmpKey, dataStructure>::Reopen(uint64_t objKey)
{
	openCount++;
	uint64_t g, h;
	g = elements[objKey].g;
	h = elements[objKey].h;
	assert(elements[objKey].where == kClosedList);
	elements[objKey].reopened = true;
	elements[objKey].where = kOpenList;
	elements[objKey].openLocation = Add(objKey, g+h);
}

template<typename state, typename CmpKey, class dataStructure>
void BucketOpenClosed<state, CmpKey, dataStructure>::FindNewMin()
{
	//assert(OpenSize() > 0);
	if (OpenSize() == 0)
	{ minBucket = -1; return; }

	minBucket = -1;
	for (int x = 0; x < pQueue.size(); x++)
	{
		while (pQueue[x].entries.size() > 0 && pQueue[x].entries.back() == -1)
		{
			pQueue[x].entries.pop_back();
		}
		if (pQueue[x].entries.size() == 0)
		{
			pQueue[x].fCost = -1;
		}
		else {
			if (minBucket == -1)
			{
				minBucket = x;
				continue;
			}
			else {
				if (pQueue[x].fCost < pQueue[minBucket].fCost)
					minBucket = x;
			}
		}
	}
}

template<typename state, typename CmpKey, class dataStructure>
uint64_t BucketOpenClosed<state, CmpKey, dataStructure>::GetOpenItem(unsigned int which)
{
	for (unsigned int x = 0; x < elements.size(); x++)
	{
		if (elements[x].where == kOpenList)
		{
			if (which == 0)
				return x;
			which--;
		}
		return -1;
	}
	return -1;
}

template<typename state, typename CmpKey, class dataStructure>
void BucketOpenClosed<state, CmpKey, dataStructure>::Print()
{
	int cnt = 0;
	printf("**pQueue[%d] items [%d, %d]:\n", OpenSize(), minBucket, minSubBucket);
	for (unsigned int x = 0; x < pQueue.size(); x++)
	{
		for (unsigned int y = 0; y < pQueue[x].size(); y++)
		{
			if (pQueue[x][y].size() > 0)
				printf("**[%d][%d] has %d elements\n", x, y, pQueue[x][y].size());
			cnt += pQueue[x][y].size();
		}
	}
	if (cnt != OpenSize())
	{
		assert(!"ERROR; sizes inconsistent");
	}
}

template<typename state, typename CmpKey, class dataStructure>
uint64_t BucketOpenClosed<state, CmpKey, dataStructure>::Add(uint64_t key, uint64_t fCost)
{
	for (int x = 0; x < pQueue.size(); x++)
	{
		if (pQueue[x].fCost == fCost)
		{
			pQueue[x].entries.push_back(key);
			FindNewMin();
			return pQueue[x].entries.size()-1;
		}
	}
	for (int x = 0; x < pQueue.size(); x++)
	{
		if (pQueue[x].fCost == -1)
		{
			pQueue[x].fCost = fCost;
			pQueue[x].entries.push_back(key);
			FindNewMin();
			return pQueue[x].entries.size()-1;
		}
	}
	assert(!"No space found to insert element");
}

template<typename state, typename CmpKey, class dataStructure>
void BucketOpenClosed<state, CmpKey, dataStructure>::Remove(uint64_t key, uint64_t location)
{
	for (int x = 0; x < pQueue.size(); x++)
	{
		if (pQueue[x].entries.size() > location && pQueue[x].entries[location] == key)
		{
			if (location == pQueue[x].entries.size()-1)
			{
				pQueue[x].entries.pop_back();
			}
			else {
				pQueue[x].entries[location] = -1;
				if (pQueue[x].entries.size() == 1)
					pQueue[x].fCost = -1;
			}
			FindNewMin();
			return;
		}
	}
	assert(!"Element not found to remove");
}

#endif
