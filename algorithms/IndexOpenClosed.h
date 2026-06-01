//
//  IndexOpenClosed.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/18/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef IndexOpenClosed_h
#define IndexOpenClosed_h

/**
 * This open/closed list is designed for state spaces where the hash is an index that
 * is small enough that we can create a table for all indices. This avoids the need for
 * a hash table for looking up states - they can be looked up directly.
 *
 */

#include "AStarOpenClosed.h"

template<typename state>
class IndexOpenClosedData {
public:
	IndexOpenClosedData() {}
	IndexOpenClosedData(const state &theData, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location) { reopened = false; }
	state data;
	double g;
	double h;
	uint64_t parentID;
	uint64_t openLocation;
	uint64_t round;
	bool reopened;
	dataLocation where;
};

template<typename state>
class IndexOpenClosedDataWithF {
public:
	IndexOpenClosedDataWithF() {}
	IndexOpenClosedDataWithF(const state &theData, double fCost, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), f(fCost), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location) { reopened = false; }
	state data;
	double f;
	double g;
	double h;
	uint64_t parentID;
	uint64_t openLocation;
	uint64_t round;
	bool reopened;
	dataLocation where;
};


template <class state>
struct IndexCompare {
	bool operator()(const IndexOpenClosedData<state> &i1, const IndexOpenClosedData<state> &i2) const
	{
		if (fequal(i1.g+i1.h, i2.g+i2.h))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(i1.g+i1.h, i2.g+i2.h));
	}
};

template <class state>
struct IndexCompareWithF {
	bool operator()(const IndexOpenClosedDataWithF<state> &i1, const IndexOpenClosedDataWithF<state> &i2) const
	{
		if (fequal(i1.f, i2.f))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(i1.f, i2.f));
	}
};

template<typename state, typename CmpKey = IndexCompareWithF<state>, class dataStructure = IndexOpenClosedDataWithF<state> >
class IndexOpenClosed {
public:
	IndexOpenClosed();
	~IndexOpenClosed();
	void Reset(int maxID);
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddOpenNode(const state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddClosedNode(state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	void KeyChanged(uint64_t objKey);
	//void IncreaseKey(uint64_t objKey);
	dataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	uint64_t GetRound() const { return currentRound; }
	uint64_t Peek() const;
	uint64_t Close();
	void Reopen(uint64_t objKey);
	
	uint64_t GetOpenItem(unsigned int which) const { return theHeap[which]; }
	size_t OpenSize() const { return theHeap.size(); }
	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }
	//	void verifyData();
private:
	bool HeapifyUp(unsigned int index);
	void HeapifyDown(unsigned int index);

	uint64_t currentRound;
	std::vector<uint64_t> theHeap;
	std::vector<dataStructure> elements;
};


template<typename state, typename CmpKey, class dataStructure>
IndexOpenClosed<state, CmpKey, dataStructure>::IndexOpenClosed()
{
	currentRound = 0;
//	elements.resize(maxID);
//	for (auto &x : elements)
//		x.round = currentRound;
}

template<typename state, typename CmpKey, class dataStructure>
IndexOpenClosed<state, CmpKey, dataStructure>::~IndexOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey, class dataStructure>
void IndexOpenClosed<state, CmpKey, dataStructure>::Reset(int maxID)
{
	theHeap.resize(0);
	if (elements.size() != maxID)
	{
		elements.resize(maxID);
		for (auto &x : elements)
			x.round = currentRound;
	}
	currentRound++;
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t IndexOpenClosed<state, CmpKey, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	if (elements[hash].round == currentRound)
	{
		// shouldn't be in open/closed already
		assert(false);
	}
	elements[hash] = dataStructure(val, g, h, parent, theHeap.size(), kOpenList);
	elements[hash].round = currentRound;
	if (parent == kTAStarNoNode)
		elements[hash].parentID = hash;
	theHeap.push_back(hash); // adding element id to back of heap
	HeapifyUp(theHeap.size()-1); // heapify from back of the heap
	return hash;
}

template<typename state, typename CmpKey, class dataStructure>
uint64_t IndexOpenClosed<state, CmpKey, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double f, double g, double h, uint64_t parent)
{
	if (elements[hash].round == currentRound)
	{
		// shouldn't be in open/closed already
		assert(false);
	}
	elements[hash] = dataStructure(val, f, g, h, parent, theHeap.size(), kOpenList);
	elements[hash].round = currentRound;
	if (parent == kTAStarNoNode)
		elements[hash].parentID = hash;
	theHeap.push_back(hash); // adding element id to back of heap
	HeapifyUp(theHeap.size()-1); // heapify from back of the heap
	return hash;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t IndexOpenClosed<state, CmpKey, dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	// shouldn't be in open/closed already
	assert(elements[hash].round != currentRound);
	elements[hash] = dataStructure(val, g, h, parent, 0, kClosedList);
	elements[hash].round = currentRound;
	if (parent == kTAStarNoNode)
		elements[hash].parentID = hash;
	return hash;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t IndexOpenClosed<state, CmpKey, dataStructure>::AddClosedNode(state &val, uint64_t hash, double f, double g, double h, uint64_t parent)
{
	// shouldn't be in open/closed already
	assert(elements[hash].round != currentRound);
	elements[hash] = dataStructure(val, f, g, h, parent, 0, kClosedList);
	elements[hash].round = currentRound;
	if (parent == kTAStarNoNode)
		elements[hash].parentID = hash;
	return hash;
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey, class dataStructure>
void IndexOpenClosed<state, CmpKey, dataStructure>::KeyChanged(uint64_t val)
{
	if (!HeapifyUp(elements[val].openLocation))
		HeapifyDown(elements[val].openLocation);
}

/**
 * Returns location of object as well as object key.
 */
template<typename state, typename CmpKey, class dataStructure>
dataLocation IndexOpenClosed<state, CmpKey, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	objKey = hashKey;
	if (elements[hashKey].round == currentRound)
		return elements[hashKey].where;
	return kNotFound;
}


/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t IndexOpenClosed<state, CmpKey, dataStructure>::Peek() const
{
	assert(OpenSize() != 0);
	
	return theHeap[0];
}

/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t IndexOpenClosed<state, CmpKey, dataStructure>::Close()
{
	assert(OpenSize() != 0);
	
	uint64_t ans = theHeap[0];
	elements[ans].where = kClosedList;
	theHeap[0] = theHeap[theHeap.size()-1];
	elements[theHeap[0]].openLocation = 0;
	theHeap.pop_back();
	HeapifyDown(0);
	
	return ans;
}

/**
 * Move item off the closed list and back onto the open list.
 */
template<typename state, typename CmpKey, class dataStructure>
void IndexOpenClosed<state, CmpKey, dataStructure>::Reopen(uint64_t objKey)
{
	assert(elements[objKey].where == kClosedList);
	elements[objKey].reopened = true;
	elements[objKey].where = kOpenList;
	elements[objKey].openLocation = theHeap.size();
	theHeap.push_back(objKey);
	HeapifyUp(theHeap.size()-1);
}


/**
 * Moves a node up the heap. Returns true if the node was moved, false otherwise.
 */
template<typename state, typename CmpKey, class dataStructure>
bool IndexOpenClosed<state, CmpKey, dataStructure>::HeapifyUp(unsigned int index)
{
	if (index == 0) return false;
	int parent = (index-1)/2;
	CmpKey compare;
	
	if (compare(elements[theHeap[parent]], elements[theHeap[index]]))
	{
		unsigned int tmp = theHeap[parent];
		theHeap[parent] = theHeap[index];
		theHeap[index] = tmp;
		elements[theHeap[parent]].openLocation = parent;
		elements[theHeap[index]].openLocation = index;
		HeapifyUp(parent);
		return true;
	}
	return false;
}

template<typename state, typename CmpKey, class dataStructure>
void IndexOpenClosed<state, CmpKey, dataStructure>::HeapifyDown(unsigned int index)
{
	CmpKey compare;
	unsigned int child1 = index*2+1;
	unsigned int child2 = index*2+2;
	int which;
	unsigned int count = theHeap.size();
	// find smallest child
	if (child1 >= count)
		return;
	else if (child2 >= count)
		which = child1;
	else if (!(compare(elements[theHeap[child1]], elements[theHeap[child2]])))
		which = child1;
	else
		which = child2;
	
	//if (fless(theHeap[which]->GetKey(), theHeap[index]->GetKey()))
	if (!(compare(elements[theHeap[which]], elements[theHeap[index]])))
	{
		unsigned int tmp = theHeap[which];
		theHeap[which] = theHeap[index];
		//		table[theHeap[which]] = which;
		theHeap[index] = tmp;
		elements[theHeap[which]].openLocation = which;
		elements[theHeap[index]].openLocation = index;
		//		table[theHeap[index]] = index;
		//    theHeap[which]->key = which;
		//    theHeap[index]->key = index;
		HeapifyDown(which);
	}
}

#endif /* IndexOpenClosed_h */
