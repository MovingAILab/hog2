/*
 *  $Id: AStarOpenClosed.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/25/09.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef ASTAROPENCLOSED_H
#define ASTAROPENCLOSED_H

#include <cassert>
#include <cstddef>
#include <functional>
#include <stdint.h>
#include <unordered_map>
#include <vector>

struct AHash64 {
	size_t operator()(const uint64_t &x) const
	{ return (size_t)(x); }
};

enum dataLocation {
	kOpenList,
	kClosedList,
	kNotFound
};

const uint64_t kTAStarNoNode = 0xFFFFFFFFFFFFFFFFull;

template<typename state>
class AStarOpenClosedDataWithF {
public:
	AStarOpenClosedDataWithF() {}
	AStarOpenClosedDataWithF(const state &theData, double fCost, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), f(fCost), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location) { reopened = false; }
	state data;
	double f;
	double g;
	double h;
	uint64_t parentID;
	uint64_t openLocation;
	bool reopened;
	dataLocation where;
};

template<typename state>
class AStarOpenClosedData {
public:
	AStarOpenClosedData() {}
	AStarOpenClosedData(const state &theData, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location) { reopened = false; }
	state data;
	// Refactor TODO:
	// Most of the data here is required by the data structure for sorting purposes.
	// But, the g/h/(f) cost is specific to a given algorithm. So, g/h/(f) should be factored out
	// and that should be the templat class, not the full data here.
	// Bonus: for Canonical A* we can then store the parent action directly in the open list
	// to avoiding re-copying states in the A* implementation
	double g;
	double h;
	uint64_t parentID;
	uint64_t openLocation;
	bool reopened;
	dataLocation where;
};


template<typename state, typename CmpKey, class dataStructure = AStarOpenClosedData<state> >
class AStarOpenClosed {
public:
	AStarOpenClosed();
	~AStarOpenClosed();
	void Reset(int val=0);
	// TODO: replace f/g/h by a different data structure
	uint64_t AddOpenNode(const state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddClosedNode(state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode);
	void KeyChanged(uint64_t objKey);
	dataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	void Remove(uint64_t hash);
	uint64_t Peek() const;
	uint64_t Close(uint64_t objKey);
	uint64_t Close();
	void Reopen(uint64_t objKey);

	void CloseAllOnOpen()
	{
		for (int x = 0; x < theHeap.size(); x++)
			elements[theHeap[x]].where = kClosedList;
		theHeap.resize(0);
	}
	uint64_t GetOpenItem(unsigned int which) { return theHeap[which]; }
	size_t OpenSize() const { return theHeap.size(); }
	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }
	bool empty() const {return (theHeap.size() == 0);}
	//	void verifyData();
private:
	bool HeapifyUp(uint64_t index);
	void HeapifyDown(uint64_t index);
	// TODO: don't pass in the hash value any more - use the C++ hash function specialization
	//	std::hash<state> hashFcn;

	std::vector<uint64_t> theHeap;
	// storing the element id; looking up with...hash?
	// TODO: replace this with C++11 data structures
	typedef std::unordered_map<uint64_t, uint64_t, AHash64> IndexTable;
	IndexTable table;
	std::vector<dataStructure> elements;
};


template<typename state, typename CmpKey, class dataStructure>
AStarOpenClosed<state, CmpKey, dataStructure>::AStarOpenClosed()
{
}

template<typename state, typename CmpKey, class dataStructure>
AStarOpenClosed<state, CmpKey, dataStructure>::~AStarOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey, class dataStructure>
void AStarOpenClosed<state, CmpKey, dataStructure>::Reset(int)
{
	table.clear();
	elements.clear();
	theHeap.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double f, double g, double h, uint64_t parent)
{
	//size_t hash = hashFcn(val);
	// Change to behavior: if we have a duplicate state instead throwing and error,
	// we update if the path is shorter, otherwise return the old state
	auto i = table.find(hash);
	if (i != table.end())
	{
		//return -1; // TODO: find correct id and return
		//assert(false);
		uint64_t index = i->second;
		if (fless(g, elements[index].g))
		{
			elements[index].parentID = parent;
			elements[index].g = g;
			elements[index].f = f;
			KeyChanged(index);
		}
		return index;
	}
	elements.push_back(dataStructure(val, f, g, h, parent, theHeap.size(), kOpenList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	theHeap.push_back(elements.size()-1); // adding element id to back of heap
	HeapifyUp(theHeap.size()-1); // heapify from back of the heap
	return elements.size()-1;
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	// Change to behavior: if we have a duplicate state instead throwing and error,
	// we update if the path is shorter, otherwise return the old state
	auto i = table.find(hash);
	if (i != table.end())
	{
		//return -1; // TODO: find correct id and return
		//assert(false);
		uint64_t index = i->second;
		if (fless(g, elements[index].g))
		{
			elements[index].parentID = parent;
			elements[index].g = g;
			KeyChanged(index);
		}
		return index;
	}
	elements.push_back(dataStructure(val, g, h, parent, theHeap.size(), kOpenList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	theHeap.push_back(elements.size()-1); // adding element id to back of heap
	HeapifyUp(theHeap.size()-1); // heapify from back of the heap
	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::AddClosedNode(state &val, uint64_t hash, double f, double g, double h, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, f, g, h, parent, 0, kClosedList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	return elements.size()-1;
}

template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, g, h, parent, 0, kClosedList));
	if (parent == kTAStarNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	return elements.size()-1;
}

/**
 * Remove item from open/closed
 */
template<typename state, typename CmpKey, class dataStructure>
void AStarOpenClosed<state, CmpKey, dataStructure>::Remove(uint64_t hash)
{
	uint64_t index = table[hash];
	uint64_t openLoc = elements[index].openLocation;
	uint64_t swappedItem = theHeap.back();
	table.erase(table.find(hash));
	theHeap[openLoc] = theHeap.back();
	theHeap.pop_back();
	elements[swappedItem].openLocation = openLoc;
	KeyChanged(index);
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey, class dataStructure>
void AStarOpenClosed<state, CmpKey, dataStructure>::KeyChanged(uint64_t val)
{
	if (!HeapifyUp(elements[val].openLocation))
		HeapifyDown(elements[val].openLocation);
}

/**
 * Returns location of object as well as object key.
 */
template<typename state, typename CmpKey, class dataStructure>
dataLocation AStarOpenClosed<state, CmpKey, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
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
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::Peek() const
{
	assert(OpenSize() != 0);

	return theHeap[0];
}

/**
 * Move the given item to the closed list and return key.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::Close(uint64_t objKey)
{
	assert(OpenSize() != 0);
	uint64_t index = elements[objKey].openLocation;
	uint64_t ans = theHeap[index];
	assert(ans == objKey);
	elements[ans].where = kClosedList;
	theHeap[index] = theHeap[theHeap.size()-1];
	elements[theHeap[index]].openLocation = index;
	theHeap.pop_back();
	if (!HeapifyUp(index))
		HeapifyDown(index);

	return ans;
}

/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey, class dataStructure>
uint64_t AStarOpenClosed<state, CmpKey, dataStructure>::Close()
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
void AStarOpenClosed<state, CmpKey, dataStructure>::Reopen(uint64_t objKey)
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
bool AStarOpenClosed<state, CmpKey, dataStructure>::HeapifyUp(uint64_t index)
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
void AStarOpenClosed<state, CmpKey, dataStructure>::HeapifyDown(uint64_t index)
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

	if (!(compare(elements[theHeap[which]], elements[theHeap[index]])))
	{
		unsigned int tmp = theHeap[which];
		theHeap[which] = theHeap[index];
		theHeap[index] = tmp;
		elements[theHeap[which]].openLocation = which;
		elements[theHeap[index]].openLocation = index;
		HeapifyDown(which);
	}
}


#endif
