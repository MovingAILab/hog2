/*
 *  $Id: OpenClosedList.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/14/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef OpenClosedList_H
#define OpenClosedList_H

#include <cassert>
#include <vector>
#include <stdio.h>
#include <stdint.h>
#include <unordered_map>

/**
* A simple Heap class.
 */


template<typename OBJ, class HashKey, class EqKey, class CmpKey>
class OpenClosedList {
public:
	OpenClosedList();
	~OpenClosedList();
	void reset();
	void Add(OBJ val);
	void DecreaseKey(OBJ val);
	void IncreaseKey(OBJ val);
	bool IsIn(const OBJ val) const;
	OBJ Remove();
	void pop() { Remove(); }
	const OBJ peek() const { return _elts[0]; }
	OBJ top() { return _elts[0]; }
	OBJ find(OBJ val);
	bool Empty();
	unsigned size() { return _elts.size(); }
	//	void verifyData();
private:
	std::vector<OBJ> _elts;
	void HeapifyUp(unsigned int index);
	void HeapifyDown(unsigned int index);
	typedef std::unordered_map<OBJ, unsigned int, HashKey, EqKey > IndexTable;
	IndexTable table;
};


template<typename OBJ, class HashKey, class EqKey, class CmpKey>
OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::OpenClosedList()
{
}

template<typename OBJ, class HashKey, class EqKey, class CmpKey>
OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::~OpenClosedList()
{
}

/**
* Remove all objects from queue.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::reset()
{
	table.clear();
	_elts.resize(0);
}

/**
* Add object into OpenClosedList.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::Add(OBJ val)
{
	if ((table.find(val) != table.end()))
	{
		printf("Trying to add duplicate node.\n");
		assert(table.find(val) == table.end());
	}
	table[val] = _elts.size();
	//  val->key = count;
	_elts.push_back(val);
	//count++;
	//  HeapifyUp(val->key);
	HeapifyUp(table[val]);
}

/**
* Indicate that the key for a particular object has decreased.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::DecreaseKey(OBJ val)
{
	EqKey eq;
	assert(eq(_elts[table[val]], val));
  //HeapifyUp(val->key);
	_elts[table[val]] = val;
  HeapifyUp(table[val]);
}

/**
* Indicate that the key for a particular object has increased.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::IncreaseKey(OBJ val)
{
	EqKey eq;
	assert(eq(_elts[table[val]], val));
	_elts[table[val]] = val;
  HeapifyDown(table[val]);
}

/**
* Returns true if the object is in the OpenClosedList.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
bool OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::IsIn(const OBJ val) const
{
	EqKey eq;
	//	typedef std::unordered_map<OBJ, unsigned int, HashKey, EqKey > IndexTable;
	typename IndexTable::const_iterator it;
	it = table.find(val);
	if (it != table.end())
	{
		const unsigned int stored = (*it).second;
		if (stored < _elts.size())
		{
			if (eq(_elts[stored], val))
				return true;
		}
	}
	return false;
}

/**
* Remove the item with the lowest key from the OpenClosedList & re-heapify.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
OBJ OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::Remove()
{
  if (Empty())
		return OBJ();
  // count--;
  OBJ ans = _elts[0];
  _elts[0] = _elts[_elts.size()-1];
	table[_elts[0]] = 0;
  //_elts[0]->key = 0;
  _elts.pop_back();
	table.erase(ans);
  HeapifyDown(0);
	
  return ans;
}

/**
* find this object in the Heap and return
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
OBJ OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::find(OBJ val)
{
	if (!IsIn(val))
		return OBJ();
    return _elts[table[val]];
}

/**
* Returns true if no items are in the OpenClosedList.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey>
bool OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::Empty()
{
  return _elts.size() == 0;
}

///**
//* Verify that the Heap is internally consistent. Fails assertion if not.
// */
//template<typename OBJ, class HashKey, class EqKey, class CmpKey>
//void OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::verifyData()
//{
//	assert(_elts.size() == table.size());
//	OpenClosedList::IndexTable::iterator iter;
//	for (iter = table.begin(); iter != table.end(); iter++)
//	{
//		EqKey eq;
//		assert(eq(iter->first, _elts[iter->second])); 
//	}
//}

template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::HeapifyUp(unsigned int index)
{
  if (index == 0) return;
  int parent = (index-1)/2;
	CmpKey compare;

	if (compare(_elts[parent], _elts[index]))
	{
    OBJ tmp = _elts[parent];
    _elts[parent] = _elts[index];
    _elts[index] = tmp;
		table[_elts[parent]] = parent;
		table[_elts[index]] = index;
		EqKey eq;
		assert(!eq(_elts[parent], _elts[index]));
    HeapifyUp(parent);
  }
}

template<typename OBJ, class HashKey, class EqKey, class CmpKey>
void OpenClosedList<OBJ, HashKey, EqKey, CmpKey>::HeapifyDown(unsigned int index)
{
	CmpKey compare;
  unsigned int child1 = index*2+1;
  unsigned int child2 = index*2+2;
  int which;
	unsigned int count = _elts.size();
  // find smallest child
  if (child1 >= count)
    return;
  else if (child2 >= count)
    which = child1;
  //else if (fless(_elts[child1]->GetKey(), _elts[child2]->GetKey()))
	else if (!(compare(_elts[child1], _elts[child2])))
		which = child1;
  else
    which = child2;
	
  //if (fless(_elts[which]->GetKey(), _elts[index]->GetKey()))
	if (!(compare(_elts[which], _elts[index])))
	{
    OBJ tmp = _elts[which];
    _elts[which] = _elts[index];
		table[_elts[which]] = which;
    _elts[index] = tmp;
		table[_elts[index]] = index;
//    _elts[which]->key = which;
//    _elts[index]->key = index;
    HeapifyDown(which);
  }
}

#endif
