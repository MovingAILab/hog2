/*
 *  $Id: OpenListB.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/14/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 * Modified by Zhifu Zhang
 *
 */

#ifndef OpenListB_H
#define OpenListB_H

#include <cassert>
#include <vector>
#include <deque>
#include <unordered_map>

/**
* A simple & efficient Heap class.
 */


template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
class OpenListB {
public:
  OpenListB();
  ~OpenListB();
	void reset();
  void Add(OBJ val);
  void DecreaseKey(OBJ val);
  void IncreaseKey(OBJ val);
  bool IsIn(OBJ val);
  OBJ Remove();
	void pop() { Remove(); }
	OBJ top() { return _elts[0]; }
	OBJ find(OBJ val);
  bool Empty();
	unsigned size() { return _elts.size(); }
//	void verifyData();
  OBJ FindSpecialMin(double F);
  OBJ FindTieFMin(double F);

private:
  std::vector<OBJ> _elts;
	void HeapifyUp(unsigned int index);
  void HeapifyDown(unsigned int index);
	typedef std::unordered_map<OBJ, unsigned int, HashKey, EqKey > IndexTable;
	IndexTable table;
};


template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::OpenListB()
{
}

template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::~OpenListB()
{
}

/**
* Remove all objects from queue.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
void OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::reset()
{
	table.clear();
	_elts.resize(0);
}

/**
* Add object into OpenListB.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
void OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::Add(OBJ val)
{
    assert(table.find(val) == table.end());
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
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
void OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::DecreaseKey(OBJ val)
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
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
void OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::IncreaseKey(OBJ val)
{
	EqKey eq;
	assert(eq(_elts[table[val]], val));
	_elts[table[val]] = val;
  HeapifyDown(table[val]);
}

/**
* Returns true if the object is in the OpenListB.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
bool OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::IsIn(OBJ val)
{
	EqKey eq;
  if ((table.find(val) != table.end()) && (table[val] < _elts.size()) &&
			(eq(_elts[table[val]], val)))
		return true;
  return false;
}

/**
* Remove the item with the lowest key from the OpenListB & re-heapify.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
OBJ OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::Remove()
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
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
OBJ OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::find(OBJ val)
{
	if (!IsIn(val))
		return OBJ();
    return _elts[table[val]];
}

/**
* Returns true if no items are in the OpenListB.
 */
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
bool OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::Empty()
{
  return _elts.size() == 0;
}

///**
//* Verify that the Heap is internally consistent. Fails assertion if not.
// */
//template<typename OBJ, class HashKey, class EqKey, class CmpKey>
//void OpenListB<OBJ, HashKey, EqKey, CmpKey>::verifyData()
//{
//	assert(_elts.size() == table.size());
//	OpenListB::IndexTable::iterator iter;
//	for (iter = table.begin(); iter != table.end(); iter++)
//	{
//		EqKey eq;
//		assert(eq(iter->first, _elts[iter->second])); 
//	}
//}

template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
void OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::HeapifyUp(unsigned int index)
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

template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
void OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::HeapifyDown(unsigned int index)
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

/* returns the index of the element that is in the special case, and has min special key */
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
OBJ OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::FindSpecialMin(double F) 
{
	std::deque<unsigned int> candidates;
	CmpKeyStrictExtract extractor;
	SpecialKey spk;

	if (!fless(extractor(_elts[0]), F))
		return OBJ();

	unsigned int resultIndex = 0;
	candidates.push_back(0);

	while(candidates.size()) 
	{
		unsigned int current = candidates.front();
		candidates.pop_front();

		if (spk(_elts[resultIndex],_elts[current])) 
		{
			resultIndex = current;
		}

		unsigned int count = _elts.size();

		unsigned int child1 = current*2+1;
		unsigned int child2 = current*2+2;
		
		if (child1 < count && fless(extractor(_elts[child1]),F))
		{
			candidates.push_back(child1);
		}

		if (child2 < count && fless(extractor(_elts[child2]),F))
		{
			candidates.push_back(child2);
		}
	}

	return _elts[resultIndex];
}

/* returns the index of the element that has f==F, and has min g, as suggested by Nathan */
template<typename OBJ, class HashKey, class EqKey, class CmpKey, class SpecialKey, class CmpKeyStrictExtract>
OBJ OpenListB<OBJ, HashKey, EqKey, CmpKey, SpecialKey, CmpKeyStrictExtract>::FindTieFMin(double F) 
{
	std::deque<unsigned int> candidates;
	CmpKeyStrictExtract extractor;
	SpecialKey spk;

	if (!fequal(extractor(_elts[0]), F))
		return OBJ();

	unsigned int resultIndex = 0;
	candidates.push_back(0);

	while(candidates.size()) 
	{
		unsigned int current = candidates.front();
		candidates.pop_front();

		if (spk(_elts[resultIndex],_elts[current])) 
		{
			resultIndex = current;
		}

		unsigned int count = _elts.size();

		unsigned int child1 = current*2+1;
		unsigned int child2 = current*2+2;
		
		if (child1 < count && fequal(extractor(_elts[child1]),F))
		{
			candidates.push_back(child1);
		}

		if (child2 < count && fequal(extractor(_elts[child2]),F))
		{
			candidates.push_back(child2);
		}
	}

	return _elts[resultIndex];
}


#endif
