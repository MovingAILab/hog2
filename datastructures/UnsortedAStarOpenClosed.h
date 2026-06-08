/*
 *  $Id: UnsortedOpenClosed.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/25/09.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */


#include <cassert>
#include <functional>
#include <stdint.h>
#include <unordered_map>
#include <vector>
#include <iostream>


struct UnsortedIndexData {
public:
	UnsortedIndexData()
	{

	}
	UnsortedIndexData(uint64_t _index, bool _inOpen)
	{
		index = _index;
		inOpen = _inOpen;
	}
	uint64_t index;
	bool inOpen;
};



template<typename state>
class UnsortedOpenClosedData {
public:
	UnsortedOpenClosedData() {}
	UnsortedOpenClosedData(const state &theData, uint64_t _hash, double gCost, uint64_t _parent)
	:data(theData), g(gCost), parent(_parent), hash(_hash) {}
	state data;
	// Refactor TODO:
	// Most of the data here is required by the data structure for sorting purposes.
	// But, the g/h/(f) cost is specific to a given algorithm. So, g/h/(f) should be factored out
	// and that should be the templat class, not the full data here.
	// Bonus: for Canonical A* we can then store the parent action directly in the open list
	// to avoiding re-copying states in the A* implementation
    uint64_t hash;
	double g;
	uint64_t parent;
};




template<typename state, class dataStructure = UnsortedOpenClosedData<state> >
class UnsortedOpenClosed {
public:
	UnsortedOpenClosed();
	~UnsortedOpenClosed();
	void reset(int val=0);
	// TODO: replace f/g/h by a different data structure
	uint64_t Add(const state &val, uint64_t hash, double g, uint64_t parent=kTAStarNoNode);
	bool Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey, bool from_open = true) 
	{ 
		if (from_open)
			return elements[objKey]; 
		else
			return closed[objKey];
	}
    void Close(uint64_t hash);
	//uint64_t GetOpenItem(unsigned int which) { return theHeap[which]; }
	size_t OpenSize() const { return elements.size(); }
	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }
	bool empty() const {return (elements.size() == 0);}
	bool Contains(uint64_t hash) { return table.find(hash) != table.end();}
	typedef std::unordered_map<uint64_t, UnsortedIndexData> IndexTable;
	IndexTable table;
	std::vector<dataStructure> elements;
    std::vector<dataStructure> closed;
	std::set<uint64_t> _closed;

    void Move(uint64_t hash, uint64_t dest)
    {
        uint64_t index;
        auto atDest = elements[dest];
        Lookup(hash, index);
        dataStructure ds = Lookup(index);
        elements[dest] = ds;
        elements[index] = atDest;
        table[hash] = UnsortedIndexData(dest, true);//dest;
        table[atDest.hash] = UnsortedIndexData(index, true);//index;
    }
};


template<typename state, class dataStructure>
UnsortedOpenClosed<state, dataStructure>::UnsortedOpenClosed()
{
}

template<typename state, class dataStructure>
UnsortedOpenClosed<state, dataStructure>::~UnsortedOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, class dataStructure>
void UnsortedOpenClosed<state, dataStructure>::reset(int val)
{
	table.clear();
	elements.resize(0);
	closed.resize(0);
	if (val != 0)
	{
		table.reserve(val);
		elements.reserve(val);
		closed.reserve(val);
	}
	//_closed.clear();
}



/**
 * Add object into open list.
 */
template<typename state, class dataStructure>
uint64_t UnsortedOpenClosed<state, dataStructure>::Add(const state &val, uint64_t hash, double g, uint64_t parent)
{
	// Change to behavior: if we have a duplicate state instead throwing and error,
	// we update if the path is shorter, otherwise return the old state
	auto i = table.find(hash);
	if (i != table.end())
	{
		//return -1; // TODO: find correct id and return
		//assert(false);
		uint64_t index = i->second.index;
		return index;
	}
	elements.push_back(dataStructure(val, hash, g, parent));
	table[hash] = UnsortedIndexData(elements.size()-1, true); // hashing to element list location
	//std::cout << hash << " " << elements.size()-1 << std::endl;
	//table.insert({hash, elements.size()-1});
	return elements.size()-1;
}




/**
 * Returns location of object as well as object key.
 */
template<typename state, class dataStructure>
bool UnsortedOpenClosed<state, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	typename IndexTable::const_iterator it;
	it = table.find(hashKey);
	if (it != table.end())
	{
		auto d = (*it).second;
		objKey = d.index;
		return d.inOpen;
	}
	else
	{
		std::cout << "Lookup: hash not found" << std::endl;
	}
	return false;
}


template<typename state, class dataStructure>
void UnsortedOpenClosed<state, dataStructure>::Close(uint64_t hash)
{
    Move(hash, size()-1);
    dataStructure ds = Lookup(size()-1);
    elements.pop_back();
    closed.push_back(ds);
    table[hash] = UnsortedIndexData(closed.size()-1, false);
	//_closed.insert(hash);
}
