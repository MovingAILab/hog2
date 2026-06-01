//
//  DVCBSOpenClosed.h
//  hog2 mac native demos
//
//  Created by Shahaf S. Shperberg
//

#ifndef DVCBSOpenClosed_h
#define DVCBSOpenClosed_h

#include <cassert>
#include <vector>
#include <unordered_map>
#include <stdint.h>
#include "AStarOpenClosed.h"
#include <unordered_map>
#include <map>
#include <set>


template<typename state>
class DVCBSOpenClosedData {
public:
	DVCBSOpenClosedData() {}
	DVCBSOpenClosedData(const state &theData, double gCost, double hCost, uint64_t parent, stateLocation location)
	:data(theData), g(gCost), h(hCost), parentID(parent), where(location) { reopened = false; }
	state data;
	double g;
	double h;
	uint64_t parentID;
	uint64_t openLocation;
	bool reopened;
	stateLocation where;
};

template<typename state, class dataStructure = DVCBSOpenClosedData<state> >
class DVCBSOpenClosed {
public:
	DVCBSOpenClosed();
	~DVCBSOpenClosed();
	void Reset();
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent=kTBDNoNode, stateLocation whichQueue = kOpenWaiting);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent=kTBDNoNode);
	void KeyChanged(uint64_t objKey,double oldVal);
	void eraseFromNodesMap(stateLocation whichQueue,uint64_t val);
	void eraseFromNodesMap(stateLocation whichQueue,uint64_t val,double oldKey);
	void eraseFirstClusterFromNodesMap(stateLocation whichQueue);
	void putInNodesMap(stateLocation whichQueue,uint64_t val);
	void Remove(uint64_t objKey);
	//void IncreaseKey(uint64_t objKey);
	stateLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	double getFirstKey(stateLocation whichQueue) const { return nodesMap[whichQueue].begin()->first; }
	
	//if exist min pair, return true, else(one queue is empty) return false;
	//if it returns true, left and right should be set to the pair that is supposed to be returned.
	//bool ExtractMinPair(uint64_t& left, uint64_t& right) ;
	uint64_t Close();
	uint64_t CloseAtIndex(uint64_t index);
	void PutToReady();
	//void Reopen(uint64_t objKey);
	
	size_t OpenReadySize() const { return nodesMapSize[kOpenReady]; }
	size_t OpenWaitingSize() const { return nodesMapSize[kOpenWaiting]; }
	size_t OpenSize() const { return OpenReadySize()+OpenWaitingSize(); }
	
	std::map<double,std::set<uint64_t> >::iterator getNodesMapBegin(stateLocation whichQueue) {return nodesMap[whichQueue].begin();}
	std::map<double,std::set<uint64_t> >::iterator getNodesMapEnd(stateLocation whichQueue) {return nodesMap[whichQueue].end();}
	std::set<uint64_t> getNodesMapElements(stateLocation whichQueue,double key) {return nodesMap[whichQueue][key];}
	
	
	size_t ClosedSize() const { return size()-OpenReadySize()-OpenWaitingSize(); }
	size_t size() const { return elements.size(); }
	void verifyData();
private:
	
	//2 queues:
	//priorityQueues[0] is openReady, priorityQueues[1] is openWaiting
	std::vector<std::map<double,std::set<uint64_t> > > nodesMap;
	std::vector<double> nodesMapSize;
	//std::vector<uint64_t> readyQueue;
	//std::vector<uint64_t> waitingQueue;
	
	// storing the element id; looking up with...hash?
	typedef std::unordered_map<uint64_t, uint64_t, AHash64> IndexTable;
	
	IndexTable table;
	//all the elements, open or closed
	std::vector<dataStructure> elements;
};


template<typename state, class dataStructure>
DVCBSOpenClosed<state, dataStructure>::DVCBSOpenClosed()
{
	std::map<double,std::set<uint64_t> > mapReady;
	std::map<double,std::set<uint64_t> > mapWaiting;
	nodesMap.push_back(mapReady);
	nodesMap.push_back(mapWaiting);
	nodesMapSize.push_back(0);
	nodesMapSize.push_back(0);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

template<typename state, class dataStructure>
DVCBSOpenClosed<state, dataStructure>::~DVCBSOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state,   class dataStructure>
void DVCBSOpenClosed<state,   dataStructure>::Reset()
{
	table.clear();
	elements.clear();
	nodesMap[0].clear();
	nodesMap[1].clear();
	nodesMapSize[0]=0;
	nodesMapSize[1]=0;
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state,   class dataStructure>
uint64_t DVCBSOpenClosed<state,   dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent, stateLocation whichQueue)
{
	// should do lookup here...
	if (table.find(hash) != table.end())
	{
		//return -1; // TODO: find correct id and return
		assert(false);
	}
	if (whichQueue == kOpenReady)
	{
		elements.push_back(dataStructure(val, g, h, parent, kOpenReady));
		
	}
	else if (whichQueue == kOpenWaiting)
	{
		elements.push_back(dataStructure(val, g, h, parent, kOpenWaiting));
	}
	
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	
	putInNodesMap(whichQueue,elements.size() - 1);
	
	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state,   class dataStructure>
uint64_t DVCBSOpenClosed<state,   dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, g, h, parent,kClosed));
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	return elements.size()-1;
}

/**
 * Indicate that the key for a particular object has changed.
 */

template<typename state,   class dataStructure>
void DVCBSOpenClosed<state,   dataStructure>::KeyChanged(uint64_t val,double oldKey)
{
	eraseFromNodesMap(elements[val].where,val,oldKey);
	putInNodesMap(elements[val].where,val);
}

template<typename state,   class dataStructure>
void DVCBSOpenClosed<state,   dataStructure>::putInNodesMap(stateLocation whichQueue,uint64_t val)
{
	double key;
	if (whichQueue == kOpenReady){
		key = elements[val].g;
	}
	else if (whichQueue == kOpenWaiting){
		key = elements[val].g+elements[val].h;
	}
	else{
		return;
	}
	if (nodesMap[whichQueue][key].insert(val).second){
		nodesMapSize[whichQueue] += 1;
	}
}

template<typename state,   class dataStructure>
void DVCBSOpenClosed<state,   dataStructure>::eraseFromNodesMap(stateLocation whichQueue,uint64_t val)
{
	double key;
	if (whichQueue == kOpenReady){
		key = elements[val].g;
	}
	else if (whichQueue == kOpenWaiting){
		key = elements[val].g+elements[val].h;
	}
	else{
		return;
	}
	auto containerIter = nodesMap[whichQueue].find(key);
	if (containerIter != nodesMap[whichQueue].end()){
		auto toDelete = containerIter->second.find(val);
		if (toDelete != containerIter->second.end()){
			containerIter->second.erase(toDelete);
			nodesMapSize[whichQueue] -= 1;
			if (containerIter->second.empty()){
				nodesMap[whichQueue].erase(containerIter);
			}
		}
	}
}

template<typename state,   class dataStructure>
void DVCBSOpenClosed<state,   dataStructure>::eraseFromNodesMap(stateLocation whichQueue,uint64_t val,double oldKey)
{
	auto containerIter = nodesMap[whichQueue].find(oldKey);
	if (containerIter != nodesMap[whichQueue].end()){
		auto toDelete = containerIter->second.find(val);
		if (toDelete != containerIter->second.end()){
			containerIter->second.erase(toDelete);
			nodesMapSize[whichQueue] -= 1;
			if (containerIter->second.empty()){
				nodesMap[whichQueue].erase(containerIter);
			}
		}
	}
}

template<typename state,   class dataStructure>
void DVCBSOpenClosed<state,   dataStructure>::eraseFirstClusterFromNodesMap(stateLocation whichQueue)
{
	auto containerIter = nodesMap[whichQueue].begin();
	auto oldsize = nodesMap[whichQueue].size();
	if (containerIter != nodesMap[whichQueue].end()){
		nodesMapSize[whichQueue] -= containerIter->second.size();
		nodesMap[whichQueue].erase(containerIter);
	}
}

template<typename state, class dataStructure>
void DVCBSOpenClosed<state, dataStructure>::Remove(uint64_t val)
{
	stateLocation whichQueue = elements[val].where;
	elements[val].where = kClosed;
	eraseFromNodesMap(whichQueue,val);
}

template<typename state,   class dataStructure>
stateLocation DVCBSOpenClosed<state,   dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	typename IndexTable::const_iterator it;
	it = table.find(hashKey);
	if (it != table.end())
	{
		objKey = (*it).second;
		return elements[objKey].where;
	}
	return kUnseen;
}

template<typename state,   class dataStructure>
uint64_t DVCBSOpenClosed<state,   dataStructure>::CloseAtIndex(uint64_t val)
{
	
	stateLocation whichQueue = elements[val].where;
	elements[val].where = kClosed;
	eraseFromNodesMap(whichQueue,val);
	return val;
}

template<typename state, class dataStructure>
void DVCBSOpenClosed<state, dataStructure>::PutToReady()
{
	assert(OpenWaitingSize() != 0);
	auto container = nodesMap[kOpenWaiting].begin()->second;
	for (auto i = container.begin();i != container.end(); i++){
		putInNodesMap(kOpenReady,*i);
		elements[*i].where = kOpenReady;
	}
	eraseFirstClusterFromNodesMap(kOpenWaiting);
}



#endif /* DVCBSOpenClosed_h */
