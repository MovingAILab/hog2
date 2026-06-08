//
//  FocalOpenClosed.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 12/27/19.
//
// A focal list + open + closed list where open is a treap and focal is a heap[?]
// Code is work in progress

#ifndef FocalOpenClosed_h
#define FocalOpenClosed_h

namespace focalOpenClosed {

enum dataLocation {
	kOpenList,
	kClosedList,
	kFocalList,
	kNotFound
};

template<typename state>
class FocalOpenClosedData {
public:
	FocalOpenClosedData() {}
	FocalOpenClosedData(const state &theData, double fCost, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), f(fCost), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location) { reopened = false; }
	state data;
	double f;
	double g;
	double h;
	uint64_t parentID;
	bool reopened;
	dataLocation where;
};

template<typename state, typename CmpKey, class dataStructure = FocalOpenClosedData<state> >
class FocalOpenClosed {
public:
	FocalOpenClosed();
	~FocalOpenClosed();
	void Reset(uint64_t maxKey=0);
	uint64_t AddOpenNode(const state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	uint64_t AddClosedNode(state &val, uint64_t hash, double f, double g, double h, uint64_t parent=kTAStarNoNode);
	void KeyChanged(uint64_t objKey);
	dataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	void Remove(uint64_t hash);
	uint64_t PeekOpen() const;
	uint64_t PeekFocal() const;
	uint64_t Close(uint64_t objKey);
	void Reopen(uint64_t objKey);
	
	uint64_t GetOpenItem(unsigned int which);
	size_t OpenSize() const;
	uint64_t GetOpenItem(unsigned int which);
	size_t FocalSize() const;
	size_t ClosedSize() const;
	size_t size() const;
private:
	Treap open;
	Heap focal;
};

}

#endif /* FocalOpenClosed_h */
