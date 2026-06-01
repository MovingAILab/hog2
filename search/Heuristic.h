//
//  Heuristic.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/19/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_Heuristic_h
#define hog2_glut_Heuristic_h

#include <vector>
#include <cstdint>
#include <cstddef>

enum HeuristicTreeNodeType {
	kMaxNode,
	kAddNode,
	kLeafNode
};

struct HeuristicTreeNode
{
	HeuristicTreeNodeType nodeType;
	unsigned int whichNode;
	unsigned int numChildren;
};

template <class state>
class Heuristic {
public:
	Heuristic() { for (int x = 0; x < 256; x++) histogram[x] = 0; }
	virtual ~Heuristic() {}
	virtual double HCost(const state &a, const state &b) const;
	std::vector<HeuristicTreeNode> lookups;
	std::vector<Heuristic*> heuristics;
	mutable uint64_t histogram[256];
private:
	double HCost(const state &s1, const state &s2, int treeNode) const;
};

template <class state>
class ZeroHeuristic : public Heuristic<state> {
public:
	double HCost(const state &a, const state &b) const { return 0; }
};

template <class state>
class WeightedHeuristic : public Heuristic<state> {
public:
	WeightedHeuristic(Heuristic<state> *h, double weight) :h(h), weight(weight){}
	double HCost(const state &a, const state &b) const { return weight*h->HCost(a, b); }
	void SetWeight(double w) { weight = w; }
//private:
	Heuristic<state> *h;
	double weight;
};

template <class state>
class OffsetHeuristic : public Heuristic<state> {
public:
	OffsetHeuristic(Heuristic<state> *h, double off) :h(h), offset(off){}
	double HCost(const state &a, const state &b) const { return std::max(h->HCost(a, b)-offset, 0.0); }
	void SetOffset(double w) { offset = w; }
	//private:
	Heuristic<state> *h;
	double offset;
};



template <class state>
double Heuristic<state>::HCost(const state &s1, const state &s2) const
{
	return HCost(s1, s2, 0);
}

template <class state>
double Heuristic<state>::HCost(const state &s1, const state &s2, int treeNode) const
{
	double hval = 0;
	switch (lookups[treeNode].nodeType)
	{
		case kMaxNode:
		{
			for (size_t x = 0; x < lookups[treeNode].numChildren; x++)
			{
				hval = std::max(hval, HCost(s1, s2, lookups[treeNode].whichNode+x));
			}
		} break;
		case kAddNode:
		{
			for (size_t x = 0; x < lookups[treeNode].numChildren; x++)
			{
				hval += HCost(s1, s2, lookups[treeNode].whichNode+x);
			}
		} break;
		case kLeafNode:
		{
			hval = heuristics[lookups[treeNode].whichNode]->HCost(s1, s2);
		} break;
	}
//	histogram[int(hval)]++;
	return hval;
}

#endif
