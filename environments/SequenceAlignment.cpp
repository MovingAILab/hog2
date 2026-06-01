/*
 *  SequenceAlignment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/20/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#include "SequenceAlignment.h"

void SequenceAlignment::GetSuccessors(const SequenceAlignmentState &nodeID, std::vector<SequenceAlignmentState> &neighbors) const
{
	neighbors.resize(0);
	if ((nodeID%d) < d-1)
		neighbors.push_back(nodeID+1);
	if ((nodeID/d) < d-1)
		neighbors.push_back(nodeID+d);
	if (((nodeID%d) < d-1) && ((nodeID/d) < d-1))
		neighbors.push_back(nodeID+d+1);
}

void SequenceAlignment::GetActions(const SequenceAlignmentState &nodeID, std::vector<SequenceAlignmentAction> &actions) const
{
	if ((nodeID%d) < d-1)
		actions.push_back(1);
	if ((nodeID/d) < d-1)
		actions.push_back(3);
	if (((nodeID%d) < d-1) && ((nodeID/d) < d-1))
		actions.push_back(2);
}

SequenceAlignmentAction SequenceAlignment::GetAction(const SequenceAlignmentState &s1, const SequenceAlignmentState &s2) const
{
	assert(false);
	return 0;
}

void SequenceAlignment::ApplyAction(SequenceAlignmentState &s, SequenceAlignmentAction a) const
{
	switch (a)
	{
		case -3: s=s-d; break;
		case -2: s=s-d-1; break;
		case -1: s=s-1; break;
		case  3: s=s+d; break;
		case  2: s=s+d+1; break;
		case  1: s=s+1; break;
		default: assert(false);
	}
}


void SequenceAlignment::GetNextState(const SequenceAlignmentState &s1, SequenceAlignmentAction a, SequenceAlignmentState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool SequenceAlignment::InvertAction(SequenceAlignmentAction &a) const
{ a=-a; return true; }

double SequenceAlignment::HCost(const SequenceAlignmentState &node1, const SequenceAlignmentState &node2) const
{ if (node1==node2) return 0; return 1; }

double SequenceAlignment::GCost(const SequenceAlignmentState &node1, const SequenceAlignmentState &node2) const
{ if (((node1%d) == (node2%d)) || (node1/d == node2/d)) return 1; return 1.5; }
double SequenceAlignment::GCost(const SequenceAlignmentState &node, const SequenceAlignmentAction &act) const
{ if ((act == 2)||(act == -2)) return 1.5; return 1; }
bool SequenceAlignment::GoalTest(const SequenceAlignmentState &node, const SequenceAlignmentState &goal) const
{ return node == goal; }

uint64_t SequenceAlignment::GetStateHash(const SequenceAlignmentState &node) const { return node; }
uint64_t SequenceAlignment::GetActionHash(SequenceAlignmentAction act) const { return act+3;}
