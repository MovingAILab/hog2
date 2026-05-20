//
//  RubiksCube.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/6/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__RubiksCube__
#define __hog2_glut__RubiksCube__

#include <iostream>
#include <stdint.h>
#include <unordered_map>
#include <vector>
#include "RubiksCubeCorners.h"
#include "RubiksCubeEdges.h"
#include "SearchEnvironment.h"
#include "RubiksCube7Edges.h"
#include "FourBitArray.h"
#include "DiskBitFile.h"
#include "EnvUtil.h"
#include "Bloom.h"
#include "MinBloom.h"
#include "PDBHeuristic.h"

class RubiksState
{
public:
	RubiksState()
	{
		Reset();
	}
	void Reset()
	{
		corner.Reset();
		edge.Reset();
		//edge7.Reset();
	}
	RubiksCornerState corner;
	RubikEdgeState edge;
//	Rubik7EdgeState edge7;
};

namespace std {
	template <> struct hash<RubiksState>
	{
		size_t operator()(const RubiksState & x) const
		{
			return std::hash<RubikEdgeState>{}(x.edge)^(std::hash<RubiksCornerState>{}(x.corner)<<2);
		}
	};
}

static bool operator==(const RubiksState &l1, const RubiksState &l2)
{
//	return (l1.corner.state == l2.corner.state) && (l1.edge.state == l2.edge.state);
	return (l1.corner == l2.corner) && (l1.edge == l2.edge);
}

static bool operator!=(const RubiksState &l1, const RubiksState &l2)
{
	return !(l1 == l2);
}

static std::ostream &operator<<(std::ostream &out, const RubiksState &s)
{
	out << "{" << s.edge << ", " << s.corner << "}";
	return out;
}

typedef int RubiksAction;

//static std::ostream &operator<<(std::ostream &out, const RubiksAction &s)
//{
//	switch (s) {
//		case 0: out << "U "; break;
//		case 1: out << "U- "; break;
//		case 2: out << "U2 "; break;
//		case 3: out << "D "; break;
//		case 4: out << "D- "; break;
//		case 5: out << "D2 "; break;
//		case 6: out << "F "; break;
//		case 7: out << "F- "; break;
//		case 8: out << "F2 "; break;
//		case 9: out << "B "; break;
//		case 10: out << "B- "; break;
//		case 11: out << "B2 "; break;
//		case 12: out << "L "; break;
//		case 13: out << "L- "; break;
//		case 14: out << "L2 "; break;
//		case 15: out << "R "; break;
//		case 16: out << "R- "; break;
//		case 17: out << "R2 "; break;
//	}
//	return out;
//}

//class RubikCornerMove {
//public:
//	RubiksCornersAction act;
//	RubikCornerMove *next;
//	int length() { if (next == 0) return 1; return 1+next->length(); }
//};

class RubiksCube : public SearchEnvironment<RubiksState, RubiksAction>
{
public:
	RubiksCube()
//:f("/home/sturtevant/sturtevant/code/cc/rubik/RC-10edge")
//:f("/data/cc/rubik/10/RC-10edge")
//:f("/store/rubik/RC")
	{
		f = 0;
		pruneSuccessors = false;
		minCompression = true;
		bloomFilter = false;
		uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(false);
		InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
		InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);

		edgeDist.resize(16);
		cornDist.resize(16);
		depth8 = 0;
		depth9 = 0;
		//		for (int x = 0; x < 18; x++)
//		{
//			moves[x].act = x;
//			if (x != 17)
//				moves[x].next = &moves[x+1];
//		} moves[17].next = 0;
	}
	~RubiksCube() { /*delete depth8; delete depth9;*/ }
	std::string GetName() { return "Rubiks Cube"; }
	void SetPruneSuccessors(bool val) { pruneSuccessors = val; history.resize(0); }
	virtual void GetSuccessors(const RubiksState &nodeID, std::vector<RubiksState> &neighbors) const;
	virtual void GetActions(const RubiksState &nodeID, std::vector<RubiksAction> &actions) const;
	void GetActions(const RubiksState &nodeID, std::vector<RubiksAction> &actions, const RubiksAction &lastAction) const;
	virtual void GetPrunedActions(const RubiksState &nodeID, RubiksAction lastAction, std::vector<RubiksAction> &actions) const;
	virtual RubiksAction GetAction(const RubiksState &s1, const RubiksState &s2) const;
	virtual void ApplyAction(RubiksState &s, RubiksAction a) const;
	virtual void UndoAction(RubiksState &s, RubiksAction a) const;

	virtual void GetNextState(const RubiksState &, RubiksAction , RubiksState &) const;
	
	virtual bool InvertAction(RubiksAction &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const RubiksState &node1, const RubiksState &node2) const;
	virtual double HCost(const RubiksState &node1, const RubiksState &node2, double parentHCost) const;
	int Edge12PDBDist(const RubiksState &s);
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const RubiksState &node) const;
	
	virtual double GCost(const RubiksState &node1, const RubiksState &node2) const { return 1.0; }
	virtual double GCost(const RubiksState &node, const RubiksAction &act) const { return 1.0; }
	virtual bool GoalTest(const RubiksState &node, const RubiksState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const RubiksState &node) const;
	
	virtual uint64_t GetStateHash(const RubiksState &node) const;
	virtual uint64_t GetCornerHash(const RubiksState &node) const;
	virtual uint64_t GetEdgeHash(const RubiksState &node) const;

	virtual uint64_t GetActionHash(RubiksAction act) const { return act; }
	virtual void GetStateFromHash(uint64_t hash, RubiksState &node) const;
	virtual void GetStateFromHash(uint64_t cornerHash, uint64_t edgeHash, RubiksState &node) const;
	
//	FourBitArray &GetCornerPDB() { return cornerPDB; }
//	FourBitArray &GetEdgePDB() { return edgePDB; }
	//FourBitArray &GetEdge7PDB(bool min) { if (min) return edge7PDBmin; return edge7PDBint; }

	virtual void Draw(Graphics::Display &display, const RubiksState &s) const
	{ 
//		c.Draw(display, s.corner);
		e.Draw(display, s.edge);
	}

//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const RubiksState&) const;
//	virtual void OpenGLDrawCorners(const RubiksState&) const;
//	virtual void OpenGLDrawEdges(const RubiksState&) const;
//	virtual void OpenGLDrawEdgeDual(const RubiksState&) const;
//	virtual void OpenGLDrawCenters() const;
//	virtual void OpenGLDrawCubeBackground() const;
//	/** Draw the transition at some percentage 0...1 between two states */
//	virtual void OpenGLDraw(const RubiksState&, const RubiksState&, float) const;
//	virtual void OpenGLDraw(const RubiksState&, const RubiksAction&) const;
	
	//int64_t percentage;
	int compressionFactor;
	bool minCompression;
	bool bloomFilter;
	bool minBloomFilter;
	std::vector<uint64_t> edgeDist;
	std::vector<uint64_t> cornDist;
	std::unordered_map<uint64_t, uint8_t> depthTable;
	BloomFilter *depth8, *depth9;
	MinBloomFilter *minBloom;
//private:
//	void OpenGLDrawCube(int cube) const;
//	void SetFaceColor(int face) const;
	mutable std::vector<RubiksAction> history;
	RubiksCorner c;
	RubikEdge e;
	mutable RubikEdgeState dual;
	mutable Rubik7EdgeState e7dual;
	Rubik7Edge e7;
//	FourBitArray cornerPDB;
//	FourBitArray edgePDB;
	//FourBitArray edge7PDBmin;
	//FourBitArray edge7PDBint;
	
	
	DiskBitFile *f;
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;

	bool pruneSuccessors;
};


class RubikPDB : public PDBHeuristic<RubiksState, RubiksAction, RubiksCube, RubiksState, 4> {
public:
	RubikPDB(RubiksCube *e, const RubiksState &s, std::vector<int> distinctEdges, std::vector<int> distinctCorners);
	uint64_t GetStateHash(const RubiksState &s) const;
	void GetStateFromHash(RubiksState &s, uint64_t hash) const;

	uint64_t GetPDBSize() const;
	
	uint64_t GetPDBHash(const RubiksState &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const RubiksState &s, int threadID = 0) const { return GetPDBHash(s); }
	void GetStateFromPDBHash(uint64_t hash, RubiksState &s, int threadID = 0) const;
	RubiksState GetStateFromAbstractState(RubiksState &s) const { return s; }

//	void OpenGLDraw() const
//	{
//		RubiksState s;
//		GetStateFromPDBHash(0, s);
//		env->OpenGLDraw(s);
//	}
	
	//	const char *GetName();
	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	std::string GetFileName(const char *prefix);
private:
	RubikEdgePDB ePDB;
	RubikCornerPDB cPDB;
	std::vector<int> edges;
	std::vector<int> corners;
};

class RubikDualPDB : public Heuristic<RubiksState> {
public:
	RubikDualPDB(RubikPDB *pdb);
	virtual double HCost(const RubiksState &a, const RubiksState &b) const;
private:
	RubikPDB *pdb;
};

class RubikArbitraryGoalPDB : public Heuristic<RubiksState> {
public:
	RubikArbitraryGoalPDB(RubikPDB *pdb);
	virtual double HCost(const RubiksState &a, const RubiksState &b) const;
private:
	RubikPDB *pdb;
};


#endif /* defined(__hog2_glut__RubiksCube__) */
