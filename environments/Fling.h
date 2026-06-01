/*
 *  Fling.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/5/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#include <iostream>
#include "SearchEnvironment.h"

class FlingMove;

class FlingBoard
{
public:
	FlingBoard(unsigned int len=7, unsigned int high=8) :width(len), height(high)
	{ board = 0; obstacles = 0; holes = 0; /*board.resize(len*high);*/ currId = 0;}
	void Reset()
	{
		currId = 0;
		board = 0; locs.resize(0);
		holes = 0; obstacles = 0;
	}
	void AddFling(unsigned int x, unsigned int y);
	void AddFling(unsigned int offset);
	void RemoveFling(unsigned int x, unsigned int y);
	void RemoveFling(unsigned int offset);
	bool CanMove(int which, int x, int y) const;
	void Move(int which, int x, int y);
	int LocationAfterAction(FlingMove m);
	int NumPieces() const { return int(locs.size()); }
	int GetPieceLocation(int which) const { return locs[which].first; }
	bool HasPiece(int x, int y) const;
	bool HasPiece(int offset) const;
	bool HasHole(int x, int y) const;
	bool HasHole(int offset) const;
	bool HasObstacle(int x, int y) const;
	bool HasObstacle(int offset) const;
	int GetIndexInLocs(int x, int y) const;
	int GetIndexInLocs(int offset) const;
	unsigned int width;
	unsigned int height;
	
	std::vector<std::pair<int, int>> locs;
	
	void SetPiece(int which);
	void ClearPiece(int which);
	void SetHole(int which);
	void ClearHole(int which);
	void SetObstacle(int which);
	void ClearObstacle(int which);
	uint64_t GetObstacles() { return obstacles; }
	void SetObstacles(uint64_t o) { obstacles = o; }
	uint64_t GetRawBoard() const { return board; }
	uint64_t GetRawObstacles() const { return obstacles; }
	uint64_t GetRawHoles() const { return holes; }

	int currId;
private:
	uint64_t board;
	uint64_t obstacles;
	uint64_t holes;
};

inline int GetX(int loc) { return loc%7; }
inline int GetY(int loc) { return loc/7; }
void GetMirror(const FlingBoard &in, FlingBoard &out, bool horiz, bool vert);
void ShiftToCorner(FlingBoard &in);
uint64_t GetCanonicalHash(uint64_t which);

enum tFlingDir {
	kLeft = 0, kUp = 1, kRight = 2,kDown = 3
};

class FlingMove
{
public:
	uint8_t startLoc;
	tFlingDir dir;
};

static std::ostream& operator <<(std::ostream & out, const FlingBoard &loc)
{
	for (unsigned int y= 0; y < loc.height; y++)
	{
		for (unsigned int x = 0; x < loc.width; x++)
		{
			if (loc.HasPiece(x, y))// board[y*loc.width+x])
			{
				out << "o";
			}
			else if (loc.HasHole(x, y))
			{
				out << " ";
			}
			else if (loc.HasObstacle(x, y))
			{
				out << "x";
			}
			else {
				out << ".";
			}
		}
		out << std::endl;
	}
	return out;
}

static std::ostream& operator <<(std::ostream & out, const FlingMove &loc)
{
	out << +loc.startLoc << "->";
	switch (loc.dir)
	{
		case kLeft: out << "Left"; break;
		case kRight: out << "Right"; break;
		case kUp: out << "Up"; break;
		case kDown: out << "Down"; break;
	}
	return out;
}

static bool operator==(const FlingBoard &l1, const FlingBoard &l2) {
	return (l1.width == l2.width && l1.height == l2.height && l1.GetRawBoard() == l2.GetRawBoard());
}

static bool operator!=(const FlingBoard &l1, const FlingBoard &l2) {
	return !(l1.width == l2.width && l1.height == l2.height && l1.GetRawBoard() == l2.GetRawBoard());
}

static bool operator==(const FlingMove &l1, const FlingMove &l2) {
	return (l1.startLoc == l2.startLoc && l1.dir == l2.dir);
}


class Fling : public SearchEnvironment<FlingBoard, FlingMove> {
public:
	Fling();
	
	virtual void GetSuccessors(const FlingBoard &nodeID, std::vector<FlingBoard> &neighbors) const;
	virtual void GetActions(const FlingBoard &nodeID, std::vector<FlingMove> &actions) const;
	
	virtual FlingMove GetAction(const FlingBoard &s1, const FlingBoard &s2) const;
	virtual void ApplyAction(FlingBoard &s, FlingMove a) const;
	virtual void UndoAction(FlingBoard &s, FlingMove a) const;
	virtual void GetNextState(const FlingBoard &, FlingMove , FlingBoard &) const;
	bool LegalMove(const FlingBoard &, FlingMove);
	virtual bool InvertAction(FlingMove &a) const { assert(false); }
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const FlingBoard &node1, const FlingBoard &node2) const { return 0; }
	
	virtual double GCost(const FlingBoard &node1, const FlingBoard &node2) const { return 1; }
	virtual double GCost(const FlingBoard &node, const FlingMove &act) const { return 1; }
	
	void SetGoalPanda(int which);
	void ClearGoalPanda();
	void SetGoalLoc(int val);
	void ClearGoalLoc();
	virtual bool GoalTest(const FlingBoard &node, const FlingBoard &goal) const;
	
	virtual uint64_t GetStateHash(const FlingBoard &node) const;
	virtual void GetStateFromHash(uint64_t parent, FlingBoard &s) const;
	virtual uint64_t GetActionHash(FlingMove act) const;
	
	bool GetXYFromPoint(const FlingBoard &b, point3d loc, int &x, int &y) const;
	void IncrementRank(FlingBoard &b) const;

	int64_t getMaxSinglePlayerRank(int spots, int numPieces);
	int64_t getMaxSinglePlayerRank2(int spots, int numPieces);
	int64_t getMaxSinglePlayerRank2(int spots, int numPieces, int64_t firstIndex);
	int64_t rankPlayer(FlingBoard &s);
	void rankPlayer(FlingBoard &s, int64_t &index1, int64_t &index2);
	void rankPlayerFirstTwo(FlingBoard &s, int64_t &index1);
	void rankPlayerRemaining(FlingBoard &s, int64_t &index2);
	// returns true if it is a valid unranking given existing pieces
	bool unrankPlayer(int64_t theRank, int pieces, FlingBoard &s);

//	void initBinomialSums();
	int64_t binomialSum(unsigned int n1, unsigned int n2, unsigned int k);
	void initBinomial();
	int64_t binomial(unsigned int n, unsigned int k);
	int64_t bi(unsigned int n, unsigned int k);

	
//	virtual void OpenGLDraw() const {}
//	virtual void OpenGLDraw(const FlingBoard&) const;
//	virtual void OpenGLDrawAlternate(const FlingBoard&) const;
//	virtual void OpenGLDrawPlain(const FlingBoard&b) const;
//
//	virtual void OpenGLDraw(const FlingBoard&, const FlingMove&) const;
//	virtual void GLLabelState(const FlingBoard&, const char *) const;

private:
	int IncrementRank(FlingBoard &b, int piece) const;

	bool specificGoalLoc;
	bool specificGoalPanda;
	int goalLoc;
	std::vector<int64_t> theSums;
	std::vector<int64_t> binomials;

};

namespace std {
	
	template <>
	struct hash<FlingBoard>
	{
		std::size_t operator()(const FlingBoard& node) const
		{
			std::size_t hash = 0;
			for (unsigned int x = 0; x < node.locs.size(); x++)
			{
				hash |= (1ull<<node.locs[x].first);
			}
			return hash;
		}
	};
	
}
