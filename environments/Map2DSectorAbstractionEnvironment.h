//
//  Map2DSectorAbstractionEnvironment.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/23/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#ifndef Map2DSectorAbstractionEnvironment_h
#define Map2DSectorAbstractionEnvironment_h

#include <stdio.h>
#include "Map2DEnvironment.h"
#include "MinimalSectorAbstraction.h"

struct abstractGridState
{
	int sector;
	int region;
};

struct abstractMove {
	int direction;
	int region;
};

bool operator==(const abstractGridState &, const abstractGridState &);
bool operator==(const abstractMove &, const abstractMove &);

class Map2DSectorAbstraction : public SearchEnvironment<abstractGridState, abstractMove>  {
public:
	Map2DSectorAbstraction(Map *m, int sectorSize);
	~Map2DSectorAbstraction();
	void GetSuccessors(const abstractGridState &nodeID, std::vector<abstractGridState> &neighbors) const;
	void GetActions(const abstractGridState &nodeID, std::vector<abstractMove> &actions) const;
	
	virtual void ApplyAction(abstractGridState &s, abstractMove a) const;

	xyLoc GetState(const abstractGridState &loc) const;
	abstractGridState GetAbstractState(const xyLoc &loc) const;

	bool InvertAction(abstractMove &a) const
	{ return false; }
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const abstractGridState &node1, const abstractGridState &node2) const;
	
	virtual double GCost(const abstractGridState &node1, const abstractGridState &node2) const;
	virtual double GCost(const abstractGridState &node, const abstractMove &act) const;
	virtual bool GoalTest(const abstractGridState &node, const abstractGridState &goal) const;
	
	virtual uint64_t GetStateHash(const abstractGridState &node) const;
	
	virtual uint64_t GetActionHash(abstractMove act) const;
	
//	virtual void OpenGLDraw() const {};
//	virtual void OpenGLDraw(const abstractGridState&) const {};
//	/** Draw the transition at some percentage 0...1 between two abstractGridStates */
//	virtual void OpenGLDraw(const abstractGridState&, const abstractGridState&, float) const {}
//	virtual void OpenGLDraw(const abstractGridState&, const abstractMove&) const {};
//	virtual void GLLabelabstractGridState(const abstractGridState&, const char *) const {}
//	virtual void GLDrawLine(const abstractGridState &x, const abstractGridState &y) const {}
	
	virtual void Draw(Graphics::Display &display) const;
	virtual void Draw(Graphics::Display &display, const abstractGridState&) const;
	virtual void DrawLine(Graphics::Display &display, const abstractGridState &x, const abstractGridState &y, float width = 1.0) const;
	
private:
	Map *map;
	MapEnvironment *me;
	MinimalSectorAbstraction *msa;
	mutable std::vector<tempEdgeData> actions;
	mutable std::vector<abstractGridState> nbr;
};

#endif /* Map2DSectorAbstractionEnvironment_h */
