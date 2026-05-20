//
//  Map2DConstrainedEnvironment.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/3/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__Map2DConstrainedEnvironment__
#define __hog2_glut__Map2DConstrainedEnvironment__

#include <iostream>

#include "Map2DEnvironment.h"

struct xytLoc {
	xytLoc(xyLoc loc, int time) :l(loc), t(time) {}
	xytLoc() { t = 0; }
	xyLoc l;
	uint32_t t;
};

struct constraint {
	xytLoc loc;
	tDirection dir;
};

static std::ostream& operator <<(std::ostream & out, const xytLoc &loc)
{
	out << "(" << loc.l.x << ", " << loc.l.y << ": " << loc.t << ")";
	return out;
}
	
bool operator==(const xytLoc &l1, const xytLoc &l2);


class Map2DConstrainedEnvironment : public SearchEnvironment<xytLoc, tDirection>
{
public:
	Map2DConstrainedEnvironment(Map *m);
	void AddConstraint(constraint c);
	void AddConstraint(xytLoc loc);
	void AddConstraint(xytLoc loc, tDirection dir);
	void ClearConstraints();

	virtual void GetSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const;
	virtual void GetActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const;
	virtual tDirection GetAction(const xytLoc &s1, const xytLoc &s2) const;
	virtual void ApplyAction(xytLoc &s, tDirection a) const;
	virtual void UndoAction(xytLoc &s, tDirection a) const;
	
	virtual bool InvertAction(tDirection &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const xytLoc &node1, const xytLoc &node2) const;
	virtual double GCost(const xytLoc &node1, const xytLoc &node2) const { return 1; }
	virtual double GCost(const xytLoc &node, const tDirection &act) const { return 1; }
	virtual bool GoalTest(const xytLoc &node, const xytLoc &goal) const;
	
	virtual uint64_t GetStateHash(const xytLoc &node) const;
	virtual uint64_t GetActionHash(tDirection act) const;

//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const xytLoc&) const;
//	virtual void OpenGLDraw(const xytLoc&, const tDirection&) const;
//	virtual void GLDrawLine(const xytLoc &x, const xytLoc &y) const;
private:
	bool ViolatesConstraint(const xyLoc &from, const xyLoc &to, int time) const;

	std::vector<constraint> constraints;
	MapEnvironment *mapEnv;
};

#endif /* defined(__hog2_glut__Map2DConstrainedEnvironment__) */
