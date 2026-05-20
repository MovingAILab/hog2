/*
 *  ConfigEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/9/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include "Constants.h"
#include "SearchEnvironment.h"
#include <stdlib.h>
#include <stdio.h>

using Graphics::line2d;

class ConfigEnvironment : public SearchEnvironment<Graphics::point, line2d>
{
public:
	ConfigEnvironment();
	virtual ~ConfigEnvironment();

	void AddObstacle(line2d obs) { obstacles.push_back(obs); }
	void PopObstacle() { obstacles.pop_back(); }
	void GetSuccessors(const Graphics::point &nodeID, std::vector<Graphics::point> &neighbors) const;
	void GetActions(const Graphics::point &nodeID, std::vector<line2d> &actions) const;
	line2d GetAction(const Graphics::point &s1, const Graphics::point &s2) const;
	virtual void ApplyAction(Graphics::point &s, line2d dir) const;

	virtual bool InvertAction(line2d &a) const;

	virtual double HCost(const Graphics::point &) const {
		printf("Single State HCost Failure: method not implemented for ConfigEnvironment\n");
		exit(0); return -1.0;}

	virtual double HCost(const Graphics::point &node1, const Graphics::point &node2) const;
	virtual double GCost(const Graphics::point &node1, const Graphics::point &node2) const;
	virtual double GCost(const Graphics::point &node1, const line2d &act) const;
	bool GoalTest(const Graphics::point &node, const Graphics::point &goal) const;

	bool GoalTest(const Graphics::point &) const {
		printf("Single State Goal Test Failure: method not implemented for ConfigEnvironment\n");
		exit(0); return false;}

	uint64_t GetStateHash(const Graphics::point &node) const;
	uint64_t GetActionHash(line2d act) const;

//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const Graphics::point &l) const;
//	virtual void OpenGLDraw(const Graphics::point &, const line2d &) const;
//	virtual void OpenGLDraw(const Graphics::point&, const Graphics::point&, float) const {}

	virtual void GetNextState(const Graphics::point &currents, line2d dir, Graphics::point &news) const;

	void StoreGoal(Graphics::point &g) { goal = g; goal_stored = true;} // stores the locations for the given goal state
	void ClearGoal() {goal_stored = false;}
	bool IsGoalStored() const {return false;}


//	bool LegalState(line2d &a);
//	bool LegalArmConfig(line2d &a);
private:
	bool Legal(const Graphics::point &a, const Graphics::point &b) const;
	Graphics::point goal;
//	void DrawLine(line2d l) const;
	std::vector<line2d> obstacles;

	bool goal_stored;
};
