//
//  CanonicalGrid.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/30/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__CanonicalGrid__
#define __hog2_glut__CanonicalGrid__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include "Map.h"
#include "SearchEnvironment.h"
#include <cassert>
#include "Graphics.h"

namespace CanonicalGrid {
	
	enum tDirection {
		kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE,
		kSE=kS|kE, kSW=kS|kW, kStay=0, kAll=kSW|kNE
	};

	struct xyLoc {
	public:
		xyLoc() { x = -1; y = -1; parent = kAll; }
		xyLoc(uint16_t _x, uint16_t _y, tDirection _parent = kAll) :x(_x), y(_y), parent(_parent) {}
		uint16_t x;
		uint16_t y;
		tDirection parent;
	};
	
	static std::ostream& operator <<(std::ostream & out, const xyLoc &loc)
	{
		out << "(" << loc.x << ", " << loc.y << " : " << loc.parent << ")";
		return out;
	}
	
	static bool operator==(const xyLoc &l1, const xyLoc &l2) {
		return (l1.x == l2.x) && (l1.y == l2.y);
	}
	
	static bool operator!=(const xyLoc &l1, const xyLoc &l2) {
		return (l1.x != l2.x) || (l1.y != l2.y);
	}
	
	class CanonicalGrid : public SearchEnvironment<xyLoc, tDirection>
	{
	public:
		CanonicalGrid(Map *m);
		void GetSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;
		void GetActions(const xyLoc &nodeID, std::vector<tDirection> &actions) const;
		tDirection GetAction(const xyLoc &s1, const xyLoc &s2) const;
		void ApplyAction(xyLoc &s, tDirection dir) const;
		
		bool InvertAction(tDirection &a) const;
		
		double HCost(const xyLoc &) const {
			fprintf(stderr, "ERROR: Single State HCost not implemented for CanonicalGrid\n");
			exit(1); return -1.0;}
		double HCost(const xyLoc &node1, const xyLoc &node2) const;
		double GCost(const xyLoc &node1, const xyLoc &node2) const;
		double GCost(const xyLoc &node1, const tDirection &act) const;
		bool GoalTest(const xyLoc &node, const xyLoc &goal) const;
		
		bool GoalTest(const xyLoc &) const {
			fprintf(stderr, "ERROR: Single State Goal Test not implemented for CanonicalGrid\n");
			exit(1); return false;}
		
		uint64_t GetMaxHash() const;
		uint64_t GetStateHash(const xyLoc &node) const;
		uint64_t GetActionHash(tDirection act) const;
//		void OpenGLDraw() const;
//		void OpenGLDraw(const xyLoc &l) const;
//		void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const;
//		void OpenGLDraw(const xyLoc &, const tDirection &) const;
//		void GLLabelState(const xyLoc &, const char *) const;
//		void GLLabelState(const xyLoc &s, const char *str, double scale) const;
//		void GLDrawLine(const xyLoc &x, const xyLoc &y) const;
		Map* GetMap() const { return map; }
		
		std::string SVGHeader();
		std::string SVGDraw();
		std::string SVGDraw(const xyLoc &);
		std::string SVGLabelState(const xyLoc &, const char *, double scale) const;
		std::string SVGDrawLine(const xyLoc &x, const xyLoc &y, int width=1) const;
		std::string SVGFrameRect(int left, int top, int right, int bottom, int width = 1);
		
		virtual void Draw(Graphics::Display &disp) const;
		virtual void Draw(Graphics::Display &disp, const xyLoc &l) const;
		virtual void DrawAlternate(Graphics::Display &disp, const xyLoc &l) const;
		virtual void Draw(Graphics::Display &disp, const xyLoc &l1, const xyLoc &l2, float v) const;
		virtual void DrawStateLabel(Graphics::Display &disp, const xyLoc &l1, const char *txt) const;
		virtual void DrawStateLabel(Graphics::Display &disp, const xyLoc &l1, const xyLoc &l2, float v, const char *txt) const;
		virtual void DrawLine(Graphics::Display &disp, const xyLoc &x, const xyLoc &y, double width = 1.0) const;
		virtual void DrawArrow(Graphics::Display &disp, const xyLoc &x, const xyLoc &y, double width = 1.0) const;

		void DrawBasicOrdering(::Graphics::Display &disp, const xyLoc l) const; // Only draws canonical ordering
		void DrawOrdering(::Graphics::Display &disp, const xyLoc l) const; // Only draws canonical ordering
		void GetFirstJumpPoints(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;

		void GetNextState(const xyLoc &currents, tDirection dir, xyLoc &news) const;
		
		void StoreGoal(xyLoc &) {} // stores the locations for the given goal state
		void ClearGoal() {}
		bool IsGoalStored() const {return false;}
		void SetDiagonalCost(double val) { DIAGONAL_COST = val; }
		double GetDiagonalCost() { return DIAGONAL_COST; }
		bool FourConnected() { return fourConnected; }
		bool EightConnected() { return !fourConnected; }
		void SetFourConnected() { fourConnected = true; }
		void SetEightConnected() { fourConnected = false; }
	protected:
		void GetBasicSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;
		Map *map;
		double DIAGONAL_COST;
		bool fourConnected;
		std::vector<bool> grid;
	};
	
}



#endif /* defined(__hog2_glut__CanonicalGrid__) */
