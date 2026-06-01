/*
 *  StephenPuzzle.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef StephenPuzzle_H
#define StephenPuzzle_H

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include "SearchEnvironment.h"
#include "Graphics.h"
#include "Map.h"

#include <cassert>

namespace SPState {
	
	struct puzzleState {
	public:
		puzzleState() { x = -1; y = -1; }
		puzzleState(uint16_t _x, uint16_t _y) :x(_x), y(_y) {}
		uint16_t x;
		uint16_t y;
		bool carry;
		uint16_t object_x;
		uint16_t object_y;
	};
	
	
	static std::ostream& operator <<(std::ostream & out, const puzzleState &loc)
	{
		out << "(" << loc.x << ", " << loc.y << ")";
		return out;
	}
	
	static bool operator==(const puzzleState &l1, const puzzleState &l2) {
		return (l1.x == l2.x) && (l1.y == l2.y);
	}
	
	static bool operator!=(const puzzleState &l1, const puzzleState &l2) {
		return (l1.x != l2.x) || (l1.y != l2.y);
	}
	
	
	enum tDirection {
		kN, kE, kS, kW, kPickUp, kDrop
	};
	
	class StephenPuzzle : public SearchEnvironment<puzzleState, tDirection>
	{
	public:
		StephenPuzzle();
		virtual ~StephenPuzzle();
		puzzleState LoadPuzzle(Map *m);
		virtual void GetSuccessors(const puzzleState &nodeID, std::vector<puzzleState> &neighbors) const;
		void GetActions(const puzzleState &nodeID, std::vector<tDirection> &actions) const;
		tDirection GetAction(const puzzleState &s1, const puzzleState &s2) const;
		virtual void ApplyAction(puzzleState &s, tDirection dir) const;
		virtual void GetNextState(const puzzleState &currents, tDirection dir, puzzleState &news) const;

		virtual bool InvertAction(tDirection &a) const;
		std::string GetName() { return "StephenPuzzle"; }
		//	bool Contractable(const puzzleState &where);
		
		virtual double HCost(const puzzleState &) const
		{ return 0;}
		virtual double HCost(const puzzleState &node1, const puzzleState &node2) const
		{ return 0;}
		
		virtual double GCost(const puzzleState &node1, const puzzleState &node2) const
		{ return 1;}

		virtual double GCost(const puzzleState &node1, const tDirection &act) const
		{ return 1;}

		bool GoalTest(const puzzleState &node, const puzzleState &goal) const
		{
			return GoalTest(node);
		}
		
		bool GoalTest(const puzzleState &) const;
		
		uint64_t GetMaxHash() const;
		uint64_t GetStateHash(const puzzleState &node) const;
		uint64_t GetActionHash(tDirection act) const;

		virtual void OpenGLDraw() const {};
		virtual void OpenGLDraw(const puzzleState&) const {};
		virtual void OpenGLDraw(const puzzleState&, const tDirection&) const {};

		void Draw(Graphics::Display &disp) const;
		void Draw(Graphics::Display &disp, const puzzleState &l) const;
//		void DrawAlternate(Graphics::Display &disp, const puzzleState &l) const;
//		void Draw(Graphics::Display &disp, const puzzleState &l1, const puzzleState &l2, float v) const;
//		void DrawStateLabel(Graphics::Display &disp, const puzzleState &l1, const char *txt) const;
//		void DrawStateLabel(Graphics::Display &disp, const puzzleState &l1, const puzzleState &l2, float v, const char *txt) const;
//		void DrawLine(Graphics::Display &disp, const puzzleState &x, const puzzleState &y, double width = 1.0) const;
//		void DrawArrow(Graphics::Display &disp, const puzzleState &x, const puzzleState &y, double width = 1.0) const;
		
		Map* GetMap() const { return map; }
		
	protected:
		bool CanMove(int fromx, int fromy, int tox, int toy) const;
		Map *map;
	};
	
	
}

namespace std {
	template <>
	struct hash<SPState::puzzleState>
	{
		std::size_t operator()(const SPState::puzzleState& k) const
		{
			return (((std::size_t)k.x)<<16)|k.y;
		}
	};
}

#endif
