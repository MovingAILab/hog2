/*
 *  StephenPuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */
#include "StephenPuzzle.h"
#include "FPUtil.h"
#include "SVGUtil.h"
#include <cstring>
#include <unordered_map>
#include "Graphics.h"

using namespace Graphics;

namespace SPState {
	
	StephenPuzzle::StephenPuzzle()
	{
		map = new Map(5, 5);
	}
	
	StephenPuzzle::~StephenPuzzle()
	{
		delete map;
	}
	
	puzzleState StephenPuzzle::LoadPuzzle(Map *m)
	{
		delete map;
		map = new Map(m);

		puzzleState s;
		s.carry = false;
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			for (int x = 0; x < map->GetMapWidth(); x++)
			{
				if (map->GetTerrainType(x, y) == kGrass)
				{
					map->SetTerrainType(x, y, kGround);
					s.x = x;
					s.y = y;
					return s;
				}
			}
		}
		s.x = 0;
		s.y = 0;
		return s;
	}
	
	void StephenPuzzle::GetSuccessors(const puzzleState &nodeID, std::vector<puzzleState> &neighbors) const
	{
		
	}
	
	void StephenPuzzle::GetActions(const puzzleState &nodeID, std::vector<tDirection> &actions) const
	{
		
	}
	
	tDirection StephenPuzzle::GetAction(const puzzleState &s1, const puzzleState &s2) const
	{
		
	}
	
	void StephenPuzzle::ApplyAction(puzzleState &s, tDirection dir) const
	{
		if (s.carry)
		{
			switch (dir)
			{
				case kN:
				{
					if (CanMove(s.x, s.y, s.x, s.y-1) && CanMove(s.object_x, s.object_y, s.object_x, s.object_y-1))
					{
						s.y-=1;
						s.object_y-=1;
					}
					break;
				}
				case kS:
				{
					if (CanMove(s.x, s.y, s.x, s.y+1) && CanMove(s.object_x, s.object_y, s.object_x, s.object_y+1))
					{
						s.y+=1;
						s.object_y+=1;
					}
					break;
				}

				case kE:
				{
					if (CanMove(s.x, s.y, s.x+1, s.y) && CanMove(s.object_x, s.object_y, s.object_x+1, s.object_y))
					{
						s.x+=1;
						s.object_x+=1;
					}
					break;
				}

				case kW:
				{
					if (CanMove(s.x, s.y, s.x-1, s.y) && CanMove(s.object_x, s.object_y, s.object_x-1, s.object_y))
					{
						s.x-=1;
						s.object_x-=1;
					}
					break;
				}
				case kDrop:
				{
					if (map->GetTerrainType(s.object_x, s.object_y) == kGround)
					{
						map->SetTerrainType(s.object_x, s.object_y, kTrees);
						s.carry = false;
					}
					break;
				}
			}
		}
		else {
			switch (dir)
			{
				case kN:
				{
					if (CanMove(s.x, s.y, s.x, s.y-1))
						s.y-=1;
					break;
				}
				case kS:
				{
					if (CanMove(s.x, s.y, s.x, s.y+1))
						s.y+=1;
					break;
				}
					
				case kE:
				{
					if (CanMove(s.x, s.y, s.x+1, s.y))
						s.x+=1;
					break;
				}
					
				case kW:
				{
					if (CanMove(s.x, s.y, s.x-1, s.y))
						s.x-=1;
					break;
				}

				case kPickUp:
				{
					if (map->GetTerrainType(s.x+1, s.y) == kTrees)
					{
						s.carry = true;
						s.object_y = s.y;
						s.object_x = s.x+1;
						map->SetTerrainType(s.x+1, s.y, kGround);
					}
					else if (map->GetTerrainType(s.x-1, s.y) == kTrees)
					{
						s.carry = true;
						s.object_y = s.y;
						s.object_x = s.x-1;
						map->SetTerrainType(s.x-1, s.y, kGround);
					}
					else if (map->GetTerrainType(s.x, s.y+1) == kTrees)
					{
						s.carry = true;
						s.object_y = s.y+1;
						s.object_x = s.x;
						map->SetTerrainType(s.x, s.y+1, kGround);
					}
					else if (map->GetTerrainType(s.x, s.y-1) == kTrees)
					{
						s.carry = true;
						s.object_y = s.y-1;
						s.object_x = s.x;
						map->SetTerrainType(s.x, s.y-1, kGround);
					}
					break;
				}
			}
		}
	}
	
	void StephenPuzzle::GetNextState(const puzzleState &currents, tDirection dir, puzzleState &news) const
	{
		news = currents;
		ApplyAction(news, dir);
	}
	
	
	bool StephenPuzzle::InvertAction(tDirection &a) const
	{
		switch (a)
		{
			case kN: a = kS; return true;
			case kS: a = kN; return true;
			case kE: a = kW; return true;
			case kW: a = kE; return true;
			case kPickUp: a = kDrop; return true;
			case kDrop: a = kPickUp; return true;
		}
		assert(false);
		return false;
	}
	
	
	bool StephenPuzzle::GoalTest(const puzzleState &node) const
	{
		if (map->GetTerrainType(node.x, node.y) == kSwamp &&
			map->GetTerrainType(node.object_x, node.object_y) == kSwamp)
			return true;
		return false;
	}
	
	
	uint64_t StephenPuzzle::GetMaxHash() const
	{
		
	}
	
	uint64_t StephenPuzzle::GetStateHash(const puzzleState &node) const
	{
		
	}
	
	uint64_t StephenPuzzle::GetActionHash(tDirection act) const
	{
		
	}

	bool StephenPuzzle::CanMove(int fromx, int fromy, int tox, int toy) const
	{
		return CanPass(map->GetTerrainType(fromx, fromy),map->GetTerrainType(tox, toy));
	}

	void StephenPuzzle::Draw(Graphics::Display &disp) const
	{
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			for (int x = 0; x < map->GetMapWidth(); x++)
			{
				double xx, yy, zz, rr;
				map->GetCoord(x, y, xx, yy, zz, rr);
				switch (map->GetTerrainType(x, y))
				{
					case kGround:
						disp.FillSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::brown);
						break;
					case kTrees:
					{
						disp.FillSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::brown);
						rr *= 0.8;
						disp.FillSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::yellow);
						disp.FrameSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::blue, 1.0);
						disp.FillCircle({static_cast<float>(xx-rr/4), static_cast<float>(yy-rr/4)}, rr/4, Colors::red);
						disp.FillCircle({static_cast<float>(xx+rr/4), static_cast<float>(yy-rr/4)}, rr/4, Colors::red);
						disp.FillNGon({static_cast<float>(xx), static_cast<float>(yy)}, 5.f*rr/8.f, 3, 0, Colors::red);
					}
						break;
					case kGrass:
						disp.FillSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::lightgreen);
						break;
					case kSwamp:
						disp.FillSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::blue);
						break;
					default:
						disp.FillSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::gray);
						break;
				}
			}
		}
	}
	
	void StephenPuzzle::Draw(Graphics::Display &disp, const puzzleState &l) const
	{
		double xx, yy, zz, rr;
		map->GetCoord(l.x, l.y, xx, yy, zz, rr);
//		disp.FillCircle({static_cast<float>(xx+rr/10.f), static_cast<float>(yy+rr/10.f)}, rr, Colors::darkgray);
		disp.FillCircle({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::white);

		if (l.carry)
		{
			map->GetCoord(l.object_x, l.object_y, xx, yy, zz, rr);
			rr *= 0.8;
			disp.FillSquare({static_cast<float>(xx+rr/10.f), static_cast<float>(yy+rr/10.f)}, rr, Colors::darkgray);
			disp.FillSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::orange);
			disp.FrameSquare({static_cast<float>(xx), static_cast<float>(yy)}, rr, Colors::blue, 1.0);
			disp.FillCircle({static_cast<float>(xx-rr/4), static_cast<float>(yy-rr/4)}, rr/4, Colors::red);
			disp.FillCircle({static_cast<float>(xx+rr/4), static_cast<float>(yy-rr/4)}, rr/4, Colors::red);
			disp.FillNGon({static_cast<float>(xx), static_cast<float>(yy)}, 5.f*rr/8.f, 3, 0, Colors::red);
		}
	}


}
