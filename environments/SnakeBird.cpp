//
//  SnakeBird.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/26/20.
//  Copyright © 2020 University of Denver. All rights reserved.
//

#include "SnakeBird.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace SnakeBird {

/**
 * Construct SnakeBird class.
 * Note that width and height are currently fixed and can't be changed.
 * Eventually this should be fixed.
 */
SnakeBird::SnakeBird(int width, int height)
:width(width), height(height), editing(false)
{
	assert(width*height <= 512);
	assert(width >= 5 && height >= 5);
	portal1Loc = portal2Loc = -1;
	exitLoc = -1;
	std::fill_n(&world[0], 512, kEmpty);
	for (int x = 0; x < width; x++)
		world[GetIndex(x, height-1)] = kSpikes;
}

/** Reset to initial state and default 20(w)x16(h) map size. */
void SnakeBird::Reset()
{
	width = 20;
	height = 16;
	portal1Loc = portal2Loc = -1;
	exitLoc = -1;
	std::fill_n(&world[0], 512, kEmpty);
	for (int x = 0; x < width; x++)
		world[GetIndex(x, height-1)] = kSpikes;
	startState.Reset();
	fruit.clear();
	for (int x = 0; x < objects.size(); x++)
		objects[x].clear();
}

void SnakeBird::BiggerMapHeight()
{
	if (width*(height+1) <= 512) // map cannot have more than 512 things in it
	{
		std::array<SnakeBirdWorldObject, 512> worldCopy;
		worldCopy = world;
		for (int x = 0; x < (width*height); x++)
			world[x] = kEmpty;
		
		for (int snake = 0; startState.GetNumSnakes()-1 >= snake; snake++) // for snake
		{
			int snakeHead = startState.GetSnakeHeadLoc(snake);
			std::vector<snakeDir> snakeBody;
			for (int t = 0; t <= startState.GetSnakeLength(snake)-2; t++)
			{
				snakeBody.push_back(startState.GetSnakeDir(snake, t));
			}

			for (int i = 0; i <= height+1; i++)
			{
				if (snakeHead/(height) >= i && snakeHead/(height) < i+1)
				{
					startState.SetSnakeHeadLoc(snake, snakeHead+i+1);
					startState.SetSnakeLength(snake, 1);
					break;
				}
			}
			if (snakeBody.size() >= 1)
			{
			startState.SetSnakeLength(snake, snakeBody.size()+1);
			for (int x = 0; x < snakeBody.size(); x++)
				startState.SetSnakeDir(snake, x, snakeBody[x]);
			}
		}

		height++;

		//TODO:: 1. make the blocks(ground) print accurately after y = 15 or 18
		//TODO:: 2. figure out the offset for blocks/make them reprint accruetly
		
		for (int x = 0; x <= worldCopy.size()-1; x++) //put ground back in
		{
			if (worldCopy[x] != kEmpty)
			{
				for (int i = 0; i <= height; i++)
				{
					if (x/(height-1) >= i && x/(height-1) < i+1)
					{
						if (i >= 18)
						{
							world[x+i+1] = worldCopy[x];
							break;
						}
						else if (i <= 17)
						{
							world[x+i+1] = worldCopy[x];
							break;
						}
					}
				}
			}
		}

		if (portal1Loc != -1) //put portals back in
		{
			for (int i = 0; i <= height; i++)
			{
				if (portal1Loc/(height-1) >= i && portal1Loc/(height-1) < i+1)
				{
					portal1Loc = portal1Loc+i+1;
					break;
				}
			}
		}

		if (portal2Loc != -1) //put portals back in
		{
			for (int i = 0; i <= height; i++)
			{
				if (portal2Loc/(height-1) >= i && portal2Loc/(height-1) < i+1)
				{
					portal2Loc = portal2Loc+i+1;
					break;
				}
			}
		}
		
		if (exitLoc != -1) //put exit back in
		{
			for (int i = 1; i <= width; i++)
			{
				if (exitLoc/(height-1) >= i && exitLoc/(height-1) < i+1)
				{
					exitLoc = exitLoc+i+1;
					break;
				}
			}
		}
		if (fruit.size() >= 1)
		{
			for (int x = 0; x <= fruit.size()-1; x++) //put grounds back in
			{
				for (int i = 1; i <= height; i++)
				{
					if (fruit[x]/(height-1) >= i && fruit[x]/(height-1) < i+1)
					{
						fruit[x] = fruit[x]+i+1;
						break;
					}
				}
			}
		}
		
		for (int which = 0; which <= 3; which++)
		{
			if (objects[which].size() >= 1)
			{
				for (int vector = 0; vector <= objects[which].size()-1; vector++)
				{
					for (int i = 1; i <= height; i++)
					{
						if (objects[which][vector]/(height-1) >= i && objects[which][vector]/(height-1) < i+1) //(portal1Loc/(height-1) >= i && portal1Loc/(height-1) < i+1)
						{
							objects[which][vector] = objects[which][vector]+i+1; //fruit[x] = fruit[x]+i+1;
							break;
						}
					}
				}
			}
			
//			int newX = 0, newY=0;
//			int xOffset = 0, yOffset = 0;
//			if (objects[which].size() > 0) // other objects - get their offset base
//			{
//				xOffset = GetX(startState.GetObjectLocation(which));
//				yOffset = GetY(startState.GetObjectLocation(which));
//			}
//
//			// Get new base location (minx/y)
//			for (int i = 0; i < objects[which].size(); i++)
//			{
//				newX = std::min(newX, GetX(objects[which][i])+xOffset);
//				newY = std::min(newY, GetY(objects[which][i])+yOffset);
//			}
//			startState.SetObjectLocation(which, GetIndex(newX, newY));
//			for (int i = 0; i < objects[which].size(); i++)
//			{
//				// reset locations based on new base location
//				objects[which][i] = GetIndex(GetX(objects[which][i])+xOffset - newX,
//											 GetY(objects[which][i])+yOffset - newY);
//			}
//			// add new piece of new object
//			objects[which].push_back(GetIndex(x-newX, y-newY));
		}
		
	}
}

void SnakeBird::BiggerMapWidth()
{
	if ((width+1)*height <= 512)
		width++;
}


void SnakeBird::SmallerMapHeight()
{
	if ((height-1) >= 5)
	{
		std::array<SnakeBirdWorldObject, 512> worldCopy;
		worldCopy = world;
		for (int x = 0; x < (width*height); x++)
			world[x] = kEmpty;

		//TODO:: 1. make the snakes work (reprinting them when they go off the screen)
		//TODO:: 2. figure out the offset for blocks after y = 16 or around there
		//TODO:: 2. make the blocks(ground) reprint accurately
		
		for (int snake = 0; startState.GetNumSnakes()-1 >= snake; snake++) // for snake
		{
			int snakeHead = startState.GetSnakeHeadLoc(snake);
			int endofSnakeX = GetX(snakeHead);
			int endofSnakeY = GetY(snakeHead);
			std::vector<snakeDir> snakeBody;
			std::vector<int> snakeBodyCoordinates;
			for (int t = 0; t <= startState.GetSnakeLength(snake)-2; t++)
			{
				if (startState.GetSnakeDir(snake, t) == kRight)
				{
					endofSnakeX++;
				}
				else if (startState.GetSnakeDir(snake, t) == kLeft)
				{
					endofSnakeX--;
				}
				else if (startState.GetSnakeDir(snake, t) == kDown)
				{
					endofSnakeY++;
				}
				else if (startState.GetSnakeDir(snake, t) == kUp)
				{
					endofSnakeY--;
				}
				snakeBody.push_back(startState.GetSnakeDir(snake, t));
			}
			if (startState.GetSnakeHeadLoc(snake)%height == 0)
			{
				for (int i = 0; i <= height-1; i++)
				{
					if ((snakeHead-1)/(height) >= i && (snakeHead-1)/(height) < i+1)
					{
						startState.SetSnakeHeadLoc(snake, snakeHead-i-2); //GetIndex(GetX(snakeHead), GetY(snakeHead)-1)-i-1
						startState.SetSnakeLength(snake, snakeBody.size()+1);
						if (snakeBody.size() >= 1)
						{
							for (int x = 0; x < snakeBody.size(); x++)
								startState.SetSnakeDir(snake, x, snakeBody[x]);
						}
					}
				}
			}
			else
			{
				for (int i = 0; i <= height-1; i++)
				{
					if ((snakeHead-1)/(height) >= i && (snakeHead-1)/(height) < i+1)
					{
						startState.SetSnakeHeadLoc(snake, snakeHead-i-1);
						startState.SetSnakeLength(snake, 1);
						break;
					}
				}
				if (snakeBody.size() >= 1)
				{
					startState.SetSnakeLength(snake, snakeBody.size()+1);
					for (int x = 0; x < snakeBody.size(); x++)
						startState.SetSnakeDir(snake, x, snakeBody[x]);
				}
			}
		}
		
		height--;
		
		for (int x = 0; x <= worldCopy.size()-1; x++) //put ground back in
		{
			if (worldCopy[x] != kEmpty)
			{
				for (int i = 0; i <= width; i++)
				{
					if ((x-1)/(height+1) >= i && (x-1)/(height+1) < i+1 && x%(height+1) != 0)
					{
						if (i >= 15)
						{
							world[x-i-1-15] = worldCopy[x];
							break;
						}
						else if (i <= 14)
						{
							world[x-i-1] = worldCopy[x];
							break;
						}
					}
				}
			}
		}
		
		if (portal1Loc != -1) //put portals back in
		{
			for (int i = 0; i <= height; i++)
			{
				if (portal1Loc%(height+1) == 0)
				{
					portal1Loc = -1;
					break;
				}
				if ((portal1Loc-1)/(height+1) >= i && (portal1Loc-1)/(height+1) < i+1)
				{
					portal1Loc = portal1Loc-i-1;
					break;
				}
			}
		}
		
		if (portal2Loc != -1) //put portals back in
		{
			for (int i = 0; i <= height; i++)
			{
				if (portal2Loc%(height+1) == 0)
				{
					portal2Loc = -1;
					break;
				}
				if ((portal2Loc-1)/(height+1) >= i && (portal2Loc-1)/(height+1) < i+1)
				{
					portal2Loc = portal2Loc-i-1;
					break;
				}
			}
		}
		
		if (exitLoc != -1) //put exit back in
		{
			for (int i = 1; i <= width; i++)
			{
				if (exitLoc%(height+1) == 0)
				{
					exitLoc = -1;
					break;
				}
				if ((exitLoc-1)/(height+1) >= i && (exitLoc-1)/(height+1) < i+1)
				{
					exitLoc = exitLoc-i-1;
					break;
				}
			}
		}
		
		if (fruit.size() >= 1)
		{
			for (int x = 0; x <= fruit.size()-1; x++) //put grounds back in
			{
				for (int i = 1; i <= height; i++)
				{
					if (fruit[x]%(height+1) == 0)
					{
						std::vector<int>::iterator tor = std::find(fruit.begin(), fruit.end(), GetIndex(x, 0)); //comeback for the y value
						if (tor != fruit.end())
						{
							fruit.erase(tor);
						}
						break;
					}
					if ((fruit[x]-1)/(height+1) >= i && (fruit[x]-1)/(height+1) < i+1)
					{
						fruit[x] = fruit[x]-i-1;
						break;
					}
				}
			}
		}

		for (int which = 0; which <= 3; which++)
		{
			if (objects[which].size() >= 1)
			{
				for (int vector = 0; vector <= objects[which].size()-1; vector++)
				{
					for (int i = 1; i <= height; i++)
					{
						if ((objects[which][vector]+1)/(height+1) >= i && (objects[which][vector]-1)/(height+1) < i+1 && objects[which][vector]%(height+1) != 0)
						{
							objects[which][vector] = objects[which][vector]-i-1;
							break;
						}
					}
				}
			}
		}
		
	}
}

void SnakeBird::SmallerMapWidth()
{
	if ((width-1) >= 5)
	{
		//TODO:: 1. make the snakes work (reprinting them when they go off the screen)
		
		for (int snake = 0; startState.GetNumSnakes()-1 >= snake; snake++) // for snake
		{
			int snakeHead = startState.GetSnakeHeadLoc(snake);
			int endofSnakeX = GetX(snakeHead);
			int endofSnakeY = GetY(snakeHead);
			std::vector<snakeDir> snakeBody;
			for (int t = 0; t <= startState.GetSnakeLength(snake)-2; t++)
			{
				if (startState.GetSnakeDir(snake, t) == kRight)
				{
					endofSnakeX++;
				}
				else if (startState.GetSnakeDir(snake, t) == kLeft)
				{
					endofSnakeX--;
				}
				else if (startState.GetSnakeDir(snake, t) == kDown)
				{
					endofSnakeY++;
				}
				else if (startState.GetSnakeDir(snake, t) == kUp)
				{
					endofSnakeY--;
				}
				
				if (endofSnakeX == width-1)
				{
					startState.SetSnakeLength(snake, t);
					break;
				}
				snakeBody.push_back(startState.GetSnakeDir(snake, t));
			}
			if (GetX(startState.GetSnakeHeadLoc(snake)) == width-1)
			{
				startState.SetSnakeHeadLoc(snake, GetIndex(GetX(snakeHead)-1, GetY(snakeHead)));
				startState.SetSnakeLength(snake, snakeBody.size()+1);
				if (snakeBody.size() >= 1)
				{
					for (int x = 0; x < snakeBody.size(); x++)
						startState.SetSnakeDir(snake, x, snakeBody[x]);
				}
			}
		}
		width--;
	}
}

/**
 * Before the map is changed, this should be called. It allows for internal optimizations after
 * editing ends.
 */
void SnakeBird::BeginEditing()
{
	editing = true;
}

/**
 * Stop editing the map & apply optimizations to internal data structures.
 */
void SnakeBird::EndEditing()
{
	editing = false;
	for (int x = 0; x < 4; x++)
	{
		bool connected = false;
		
		for (int u = 0; u < objects[x].size(); u++)
		{
			for (int v = 0; v < objects[x].size(); v++)
			{
				if (Distance(objects[x][u], objects[x][v]) == 1)
					connected = true;
			}
		}
		objectFullyConnected[x] = connected;
	}
}


/**
 * The SnakeBird start location and fruit consumption returned is part of the
 * dynamic state. Everything inside the SnakeBird class is static.
 * The location of the fruit is static; whether it's been eaten is dynamic.
 */
SnakeBirdState SnakeBird::GetStart() const
{
	return startState;
}

/**
 * Set the initial state of the game.
 */
void SnakeBird::SetStart(const SnakeBirdState &start)
{
	startState = start;
}


/**
 * Add a snake to the level at a given location.
 * Note: Will need to be enhanced to set location of a particular snake.
 */
void SnakeBird::AddSnake(int x, int y, const std::vector<snakeDir> &body)
{
	int count = startState.GetNumSnakes();
	startState.SetNumSnakes(count+1);
	startState.SetSnakeHeadLoc(count, GetIndex(x, y));
	startState.SetSnakeLength(count, body.size()+1);
	for (int x = 0; x < body.size(); x++)
		startState.SetSnakeDir(count, x, body[x]);
}

void SnakeBird::AddSnakeHead(int x, int y, int whichSnake)
{
	if (GetIndex(x, y) >= 0 && GetIndex(x, y) < width*height && y <= height-2 && x <= width-1 && GetIndex(x, y) != startState.GetSnakeHeadLoc(whichSnake))
	{
		if (startState.GetNumSnakes() == whichSnake)
			startState.SetNumSnakes(whichSnake+1);
		startState.SetSnakeHeadLoc(whichSnake, GetIndex(x, y));
		startState.SetSnakeLength(whichSnake, 1);
	}
}

snakeDir SnakeBird::GetAddingDirection(int x, int y, int endX, int endY)
{
	if ((x-1 == endX) && (y == endY))
	{
		return kRight;
	}
	else if ((x+1 == endX) && (y == endY))
	{
		return kLeft;
	}
	else if ((y-1 == endY) && (x == endX))
	{
		return kDown;
	}
	else if ((y+1 == endY) && (x == endX))
	{
		return kUp;
	}
	return kNoDirection;
}

void SnakeBird::AddSnakeBody(int x, int y, int whichSnake)
{
	if (GetIndex(x, y) >= 0 && GetIndex(x, y) < width*height && y <= height-2 && x <= width-1)
	{
		int endofSnakeX = GetX(startState.GetSnakeHeadLoc(whichSnake)); // these track where the coordinates of the snake 'butt'(end) are
		int endofSnakeY = GetY(startState.GetSnakeHeadLoc(whichSnake));
		if (startState.GetSnakeLength(whichSnake) >= 2 && startState.GetSnakeLength(whichSnake) <= 30)
		{
			for (int t = 0; t <= startState.GetSnakeLength(whichSnake)-2; t++) //Go through the snake segments and find where the end of the snake is
			{
				if (startState.GetSnakeDir(whichSnake, t) == kRight)
				{
					endofSnakeX++;
				}
				else if (startState.GetSnakeDir(whichSnake, t) == kLeft)
				{
					endofSnakeX--;
				}
				else if (startState.GetSnakeDir(whichSnake, t) == kDown)
				{
					endofSnakeY++;
				}
				else if (startState.GetSnakeDir(whichSnake, t) == kUp)
				{
					endofSnakeY--;
				}
			}
		}
		if (GetAddingDirection(x, y, endofSnakeX, endofSnakeY) != kNoDirection)
		{
			if (whichSnake == 0 && startState.GetNumSnakes() == 2)
			{
				startState.MakeSnakeLonger(whichSnake, GetAddingDirection(x, y, endofSnakeX, endofSnakeY));
			}
			else
			{
				startState.SetSnakeLength(whichSnake, startState.GetSnakeLength(whichSnake)+1);
				startState.SetSnakeDir(whichSnake, startState.GetSnakeLength(whichSnake)-2, GetAddingDirection(x, y, endofSnakeX, endofSnakeY));
				//TODO:: There's an issue here with the program in the adding direction after 4 segments
			}
		}
	}
}


void SnakeBird::RemoveSnake(int x, int y, int o, int whichSnake)
{
	int endofSnakeX = GetX(startState.GetSnakeHeadLoc(whichSnake));
	int endofSnakeY = GetY(startState.GetSnakeHeadLoc(whichSnake));
	if (startState.GetSnakeLength(whichSnake) >= 2 && startState.GetSnakeLength(whichSnake) <= 30)
	{
		for (int t = 0; t <= startState.GetSnakeLength(whichSnake)-3; t++) //Go through the snake segments and find where the end of the snake is
		{
			if (startState.GetSnakeDir(whichSnake, t) == kRight)
			{
				endofSnakeX++;
			}
			else if (startState.GetSnakeDir(whichSnake, t) == kLeft)
			{
				endofSnakeX--;
			}
			else if (startState.GetSnakeDir(whichSnake, t) == kDown)
			{
				endofSnakeY++;
			}
			else if (startState.GetSnakeDir(whichSnake, t) == kUp)
			{
				endofSnakeY--;
			}
		}
	}
	if (whichSnake == 0)
	{
		if (o == 1) // if click
		{
			if (x == endofSnakeX && y == endofSnakeY && startState.GetSnakeLength(whichSnake) >= 3)
			{
				startState.SetSnakeLength(whichSnake, startState.GetSnakeLength(whichSnake)-1);
			}
			else if (GetIndex(x, y) == startState.GetSnakeHeadLoc(whichSnake) && startState.GetNumSnakes() >= 2 && startState.GetNumSnakes() == 2)
			{
				std::vector<snakeDir> snakeBody;
				int snakeHead = startState.GetSnakeHeadLoc(1);
				for (int t = 0; t <= startState.GetSnakeLength(1)-2; t++)
				{
					if (startState.GetSnakeDir(1, t) == kRight)
					{
						snakeBody.push_back(kRight);
					}
					else if (startState.GetSnakeDir(1, t) == kLeft)
					{
						snakeBody.push_back(kLeft);
					}
					else if (startState.GetSnakeDir(1, t) == kDown)
					{
						snakeBody.push_back(kDown);
					}
					else if (startState.GetSnakeDir(1, t) == kUp)
					{
						snakeBody.push_back(kUp);
					}
				}
				startState.SetSnakeLength(whichSnake, 0);
				startState.SetSnakeLength(whichSnake, snakeBody.size()+1);
				startState.SetSnakeHeadLoc(whichSnake, snakeHead);
				for (int b = 0; b <= snakeBody.size(); b++)
					startState.SetSnakeDir(whichSnake, b, snakeBody[b]);
				startState.SetNumSnakes(startState.GetNumSnakes()-1);
			}
			else
			{
				std::vector<int> snakeBody;
				int snakeBodyCounterX = GetX(startState.GetSnakeHeadLoc(whichSnake));
				int snakeBodyCounterY = GetY(startState.GetSnakeHeadLoc(whichSnake));
				for (int t = 0; t <= startState.GetSnakeLength(whichSnake)-2; t++)
				{
					if (startState.GetSnakeDir(whichSnake, t) == kRight)
					{
						snakeBodyCounterX++;
						snakeBody.push_back(GetIndex(snakeBodyCounterX+1, snakeBodyCounterY));
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kLeft)
					{
						snakeBodyCounterX--;
						snakeBody.push_back(GetIndex(snakeBodyCounterX-1, snakeBodyCounterY));
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kDown)
					{
						snakeBodyCounterY++;
						snakeBody.push_back(GetIndex(snakeBodyCounterX, snakeBodyCounterY+1));
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kUp)
					{
						snakeBodyCounterY--;
						snakeBody.push_back(GetIndex(snakeBodyCounterX, snakeBodyCounterY-1));
					}
				}

				for (int i = 0; i <= snakeBody.size()-1; i++)
				{
					if (GetIndex(x, y) == snakeBody[i])
					{
						startState.SetSnakeLength(whichSnake, startState.GetSnakeLength(whichSnake)-snakeBody.size()-i);
					}
				}
			}
		}
		else if (o == 0) // if drag
		{
			if (x == endofSnakeX && y == endofSnakeY && startState.GetSnakeLength(whichSnake) >= 3)
			{
				startState.SetSnakeLength(whichSnake, startState.GetSnakeLength(whichSnake)-1);
			}
			else if (GetIndex(x, y) == startState.GetSnakeHeadLoc(whichSnake) && startState.GetSnakeLength(whichSnake) >= 2 && startState.GetNumSnakes() >= 2)
			{
				std::vector<snakeDir> snakeBody;
				int snakeHead = startState.GetSnakeHeadLoc(whichSnake);
				for (int t = 0; t <= startState.GetSnakeLength(whichSnake)-2; t++)
				{
					if (startState.GetSnakeDir(whichSnake, t) == kRight)
					{
						snakeBody.push_back(kRight);
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kLeft)
					{
						snakeBody.push_back(kLeft);
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kDown)
					{
						snakeBody.push_back(kDown);
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kUp)
					{
						snakeBody.push_back(kUp);
					}
				}
				startState.SetSnakeLength(whichSnake, 0);
				startState.SetSnakeLength(whichSnake, snakeBody.size()+1);
				startState.SetSnakeHeadLoc(whichSnake, snakeHead);
				if (startState.GetSnakeLength(whichSnake) > 1)
				{
					for (int b = 0; b <= snakeBody.size(); b++)
						startState.SetSnakeDir(whichSnake, b, snakeBody[b]);
				}
				startState.SetNumSnakes(startState.GetNumSnakes()-1);
			}
		}
	}
	if (whichSnake == 1)
	{
		if (o == 1) // if click
		{
			if (x == endofSnakeX && y == endofSnakeY)
			{
				startState.SetSnakeLength(whichSnake, startState.GetSnakeLength(whichSnake)-1);
			}
			else if (GetIndex(x, y) == startState.GetSnakeHeadLoc(whichSnake) && startState.GetSnakeLength(whichSnake) >= 2)
			{
				startState.SetNumSnakes(startState.GetNumSnakes()-1);
			}
			else
			{
				std::vector<int> snakeBody;
				int snakeBodyCounterX = GetX(startState.GetSnakeHeadLoc(whichSnake));
				int snakeBodyCounterY = GetY(startState.GetSnakeHeadLoc(whichSnake));
				for (int t = 0; t <= startState.GetSnakeLength(whichSnake)-2; t++)
				{
					if (startState.GetSnakeDir(whichSnake, t) == kRight)
					{
						snakeBodyCounterX++;
						snakeBody.push_back(GetIndex(snakeBodyCounterX+1, snakeBodyCounterY));
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kLeft)
					{
						snakeBodyCounterX--;
						snakeBody.push_back(GetIndex(snakeBodyCounterX-1, snakeBodyCounterY));
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kDown)
					{
						snakeBodyCounterY++;
						snakeBody.push_back(GetIndex(snakeBodyCounterX, snakeBodyCounterY+1));
					}
					else if (startState.GetSnakeDir(whichSnake, t) == kUp)
					{
						snakeBodyCounterY--;
						snakeBody.push_back(GetIndex(snakeBodyCounterX, snakeBodyCounterY-1));
					}
				}
				
				for (int i = 0; i <= snakeBody.size()-1; i++)
				{
					if (GetIndex(x, y) == snakeBody[i])
					{
						startState.SetSnakeLength(whichSnake, startState.GetSnakeLength(whichSnake)-snakeBody.size()-i);
					}
				}
			}
		}
		else if (o == 0) // if drag
		{
			if (x == endofSnakeX && y == endofSnakeY && startState.GetSnakeLength(whichSnake) >= 3)
			{
				startState.SetSnakeLength(whichSnake, startState.GetSnakeLength(whichSnake)-1);
			}
			if (GetIndex(x, y) == startState.GetSnakeHeadLoc(whichSnake) && startState.GetSnakeLength(whichSnake) >= 2)
			{
				startState.SetSnakeLength(whichSnake, 0);
				startState.SetNumSnakes(startState.GetNumSnakes()-1);
			}
		}
	}
}

std::vector<snakeDir> LoadSnake(std:: vector<snakeDir> snakeBod, int width, int pos,std::vector<char> lvl)
{
	bool snakeBuilt = false;
	while(!snakeBuilt)
	{
			//build_snake:
		if (lvl[pos+1] == 1+lvl[pos]){
			snakeBod.push_back(kRight);
			pos++;
			//goto build_snake;
		}
		else if (lvl[pos-1] == 1+lvl[pos])
		{
			snakeBod.push_back(kLeft);
			pos--;
			//goto build_snake;
		}
		else if (lvl[pos+width+1] == 1+lvl[pos])
		{
			snakeBod.push_back(kDown);
			pos+= width +1;
			//goto build_snake;
		}
		else if (lvl[pos-width-1] == 1+lvl[pos])
		{
			snakeBod.push_back(kUp);
			pos-= width +1;
			//goto build_snake;
		}
		else
		{
			snakeBuilt = true;
			return snakeBod;
		}
	}
	assert(false);
	return snakeBod;
}



/**
 * Load a map from a filename.
 */
bool SnakeBird::Load(const char *filename)
{
	BeginEditing();
	
	std::ifstream infile;
	infile.open(filename);
	if (infile.fail()) {
		printf("File could not be opened.\n");
		EndEditing();
		return false;
	}
	if (infile.is_open()){
		char ch, ach;
		int x = 0;
		int y = 0;
		int ct = 0;
	
		std::string hLine;
		getline(infile, hLine);
	 	
		int wWidth;
		int wHeight;
		sscanf(hLine.c_str(), "snake%dx%d", &wWidth, &wHeight);
		
//		std::string sHeight = hLine.substr(8,2);
//		std::string sWidth = hLine.substr(5,2);
//
//		int wWidth = std::stoi(sWidth, nullptr, 10);
//		int wHeight = std::stoi(sHeight, nullptr,10);

		portal1Loc = portal2Loc = -1;
		exitLoc = -1;
		std::fill_n(&world[0], 512, kEmpty);
		startState.Reset();
		fruit.clear();
		for (int x = 0; x < objects.size(); x++)
			objects[x].clear();

		//Note that you can't call the constructor again at this point
		//SnakeBird(wWidth, wHeight);
		this->width = wWidth;
		this->height = wHeight;
		for (int x = 0; x < width; x++)
			world[GetIndex(x, height-1)] = kSpikes;

		std::vector<char> lvlary;
		while (infile.get(ch)) {
			
			lvlary.push_back(ch);
		}
		
		std:: vector<snakeDir> bod;
		bool snakeBuilt = false;
		bool madePortal = false;
		for(int a = 0; a <= lvlary.size(); a++)
		{
			if (y == height-1)
			{
				break;
			}
			int b = 0;
			switch (lvlary[a])
			{
				case '.': SetGroundType(x,y,SnakeBirdWorldObject::kEmpty); x++; break;
				case 'G': SetGroundType(x,y,SnakeBirdWorldObject::kGround); x++; break;
				case 'S': SetGroundType(x,y,SnakeBirdWorldObject::kSpikes); x++; break;
				case 'O':
					if (!madePortal)
					{
						SetGroundType(x,y,SnakeBirdWorldObject::kPortal1);
						madePortal = true;
					}
					else {
						SetGroundType(x,y,SnakeBirdWorldObject::kPortal2);
					}
						x++;
						break;
				case 'E': SetGroundType(x,y,SnakeBirdWorldObject::kExit); x++; break;
				case 'F': SetGroundType(x,y,SnakeBirdWorldObject::kFruit); x++; break;
				case '1': SetGroundType(x, y, kBlock1); x++; break;
				case '2': SetGroundType(x, y, kBlock2); x++; break;
				case '3': SetGroundType(x, y, kBlock3); x++; break;
				case '4': SetGroundType(x, y, kBlock4); x++; break;
				case '\n': x = 0; y++; break;
				case '<': 
					bod.push_back(kRight);
					b = a;
					b++;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					SetGroundType(x,y,SnakeBirdWorldObject::kEmpty);
					x++;
					break;
				case '>':
					bod.push_back(kLeft);
					b = a;
					b--;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					x++;
					break;
				case '^':
					bod.push_back(kDown);
					b = a;
					b += wWidth+1;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					x++;
					break;
				case 'v':
					bod.push_back(kUp);
					b = a;
					b -= wWidth+1;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					x++;
					break;
				default: SetGroundType(x,y,SnakeBirdWorldObject::kEmpty); x++; break;
			}		
		}
		infile.close();
		EndEditing();
		return  true;
	}
	else {
		std::cout << "Error loading level.";
		EndEditing();
		return false;
	}	
}

/**
 * Currently not supported.
 */
bool SnakeBird::Save(const char *filename)
{
	return false;
}

std::string SnakeBird::Code(int v) const
{
	//static const char* digits = "0123456789ABCDEF";
    static const char* digits = "abcdefghijklmnopqrstuvwxyz";
	std::string res;
	if (v >= 26*26)
		return "!";
	//	res  = digits[v/(26*26)];
	res = digits[v/26];
	res += digits[v%26];
//	printf("%d -> %s\n", v, res.c_str());
	return res;
}

int SnakeBird::DeCode(const std::string &s, size_t offset) const
{
	int val = 0;
	if (s.size()-offset < codeSize)
		return -1;
	for (int x = 0; x < codeSize; x++)
	{
		switch (s[offset+x])
		{
			case 'a': val = val*26+0; break;
			case 'b': val = val*26+1; break;
			case 'c': val = val*26+2; break;
			case 'd': val = val*26+3; break;
			case 'e': val = val*26+4; break;
			case 'f': val = val*26+5; break;
			case 'g': val = val*26+6; break;
			case 'h': val = val*26+7; break;
			case 'i': val = val*26+8; break;
			case 'j': val = val*26+9; break;
			case 'k': val = val*26+10; break;
			case 'l': val = val*26+11; break;
			case 'm': val = val*26+12; break;
			case 'n': val = val*26+13; break;
			case 'o': val = val*26+14; break;
			case 'p': val = val*26+15; break;
			case 'q': val = val*26+16; break;
			case 'r': val = val*26+17; break;
			case 's': val = val*26+18; break;
			case 't': val = val*26+19; break;
			case 'u': val = val*26+20; break;
			case 'v': val = val*26+21; break;
			case 'w': val = val*26+22; break;
			case 'x': val = val*26+23; break;
			case 'y': val = val*26+24; break;
			case 'z': val = val*26+25; break;
			default: return -1; break;
		}
	}
//	printf("%c%c -> %d\n", s[offset], s[offset+1], val);
	return val;
}

/**
 * Convert an ASCII map encoding back to a level.
 */
std::string SnakeBird::EncodeLevel() const
{
	std::string encoding;
	// width & height - 2 bytes each
	encoding += std::to_string(width);
	encoding += "b";
	encoding += std::to_string(height);
	
	// exit
	encoding += "E";
	encoding += Code(exitLoc);
	
	// ground
	encoding += "G";
	for (int i = 0; i < width*height; i++)
	{
		if (world[i] == kGround)
			encoding += Code(i);
	}
	
	// spikes
	encoding += "K";
	for (int i = 0; i < width*height; i++)
	{
		if (world[i] == kSpikes && GetY(i) != height-1)
			encoding += Code(i);
	}

	// fruit
	encoding += "F";
	if (fruit.size() > 0)
	{
		for (auto i : fruit)
		{
			encoding += Code(i);
		}
	}
	
	// portal
	if (portal1Loc != -1 && portal2Loc != -1)
	{
		encoding += "P";
		encoding += Code(portal1Loc);
		encoding += Code(portal2Loc);
	}
	
	// blocks
	for (int x = 0; x < objects.size(); x++)
	{
		if (objects[x].size() > 0)
			encoding += "B";
		for (int i = 0; i < objects[x].size(); i++)
		{
			int index = GetIndex(GetX(objects[x][i])+GetX(startState.GetObjectLocation(x)),
								 GetY(objects[x][i])+GetY(startState.GetObjectLocation(x)));
			encoding += Code(index);
		}
	}
	
	// snakes
	for (int x = 0; x < startState.GetNumSnakes(); x++)
	{
		encoding+="S";
		encoding+=Code(startState.GetSnakeHeadLoc(x));
		
		for (int y = 0; y < startState.GetSnakeLength(x)-1; y++)
		{
			switch (startState.GetSnakeDir(x, y))
			{
				case kLeft: encoding+="L"; break;
				case kRight: encoding+="R"; break;
				case kDown: encoding+="D"; break;
				case kUp: encoding+="U"; break;
			}
		}
	}
	
	return encoding;
}

/**
 * Convert a map to an ASCII string.
 */
bool SnakeBird::DecodeLevel(const std::string &encoding)
{
	portal1Loc = portal2Loc = -1;
	exitLoc = -1;
	std::fill_n(&world[0], 512, kEmpty);
	startState.Reset();
	fruit.clear();
	for (int x = 0; x < objects.size(); x++)
		objects[x].clear();

	BeginEditing();
	
	int cnt = sscanf(encoding.c_str(), "%db%dE", &width, &height);
	if (cnt != 2 || width < 5 || height < 5 || width*height > 512)
	{ Reset(); return false; }

	size_t start = encoding.find_first_of("E");
	if (start==std::string::npos)
	{ Reset(); printf("F1\n"); return false; }
	start++;
	
	exitLoc = DeCode(encoding, start);
	if (exitLoc == -1)
	{
		Reset();
		return false;
	}
	SetGroundType(exitLoc, kExit);
	start+=codeSize;
	
	if (start >= encoding.size() || encoding[start] != 'G')
	{ Reset(); printf("F2\n");return false; }
	start++;
	
	while (start < encoding.size() && encoding[start] != 'K') // go until we see spikes
	{
		int next = DeCode(encoding, start);
		if (next == -1)
		{ Reset(); printf("F3\n");return false; }
		start+=codeSize;
		SetGroundType(next, kGround);
	}
	
	// spikes
	if (start >= encoding.size() || encoding[start] != 'K')
	{ Reset(); printf("F4\n");return false; }
	start++;
	
	while (start < encoding.size() && encoding[start] != 'F')  // go until we see fruit
	{
		int next = DeCode(encoding, start);
		if (next == -1)
		{ Reset(); printf("F5\n");return false; }
		start+=codeSize;
		SetGroundType(next, kSpikes);
	}

	if (start >= encoding.size() || encoding[start] != 'F')
	{ Reset(); printf("F6\n");return false; }
	start++;

	while (start < encoding.size() && // might see Portal, Blocks, or Snakes next
		   encoding[start] != 'P' && encoding[start] != 'B' && encoding[start] != 'S')  // go until we see
	{
		int next = DeCode(encoding, start);
		if (next == -1)
		{ Reset(); printf("F7\n");return false; }
		start+=codeSize;
		SetGroundType(next, kFruit);
	}

	// portal
	if (start >= encoding.size())
	{ Reset(); printf("F8\n");return false; }
	if (encoding[start] == 'P' && start+1+2*codeSize <= encoding.size())
	{
		SetGroundType(DeCode(encoding, start+1), kPortal1);
		SetGroundType(DeCode(encoding, start+1+codeSize), kPortal2);
		start += 2*codeSize+1;
	}
		
	// blocks
	SnakeBirdWorldObject blocks[] = {kBlock1, kBlock2, kBlock3, kBlock4};
	int whichBlock = 0;
	while (start < encoding.size() && encoding[start] == 'B')
	{
		start++;
		// Found B, read locations until we get another B or S
		do {
			int next = DeCode(encoding, start);
			if (next == -1)
			{ Reset(); printf("F9\n");return false; }
			SetGroundType(next, blocks[whichBlock]);
			start+=codeSize;
		} while (start < encoding.size() && encoding[start] != 'B' && encoding[start] != 'S');
		whichBlock++;
	}
	
	if (encoding[start] != 'S') // snakes now
	{ Reset(); printf("F10\n");return false; }
	start++;
	
	// snakes
	int whichSnake = 0;
	while (start < encoding.size())
	{
		std::vector<snakeDir> body;
		// Found S, read locations until we run out
		do {
			int next = DeCode(encoding, start); // head location
			if (next == -1)
			{ Reset(); printf("F11\n");return false; }
			start += codeSize;
			
			body.clear();
			bool done = false;
			while (!done)
			{
				switch (encoding[start]) // body
				{
					case 'U': body.push_back(kUp); break;
					case 'D': body.push_back(kDown); break;
					case 'L': body.push_back(kLeft); break;
					case 'R': body.push_back(kRight); break;
					case 'S':
						done = true;
						break;
				}
				start++;
				if (start >= encoding.size())
					done = true;
			}
			if (body.size() == 0)
			{ Reset(); printf("F12\n");return false; }

			startState.SetNumSnakes(whichSnake+1);
			startState.SetSnakeHeadLoc(whichSnake, next);
			startState.SetSnakeLength(whichSnake, body.size()+1);
			for (int x = 0; x < body.size(); x++)
				startState.SetSnakeDir(whichSnake, x, body[x]);
			whichSnake++;
		} while (start < encoding.size());
	}
	for (int x = 0; x < width; x++)
		world[GetIndex(x, height-1)] = kSpikes;
	EndEditing();
	return true;
}


/**
 * Return (via references) all of the legal actions in a state.
 */
void SnakeBird::GetSuccessors(const SnakeBirdState &nodeID, std::vector<SnakeBirdState> &neighbors) const
{
	static std::vector<SnakeBirdAction> acts;
	GetActions(nodeID, acts);
	neighbors.clear();
	for (auto &a : acts)
	{
		SnakeBirdState s = nodeID;
		ApplyAction(s, a);
		bool valid = true;
		for (int x = s.GetNumSnakes()-1; x >= 0; x--)
		{
			if (s.GetSnakeHeadLoc(x) == kDead)
			{
				valid = false;
				break;
			}
		}
		if (valid)
		{
			// Validate that all snakes are in legal locations
//			for (int x = 0; x < s.GetNumSnakes(); x++)
//			{
//				if (s.GetSnakeHeadLoc(x) >= GetWidth()*GetHeight() && s.GetSnakeHeadLoc(x) < kDead)
//				{
//					for (int y = 0; y < s.GetNumSnakes(); y++)
//					printf("%d: %d\n", y, s.GetSnakeHeadLoc(y));
//					printf("Error - illegal successor\n");
//					s = nodeID;
//					ApplyAction(s, a);
//				}
//			}
			neighbors.push_back(s);
		}
	}
}

/**
 * Return whether an action is legal.
 */
bool SnakeBird::Legal(SnakeBirdState &s, SnakeBirdAction a)
{
	static std::vector<SnakeBirdAction> acts;
	GetActions(s, acts);
	for (auto &act : acts)
		if (act == a)
			return true;
	return false;
}

bool SnakeBird::IsOnSpikes(const SnakeBirdState &s, int snake) const
{
	int loc = s.GetSnakeHeadLoc(snake);
	if (loc == kInGoal)
		return false;
	if (loc == kDead) // no legal actions for any snake if one is dead
		return false;
	if (world[loc] == kSpikes)
		return true;
	int len = s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1);
	for (int x = 0; x < len; x++)
	{
		switch (s.GetSnakeDir(snake, x))
		{
			case kLeft: loc-=height; break;
			case kRight: loc+=height; break;
			case kUp: loc-=1; break;
			case kDown: loc+=1; break;
		}
		assert(loc >= 0 && loc < width*height);
		if (world[loc] == kSpikes)
			return true;
	}
	return false;
}


bool SnakeBird::Render(const SnakeBirdState &s) const
{
	SnakeBirdWorldObject obj[4] = {kSnake1, kSnake2, kSnake3, kSnake4};
	std::fill_n(&render[0], 512, kEmpty);
	
	// render snakes into world
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		int loc = s.GetSnakeHeadLoc(snake);
		if (loc == kInGoal)
			continue;
		if (loc == kDead) // no legal actions for any snake if one is dead
			return false;
		if (loc < 0 && loc >= width*height) // pushed offscreen
			return false;
		render[loc] = obj[snake];
		if (world[loc] == kSpikes)
			return false;
		int len = s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1);
		for (int x = 0; x < len; x++)
		{
			switch (s.GetSnakeDir(snake, x))
			{
				case kLeft: loc-=height; break;
				case kRight: loc+=height; break;
				case kUp: loc-=1; break;
				case kDown: loc+=1; break;
			}
//  			assert(loc >= 0 && loc < width*height);
			if (loc < 0 && loc >= width*height)
				return false;
			render[loc] = obj[snake];
			if (world[loc] == kSpikes)
				return false;
		}
	}
	// render fruit into world
	for (size_t f = 0; f < fruit.size(); f++)
	{
		if (s.GetFruitPresent(f))
			render[fruit[f]] = kFruit;
	}

	// render objects into world
	for (int x = 0; x < 4; x++)
	{
		SnakeBirdWorldObject item[4] = {kBlock1, kBlock2, kBlock3, kBlock4};
		if (s.GetObjectLocation(x) == kDead)
			//continue;  //    allow pushing objects off the board
			return false; // disallow pushing objects off the board

		for (int i = 0; i < objects[x].size(); i++)
		{
			// hack for S4 - don't allow objects on the ground
//			if (GetY(objects[x][i])+GetY(s.GetObjectLocation(x)) >= 11)
//				return false;
			
			render[GetIndex(GetX(objects[x][i])+GetX(s.GetObjectLocation(x)),
							GetY(objects[x][i])+GetY(s.GetObjectLocation(x)))] = item[x];
		}
	}
	return true;
}

/**
 * Return (by reference) the legal actions in a state.
 */
void SnakeBird::GetActions(const SnakeBirdState &s, std::vector<SnakeBirdAction> &actions) const
{
	actions.clear();
	SnakeBirdAction a;
	
	bool success = Render(s);
	if (!success) // something illegal about state - usually has dead snakes
		return;
	SnakeBirdWorldObject obj[4] = {kSnake1, kSnake2, kSnake3, kSnake4};

	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		// first pass - don't allow to move back onto yourself or onto other obstacles
		int loc = s.GetSnakeHeadLoc(snake);
		if (loc == kInGoal)
			continue;

		a.bird = snake;
		a.pushed = 0;
		// kDown - can never push any object down due to gravity
		if (GetY(loc)+1 < height && // not off screen
			!(kGroundMask == (world[loc+1]&kGroundMask)) && // not ground or spikes
			!(kSnakeMask == (render[loc+1]&kSnakeMask)) && // not snake
			!(kBlockMask == (render[loc+1]&kBlockMask))) // not block
		{
			if (s.GetSnakeDir(snake, 0) != kDown) // not blocked by own snake
			{
				a.direction = kDown;
				actions.push_back(a);
			}
		}

		a.pushed = 0;
		// kUp
		if (GetY(loc) > 0 && // not off screen
			!(kGroundMask == (world[loc-1]&kGroundMask)) && // not ground or spikes
			render[loc-1] != obj[snake]) // not self
		{
			// check if snake or object which can be pushed
			if ((kSnakeMask == (render[loc-1]&kSnakeMask)) || (kBlockMask == (render[loc-1]&kBlockMask)))
			{
				if (CanPush(s, snake, render[loc-1], kUp, a))
				{
					a.direction = kUp;
					actions.push_back(a);
				}
			}
			else {
				assert(kCanEnterMask == (world[loc-1]&kCanEnterMask));
				a.direction = kUp;
				actions.push_back(a);
			}
		}

		a.pushed = 0;
		// right
		if (GetX(loc)+1 < width && // not off screen
			!(kGroundMask == (world[loc+height]&kGroundMask)) && // not ground or spikes
			render[loc+height] != obj[snake]) // not self
		{
			// check if snake or object which can be pushed
			if ((kSnakeMask == (render[loc+height]&kSnakeMask)) || (kBlockMask == (render[loc+height]&kBlockMask)))
			{
				if (CanPush(s, snake, render[loc+height], kRight, a))
				{
					a.direction = kRight;
					actions.push_back(a);
				}
			}
			else {
				assert(kCanEnterMask == (world[loc+height]&kCanEnterMask));
				a.direction = kRight;
				actions.push_back(a);
			}
		}
		
		a.pushed = 0;
		// left
		if (GetX(loc)-1 >= 0 && // not off screen
			!(kGroundMask == (world[loc-height]&kGroundMask)) && // not ground or spikes
			render[loc-height] != obj[snake]) // not self
		{
			// check if snake or object which can be pushed
			if ((kSnakeMask == (render[loc-height]&kSnakeMask)) || (kBlockMask == (render[loc-height]&kBlockMask)))
			{
				if (CanPush(s, snake, render[loc-height], kLeft, a))
				{
					a.direction = kLeft;
					actions.push_back(a);
				}
			}
			else {
				//std::cout << +world[loc-height] << std::endl;
				assert(kCanEnterMask == (world[loc-height]&kCanEnterMask));
				a.direction = kLeft;
				actions.push_back(a);
			}
		}
	}
}

/**
 * Can the snake push the given object in the given direction?
 * \param s The current state of the game
 * \param snake The snake pushing
 * \param obj The object being pushed
 * \param dir The direction it is pushed
 * \param a The final action, which will contain the objects being used
 */
bool SnakeBird::CanPush(const SnakeBirdState &s, int snake, SnakeBirdWorldObject obj, snakeDir dir,
						SnakeBirdAction &a) const
{
//	if (pushObject >= kMaxPushedObjects)
//		return false;
	int pushed = -1;
	// can't push yourself
	//if (pushObject+1 < kMaxPushedObjects)
	//a.pushed[pushObject+1] = kNothingPushed;
	switch (obj) {
		case kSnake1: if (snake == 0) return false; pushed = 0; break;
		case kSnake2: if (snake == 1) return false; pushed = 1; break;
		case kSnake3: if (snake == 2) return false; pushed = 2; break;
		case kSnake4: if (snake == 3) return false; pushed = 3; break;
		case kBlock1: pushed = 4; break;
		case kBlock2: pushed = 5; break;
		case kBlock3: pushed = 6; break;
		case kBlock4: pushed = 7; break;
		default: return false; break;
	}
	// already pushing this
	if ((a.pushed>>pushed)&0x1)
		return true;
	
	// mark object as being pushed
	a.pushed |= (1<<pushed);
	
	if (pushed <= 3) // snake is pushed
	{
		// Render pushed obj one step in (dir)
		int loc = s.GetSnakeHeadLoc(pushed);
		switch (dir)
		{
			case kLeft: if (loc < height) return false; loc-=height; break;
			case kRight: if (loc+height > width*height) return false; loc+=height; break;
			case kUp: if (loc == 0) return false; loc-=1; break;
			case kDown: if (loc+1 == width*height) return false; loc+=1; break;
		}
		// cannot push into solid object - except when going down
		if ((((world[loc]&kGroundMask) == kGroundMask) && dir != kDown) ||
			(dir == kDown && world[loc] == kGround))
			return false;
		if (render[loc]==kFruit) // cannot push into fruit
			return false;
		if (((render[loc]&kSnakeMask) == kSnakeMask) || (render[loc]&kBlockMask) == kBlockMask) // pushable object
		{
			if (!CanPush(s, snake, render[loc], dir, a))
				return false;
		}
		
		int len = s.GetSnakeBodyEnd(pushed)-s.GetSnakeBodyEnd(pushed-1);
		for (int x = 0; x < len; x++)
		{
			switch (s.GetSnakeDir(pushed, x))
			{
				case kLeft: if (loc < height) return false; loc-=height; break;
				case kRight: if (loc+height > width*height) return false; loc+=height; break;
				case kUp: if (loc == 0) return false; loc-=1; break;
				case kDown: if (loc+1 == width*height) return false; loc+=1; break;
			}
			// TODO: check if snake renders into spikes
			// cannot push into solid object - except when going down
			if ((((world[loc]&kGroundMask) == kGroundMask) && dir != kDown) ||
				(dir == kDown && world[loc] == kGround))
				return false;
			if (render[loc]==kFruit) // cannot push into fruit
				return false;
			if (((render[loc]&kSnakeMask) == kSnakeMask) || (render[loc]&kBlockMask) == kBlockMask) // pushable object
			{
				if (!CanPush(s, snake, render[loc], dir, a))
					return false;
			}
		}
	}
	else if (pushed >= 4) // object is pushed
	{
		pushed-=4;
		// Render pushed obj one step in (dir)
		int loc = s.GetObjectLocation(pushed); // s.GetSnakeHeadLoc(pushed);
		if (loc == kDead)
			return false;
		for (int x = 0; x < objects[pushed].size(); x++)
		{
			int pieceLoc = objects[pushed][x];
			pieceLoc = GetIndex(GetX(objects[pushed][x])+GetX(loc),
								GetY(objects[pushed][x])+GetY(loc));
			switch (dir)
			{
				case kLeft: if (pieceLoc < height) return false; pieceLoc-=height; break;
				case kRight: if (pieceLoc+height > width*height) return false; pieceLoc+=height; break;
				case kUp: if (pieceLoc == 0) return false; pieceLoc-=1; break;
				case kDown: if (pieceLoc+1 == width*height) return false; pieceLoc+=1; break;
			}
			if ((world[pieceLoc]&kGroundMask) == kGroundMask) // cannot push into solid object
				return false;
			if (render[pieceLoc]==kFruit) // cannot push into fruit
				return false;
//			if (s.GetSnakeHeadLoc(snake))
			if (((render[pieceLoc]&kSnakeMask) == kSnakeMask) || (render[pieceLoc]&kBlockMask) == kBlockMask) // pushable object
			{
				if (!CanPush(s, snake, render[pieceLoc], dir, a))
					return false;
			}
		}
	}
	// temporarily return false to not push other snakes
	//return ;
	return true;
}


SnakeBirdAnimation SnakeBird::DoFirstMovement(const SnakeBirdAction &a, int offset,
								snakeDir opposite, SnakeBirdState &s) const
{
	bool didExit = false;
	if (world[s.GetSnakeHeadLoc(a.bird)+offset] == kExit && s.KFruitEaten(fruit.size()))
	{
		// move onto goal
		s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+offset);
		s.InsertSnakeDir(a.bird, opposite);
		
		// stop to animate exiting level]]
		didExit = true;
		// previously returned, but still need to push objects and let them fall
		//return;
	}
	// eating fruit (can't push and eat fruit in the same action)
	else if (world[s.GetSnakeHeadLoc(a.bird)+offset] == kFruit &&
			 s.GetFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+offset)))
	{
		s.ToggleFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+offset));
		for (int x = a.bird; x < s.GetNumSnakes(); x++)
			s.SetSnakeBodyEnd(x, s.GetSnakeBodyEnd(x)+1);
		s.InsertSnakeHeadDir(a.bird, opposite);
		s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+offset);

		// If another snake is on the goal, set didEdit
		if (s.KFruitEaten(fruit.size()))
		{
			for (int x = 0; x < s.GetNumSnakes(); x++)
			{
				if (s.GetSnakeHeadLoc(x) == exitLoc)
				{
					didExit = true;
				}
			}
		}
	}
	else { // normal movement or pushing
		s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+offset);
		s.InsertSnakeDir(a.bird, opposite);
		
		// check for pushed snakes
		for (int i = 0; i < 4 && (a.pushed>>i); i++)
		{
			if ((a.pushed>>i)&0x1)
			{
				// This pushing action could push a snake offscreen, which is illegal
				// So, we check the boundary conditions and kill the snake if it is pushed offscreen
				// Snakes will later be rendered and killed if they are offscreen
				if (s.GetSnakeHeadLoc(i)+offset < 0 || s.GetSnakeHeadLoc(i)+offset >= GetWidth()*GetHeight())
					s.SetSnakeHeadLoc(i, kDead);
				else if (abs(GetX(s.GetSnakeHeadLoc(i))-GetX(s.GetSnakeHeadLoc(i)+offset)) > 1)
					s.SetSnakeHeadLoc(i, kDead);
				else if (abs(GetY(s.GetSnakeHeadLoc(i))-GetY(s.GetSnakeHeadLoc(i)+offset)) > 1)
					s.SetSnakeHeadLoc(i, kDead);
				else
					s.SetSnakeHeadLoc(i, s.GetSnakeHeadLoc(i)+offset);

				if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
				{
					didExit = true;
					//s.SetSnakeHeadLoc(i, kInGoal);
				}
			}
		}
		// check for pushed objects
		for (int i = 4; i < 8 && (a.pushed>>i); i++)
		{
			if ((a.pushed>>i)&0x1)
			{
				if (s.GetObjectLocation(i-4) == kDead)
					continue;
				int newloc = s.GetObjectLocation(i-4)+offset;
				s.SetObjectLocation(i-4, newloc);
			}
		}
	}
	if (didExit)
		return kWentInGoal;
	else
		return kInitialTeleport;

}

SnakeBirdAnimation SnakeBird::DoFall(SnakeBirdAction &a, SnakeBirdState &s) const
{
	const SnakeBirdWorldObject objs[8] = {kSnake1, kSnake2, kSnake3, kSnake4, kBlock1, kBlock2, kBlock3, kBlock4};
	uint8_t falling = 0;
	SnakeBirdAction move;
	// Find which objects can fall
	for (int i = 0; i < 8; i++)
	{
		move.pushed = 0;
		// snake isn't in the world
		if ((i < 4 && i >= s.GetNumSnakes()) || s.GetSnakeHeadLoc(i) == kInGoal)
			continue;
		// block isn't in the world (fell out or not there to begin with)
		if (i >= 4 && (objects[i-4].size() == 0 || s.GetObjectLocation(i-4) == kDead))
			continue;
		
		if (CanPush(s, -1, objs[i], kDown, move))
		{
			falling |= move.pushed;
		}
	}
	// If nothing can fall, we are done
	if (falling == 0)
		return kDoneAnimation;
	
	// Actually move the objects down
	bool hitExit = false;
	for (int i = 0; i < 8; i++)
	{
		if (i < 4 && ((falling>>i)&0x1))
		{
			s.SetSnakeHeadLoc(i, s.GetSnakeHeadLoc(i)+1);
			if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
			{
				hitExit = true;
//				s.SetSnakeHeadLoc(i, kInGoal);
			}
		}
		if (i >= 4 && ((falling>>i)&0x1))
			s.SetObjectLocation(i-4, s.GetObjectLocation(i-4)+1);
	}

	a.pushed = falling;
	
	// check if an object hit the bottom of the world - if so it is dead
	for (size_t i = 0; i < objects.size(); i++)
	{
		int objLoc = s.GetObjectLocation(i);
		if (objLoc == kDead)
			continue;
		int objectY = GetY(objLoc);
		for (size_t j = 0; j < objects[i].size(); j++)
		{
			if (objectY+GetY(objects[i][j]) >= height-2)
			{
				s.SetObjectLocation(i, kDead);
				break;
			}
		}
	}

	if (hitExit)
		return kFellInGoal;

	return kTeleport;
}

/**
 * Do next step of animation. Call until true is returned (when action is fully applied).
 */
bool SnakeBird::ApplyPartialAction(SnakeBirdState &s, SnakeBirdAction act, SnakeBirdAnimationStep &step) const
{
//	ApplyAction(s, act);
//	return true;
	if (step.anim == kPauseWhenDead)
		return true;

	
	if (step.anim == kNeedsInitialization)
	{
		step.anim = kMovement;
		step.a = act;
	}
	int offset;
	snakeDir opposite;
	snakeDir lastAction = (snakeDir)act.direction;
	switch (step.a.direction)
	{
		case kLeft: offset = -height; opposite = kRight; break;
		case kRight: offset = +height; opposite = kLeft; break;
		case kUp: offset = -1; opposite = kDown; break;
		case kDown: offset = 1; opposite = kUp; break;
		default: printf("Warning: Applied illegal action\n"); return true;
	}
	
	if (step.anim == kMovement)
	{
		// exited level
		step.anim = DoFirstMovement(step.a, offset, opposite, s);
		step.animationDuration = 0.1;
		return false;
	}

	if (step.anim == kWentInGoal)
	{
		for (int i = 0; i < 4; i++)
		{
			if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(i, kInGoal);
				step.animationDuration = s.GetSnakeLength(i)*0.1;
				if (step.animationDuration < 0)
					step.animationDuration = 0.01;
				break;
			}
		}
		step.anim = kInitialTeleport;
		return false;
	}

	
	// Small hack to make the following loop more generic:
	// Add current bird to list of pushed birds
	// This will cause us to check it for portals
	if (step.anim == kInitialTeleport)
	{
		step.a.pushed |= (1<<step.a.bird);
	}
	else {
		lastAction = kDown;
		opposite = kUp;
	}
	
	// Now for gravity and portals.
	// 1. Check if any portals are activated.
	// 2. Then try all moved objects to see if they can be pushed down.
	// 2a. Merge the pushable objects into a single bit vector
	// 3. If no objects moved, stop; else repeat to 1.
	bool success = Render(s);
	if (!success) // someone died as a result of the action
	{
		step.animationDuration = 0.5; // then undo automatically
		step.anim = kPauseWhenDead;
		return false;
	}

	if (step.anim == kTeleport || step.anim == kInitialTeleport)
	{
		auto didTeleport = HandleTeleports(s, step.a, lastAction, opposite, step);
		if (didTeleport == kTeleportSuccess)
		{
			step.anim = kFall;
			step.animationDuration = 0.1;
			step.teleportCount++;
			return false;
		}
		else if (didTeleport == kTeleportToExit)
		{
			step.anim = kFellInGoal;
			step.animationDuration = 0.1;
			return false;
		}
		step.anim = kFall;
	}

	if (step.teleportCount > 10)
	{
		for (int x = 0; x < s.GetNumSnakes(); x++)
		s.SetSnakeHeadLoc(x, kDead);
		step.animationDuration = 0.5; // then undo automatically
		step.anim = kPauseWhenDead;
		return false;
	}

	if (step.anim == kFall)
	{
		step.anim = DoFall(step.a, s);
		if (step.anim == kDoneAnimation)
			return true;
		step.animationDuration = 0.05; // then undo automatically
		return false;
	}
	
	if (step.anim == kFellInGoal)
	{
		for (int i = 0; i < 4; i++)
		{
			if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(i, kInGoal);
				step.animationDuration = s.GetSnakeLength(i)*0.1;
				if (step.animationDuration < 0)
					step.animationDuration = 0.01;
				break;
			}
		}
		step.anim = kFall;
	}

	return false;
}

/**
 * Apply a single action, modifying to the next state.
 */
//SnakeBirdAction GetAction(const SnakeBirdState &s1, const SnakeBirdState &s2) const;
void SnakeBird::ApplyAction(SnakeBirdState &s, SnakeBirdAction a) const
{
	SnakeBirdAnimationStep step;
	step.anim = kMovement;
	int offset;
	snakeDir opposite;
	snakeDir lastAction = (snakeDir)a.direction;
	switch (a.direction)
	{
		case kLeft: offset = -height; opposite = kRight; break;
		case kRight: offset = +height; opposite = kLeft; break;
		case kUp: offset = -1; opposite = kDown; break;
		case kDown: offset = 1; opposite = kUp; break;
		default: printf("Warning: Applied illegal action\n"); return;
	}
	
	if (step.anim == kMovement)
	{
		// exited level
		step.anim = DoFirstMovement(a, offset, opposite, s);
	}
	
	if (step.anim == kWentInGoal)
	{
		for (int i = 0; i < 4; i++)
		{
			if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(i, kInGoal);
			}
		}
		step.anim = kInitialTeleport;
	}

	// Small hack to make the following loop more generic:
	// Add current bird to list of pushed birds
	// This will cause us to check it for portals
	a.pushed |= (1<<a.bird);
	
	// Now for gravity and portals.
	// 1. Check if any portals are activated.
	// 2. Then try all moved objects to see if they can be pushed down.
	// 2a. Merge the pushable objects into a single bit vector
	// 3. If no objects moved, stop; else repeat to 1.
	while (true) {
		bool success = Render(s);
		if (!success) // someone died as a result of the action
			return;

		if (step.anim == kTeleport)
		{
			auto didTeleport = HandleTeleports(s, a, kDown, kUp, step);
			if (didTeleport == kTeleportSuccess)
			{
				step.teleportCount++;
				step.anim = kFall;
			}
			else if (didTeleport == kTeleportToExit)
			{
				step.anim = kFellInGoal;
			}
			else {
				step.anim = kFall;
			}
		}
		else if (step.anim == kInitialTeleport) {
			auto didTeleport = HandleTeleports(s, a, lastAction, opposite, step);
			if (didTeleport == kTeleportSuccess)
			{
				step.teleportCount++;
				step.anim = kFall;
			}
			else if (didTeleport == kTeleportToExit)
			{
				step.anim = kFellInGoal;
			}
			else {
				step.anim = kFall;
			}
		}
		
		if (step.teleportCount > 10)
		{
			for (int x = 0; x < s.GetNumSnakes(); x++)
			s.SetSnakeHeadLoc(x, kDead);
			return;
		}
		
		if (step.anim == kFall)
		{
			step.anim = DoFall(a, s);
			if (step.anim == kDoneAnimation)
				return;
		}
		
		if (step.anim == kFellInGoal)
		{
			// check if we died at the same time
			bool success = Render(s);
			if (!success)
				return;
			
			for (int i = 0; i < 4; i++)
			{
				if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
				{
					s.SetSnakeHeadLoc(i, kInGoal);
				}
			}
			step.anim = kFall;
		}

		//		step.anim = kTeleport;
	}
}

TeleportResult SnakeBird::HandleTeleports(SnakeBirdState &s, SnakeBirdAction &a,
										  snakeDir lastAction, snakeDir opposite, SnakeBirdAnimationStep step) const
{
	const SnakeBirdWorldObject objs[8] = {kSnake1, kSnake2, kSnake3, kSnake4, kBlock1, kBlock2, kBlock3, kBlock4};
	bool didTeleport = false;
	bool didExit = false;
	// Check for snakes which are now on portals
	// *and* didn't have an adjacent segment on a portal in the last time step
	for (int i = 0; i < 4 && (a.pushed>>i); i++)
	{
		int snake = i;
		if ((a.pushed>>i)&0x1) // ignore if it wasn't moved
		{
			int newloc = s.GetSnakeHeadLoc(snake);
			int piece = newloc;
			for (int x = 0; x < s.GetSnakeLength(snake); x++)
			{
				//int lastPiece = piece;
				// in the first round of gravity/portals
				// only check the head of the bird that was moved
				if (i == a.bird && step.anim == kInitialTeleport && x > 0)
				{
					break;
				}
				if (x > 0)
				{
					switch (s.GetSnakeDir(snake, x-1))
					{
						case kLeft: piece-=height; break;
						case kRight: piece+=height; break;
						case kUp: piece-=1; break;
						case kDown: piece+=1; break;
					}
				}
				int p1, p2 = -1;
				if (piece == portal1Loc)
				{
					p1 = portal1Loc;
					p2 = portal2Loc;
				}
				else if (piece == portal2Loc)
				{
					p1 = portal2Loc;
					p2 = portal1Loc;
				}

				// Did the snake just move to adjacent locations?
				if (p2 != -1)
				{
					// We are looking at the snake location after it was moved.
					// Need to check in forward and backwards directions
					if (x > 0 && s.GetSnakeDir(snake, x-1) == opposite)
					{ p2 = -1; }
					else if (x < s.GetSnakeLength(snake)-1 && s.GetSnakeDir(snake, x) == lastAction)
					{ p2 = -1; }
				}

				// We have a portal to teleport to
				if (p2 != -1)
				{
					int xChange = -GetX(p1)+GetX(p2);
					int yChange = -GetY(p1)+GetY(p2);

					// Validate if the object can be placed in the new location
					bool valid = true;
					int newPiece = GetIndex(GetX(newloc)+xChange, GetY(newloc)+yChange);
					//for (int v = 0; v < objects[snake].size(); v++)
					for (int v = 0; v < s.GetSnakeLength(snake); v++)
					{
						if (v > 0)
						{
							switch (s.GetSnakeDir(snake, v-1))
							{
								case kLeft: newPiece-=height; break;
								case kRight: newPiece+=height; break;
								case kUp: newPiece-=1; break;
								case kDown: newPiece+=1; break;
							}
						}

						if (GetX(newPiece) < 0 ||
							GetX(newPiece) >= width ||
							GetY(newPiece) < 0 ||
							GetY(newPiece) >= height)
						{
							valid = false;
							break;
						}
						
						if (((render[newPiece]&kSnakeMask) == kSnakeMask &&
							 render[newPiece] != objs[snake]) || // hit another snake
							((world[newPiece]&kGroundMask) == kGroundMask) || // hit ground
							((render[newPiece]&kBlockMask) == kBlockMask) || // hit piece
							(render[newPiece] == kFruit)) // hit fruit
						{
							valid = false;
							break;
						}
					}
					if (valid)
					{
						int offsetFromPortalX = GetX(newloc)+xChange;
						int offsetFromPortalY = GetY(newloc)+yChange;
						s.SetSnakeHeadLoc(i, GetIndex(offsetFromPortalX, offsetFromPortalY));
						if (s.GetSnakeHeadLoc(i) == exitLoc && s.KFruitEaten(fruit.size()))
							didExit = true;
						didTeleport = true;
						bool success = Render(s);
						assert(success == true);
//						if (!success) // someone died as a result of the action
//							return;
					}
				}
			}
		}
	}
	
	// Check for objects which are now on portals
	// and didn't have an adjacent segment on a portal in the last step
	// If an object teleports we have to re-render
	// Note - only one object can teleport, so don't check if a snake teleported
	for (int i = 4; i < 8 && (a.pushed>>i) && !didTeleport; i++)
	{
		int object = i-4;
		if ((a.pushed>>i)&0x1) // ignore if it wasn't moved
		{
			int newloc = s.GetObjectLocation(object);
			if (newloc == kDead)
				continue;
			for (size_t x = 0; x < objects[object].size(); x++)
			{
				int piece = GetIndex(GetX(objects[object][x])+GetX(newloc),
									 GetY(objects[object][x])+GetY(newloc));
				int p1=-1, p2 = -1;
				if (piece == portal1Loc)
				{
					p1 = portal1Loc;
					p2 = portal2Loc;
				}
				else if (piece == portal2Loc)
				{
					p1 = portal2Loc;
					p2 = portal1Loc;
				}
				// Check if our last action suggests that the object was in the portal previously
				if (p2 != -1)
				{
					switch (lastAction)
					{
						case kLeft:
							if (p1-height >= 0 && render[p1-height] == objs[i]) // can't teleport
								p2 = -1;
							break;
						case kRight:
							if (p1+height < width*height && render[p1+height] == objs[i]) // can't teleport
								p2 = -1;
							break;
						case kUp:
							if (p1-1 >= 0 && render[p1-1] == objs[i]) // can't teleport
								p2 = -1;
							break;
						case kDown:
							if (p1+1 < width*height && render[p1+1] == objs[i]) // can't teleport
								p2 = -1;
							break;
					}
				}
				
				if (p2 != -1)
				{
					// TODO: Check if it can be placed in new loc relative to self
					// TODO: Check if it is offscreen
					int xChange = -GetX(p1)+GetX(p2);
					int yChange = -GetY(p1)+GetY(p2);

					// Validate if the object can be placed in the new location
					bool valid = true;
					for (size_t v = 0; v < objects[object].size(); v++)
					{
						if (GetX(newloc)+GetX(objects[object][v])+xChange < 0 ||
							GetX(newloc)+GetX(objects[object][v])+xChange >= width ||
							GetY(newloc)+GetY(objects[object][v])+yChange < 0 ||
							GetY(newloc)+GetY(objects[object][v])+yChange >= height)
						{
							valid = false;
							break;
						}
						int newPiece = GetIndex(GetX(newloc)+GetX(objects[object][v])+xChange,
												GetY(newloc)+GetY(objects[object][v])+yChange);
						if (((render[newPiece]&kSnakeMask) == kSnakeMask) || // hit snake
							((world[newPiece]&kGroundMask) == kGroundMask) || // hit ground
							((render[newPiece]&kBlockMask) == kBlockMask && render[newPiece] != objs[i])) // hit another piece
						{
							valid = false;
							break;
						}
					}
					if (valid)
					{
						int offsetFromPortalX = GetX(newloc)+xChange;
						int offsetFromPortalY = GetY(newloc)+yChange;
						s.SetObjectLocation(object, GetIndex(offsetFromPortalX, offsetFromPortalY));
						didTeleport = true;

						bool success = Render(s);
						assert(success == true);
//						if (!success) // someone died as a result of the action
//							return;
					}
				}
			}
		}
	}
	if (didExit && didTeleport)
		return kTeleportToExit;
	if (didTeleport)
		return kTeleportSuccess;
	return kNoTeleport;
//	return didTeleport;
}

void SnakeBird::SetGroundType(int index, SnakeBirdWorldObject o)
{
	SetGroundType(GetX(index), GetY(index), o);
}

/**
 * Similar to SetGroundType but only deals with removing blocks
 */
void SnakeBird::RemoveBlock(int x, int y)
{
	for (int which = 0; which < 4; which++)
	{
		if (objects[which].size() <= 0) // no other objects
			continue;

		int xOffset = 0, yOffset = 0;
		xOffset = GetX(startState.GetObjectLocation(which));
		yOffset = GetY(startState.GetObjectLocation(which));

		for (size_t i = 0; i < objects[which].size(); i++)
		{
			if (GetX(objects[which][i])+xOffset == x &&
				GetY(objects[which][i])+yOffset == y) // remove
			{
				objects[which][i] = objects[which].back();
				objects[which].pop_back();
				break;
			}
		}
		// quit loop if we removed last object
		if (objects[which].size() <= 0)
		{
			continue;
		}

		int newX = width, newY=height;
		
		// Get new base location (minx/y)
		for (size_t i = 0; i < objects[which].size(); i++)
		{
			newX = std::min(newX, GetX(objects[which][i])+xOffset);
			newY = std::min(newY, GetY(objects[which][i])+yOffset);
		}
		startState.SetObjectLocation(which, GetIndex(newX, newY));
		for (size_t i = 0; i < objects[which].size(); i++)
		{
			// reset locations based on new base location
			objects[which][i] = GetIndex(GetX(objects[which][i])+xOffset - newX,
										 GetY(objects[which][i])+yOffset - newY);
		}
	}
}

/**
 * Change the ground type to a new type. Must enable editing to have an impact.
 */
void SnakeBird::SetGroundType(int x, int y, SnakeBirdWorldObject o)
{
	if (!editing)
	{
		printf("!!WARNING: Cannot change map without enabling editing!!\n");
		return;
	}
	// TODO: check duplicates and removals for fruit and blocks
	if (y == height-1)
	{
		printf("Error - cannot add terrain at bottom of map\n");
		return;
	}
	
	if ((o&kBlockMask) == kBlockMask) // are you adding a moveable block?
	{
		int which = -1;
		switch (o)
		{
			case kBlock1:
				which = 0;
				break;
			case kBlock2:
				which = 1;
				break;
			case kBlock3:
				which = 2;
				break;
			case kBlock4:
				which = 3;
				break;
			default:
				assert(!"Can't get here - object is a block");
				break;
		}
		assert(which != -1);
		// objects are stored as offsets from 0.
		// The offset in the original world is in startState
		int newX = x, newY=y;
		int xOffset = 0, yOffset = 0;
		if (objects[which].size() > 0) // other objects - get their offset base
		{
			xOffset = GetX(startState.GetObjectLocation(which));
			yOffset = GetY(startState.GetObjectLocation(which));
		}
		
		// Get new base location (minx/y)
		for (int i = 0; i < objects[which].size(); i++)
		{
			newX = std::min(newX, GetX(objects[which][i])+xOffset);
			newY = std::min(newY, GetY(objects[which][i])+yOffset);
		}
		startState.SetObjectLocation(which, GetIndex(newX, newY));
		for (int i = 0; i < objects[which].size(); i++)
		{
			// reset locations based on new base location
			objects[which][i] = GetIndex(GetX(objects[which][i])+xOffset - newX,
										 GetY(objects[which][i])+yOffset - newY);
		}
		// add new piece of new object
		objects[which].push_back(GetIndex(x-newX, y-newY));
		
		return;
	}
	
	if (o == kPortal)
	{
		if ((portal1Loc == -1) || (portal1Loc == GetIndex(x, y)))
		{
			o = kPortal1;
		}
		else if ((portal2Loc == -1) || (portal2Loc == GetIndex(x, y)))
		{
			o = kPortal2;
		}
		else {
			printf("error cannot determine portal type");
			return;
		}
	}
	auto oldTerrainType = world[GetIndex(x, y)];
	world[GetIndex(x, y)] = o;
	// TODO: IF old ground was fruit, find it in the fruit array and remove it by
	// taking the last item out of the array  and putting it in place of the one removed,
	// and then shrinking the size by one. -> resize(size()-1)
	
	// TODO: If old type was the exit, reset the exit location to -1.
	
	if (oldTerrainType == kFruit)
	{
		std::vector<int>::iterator tor = std::find(fruit.begin(), fruit.end(), GetIndex(x, y));
		if (tor != fruit.end())
		{
			fruit.erase(tor);
		}
	}
	if (o == kFruit)
		fruit.push_back(GetIndex(x, y));
	if (oldTerrainType == kExit)
		exitLoc = -1;
	if (o == kExit)
	{
		if (exitLoc != -1 && exitLoc != GetIndex(x, y))
			world[exitLoc] = kEmpty;
		exitLoc = GetIndex(x, y);
	}
	if (o == kPortal1)
	{
		if (portal1Loc != -1 && portal1Loc != GetIndex(x, y))
			world[portal1Loc] = kEmpty;
		portal1Loc = GetIndex(x, y);
	}
	if (o == kPortal2)
	{
		if (portal2Loc != -1 && portal2Loc != GetIndex(x, y))
			world[portal2Loc] = kEmpty;
		portal2Loc = GetIndex(x, y);
	}
	if (oldTerrainType == kPortal1 && portal1Loc == GetIndex(x, y))
	{
		world[portal1Loc] = kEmpty;
		portal1Loc = -1;
	}
	if (oldTerrainType == kPortal2 && portal2Loc == GetIndex(x, y))
	{
		world[portal2Loc] = kEmpty;
		portal2Loc = -1;
	}
}

int SnakeBird::GetNumPortals()
{
	int cnt = 0;
	if (portal1Loc != -1)
		cnt++;
	if (portal2Loc != -1)
		cnt++;
	return cnt;
//	if (portal1Loc == -1 || portal2Loc == -1)
//		return true;
//	else
//		return false;
}

int SnakeBird::GetFruitOffset(int index) const
{
	for (int x = 0; x < fruit.size(); x++)
		if (fruit[x] == index)
			return x;
	assert(false);
	return -1;
}


/**
 * Return the primary ground type in the map.
 */
SnakeBirdWorldObject SnakeBird::GetGroundType(int x, int y) const
{
	return world[GetIndex(x, y)];
}

/**
 * Return the rendered ground type on the screen - eg accounting for snakes, moving objects, etc.
 */
SnakeBirdWorldObject SnakeBird::GetRenderedGroundType(const SnakeBirdState &s, int x, int y)
{
	if (GetGroundType(x, y) != kEmpty)// || GetGroundType(x, y) == kPortal1 || GetGroundType(x, y) == kPortal2)
		return GetGroundType(x, y);
	bool success = Render(s);
	if (!success)
		return kExit;
	//	assert(success);
	return render[GetIndex(x, y)];
}


int SnakeBird::GetIndex(int x, int y) const
{
	return x*height+y;
}

int SnakeBird::GetX(int index) const
{
	return index/height;
}

int SnakeBird::GetY(int index) const
{
	return index%height;
}

int SnakeBird::Distance(int index1, int index2)
{
	int x1 = GetX(index1);
	int y1 = GetY(index1);
	int x2 = GetX(index2);
	int y2 = GetY(index2);
	return  abs(x1-x2)+abs(y1-y2);
}

void SnakeBird::Draw(Graphics::Display &display, int x, int y, float width) const
{
	Graphics::point p = GetCenter(x, y);
	display.FrameRect({p, GetRadius()}, GetColor(), GetRadius()*0.3*width);
}

void SnakeBird::Draw(Graphics::Display &display) const
{
	display.FillRect({-1.1, -1.1, 1.1, 1.1}, rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5));
	Graphics::point tmp = GetCenter(width, height);
	display.FillRect({-1.1, tmp.y-GetRadius(), 1.1, 1.1}, Colors::darkblue+Colors::darkred);
	for (int x = 0; x < width*height; x++)
	{
		Graphics::point p = GetCenter(GetX(x), GetY(x));
		float radius = GetRadius()*0.95;
		switch (world[x])
		{
			case kEmpty:
				break;//display.FillSquare(p, GetRadius(), Colors::lightblue);  break;
			case kGround:
				switch ((GetX(x)*13*+11*GetY(x))%6)
				{
					case 0: display.FillSquare(p, GetRadius(), Colors::brown); break;
					case 1: display.FillSquare(p, GetRadius(), Colors::brown); break;
					case 2: display.FillSquare(p, GetRadius(), Colors::brown+Colors::gray); break;
					case 3: display.FillSquare(p, GetRadius(), Colors::brown+Colors::gray); break;
					case 4: display.FillSquare(p, GetRadius(), Colors::brown*0.8+Colors::darkgreen); break;
					case 5: display.FillSquare(p, GetRadius(), Colors::brown*0.8+Colors::darkgreen); break;
				}
				break;
			case kSpikes:
				if (GetY(x) != height-1)
				{
//					radius = GetRadius();
//					display.FillNGon(p, radius, 3, 0, Colors::darkgray);
//					display.FillNGon(p, radius, 3, 60, Colors::gray*0.75f);
					display.FillSquare({p.x, p.y+radius/2.0f}, radius/2.0f, Colors::darkgray);
					display.FillTriangle({p.x-radius/2.0f, p.y}, {p.x-radius/4.0f, p.y-radius/2}, {p.x, p.y}, Colors::darkred);
					display.FillTriangle({p.x+radius/2.0f, p.y}, {p.x+radius/4.0f, p.y-radius/2}, {p.x, p.y}, Colors::darkred);

					display.FillTriangle({p.x-radius/2.0f, p.y}, {p.x-radius, p.y+radius/4}, {p.x-radius/2.0f, p.y+radius/2}, Colors::darkred);
					display.FillTriangle({p.x-radius/2.0f, p.y+radius/2}, {p.x-radius, p.y+3*radius/4}, {p.x-radius/2.0f, p.y+radius}, Colors::darkred);

					display.FillTriangle({p.x+radius/2.0f, p.y}, {p.x+radius, p.y+radius/4}, {p.x+radius/2.0f, p.y+radius/2}, Colors::darkred);
					display.FillTriangle({p.x+radius/2.0f, p.y+radius/2}, {p.x+radius, p.y+3*radius/4}, {p.x+radius/2.0f, p.y+radius}, Colors::darkred);
				}
				break;
			default: break;
		}
	}
}

void SnakeBird::DrawObjects(Graphics::Display &display, double time) const
{
	for (int x = 0; x < width*height; x++)
	{
		Graphics::point p = GetCenter(GetX(x), GetY(x));
		double radius = GetRadius()*0.95;
		switch (world[x])
		{
			case kEmpty:
				break;//display.FillSquare(p, GetRadius(), Colors::lightblue);  break;
			case kGround:
				display.FillSquare(p, GetRadius(), Colors::brown); break;
				break;
			case kSpikes:
				if (GetY(x) != height-1)
				{
					display.FillNGon(p, radius, 3, 0, Colors::darkgray);
					display.FillNGon(p, radius, 3, 60, Colors::gray*0.75f);
				}
				break;
			case kPortal1:
			case kPortal2:
			{
				double radius = GetRadius()*0.95;
				Graphics::point p = GetCenter(GetX(x), GetY(x));
				double offset1 = (sin(time*1.0)); // -1...+1
				double offset2 = (sin(time*1.5)); // -1...+1
				double offset3 = (sin(time*2.0)); // -1...+1
				double offset4 = (sin(time*2.5)); // -1...+1
				display.FillCircle(p, radius+offset1*radius*0.25, Colors::red);
				display.FillCircle(p, radius*0.75+offset2*radius*0.2, Colors::blue);
				display.FillCircle(p, radius*0.5+offset3*radius*0.15, Colors::green);
				display.FillCircle(p, radius*0.25+offset4*radius*0.1, Colors::purple);
			}
				break;
			case kFruit:
			{
				const float rows = 5;
				const float counts[] = {3, 4, 3, 2, 1};
				Graphics::point p = GetCenter(GetX(x), GetY(x));
				float grapeRadius = GetRadius()*0.15f;
				for (int t = 0; t < rows; t++)
				{
					for (int v = 0; v < counts[t]; v++)
					{
						Graphics::point tmp = p;
						tmp.y -= rows*grapeRadius;
						tmp.y += 2*t*grapeRadius;
						tmp.x = tmp.x-grapeRadius*(counts[t]-1)+v*grapeRadius*2;
						tmp.x += sin(time*1.1+(t+v)*0.1)*GetRadius()*0.1;
						tmp.y += cos(time*0.95+(t+v)*0.1)*GetRadius()*0.1;
						display.FillCircle(tmp, grapeRadius*1.1, Colors::purple*sin(time*1.95+v+t));
					}
				}
			}
				break;
			case kExit:
			{
				Graphics::point p = GetCenter(GetX(exitLoc), GetY(exitLoc));
				double radius = GetRadius()*0.95;
				double localTime = time;
				localTime /= 4.0;
				double offset1 = (sin(localTime*1.0))*180; // -1...+1
				double offset2 = (sin(localTime*1.5))*180; // -1...+1
				double offset3 = (sin(localTime*2.0))*180; // -1...+1
				display.FillNGon(p, radius, 5, 0+offset1, Colors::yellow);
				display.FillNGon(p, radius*0.66, 5, 36+offset2, Colors::orange);
				display.FillNGon(p, radius*0.25, 5, 54+offset3, Colors::red);
				break;
			}
			default: break;
		}
	}
}

void SnakeBird::DrawObject(Graphics::Display &display, int x, int y, SnakeBirdWorldObject o, double time) const
//TODO: add ksnake1 - call the draw command for drawing the snakes - search for ksnake1
{
	rgbColor objColors[4] = {Colors::red*0.75, Colors::blue*0.75, Colors::green*0.75, Colors::yellow*0.75};
	Graphics::point p = GetCenter(x, y);
	float radius = GetRadius()*0.95;
	switch (o)
	{
		case kBlock1:
		{
			radius = GetRadius();
			Graphics::point p = GetCenter(x, y);
			display.FrameSquare(p, radius-radius*0.3, objColors[0], radius*0.6);
		}
			break;
		case kBlock2:
		{
			radius = GetRadius();
			Graphics::point p = GetCenter(x, y);
			display.FrameSquare(p, radius-radius*0.3, objColors[1], radius*0.6);
		}
			break;
		case kBlock3:
		{
			radius = GetRadius();
			Graphics::point p = GetCenter(x, y);
			display.FrameSquare(p, radius-radius*0.3, objColors[2], radius*0.6);
		}
			break;
		case kBlock4:
		{
			radius = GetRadius();
			Graphics::point p = GetCenter(x, y);
			display.FrameSquare(p, radius-radius*0.3, objColors[3], radius*0.6);
		}
			break;
		case kEmpty:
			display.FillSquare(p, GetRadius(), rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5)); break;
			break;
		case kGround:
			display.FillSquare(p, GetRadius(), Colors::brown+Colors::white*0.75); break;
			break;
		case kSpikes:
			if (y != height-1)
			{
//				display.FillNGon(p, radius, 3, 0, Colors::darkgray);
//				display.FillNGon(p, radius, 3, 60, Colors::gray*0.75f);
				display.FillSquare({p.x, p.y+radius/2.0f}, radius/2.0f, Colors::darkgray);
				display.FillTriangle({p.x-radius/2.0f, p.y}, {p.x-radius/4.0f, p.y-radius/2}, {p.x, p.y}, Colors::darkred);
				display.FillTriangle({p.x+radius/2.0f, p.y}, {p.x+radius/4.0f, p.y-radius/2}, {p.x, p.y}, Colors::darkred);

				display.FillTriangle({p.x-radius/2.0f, p.y}, {p.x-radius, p.y+radius/4}, {p.x-radius/2.0f, p.y+radius/2}, Colors::darkred);
				display.FillTriangle({p.x-radius/2.0f, p.y+radius/2}, {p.x-radius, p.y+3*radius/4}, {p.x-radius/2.0f, p.y+radius}, Colors::darkred);

				display.FillTriangle({p.x+radius/2.0f, p.y}, {p.x+radius, p.y+radius/4}, {p.x+radius/2.0f, p.y+radius/2}, Colors::darkred);
				display.FillTriangle({p.x+radius/2.0f, p.y+radius/2}, {p.x+radius, p.y+3*radius/4}, {p.x+radius/2.0f, p.y+radius}, Colors::darkred);
			}
			break;
		case kPortal1:
		case kPortal2:
		{
			double radius = GetRadius()*0.95;
			double offset1 = (sin(time*1.0)); // -1...+1
			double offset2 = (sin(time*1.5)); // -1...+1
			double offset3 = (sin(time*2.0)); // -1...+1
			double offset4 = (sin(time*2.5)); // -1...+1
			display.FillCircle(p, radius+offset1*radius*0.25, Colors::red);
			display.FillCircle(p, radius*0.75+offset2*radius*0.2, Colors::blue);
			display.FillCircle(p, radius*0.5+offset3*radius*0.15, Colors::green);
			display.FillCircle(p, radius*0.25+offset4*radius*0.1, Colors::purple);
		}
			break;
		case kFruit:
		{
			const float rows = 5;
			const float counts[] = {3, 4, 3, 2, 1};
			float grapeRadius = GetRadius()*0.15f;
			for (int t = 0; t < rows; t++)
			{
				for (int v = 0; v < counts[t]; v++)
				{
					Graphics::point tmp = p;
					tmp.y -= rows*grapeRadius;
					tmp.y += 2*t*grapeRadius;
					tmp.x = tmp.x-grapeRadius*(counts[t]-1)+v*grapeRadius*2;
					tmp.x += sin(time*1.1+(t+v)*0.1)*GetRadius()*0.1;
					tmp.y += cos(time*0.95+(t+v)*0.1)*GetRadius()*0.1;
					display.FillCircle(tmp, grapeRadius*1.1, Colors::purple*sin(time*1.95+v+t));
				}
			}
			break;
		}
		case kExit:
		{
			double radius = GetRadius()*0.95;
			double localTime = time;
			localTime /= 4.0;
			double offset1 = (sin(localTime*1.0))*180; // -1...+1
			double offset2 = (sin(localTime*1.5))*180; // -1...+1
			double offset3 = (sin(localTime*2.0))*180; // -1...+1
			display.FillNGon(p, radius, 5, 0+offset1, Colors::yellow);
			display.FillNGon(p, radius*0.66, 5, 36+offset2, Colors::orange);
			display.FillNGon(p, radius*0.25, 5, 54+offset3, Colors::red);
			break;
		}
		case kSnake1:
		{
			int snake = 0;
			rgbColor color = Colors::red;
			float smallRadius = 0.75*GetRadius();
			Graphics::rect r(p, GetRadius());
			r.top+=smallRadius;
			r.bottom-=smallRadius;
			display.FillRect(r, color);
			r.top-=smallRadius;
			r.bottom+=smallRadius;
			r.left+=smallRadius;
			r.right-=smallRadius;
			display.FillRect(r, color);
			
			display.FillCircle(p+Graphics::point(smallRadius-GetRadius(), smallRadius-GetRadius()), smallRadius, color);
			display.FillCircle(p+Graphics::point(smallRadius-GetRadius(), -smallRadius+GetRadius()), smallRadius, color);
			display.FillSquare(p+Graphics::point(-GetRadius()/2+GetRadius(), GetRadius()/2-GetRadius()), GetRadius()/2, color);
			display.FillSquare(p+Graphics::point(-GetRadius()/2+GetRadius(), -GetRadius()/2+GetRadius()), GetRadius()/2, color);
			
			float eyescale = 1+snake/5.0;
			rgbColor tmp = color*0.5;
			tmp = Colors::white;
			// draw both eyes
			p.x+=GetRadius()*0.2;
			display.FillCircle(p, GetRadius()*0.2*eyescale, tmp);
			p.x-=2*GetRadius()*0.2;
			display.FillCircle(p, GetRadius()*0.2*eyescale, tmp);
			p.x+=2*GetRadius()*0.2;
			
			// draw iris
			display.FillCircle(p, GetRadius()*0.1*eyescale, Colors::black);
			p.x-=2*GetRadius()*0.2;
			display.FillCircle(p, GetRadius()*0.1*eyescale, Colors::black);
			
			p.x+=GetRadius()*0.2;
			p.y+=2*GetRadius()*0.25;
			display.FillNGon(p, GetRadius()*0.3, 3, 240, Colors::orange);
			break;
		}
		case kSnake2:
		{
			int snake = 1;
			rgbColor color = Colors::blue;
			float smallRadius = 0.75*GetRadius();
			Graphics::rect r(p, GetRadius());
			r.top+=smallRadius;
			r.bottom-=smallRadius;
			display.FillRect(r, color);
			r.top-=smallRadius;
			r.bottom+=smallRadius;
			r.left+=smallRadius;
			r.right-=smallRadius;
			display.FillRect(r, color);
			
			display.FillCircle(p+Graphics::point(smallRadius-GetRadius(), smallRadius-GetRadius()), smallRadius, color);
			display.FillCircle(p+Graphics::point(smallRadius-GetRadius(), -smallRadius+GetRadius()), smallRadius, color);
			display.FillSquare(p+Graphics::point(-GetRadius()/2+GetRadius(), GetRadius()/2-GetRadius()), GetRadius()/2, color);
			display.FillSquare(p+Graphics::point(-GetRadius()/2+GetRadius(), -GetRadius()/2+GetRadius()), GetRadius()/2, color);
			
			float eyescale = 1+snake/5.0;
			rgbColor tmp = color*0.5;
			tmp = Colors::white;
			// draw both eyes
			p.x+=GetRadius()*0.2;
			display.FillCircle(p, GetRadius()*0.2*eyescale, tmp);
			p.x-=2*GetRadius()*0.2;
			display.FillCircle(p, GetRadius()*0.2*eyescale, tmp);
			p.x+=2*GetRadius()*0.2;
			
			// draw iris
			display.FillCircle(p, GetRadius()*0.1*eyescale, Colors::black);
			p.x-=2*GetRadius()*0.2;
			display.FillCircle(p, GetRadius()*0.1*eyescale, Colors::black);
			
			p.x+=GetRadius()*0.2;
			p.y+=2*GetRadius()*0.25;
			display.FillNGon(p, GetRadius()*0.3, 3, 240, Colors::orange);
			break;
		}
		case kNothing:
		default: break;
	}
}

void SnakeBird::Draw(Graphics::Display &display, double time) const
{
	for (int x = -2; x <= width+1; x++)
	{
		auto p = GetCenter(x, height-1);
		p.y += GetRadius()+0.25*GetRadius()*sin(x*4.0/width+time*0.43);
		p.x += fmod(time*0.25, 2)*GetRadius();
		display.FillNGon(p, GetRadius(), 3, 180, Colors::darkblue+Colors::darkred);
	}

	if (portal1Loc != -1)
	{
		double radius = GetRadius()*0.95;
		Graphics::point p = GetCenter(GetX(portal1Loc), GetY(portal1Loc));
		double offset1 = (sin(time*1.0)); // -1...+1
		double offset2 = (sin(time*1.5)); // -1...+1
		double offset3 = (sin(time*2.0)); // -1...+1
		double offset4 = (sin(time*2.5)); // -1...+1
		display.FillCircle(p, radius+offset1*radius*0.25, Colors::red);
		display.FillCircle(p, radius*0.75+offset2*radius*0.2, Colors::blue);
		display.FillCircle(p, radius*0.5+offset3*radius*0.15, Colors::green);
		display.FillCircle(p, radius*0.25+offset4*radius*0.1, Colors::purple);
	}
	if (portal2Loc != -1)
	{
		double radius = GetRadius()*0.95;
		Graphics::point p = GetCenter(GetX(portal2Loc), GetY(portal2Loc));
		double offset1 = (sin(time*1.0+0.1)); // -1...+1
		double offset2 = (sin(time*1.5+0.1)); // -1...+1
		double offset3 = (sin(time*2.0+0.1)); // -1...+1
		double offset4 = (sin(time*2.5+0.1)); // -1...+1
		display.FillCircle(p, radius+offset1*radius*0.25, Colors::red);
		display.FillCircle(p, radius*0.75+offset2*radius*0.2, Colors::blue);
		display.FillCircle(p, radius*0.5+offset3*radius*0.15, Colors::green);
		display.FillCircle(p, radius*0.25+offset4*radius*0.1, Colors::purple);
	}

}

bool SnakeBird::GetPointFromCoordinate(Graphics::point p, int &x, int &y)
{
	x = (p.x+1.0-GetRadius())/(2.0*GetRadius())+0.5;
	y = (p.y+1.0-GetRadius())/(2.0*GetRadius())+0.5;
	if (x >= 0 && x < width && y >= 0 && y < height)
		return true;
	return false;
}

Graphics::point SnakeBird::GetCenter(int x, int y) const
{
	Graphics::point p;
	p.x = -1+2*x*GetRadius()+GetRadius();
	p.y = -1+2*y*GetRadius()+GetRadius();
	return p;
}

float SnakeBird::GetRadius() const
{
	return 2.0/(std::max(width, height)*2.0);
}

void SnakeBird::Draw(Graphics::Display &display, const SnakeBirdState&s) const
{
	Draw(display, s, -1);
}

void SnakeBird::Draw(Graphics::Display &display, const SnakeBirdState&s, int active) const
{
	Draw(display, s, active, 0);
}

void SnakeBird::DrawSnakeEnteringGoal(Graphics::Display &display,
									  const SnakeBirdState &s,
									  int snake, bool isActive, double percentComplete) const
{
	const rgbColor snakeColors[4] = {Colors::red, Colors::blue, Colors::green, Colors::yellow};
	int index = s.GetSnakeHeadLoc(snake);
	
	int x = GetX(index);
	int y = GetY(index);
	int len = s.GetSnakeLength(snake);
	if (len < 0)
		return;
	float timePerSegment = 1.0f/len;
	float drawnTime = 0;
	int nextIn = percentComplete*len;
	int cnt = 0;
	
	int prevx = x, prevy = y; // 2 steps back
	for (int t = nextIn; t < len; t++)
	{
		int lastx = x;
		int lasty = y;
		switch (s.GetSnakeDir(snake, cnt))
		{
			case kUp: y-=1; break;
			case kDown: y+=1; break;
			case kRight: x+=1; break;
			case kLeft: x-=1; break;
			default: break;
		}
		bool tail = (t+1==len);

		Graphics::point p = GetCenter(lastx, lasty);
		Graphics::point p2 = GetCenter(prevx, prevy);
//		rgbColor color = snakeColors[snake]*(((++cnt+len)&1)?0.8:1.0);
		rgbColor color = snakeColors[snake]*(((lastx+lasty)&1)?0.8:1.0);
		++cnt;
		prevx = lastx;
		prevy = lasty;
		if (t == len-1)
		{
			float perc = percentComplete*len-nextIn;
			p = p2*perc+p*(1-perc);
		}
		
		DrawSnakeSegment(display, p, color,
						 (t == nextIn), // head - (was false always?)
						 tail,  // tail
						 false, // awake
						 (t == nextIn)?kUp:s.GetSnakeDir(snake, cnt-2), // dirFrom
						 tail?kUp:s.GetSnakeDir(snake, cnt-1), snake, false); //dirTo - ignored if tail is true
	}
}


void SnakeBird::DrawTranslatingSnake(Graphics::Display &display, const SnakeBirdState &old, const SnakeBirdState &s,
									 int snake, bool isActive, double percentComplete) const
{
	const rgbColor snakeColors[4] = {Colors::red, Colors::blue, Colors::green, Colors::yellow};
	int index = s.GetSnakeHeadLoc(snake);
	
	int x = GetX(index);
	int y = GetY(index);
	int deltax = GetX(old.GetSnakeHeadLoc(snake))-x;
	int deltay = GetY(old.GetSnakeHeadLoc(snake))-y;
	//		display.FillSquare(p, GetRadius(), Colors::red);
	int len = s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1);
	int oldLen = old.GetSnakeBodyEnd(snake)-old.GetSnakeBodyEnd(snake-1);
	bool deadSegment = IsOnSpikes(s, snake);//GetGroundType(deltax+x, deltay+y)==kSpikes;

	int cnt = 0;
	for (int t = 0; t < len; t++)
	{
		switch (s.GetSnakeDir(snake, t))
		{
			case kUp: y-=1; break;
			case kDown: y+=1; break;
			case kRight: x+=1; break;
			case kLeft: x-=1; break;
			default: break;
		}
		bool tail = t+1==(s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1));
		Graphics::point p = GetCenter(x, y);
		Graphics::point p2 = GetCenter(deltax+x, deltay+y);
		rgbColor color = snakeColors[snake]*(((++cnt+len)&1)?0.8:1.0);
//		rgbColor color = snakeColors[snake]*(((x+y)&1)?0.8:1.0);
//		++cnt;
		
		if (oldLen == len)
		{
			p = p*percentComplete + p2*(1-percentComplete);
		}
		else if (tail)
		{
			p = p2;
		}
		DrawSnakeSegment(display, p, color,
						 false, // head
						 tail,  // tail
						 false, // awake
						 s.GetSnakeDir(snake, t), // dirFrom
						 tail?kUp:s.GetSnakeDir(snake, t+1), snake, deadSegment); //dirTo - ignored if tail is true
	}

	// Now draw head
	index = s.GetSnakeHeadLoc(snake);
	
	x = GetX(index);
	y = GetY(index);
	deltax = GetX(old.GetSnakeHeadLoc(snake))-x;
	deltay = GetY(old.GetSnakeHeadLoc(snake))-y;
	//		display.FillSquare(p, GetRadius(), Colors::red);
	
	Graphics::point p = GetCenter(x, y);
	Graphics::point p2 = GetCenter(deltax+x, deltay+y);
	p = p*percentComplete + p2*(1-percentComplete);
//	rgbColor color = snakeColors[snake]*(((deltax+deltay+x+y)&1)?0.8:1.0);
	rgbColor color = snakeColors[snake]*((len%2)?0.8:1.0);
	//bool deadSegment = GetGroundType(deltax+x, deltay+y)==kSpikes;
	DrawSnakeSegment(display, p, color, true, false, isActive, kUp, s.GetSnakeDir(snake, 0), snake, deadSegment);
}

void SnakeBird::DrawMovingSnake(Graphics::Display &display, const SnakeBirdState &old,
								const SnakeBirdState &s,
								int snake, bool isActive, double percentComplete) const
{
	const rgbColor snakeColors[4] = {Colors::red, Colors::blue, Colors::green, Colors::yellow};
	
	int index = old.GetSnakeHeadLoc(snake);
	int x = GetX(index);
	int y = GetY(index);
	int deltax = GetX(old.GetSnakeHeadLoc(snake))-GetX(s.GetSnakeHeadLoc(snake));
	int deltay = GetY(old.GetSnakeHeadLoc(snake))-GetY(s.GetSnakeHeadLoc(snake));
	int lastx, lasty;
	bool deadSegment = IsOnSpikes(s, snake);//GetGroundType(deltax+x, deltay+y)==kSpikes;

	int len = old.GetSnakeLength(snake);
	bool ateFruit = (old.GetSnakeLength(snake) != s.GetSnakeLength(snake));
	int cnt = ateFruit?0:1;
	// 1. draw the entire old snake statically except for the tail
	snakeDir fromDir, toDir;
	fromDir = s.GetSnakeDir(snake, 0);
	toDir = old.GetSnakeDir(snake, 0);
	for (int t = 0; t < len-1; t++)
	{
		bool head = false;
		// cause the second segment to be rounded until the new head moves 
		if (percentComplete < 0.1 && t == 0)
			head = true;
		rgbColor color = snakeColors[snake]*(((++cnt+len)&1)?0.8:1.0);
//		rgbColor color = snakeColors[snake]*(((x+y)&1)?0.8:1.0);
//		++cnt;
		Graphics::point p = GetCenter(x, y);
		//bool deadSegment = GetGroundType(x, y)==kSpikes;
		DrawSnakeSegment(display, p, color,
						 head, // head
						 false, // tail
						 false, // awake
						 fromDir, // dirFrom
						 toDir, snake, deadSegment); //dirTo - ignored if tail is true

		fromDir = old.GetSnakeDir(snake, t);
		toDir = old.GetSnakeDir(snake, t+1);

		lastx = x; lasty = y;
		switch (old.GetSnakeDir(snake, t))
		{
			case kUp: y-=1; break;
			case kDown: y+=1; break;
			case kRight: x+=1; break;
			case kLeft: x-=1; break;
			default: break;
		}
	}

	// Now draw moving tail - x/y are at the end of the old snake
	if (!ateFruit)
	{
		Graphics::point p = GetCenter(lastx, lasty);
		Graphics::point p2 = GetCenter(x, y);
		p = p*percentComplete + p2*(1-percentComplete);
		color = snakeColors[snake];//*(((len)%2)?0.8:1.0);
//		color = snakeColors[snake]*(((x+y)&1)?0.8:1.0);
		//bool deadSegment = GetGroundType(x, y)==kSpikes;
		DrawSnakeSegment(display, p, color, false, true, false, old.GetSnakeDir(snake, len-2), kUp/*next*/, snake, deadSegment);
	}
	else {
		Graphics::point p = GetCenter(x, y);
		rgbColor color = snakeColors[snake]*(((++cnt+len)&1)?0.8:1.0);
//		++cnt;
//		rgbColor color = snakeColors[snake]*(((x+y)&1)?0.8:1.0);
		bool deadSegment = GetGroundType(x, y)==kSpikes;
		DrawSnakeSegment(display, p, color, false, true, false, old.GetSnakeDir(snake, len-2), kUp/*next*/, snake, deadSegment);
	}
	
	// Now draw moving head
	index = s.GetSnakeHeadLoc(snake);
	
	x = GetX(index);
	y = GetY(index);
//	int deltax = GetX(old.GetSnakeHeadLoc(snake))-x;
//	int deltay = GetY(old.GetSnakeHeadLoc(snake))-y;
	Graphics::point p = GetCenter(x, y);
	Graphics::point p2 = GetCenter(deltax+x, deltay+y);
	p = p*percentComplete + p2*(1-percentComplete);
	rgbColor color = snakeColors[snake]*((len%2)?1.0:0.8);
//	rgbColor color = snakeColors[snake]*(((deltax+deltay+x+y)&1)?0.8:1.0);
	//bool deadSegment = GetGroundType(deltax+x, deltay+y)==kSpikes;
	DrawSnakeSegment(display, p, color, true, false, isActive, kUp, s.GetSnakeDir(snake, 0), snake, deadSegment);
}


void SnakeBird::Draw(Graphics::Display &display,
					 const SnakeBirdState &old,
					 const SnakeBirdState &s,
					 int active, double percentComplete, double globalTime) const
{
	rgbColor snakeColors[4] = {Colors::red, Colors::blue, Colors::green, Colors::yellow};
	rgbColor objColors[4] = {Colors::red*0.75, Colors::blue*0.75, Colors::green*0.75, Colors::yellow*0.75};
	
	// draw objects - connections wil be behind snakes
	static std::vector<Graphics::point> points;
	for (int x = 0; x < 4; x++)
	{
		double radius = GetRadius();//*0.95;
		if (editing == true)
		{ printf("WARNING: editing not turned off.\n"); }
		points.clear();
		if (s.GetObjectLocation(x) == kDead)
			continue;
		for (int i = 0; i < objects[x].size(); i++)
		{
			Graphics::point p = GetCenter(GetX(objects[x][i])+GetX(s.GetObjectLocation(x)),
										  GetY(objects[x][i])+GetY(s.GetObjectLocation(x)));
			Graphics::point p2 = GetCenter(GetX(objects[x][i])+GetX(old.GetObjectLocation(x)),
										   GetY(objects[x][i])+GetY(old.GetObjectLocation(x)));
			p = p*percentComplete+p2*(1-percentComplete);
			display.FrameSquare(p, radius-radius*0.3, objColors[x], radius*0.6);
			if (!objectFullyConnected[x])
				points.push_back(p);
		}
		if (points.size() > 0)
		{
			for (int z = 0; z < points.size(); z++)
			{
				for (int y = z+1; y < points.size(); y++)
				{
					if (fequal(points[z].x, points[y].x))
					{
						Graphics::rect r(points[z].x-GetRadius()*0.2f,
										 points[z].y+GetRadius()-radius*0.3*2,
										 points[y].x+GetRadius()*0.2f,
										 points[y].y-GetRadius()+radius*0.3*2);
						display.FillRect(r, objColors[x]*0.5);
					}
					else if (fequal(points[z].y, points[y].y))
					{
						Graphics::rect r(points[z].x+GetRadius()-radius*0.3*2,
										 points[z].y-GetRadius()*0.2f,
										 points[y].x-GetRadius()+radius*0.3*2,
										 points[y].y+GetRadius()*0.2f);
						display.FillRect(r, objColors[x]*0.5);
					}
				}
			}
//			display.DrawLineSegments(points, GetRadius()*0.5, objColors[x]*0.8);
		}
	}

	
	// draw bodies
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		// get head loc
		int index = s.GetSnakeHeadLoc(snake);
//		if (index == kInGoal)
//			continue;
		if (index == kInGoal && old.GetSnakeHeadLoc(snake) == kInGoal)
		{
			// do nothing
		}
		else if (index != kInGoal && old.GetSnakeHeadLoc(snake) == kInGoal) // undoing an action
		{
			DrawTranslatingSnake(display, s, s, snake, (active==-1)||(active==snake), percentComplete);
		}
		else if (index == kInGoal && old.GetSnakeHeadLoc(snake) != kInGoal) // moved into goal
		{
			DrawSnakeEnteringGoal(display, old, snake, (active==-1)||(active==snake), percentComplete);
		}
		else if (s.GetSnakeLength(snake) != old.GetSnakeLength(snake))
		{
			// body stays the same - just draw head moving
			DrawMovingSnake(display, old, s, snake, (active==-1)||(active==snake), percentComplete);
			continue;
		}
		else if (s.GetBodyBits(snake) == old.GetBodyBits(snake))
		{
			// tween entire snake
			DrawTranslatingSnake(display, old, s, snake, (active==-1)||(active==snake), percentComplete);
			continue;
		}
		else {
			// whole snake moves -
			DrawMovingSnake(display, old, s, snake, (active==-1)||(active==snake), percentComplete);
			continue;
		}
	}
	
	for (int x = 0; x < fruit.size(); x++)
	{
		if (s.GetFruitPresent(x))
		{
			const float rows = 5;
			const float counts[] = {3, 4, 3, 2, 1};
			Graphics::point p = GetCenter(GetX(fruit[x]), GetY(fruit[x]));
			float grapeRadius = GetRadius()*0.15f;
			for (int t = 0; t < rows; t++)
			{
				for (int v = 0; v < counts[t]; v++)
				{
					Graphics::point tmp = p;
					tmp.y -= rows*grapeRadius;
					tmp.y += 2*t*grapeRadius;
					tmp.x = tmp.x-grapeRadius*(counts[t]-1)+v*grapeRadius*2;
					tmp.x += sin(globalTime*1.1+(t+v)*0.1)*GetRadius()*0.1;
					tmp.y += cos(globalTime*0.95+(t+v)*0.1)*GetRadius()*0.1;
					display.FillCircle(tmp, grapeRadius*1.1, Colors::purple*sin(globalTime*1.95+v+t));
				}
			}
		}
	}
	
	// draw exit
	if (exitLoc != -1)
	{
		Graphics::point p = GetCenter(GetX(exitLoc), GetY(exitLoc));
		double radius = GetRadius()*0.95;
		double time = globalTime;
		if (s.KFruitEaten(fruit.size()))
		{
			radius = radius+0.25*radius*(1+sin(globalTime*2.5));
			time *= 2.0;
		}
		else {
			time /= 4.0;
		}
		double offset1 = (sin(time*1.0))*180; // -1...+1
		double offset2 = (sin(time*1.5))*180; // -1...+1
		double offset3 = (sin(time*2.0))*180; // -1...+1
		display.FillNGon(p, radius, 5, 0+offset1, Colors::yellow);
		display.FillNGon(p, radius*0.66, 5, 36+offset2, Colors::orange);
		display.FillNGon(p, radius*0.25, 5, 54+offset3, Colors::red);
	}

}

void SnakeBird::Draw(Graphics::Display &display, const SnakeBirdState&s, int active, double globalTime) const
{
	Draw(display, s, s, active, 0, globalTime);
}

void SnakeBird::DrawSnakeSegment(Graphics::Display &display, Graphics::point p, const rgbColor &c, bool head, bool tail, bool awake, snakeDir dirFrom, snakeDir dirTo, int snake, bool isDead) const
{
//	const float cornerWidth = 0.75;
//	float offset = cornerWidth*GetRadius();
	rgbColor color = c;
	if (isDead)
		color.mix(Colors::black, 0.75);
	float smallRadius = 0.75*GetRadius();
	Graphics::rect r(p, GetRadius());
	r.top+=smallRadius;
	r.bottom-=smallRadius;
	display.FillRect(r, color);
	r.top-=smallRadius;
	r.bottom+=smallRadius;
	r.left+=smallRadius;
	r.right-=smallRadius;
	display.FillRect(r, color);

	if ((!head && (dirFrom == kDown || dirFrom == kRight)) ||
		(!tail && (dirTo == kUp || dirTo == kLeft)))
	{
		display.FillSquare(p+Graphics::point(GetRadius()/2-GetRadius(), GetRadius()/2-GetRadius()), GetRadius()/2, color);
	}
	else {
		display.FillCircle(p+Graphics::point(smallRadius-GetRadius(), smallRadius-GetRadius()), smallRadius, color);
	}

	if ((!head && (dirFrom == kUp || dirFrom == kRight)) ||
		(!tail && (dirTo == kDown || dirTo == kLeft)))
	{
		display.FillSquare(p+Graphics::point(GetRadius()/2-GetRadius(), -GetRadius()/2+GetRadius()), GetRadius()/2, color);
	}
	else {
		display.FillCircle(p+Graphics::point(smallRadius-GetRadius(), -smallRadius+GetRadius()), smallRadius, color);
	}
	
	if ((!head && (dirFrom == kDown || dirFrom == kLeft)) ||
		(!tail && (dirTo == kUp || dirTo == kRight)))
	{
		display.FillSquare(p+Graphics::point(-GetRadius()/2+GetRadius(), GetRadius()/2-GetRadius()), GetRadius()/2, color);
	}
	else {
		display.FillCircle(p+Graphics::point(-smallRadius+GetRadius(), smallRadius-GetRadius()), smallRadius, color);
	}

	if ((!head && (dirFrom == kUp || dirFrom == kLeft)) ||
		(!tail && (dirTo == kDown || dirTo == kRight)))
	{
		display.FillSquare(p+Graphics::point(-GetRadius()/2+GetRadius(), -GetRadius()/2+GetRadius()), GetRadius()/2, color);
	}
	else {
		display.FillCircle(p+Graphics::point(-smallRadius+GetRadius(), -smallRadius+GetRadius()), smallRadius, color);
	}

	if (head)
	{
		float eyescale = 1+snake/5.0;
		rgbColor tmp = color*0.5;
		if (isDead)
			tmp = Colors::black;
		else if (awake)
			tmp = Colors::white;
		// draw both eyes
		p.x+=GetRadius()*0.2;
		display.FillCircle(p, GetRadius()*0.2*eyescale, tmp);
		p.x-=2*GetRadius()*0.2;
		display.FillCircle(p, GetRadius()*0.2*eyescale, tmp);
		p.x+=2*GetRadius()*0.2;

		// draw iris
		if (awake)
			display.FillCircle(p, GetRadius()*0.1*eyescale, Colors::black);
		p.x-=2*GetRadius()*0.2;
		//display.FillCircle(p, GetRadius()*0.2*eyescale, awake?Colors::white:(color*0.5));
		if (awake)
			display.FillCircle(p, GetRadius()*0.1*eyescale, Colors::black);

		p.x+=GetRadius()*0.2;
		p.y+=2*GetRadius()*0.25;
		display.FillNGon(p, GetRadius()*0.3, 3, 240, Colors::orange);
	}
	
	
	// in spikes!
//	if (world[GetIndex(x, y)] == kSpikes)
//	{
//		display.FillNGon(p, GetRadius()*0.95, 3, 0, Colors::darkgray);
//		display.FillNGon(p, GetRadius()*0.95, 3, 60, Colors::gray*0.75f);
//		return;
//	}
}

void SnakeBird::DrawLabel(Graphics::Display &display, int x, int y, const char *str)
{
	display.DrawText(str, GetCenter(x, y), GetColor(), GetRadius()*1.5, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
}

void SnakeBird::DrawSmallLabel(Graphics::Display &display, int x, int y, const char *str)
{
	display.DrawText(str, GetCenter(x, y)+Graphics::point(GetRadius()/2, GetRadius()/2), GetColor(), 0.75*GetRadius(), Graphics::textAlignCenter, Graphics::textBaselineMiddle);
}


void SnakeBird::DrawLine(Graphics::Display &display, const SnakeBirdState &x, const SnakeBirdState &y, float width) const
{
	
}


}
