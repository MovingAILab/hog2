//
//  RC.cpp (Alternate Rubik's Cube implementation)
//
//  Created by Nathan Sturtevant on 7/9/21.
//

#include "RC.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <string>
#include <iostream>

// Helpers (for calculation)
void EditPointMult(float (&vec)[3][3], Graphics::point & p);
void MakeMatrixRotX(float (&a)[3][3], float angle);
void MakeMatrixRotY(float (&a)[3][3], float angle);
void MakeMatrixRotZ(float (&a)[3][3], float angle);
int findInArray(int arr[], int elem, int lower, int upper);
float array3 [3] = {0,0,0};
float rotation [3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };
std::vector<int> furthestZ(0);

const int edgesOnFace[6][4] =
{
	{0, 6, 4, 2},  //0
	{2, 3, 9, 1},  //1
	{0, 1, 8, 7},  //2
	{6, 7, 11, 5}, //3
	{4, 5, 10, 3}, //4
	{9, 10, 11, 8} //5
};
//	int cornersOnFace[6][4] =
//	{
//		{4 +11, 3 +11, 2 +11, 1 +11}, //0
//		{1 +11, 2 +11, 6 +11, 5 +11}, //1
//		{4 +11, 1 +11, 5 +11, 8 +11}, //2
//		{3 +11, 4 +11, 8 +11, 7 +11}, //3
//		{2 +11, 3 +11, 7 +11, 6 +11}, //4
//		{5 +11, 6 +11, 7 +11, 8 +11}  //5
//	};
const int cornersOnFace[6][4] =
{
	{15, 14, 13, 12}, //0
	{16, 12, 13, 17}, //1
	{15, 12, 16, 19}, //2
	{15, 19, 18, 14}, //3
	{13, 14, 18, 17}, //4
	{16, 17, 18, 19}  //5
};
//std::vector<std::vector<float>> rotation(3,std::vector<float>(3));

/*===============================================================================================================================================================================
 * 			CUBIE CLASS
 ===============================================================================================================================================================================*/

/* Called when cubie is initialized via the RCState
 * 
 */
void Cubie::Initialize(int RCpos, int RCind, int RCrot)
{
	RCindex = RCind; // TEMP
	// Reset previous data
	basePoints.clear();
	baseFacePoints.clear();
	points.clear();
	facePoints.clear();
	
	// Edge or corner
	bool isCorner = false;
	if (RCind >= 12) isCorner = true;
	
	// Position of the cubie on the cube (x, y, z) -------------------------------------------
	int xPos = 0, yPos = 0, zPos = 0;
	int xAdd[8] =  {3, 4, 5, 10, 13, 14, 17, 18};  //Orange
	int xSubt[8] = {0, 1, 7, 8, 12, 15, 16, 19};   //Red
	int yAdd[8] =  {8, 9, 10, 11, 16, 17, 18, 19}; //Yellow
	int ySubt[8] = {0, 2, 4, 6, 12, 13, 14, 15};   //White
	int zAdd[8] =  {5, 6, 7, 11, 14, 15, 18, 19};  //Green
	int zSubt[8] = {1, 2, 3, 9, 12, 13, 16, 17};   //Blue
	
	// Modify array search range
	int starti = 0, endi = 4;
	if (isCorner) 
	{
		starti = 4;
		endi = 8;
	}
	
	// Search within arrays and modify
	if (findInArray(xAdd, RCpos, starti, endi) != -1)
	{ 
		xPos = 1;
	} else if (findInArray(xSubt, RCpos, starti, endi) != -1)
	{ 
		xPos = -1;
	}
	if (findInArray(yAdd, RCpos, starti, endi) != -1)
	{ 
		yPos = 1;
	} else if (findInArray(ySubt, RCpos, starti, endi) != -1)
	{ 
		yPos = -1;
	}
	if (findInArray(zAdd, RCpos, starti, endi) != -1)
	{ 
		zPos = 1;
	} else if (findInArray(zSubt, RCpos, starti, endi) != -1)
	{ 
		zPos = -1;
	}
	
	// Visibible Faces -------------------------------------------------------------------------
	// Init faces
	for (int i = 0; i < 6; i++)
	{
		faceCols[i] = inCol;
		facesShown[i] = false;
	}
	
	// Do face visibility and colors depending on corner/edge
	if (isCorner)
	{
		for (int i = 0; i < 3; i++)
		{
			facesShown[facesShowing[RCpos][i]] = true; 
			// This is a colored face, update the color
			int indToEdit;
			if (RCrot == 0)
			{
				// "Correct" rotation: White/Yellow faces White/Yellow 
				indToEdit = i;
			} else if (RCrot == 1)
			{
				// Clockwise rotation from "Correct" rotation, add 1 to i
				indToEdit = (i + 1) % 3;
			} else
			{
				// Counterclockwise rotation from "Correct" rotation, subtract 1 from i
				indToEdit = (i + 2) % 3;
			}
			// Change the color
			faceCols[facesShowing[RCpos][indToEdit]] = faceColors[facesShowing[RCind][i]];		
		}	
	} else
	{ // Edge
		for (int i = 0; i < 2; i++)
		{
			facesShown[facesShowing[RCpos][i]] = true; 
			// This is a colored face, update the color
			if (RCrot != 2)
			{
				// (i + RCrot)%2 : Swap index 0/1 if the incorrect RCrot (RCrot == 1)
				faceCols[facesShowing[RCpos][i]] = faceColors[facesShowing[RCind][(i + RCrot)%2]];
				
			} else
			{
				std::cout << "ERROR: 2 OUT OF BOUNDS [Cubie::Initialize(int RCpos, int RCind, int RCrot)]" << '\n';
			}
		}
	}
	
	// The following is the same code as with Cubie::Initialize(int ind)
	//Initialize the 8 points
	for (int i = 0; i < 8; i++)
	{
		points.push_back(Graphics::point(0,0,0));
		points[i].x = -0.5f * sideLen;// + xPos * sideLen;
		if ((i%4) == 1 || (i%4) == 2)
			points[i].x += 1.f * sideLen;
		points[i].y = (-0.5f + (1.f * ((i/2)%2))) * sideLen;// + yPos * sideLen; //y
		points[i].z = (-0.5f + (1.f * (i/4))) * sideLen;// + zPos * sideLen; //z
		
		basePoints.push_back(Graphics::point(points[i].x,points[i].y,points[i].z));
	}
	//Initialize the 4 face points for all 6 faces (24)
	for (int i = 0; i < 6; i++)
	{
		facePoints.push_back(std::vector<Graphics::point>());
		baseFacePoints.push_back(std::vector<Graphics::point>());
		for (int j = 0; j < 4; j++)
		{
			float x, y, z;
			if (i == 2 || i == 4) x = 1.0f;
				else x = faceSize;
			if (i == 0 || i == 5) y = 1.0f;
				else y = faceSize;
			if (i == 1 || i == 3) z = 1.0f;
				else z = faceSize;

			Graphics::point p = points[pointsOnFace[i][j]];
			x *= p.x;
			y *= p.y;
			z *= p.z;
			facePoints[i].push_back(Graphics::point(x,y,z));
			baseFacePoints[i].push_back(Graphics::point(x,y,z));
		}
	}
	//Move all points (faces and cube) by coord
	for (int i = 0; i < 8; i++)
	{
		points[i].x += xPos * sideLen;
		points[i].y += yPos * sideLen;
		points[i].z += zPos * sideLen;
		basePoints[i].x += xPos * sideLen;
		basePoints[i].y += yPos * sideLen;
		basePoints[i].z += zPos * sideLen;
	}
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			facePoints[i][j].x += xPos * sideLen;
			facePoints[i][j].y += yPos * sideLen;
			facePoints[i][j].z += zPos * sideLen;
			baseFacePoints[i][j].x += xPos * sideLen;
			baseFacePoints[i][j].y += yPos * sideLen;
			baseFacePoints[i][j].z += zPos * sideLen;
		}
	}
}


void Cubie::Initialize(int ind)
{
	index = ind;
	int xPos = 0, yPos = 0, zPos = 0;
	//Cube index (out of 26)

	int in = index;
	if (in >= 13) in++; //Readd middle cubie for init ease
		//Initialize face colors, determine position based on index
		for (int i = 0; i < 6; i++)
		{
			faceCols[i] = inCol;
			facesShown[i] = false;
		}
	//If its cube #0,1,2,9,10,11,18,19,20, make face 0 white
	for (int i = 0; i < 3; i++)
	{
		if (in >= i*9 && in <= i*9 + 2)
		{
			faceCols[0] = faceColors[0];
			facesShown[0] = true;
			yPos--;
		}
	}
	//If its cube #0-8, make face 1 blue
	if (in >= 0 && in <= 8)
	{
		faceCols[1] = faceColors[1];
		facesShown[1] = true;
		zPos--;
	}
	//If its cube # is a multiple of 3 (including 0), make face 2 red
	if (in % 3 == 0)
	{
		faceCols[2] = faceColors[2];
		facesShown[2] = true;
		xPos--;
	}
	//If its cube #18-26, make face 3 green
	if (in >= 18 && in <= 26)
	{
		faceCols[3] = faceColors[3];
		facesShown[3] = true;
		zPos++;
		//std::cout << in << '\n';
	}
	//If its cube # - 2 is multiple of 3, make face 4 orange
	if ((in-2) % 3 == 0)
	{
		faceCols[4] = faceColors[4];
		facesShown[4] = true;
		xPos++;
	}
	//If its cube #6,7,8,15,16,17,24,25,26, make face 5 yellow
	for (int i = 0; i < 3; i++)
	{
		if (in >= i*9 + 6 && in <= i*9 + 8)
		{
			faceCols[5] = faceColors[5];
			facesShown[5] = true;
			yPos++;
		}
	}

	//Initialize the 8 points
	for (int i = 0; i < 8; i++)
	{
		points.push_back(Graphics::point(0,0,0));
		points[i].x = -0.5f * sideLen;// + xPos * sideLen;
		if ((i%4) == 1 || (i%4) == 2)
			points[i].x += 1.f * sideLen;
		points[i].y = (-0.5f + (1.f * ((i/2)%2))) * sideLen;// + yPos * sideLen; //y
		points[i].z = (-0.5f + (1.f * (i/4))) * sideLen;// + zPos * sideLen; //z
		
		basePoints.push_back(Graphics::point(points[i].x,points[i].y,points[i].z));
	}
	//Initialize the 4 face points for all 6 faces (24)
	for (int i = 0; i < 6; i++)
	{
		facePoints.push_back(std::vector<Graphics::point>());
		baseFacePoints.push_back(std::vector<Graphics::point>());
		for (int j = 0; j < 4; j++)
		{
			float x, y, z;
			if (i == 2 || i == 4) x = 1.0f;
				else x = faceSize;
			if (i == 0 || i == 5) y = 1.0f;
				else y = faceSize;
			if (i == 1 || i == 3) z = 1.0f;
				else z = faceSize;

			Graphics::point p = points[pointsOnFace[i][j]];
			x *= p.x;
			y *= p.y;
			z *= p.z;
			facePoints[i].push_back(Graphics::point(x,y,z));
			baseFacePoints[i].push_back(Graphics::point(x,y,z));
		}
	}
	//Move all points (faces and cube) by coord
	for (int i = 0; i < 8; i++)
	{
		points[i].x += xPos * sideLen;
		points[i].y += yPos * sideLen;
		points[i].z += zPos * sideLen;
		basePoints[i].x += xPos * sideLen;
		basePoints[i].y += yPos * sideLen;
		basePoints[i].z += zPos * sideLen;
	}
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			facePoints[i][j].x += xPos * sideLen;
			facePoints[i][j].y += yPos * sideLen;
			facePoints[i][j].z += zPos * sideLen;
			baseFacePoints[i][j].x += xPos * sideLen;
			baseFacePoints[i][j].y += yPos * sideLen;
			baseFacePoints[i][j].z += zPos * sideLen;
		}
	}
}

/*
 * 
 */
void Cubie::Draw(Graphics::Display &display) const
{
	//Test
//	if (RCindex == 19 || RCindex == 0)
//	{
//		//Test
//		std::cout << "start " << RCindex << "\n";
//		for (int i = 0; i < 8; i++)
//		{
//			std::cout << "p" << i << RCindex << "\n";
//			std::cout << "x " << points[i].x << "\n";
//			std::cout << "y " << points[i].y << "\n";
//			std::cout << "z " << points[i].z << "\n";
//		}
//		std::cout << "\n\n";
//	}

	//	Identify which faces will be hidden from sight (Find which points are the furthest back)
	float highest;
	furthestZ.clear();
	furthestZ.push_back(0);
	highest = points[0].z;
	for (int i = 1; i < 8; i++)
	{
		if (points[i].z > highest) {
			furthestZ.clear();
			furthestZ.push_back(i);
			highest = points[i].z;
			continue;
		}
		if (points[i].z == highest) {
			furthestZ.push_back(i);
		}
	}
	int facesDrawn = 0;
	for (int i = 0; i < 6; i++)
	{
		if (facesShown[i])
		{
			bool drawThis = true;

			for (int pInd : pointsOnFace[i])
			{
				for (int check : furthestZ)
				{
					if (pInd == check)
					{
						//This face shouldn't be shown
						drawThis = false;
						break;
					}
				}
			}

			//Draw face if not on the back
			if (drawThis)
			{
				facesDrawn++;
				DrawFace(display, i);
			}
		}
	}
}

/* Given points (in a correct order)
 * Draw a polygon (face of a cubie)
 */
void Cubie::DrawFace(Graphics::Display &display, int index) const //face Index (0-5)
{
	display.FillTriangle( points[pointsOnFace[index][0]], points[pointsOnFace[index][1]],
			points[pointsOnFace[index][2]], outCol);
	display.FillTriangle( points[pointsOnFace[index][0]], points[pointsOnFace[index][2]],
			points[pointsOnFace[index][3]], outCol);

	display.FillTriangle( facePoints[index][0], facePoints[index][1],
			facePoints[index][2], faceCols[index]);
	display.FillTriangle( facePoints[index][0], facePoints[index][2],
			facePoints[index][3], faceCols[index]);
}

/*
 * 
 */
void Cubie::RotateRelative(const float angle[3])
{
	for (Graphics::point & point : points) //!!!!
	{
		MakeMatrixRotX(rotation, angle[0]);
		EditPointMult(rotation, point);
		MakeMatrixRotY(rotation, angle[1]);
		EditPointMult(rotation, point);
		MakeMatrixRotZ(rotation, angle[2]);
		EditPointMult(rotation, point);
	}
	for (int i = 0; i < 6; i++) //!!!!
	{
		MakeMatrixRotX(rotation, angle[0]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, facePoints[i][j]);
		MakeMatrixRotY(rotation, angle[1]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, facePoints[i][j]);
		MakeMatrixRotZ(rotation, angle[2]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, facePoints[i][j]);
	}
}

/*
 * 
 */
void Cubie::RotateBase(float angle [3])
{
	for (Graphics::point & point : basePoints)
	{
		MakeMatrixRotX(rotation, angle[0]);
		EditPointMult(rotation, point);
		MakeMatrixRotY(rotation, angle[1]);
		EditPointMult(rotation, point);
		MakeMatrixRotZ(rotation, angle[2]);
		EditPointMult(rotation, point);
	}
	for (int i = 0; i < 6; i++)
	{
		MakeMatrixRotX(rotation, angle[0]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, baseFacePoints[i][j]);
		MakeMatrixRotY(rotation, angle[1]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, baseFacePoints[i][j]);
		MakeMatrixRotZ(rotation, angle[2]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, baseFacePoints[i][j]);
	}
}

/*
 * 
 */
void Cubie::RotateFacePos(bool clockwise, int axis)
{
	int temp;
	if (clockwise)
	{
		temp = faceInPos[faceOrderByAxis[axis][3]];
		for (int i = 3; i > 0; i--)
		{
			faceInPos[faceOrderByAxis[axis][i]] = faceInPos[faceOrderByAxis[axis][i-1]];
		}
		faceInPos[faceOrderByAxis[axis][0]] = temp;
	} else
	{
		temp = faceInPos[faceOrderByAxis[axis][0]];
		for (int i = 0; i < 3; i++)
		{
			faceInPos[faceOrderByAxis[axis][i]] = faceInPos[faceOrderByAxis[axis][i+1]];
		}
		faceInPos[faceOrderByAxis[axis][3]] = temp;
	}
}

/*
 * 
 */
void Cubie::ResetToBase()
{
	for (int i = 0; i < 8; i++)
	{
		points[i].x = basePoints[i].x;
		points[i].y = basePoints[i].y;
		points[i].z = basePoints[i].z;	
	}
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			facePoints[i][j].x = baseFacePoints[i][j].x;
			facePoints[i][j].y = baseFacePoints[i][j].y;
			facePoints[i][j].z = baseFacePoints[i][j].z;
		}
	}
}

/*
 * 
 */
void Cubie::ResetVisibleFace()
{
	if (blackFaceToReset != -1)
	{
		facesShown[faceInPos[blackFaceToReset]] = false;
		blackFaceToReset = -1;
	}
}

/* Called when a rotation is made. cubies must make an inner black face
 * visible while rotation is in motion (which must be invisible once it's complete)
 */
void Cubie::SetFacePositionVisible(bool toggle, int position)
{
	facesShown[faceInPos[position]] = toggle;
	blackFaceToReset = position;
}

/* For testing purposes, prints all of the Cubie's non-constant data
 * 
 */
void Cubie::PrintData()
{
	std::cout << "Index: " << index << std::endl;
	
	std::cout << "BlackFaceToReset: " << blackFaceToReset << std::endl;
	
	std::cout << "faceCols: ";
	for (int i = 0; i < 6; i ++)
	{
		//std::cout << faceCols[i] << ", ";
	} std::cout << std::endl;
	
	std::cout << "facesShown: ";
	for (int i = 0; i < 6; i ++)
	{
		std::cout << facesShown[i] << ", ";
	} std::cout << std::endl;
	
	std::cout << "faceInPos: ";
	for (int i = 0; i < 6; i ++)
	{
		std::cout << faceInPos[i] << ", ";
	} std::cout << std::endl;
	
	std::cout << "points: ";
	for (int i = 0; i < 8; i ++)
	{
		std::cout << points[i] << ", ";
	} std::cout << std::endl;
	
	std::cout << "basePoints: ";
	for (int i = 0; i < 8; i ++)
	{
		std::cout << points[i] << ", ";
	} std::cout << std::endl;	
	
	std::cout << "facePoints: " << std::endl;
	for (int i = 0; i < 6; i ++)
	{
		std::cout << "	Face " << i << ": ";
		for (int j = 0; j < 8; j++)
		{
			std::cout << facePoints[i][j] << ", ";
		} std::cout << std::endl;	
	} std::cout << std::endl;
	
	std::cout << "baseFacePoints: " << std::endl;
	for (int i = 0; i < 6; i ++)
	{
		std::cout << "	Face " << i << ": ";
		for (int j = 0; j < 8; j++)
		{
			std::cout << baseFacePoints[i][j] << ", ";
		} std::cout << std::endl;	
	} std::cout << std::endl;
}

/*===============================================================================================================================================================================
 * 			RCSTATE CLASS
 ===============================================================================================================================================================================*/

/*
 * 
 */
void RCState::RotateFace(int face, int move)
{
	// TODO: DELETE
	
	// Move the cubes
	if (move != 2)
	{
		//Rotate the Cubes
		RotateEdges(face, move);
		RotateCorners(face, move);

		bool forward;
		if (move == 0) forward = true;
		else forward = false;
		
		ShiftPositions(edgesOnFace[face], forward);
		ShiftPositions(cornersOnFace[face], forward);
	} else //180* turn
	{
		SwapPositions(edgesOnFace[face][0], edgesOnFace[face][2]);
		SwapPositions(edgesOnFace[face][1], edgesOnFace[face][3]);
		SwapPositions(cornersOnFace[face][0], cornersOnFace[face][2]);
		SwapPositions(cornersOnFace[face][1], cornersOnFace[face][3]);
	}
}

/*
 * 
 */
void RCState::RotateFace(int move)
{
	switch(move)
	{
		case 0: // Face 0, CW
			// No change in rotation
			ShiftPositionsCW(edgesOnFace[0]);
			ShiftPositionsCW(cornersOnFace[0]);
			break;
		case 1: // CCW
			// No change in rotation
			ShiftPositionsCCW(edgesOnFace[0]);
			ShiftPositionsCCW(cornersOnFace[0]);
			break;
		case 3: // Face 1, CW
			RotateEdges(move);
			RotateCorners(move);
			
			ShiftPositionsCW(edgesOnFace[1]);
			ShiftPositionsCW(cornersOnFace[1]);
			break;
		case 4: // CCW
			RotateEdges(move);
			RotateCorners(move);
			
			ShiftPositionsCCW(edgesOnFace[1]);
			ShiftPositionsCCW(cornersOnFace[1]);
			break;
		case 6: // Face 2, CW
			RotateCorners(move);
			
			ShiftPositionsCW(edgesOnFace[2]);
			ShiftPositionsCW(cornersOnFace[2]);
			break;
		case 7: // CCW
			RotateCorners(move);
			
			ShiftPositionsCCW(edgesOnFace[2]);
			ShiftPositionsCCW(cornersOnFace[2]);
			break;
		case 9: // Face 3, CW
			RotateEdges(move);
			RotateCorners(move);
			
			ShiftPositionsCW(edgesOnFace[3]);
			ShiftPositionsCW(cornersOnFace[3]);
			break;
		case 10: // CCW
			RotateEdges(move);
			RotateCorners(move);
			
			ShiftPositionsCCW(edgesOnFace[3]);
			ShiftPositionsCCW(cornersOnFace[3]);
			break;
		case 12: // Face 4, CW
			RotateCorners(move);
			
			ShiftPositionsCW(edgesOnFace[4]);
			ShiftPositionsCW(cornersOnFace[4]);
			break;
		case 13: // CCW
			RotateCorners(move);
			
			ShiftPositionsCCW(edgesOnFace[4]);
			ShiftPositionsCCW(cornersOnFace[4]);
			break;
		case 15: // Face 5, CW
			// No change in rotation
			
			ShiftPositionsCW(edgesOnFace[5]);
			ShiftPositionsCW(cornersOnFace[5]);
			break;
		case 16: // CCW
			// No change in rotation
			
			ShiftPositionsCCW(edgesOnFace[5]);
			ShiftPositionsCCW(cornersOnFace[5]);
			break;
		case 2: // All faces 180
		case 5:
		case 8:
		case 11:
		case 14:
		case 17:
			// No change in rotation
			
			int face = move/3;
			SwapPositions(edgesOnFace[face][0], edgesOnFace[face][2]);
			SwapPositions(edgesOnFace[face][1], edgesOnFace[face][3]);
			SwapPositions(cornersOnFace[face][0], cornersOnFace[face][2]);
			SwapPositions(cornersOnFace[face][1], cornersOnFace[face][3]);
			break;
	}
}

/*
 * 
 */
void RCState::RotateEdges(int move)
{
	int face = move/3;
	rotation[edgesOnFace[face][0]] = 1 - rotation[edgesOnFace[face][0]];
	rotation[edgesOnFace[face][1]] = 1 - rotation[edgesOnFace[face][1]];
	rotation[edgesOnFace[face][2]] = 1 - rotation[edgesOnFace[face][2]];
	rotation[edgesOnFace[face][3]] = 1 - rotation[edgesOnFace[face][3]];
}

/*
 * 
 */
void RCState::RotateEdges(int face, int move)
{
	// TODO: move is already checked to not be 2
	if (move != 2 && (face == 1 || face == 3))
	{
		for (int i = 0; i < 4; i++)
		{
			rotation[edgesOnFace[face][i]] = 1 - rotation[edgesOnFace[face][i]];
		}
	}
}

/*
 * 
 */
void RCState::RotateCorners(int move)
{
	// cornersOnFace[face][0] will ALWAYS be a special corner, 
	// cannot be an index A face (face =/ 0, 5)
	int face = move/3;
	// For standard corners, the amount of rotation is equal to the faceIndex (0-2)
	int add = 2 - (face % 2); // Standard corners
	rotation[cornersOnFace[face][1]] += add;
	rotation[cornersOnFace[face][1]] %= 3;
	rotation[cornersOnFace[face][3]] += add;
	rotation[cornersOnFace[face][3]] %= 3;
	
	add = 3 - add; // Special corners
	rotation[cornersOnFace[face][0]] += add;
	rotation[cornersOnFace[face][0]] %= 3;
	rotation[cornersOnFace[face][2]] += add;
	rotation[cornersOnFace[face][2]] %= 3;
}

/*
 * 
 */
void RCState::RotateCorners(int face, int move)
{
	// TODO: move is already checked to not be 2
	// If 180 turn or index A face, do not change rotation
	if (move != 2)
	{
		int index;
		if (face % 5 == 0) index = 0;
		else if (face % 2 == 1) index = 1;
		else index = 2;
		
		// If index A face, do not change rotation
		if (index != 0)
		{
			int specialCorners[4] = {13, 15, 16, 18};

			// Rotate each corner in the face
			for (int i = 0; i < 4; i++)
			{
				// Determine whether this cubie is a special corner
				bool special = false;
				for(int j = 0; j < 4; j++){
				     if(cornersOnFace[face][i] == specialCorners[j]){
				    	 special = true;
				         break;
				     }
				}
				// if Shaded corner cubie:
					// B Face: rot += 2
					// C Face: rot += 1
				// if Unshaded corner cubie:
					// B Face: rot += 1
					// C Face: rot += 2
				int add = index;
				if (special) add = 3-add;

				rotation[cornersOnFace[face][i]] += add;
				// Loop to start of rotation if rotation goes over 2
				rotation[cornersOnFace[face][i]] = rotation[cornersOnFace[face][i]] % 3;
			}
		}
	}
}

/*
 * 
 */
void RCState::SwapPositions(int p1, int p2)
{
	int temp = indices[p1];
	indices[p1] = indices[p2];
	indices[p2] = temp;
	// Also swap rotations
	temp = rotation[p1];
	rotation[p1] = rotation[p2];
	rotation[p2] = temp;
}

/*
 * 
 */
void RCState::ShiftPositionsCW(const int (&arr)[4])
{
	// Shift rotations as well
	int temp;
	int tempR;
	temp = indices[arr[3]];
	tempR = rotation[arr[3]];
	
	int i = 3;
	indices[arr[i]] = indices[arr[i-1]];
	rotation[arr[i]] = rotation[arr[i-1]];
	i--;
	indices[arr[i]] = indices[arr[i-1]];
	rotation[arr[i]] = rotation[arr[i-1]];
	i--;
	indices[arr[i]] = indices[arr[i-1]];
	rotation[arr[i]] = rotation[arr[i-1]];
	
	indices[arr[0]] = temp;
	rotation[arr[0]] = tempR;
}

/*
 * 
 */
void RCState::ShiftPositionsCCW(const int (&arr)[4])
{
	// Shift rotations as well
	int temp;
	int tempR;
	temp = indices[arr[0]];
	tempR = rotation[arr[0]];
	
	int i = 0;
	indices[arr[i]] = indices[arr[i+1]];
	rotation[arr[i]] = rotation[arr[i+1]];
	i++;
	indices[arr[i]] = indices[arr[i+1]];
	rotation[arr[i]] = rotation[arr[i+1]];
	i++;
	indices[arr[i]] = indices[arr[i+1]];
	rotation[arr[i]] = rotation[arr[i+1]];

	indices[arr[3]] = temp;
	rotation[arr[3]] = tempR;
}

/*
 * 
 */
void RCState::ShiftPositions(const int (&arr)[4], bool forward)
{
	//std::cout << "Shifting positions " << arr[0] << ", " << arr[1] << ", " << arr[2] << ", " << arr[3] << std::endl;

	// Shift rotations as well
	int temp;
	int tempR;
	if (forward) // clockwise
	{
		temp = indices[arr[3]];
		tempR = rotation[arr[3]];
		for (int i = 3; i > 0; i--)
		{
			indices[arr[i]] = indices[arr[i-1]];
			rotation[arr[i]] = rotation[arr[i-1]];
		}
		indices[arr[0]] = temp;
		rotation[arr[0]] = tempR;
	} else //CCW
	{
		temp = indices[arr[0]];
		tempR = rotation[arr[0]];
		for (int i = 0; i < 3; i++)
		{
			indices[arr[i]] = indices[arr[i+1]];
			rotation[arr[i]] = rotation[arr[i+1]];
		}
		indices[arr[3]] = temp;
		rotation[arr[3]] = tempR;
	}
}

void RCState::PrintState()
{
	// Print the two arrays: indices[] and rotation[]
	std::cout << "Indices: [";
	for (int i = 0; i < 20; i++)
	{
		std::cout << indices[i] << ", ";
	}
	std::cout << "]" << std::endl;
	std::cout << "Rotation: [";
	for (int i = 0; i < 20; i++)
	{
		std::cout << rotation[i] << ", ";
	}
	std::cout << "]" << std::endl;
}

/*===============================================================================================================================================================================
 * 			RC CLASS
 ===============================================================================================================================================================================*/

/*
 * 
 */
void RC::DrawCubies(Graphics::Display &display) const
{
	if (rotating)
	{
		DrawCubiesRotating(display);
	} else
	{
		for (int i = 0; i < 26; i++)
		{
			cubies[i].Draw(display);
		}
	}
}

/*
 * 
 */
void RC::DrawCubiesRotating(Graphics::Display &display) const
{
	bool jumped = false;
	if (rotatingFaceBehind)
	{
		 goto drawFace;
	} else goto drawCubie;
	
	drawFace:
		for (int i = 0; i < 9; i++)
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].Draw(display);	
		if (!jumped)
		{
			jumped = true;
			goto drawCubie;
		}
		return;
	drawCubie:
		for (int i = 0; i < 17; i++)
			cubies[cubieInPos[notInFaceTurning[faceTurning][i]]].Draw(display);
		if (!jumped)
		{
			jumped = true;
			goto drawFace;
		}
		return;
}

/*
 * 
 */
void RC::RotateCubies(float add[3])
{
	for (int i = 0; i < 3; i++)
	{
		rotationTotal[i] += add[i];
	}
	for (int i = 0; i < 26; i++)
	{
		cubies[i].ResetToBase();
		cubies[i].RotateRelative(rotationTotal);
	}
}

/*
 * 
 */
void RC::RotateFace(int face, int move)
{
	if (!rotating)
	{
		bool clockwise = true;
		int axis;
		turnArr[0] = 0;
		turnArr[1] = 0;
		turnArr[2] = 0;
		rotating = true;
		faceTurning = face;
		rotProgress = 0;
		
		float add = piOver2;
		if (face == 0 || face == 3 || face == 4)
		{
			add *= -1;
		}
		if (face == 5 || face == 3 || face == 4 )
		{
			clockwise = !clockwise;
		}
		
		// Move cubies on face
		int temp, temp2;
		
		if (move == 0) //CW
		{
			temp = cubieInPos[cubiesOnFace[face][edgeOrder[3]]];
			temp2 = cubieInPos[cubiesOnFace[face][cornerOrder[3]]];
			for (int i = 3; i > 0; i--)
			{
				cubieInPos[cubiesOnFace[face][edgeOrder[i]]] = cubieInPos[cubiesOnFace[face][edgeOrder[i-1]]];
				cubieInPos[cubiesOnFace[face][cornerOrder[i]]] = cubieInPos[cubiesOnFace[face][cornerOrder[i-1]]];
			}
			cubieInPos[cubiesOnFace[face][edgeOrder[0]]] = temp;
			cubieInPos[cubiesOnFace[face][cornerOrder[0]]] = temp2;
		} else if (move == 1) //CCW
		{
			add *= -1;
			
			temp = cubieInPos[cubiesOnFace[face][edgeOrder[0]]];
			temp2 = cubieInPos[cubiesOnFace[face][cornerOrder[0]]];
			for (int i = 0; i < 3; i++)
			{
				cubieInPos[cubiesOnFace[face][edgeOrder[i]]] = cubieInPos[cubiesOnFace[face][edgeOrder[i+1]]];
				cubieInPos[cubiesOnFace[face][cornerOrder[i]]] = cubieInPos[cubiesOnFace[face][cornerOrder[i+1]]];
			}
			cubieInPos[cubiesOnFace[face][edgeOrder[3]]] = temp;
			cubieInPos[cubiesOnFace[face][cornerOrder[3]]] = temp2;
			
			clockwise = !clockwise;
		} else //180
		{
			add *= 2;
			
			for (int i = 0; i < 2; i++)
			{
				temp = cubieInPos[cubiesOnFace[face][edgeOrder[i]]];
				temp2 = cubieInPos[cubiesOnFace[face][cornerOrder[i]]];
				cubieInPos[cubiesOnFace[face][edgeOrder[i]]] = cubieInPos[cubiesOnFace[face][edgeOrder[i+2]]];
				cubieInPos[cubiesOnFace[face][cornerOrder[i]]] = cubieInPos[cubiesOnFace[face][cornerOrder[i+2]]];
				cubieInPos[cubiesOnFace[face][edgeOrder[i+2]]] = temp;
				cubieInPos[cubiesOnFace[face][cornerOrder[i+2]]] = temp2;
			}
		}
		//Set direction and distance of turn
		if (face%5 == 0) {
			turnArr[1] = add;
			axis = 1;
		} else if (face == 1 || face == 3) {
			turnArr[2] = add;
			axis = 2;
		} else {
			turnArr[0] = add;
			axis = 0;
		}
		for (int i = 0; i < 9; i++)
		{
			//Edit the base to be FULLY rotated
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateBase(turnArr);
			
			//Rotate Cubie face positions
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateFacePos(clockwise, axis);
			if (move == 2) cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateFacePos(clockwise, axis);
			
			//Set black face to be visible on each rotated cubie
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].SetFacePositionVisible(true, faceBlackUnderside[faceTurning]);
			
			//Set black face to be visible on each cubie now exposed by the turn face rotating
			int toEdit = cubiesOnFace[faceTurning][i];
			if (toEdit >= 13) toEdit++;
			toEdit += fromFaceToCenter[faceTurning];
			if (toEdit != 13)
			{
				if (toEdit >= 14) toEdit--;
				cubies[cubieInPos[toEdit]].SetFacePositionVisible(true, faceTurning);
			}
		}
	}
}

/*
 * 
 */
void RC::InterpFaceRot(float progress)
{
	for (int i = 0; i < 3; i++)
	{
		//Inbetween val
		interpArr[i] = turnArr[i] * (1-progress) * -1;
	}
	// Only turns the 9 face cubies
	for (int i = 0; i < 9; i++) 
	{
		//Reset Rotation
		cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].ResetToBase();
		//Apply interpolated face rotation to straight-on cubie
		cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateRelative(interpArr);
		//Apply total rotation back
		cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateRelative(rotationTotal);
	}
	//Turn the rest of the cubie to match
	for (int i = 0; i < 17; i++)
	{
		cubies[cubieInPos[notInFaceTurning[faceTurning][i]]].ResetToBase();
		cubies[cubieInPos[notInFaceTurning[faceTurning][i]]].RotateRelative(rotationTotal);
	}
	
	//Determing whether the center cubie of the face is behind a z of 0
	float centerCubieZ = (cubies[cubieInPos[cubiesOnFace[faceTurning][4]]].points[0].z - 
			cubies[cubieInPos[cubiesOnFace[faceTurning][4]]].points[6].z)/2 + 
			cubies[cubieInPos[cubiesOnFace[faceTurning][4]]].points[6].z;
	rotatingFaceBehind = (centerCubieZ >= 0); //if it is behind (larger than) 0, it is drawn second	
}

/*
 * 
 */
void RC::TestUpdate()
{
	float testRot [3] = {0, 0, 0};
	if (passiveRot)
	{
//		for (int i = 0; i < 3; i++)
//		{
//			//if (i == 1)
//			testRot[i] = 0.03f;
//		}
		testRot[0] = 0.01f;
		testRot[1] = 0.02f;
		testRot[2] = 0.03f;
	}
	
	if (rotating)
	{
		for (int i = 0; i < 3; i++)
		{
			rotationTotal[i] += testRot[i];
		}
		
		rotProgress += turnSpd;
		if (rotProgress > 1)
		{
			rotating = false;
			rotProgress = 1;
			for (int i = 0; i < 26; i++)
			{
				cubies[i].ResetVisibleFace();
			}
		}
		InterpFaceRot(rotProgress);
	} else
	{
		RotateCubies(testRot);
	}
}

void RC::GetSuccessors(const RCState &nodeID, std::vector<RCState> &neighbors) const
{
	neighbors.resize(18);
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, neighbors[x]);
	}
}

void RC::GetPrunedActions(const RCState &nodeID, RCAction lastAction, std::vector<RCAction> &actions) const
{
	assert(false);
//	actions.resize(0);
//	for (int x = 0; x < 18; x++)
//	{
//		// 1. after any face you can't turn the same face again
//		if (x/3 == lastAction/3)
//			continue;
//
//		// 2. after faces 5, 4, 3 you can't turn 0, 2, 1 respectively
//		if ((1 == (lastAction/3)%2) &&
//			(x/3+1 == lastAction/3))
//			continue;
//
//		actions.push_back(x);
//	}
}


void RC::GetActions(const RCState &nodeID, std::vector<RCAction> &actions) const
{
	actions.resize(0);
	if (!pruneSuccessors || history.size() == 0)
	{
		for (int x = 0; x < 18; x++)
			actions.push_back(x);
	}
	else {
		// 1-3, 2-4, 0-5
		// 0, 5, 2, 4, 1, 3
		//int firstFace[6] = {true, true, false, false, true, false};
		int firstFace[6] = {-1, -1, -1, 1, 2, 0};
		for (int x = 0; x < 18; x++)
		{
			// 1. after any face you can't turn the same face again
			if (x/3 == history.back()/3)
				continue;

			// 2. after faces 0, 1, 2 you can't turn 4, 3, 5 respectively
			if (firstFace[x/3] == history.back()/3)
				continue;
//			if ((1 == (history.back()/3)%2) &&
//				(x/3+1 == history.back()/3))
//				continue;

			actions.push_back(x);
		}
	}
//	std::random_shuffle(actions.begin(), actions.end());
}

RCAction RC::GetAction(const RCState &s1, const RCState &s2) const
{
	//std::vector<RCAction> succ;
	//GetActions(s1, succ);
	RCState tmp;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(s1, x, tmp);
		if (tmp == s2)
			return x;
	}
	assert(false);
	return 0;
}

void RC::ApplyAction(RCState &s, RCAction a) const
{
	s.RotateFace(a/3, a%3);
	if (pruneSuccessors)
		history.push_back(a);
}

void RC::UndoAction(RCState &s, RCAction a) const
{
	if (pruneSuccessors)
	{
		assert(history.back() == a);
		history.pop_back();
	}
	InvertAction(a);
	s.RotateFace(a/3, a%3);
//	assert(false);
//	// TODO: verify
//	s.RotateFace(a/3, 2-(a%3));
}

void RC::GetNextState(const RCState &s1, RCAction a, RCState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool RC::InvertAction(RCAction &a) const
{
	if (2 == a%3)
		return true;
	if (1 == a%3)
	{
		a -= 1;
		return true;
	}
	a += 1;
	return true;
	return false;
}

double RC::HCost(const RCState &node1, const RCState &node2, double parentHCost) const
{
	return HCost(node1, node2);
}

/** Heuristic value between two arbitrary nodes. **/
double RC::HCost(const RCState &node1, const RCState &node2) const
{
	return 0;
}

/** Heuristic value between node and the stored goal. Asserts that the
 goal is stored **/
double RC::HCost(const RCState &node) const
{
	return HCost(node, node);
}

bool RC::GoalTest(const RCState &node, const RCState &goal) const
{
	return node == goal;
}

/** Goal Test if the goal is stored **/
bool RC::GoalTest(const RCState &node) const
{
	for (int x = 0; x < 20; x++)
	{
		if (node.indices[x] != x)
			return false;
		if (node.rotation[x] != 0)
			return false;
	}
	return false;
}

uint64_t RC::GetStateHash(const RCState &node) const
{
	// TODO: write
	return 0;
}

void RC::GetStateFromHash(uint64_t hash, RCState &node) const
{
// TODO: write
}

//void RC::OpenGLDraw() const
//{}
//
//void RC::OpenGLDraw(const RCState&s) const
//{}
//
//void RC::OpenGLDrawCorners(const RCState&s) const
//{}
//
//void RC::OpenGLDrawEdges(const RCState&s) const
//{}
//
//void RC::OpenGLDrawEdgeDual(const RCState&s) const
//{}
//
//void RC::OpenGLDrawCubeBackground() const
//{}
//
//void RC::OpenGLDrawCenters() const
//{}
//
//void RC::OpenGLDrawCube(int cube) const
//{}

void RC::SetFaceColor(int theColor) const
{}


///** Draw the transition at some percentage 0...1 between two states */
//void RC::OpenGLDraw(const RCState &s1, const RCState &s2, float t) const
//{}
//
//void RC::OpenGLDraw(const RCState&, const RCAction&) const
//{
//	
//}

// Draw a RCState. Internally do the 3d transform for the drawing.
void RC::Draw(Graphics::Display &display, const RCState &state) const
{
	// TEMP: Refresh the cube EVERY FRAME (BAD)
	for (int i = 0; i < 20; i++)
	{
		cubies[convertStatePos[i]].Initialize(i, state.indices[i], state.rotation[i]);
	}

	// For every cubie, return it to its original draw angle, and then apply the needed rotation
	// This codeblock is a section from RotateCubies(), consider creating a new function
	for (int i = 0; i < 26; i++)
	{
		cubies[i].ResetToBase();
		cubies[i].RotateRelative(rotationTotal);
	}
	
	// Draw the newly reset and rotated cubies
	DrawCubies(display);
}


/*
 * 
 */
void EditPointMult(float (&vec)[3][3], Graphics::point & p)
{
	int vecCols = 3;
	int vecRows = 3;

	array3[0] = p.x;
	array3[1] = p.y;
	array3[2] = p.z;

	for (int i = 0; i < vecRows; i++)
	{
		float sum = 0.f;

		for (int j = 0; j < vecCols; j++)
		{
			sum += vec[i][j] * array3[j];
		}

		switch (i)
		{
			case 0: p.x = sum;
				break;
			case 1: p.y = sum;
				break;
			case 2: p.z = sum;
				break;
		}
	}
}

/* Edit an existing 3x3 vector to become the
 * X rotation multiplication matrix for a given angle
 */
//void MakeMatrixRotX(std::vector<std::vector<float>> & a, float angle) {
void MakeMatrixRotX(float (&a)[3][3], float angle) {
	a[0][0] = 1;
	a[0][1] = 0;
	a[0][2] = 0;

	a[1][0] = 0;
	a[1][1] = cos(angle);
	a[1][2] = -sin(angle);

	a[2][0] = 0;
	a[2][1] = sin(angle);
	a[2][2] = cos(angle);
}

/* Edit an existing 3x3 vector to become the
 * Y rotation multiplication matrix for a given angle
 */
//void MakeMatrixRotY(std::vector<std::vector<float>> & a, float angle) {
void MakeMatrixRotY(float (&a)[3][3], float angle) {	
	a[0][0] = cosf(angle);
	a[0][1] = 0;
	a[0][2] = -sin(angle);

	a[1][0] = 0;
	a[1][1] = 1;
	a[1][2] = 0;

	a[2][0] = sin(angle);
	a[2][1] = 0;
	a[2][2] = cos(angle);
}

/* Edit an existing 3x3 vector to become the
 * Z rotation multiplication matrix for a given angle
 */
//void MakeMatrixRotZ(std::vector<std::vector<float>> & a, float angle) {
void MakeMatrixRotZ(float (&a)[3][3], float angle) {
	a[0][0] = cosf(angle);
	a[0][1] = -sin(angle);
	a[0][2] = 0;

	a[1][0] = sin(angle);
	a[1][1] = cos(angle);
	a[1][2] = 0;

	a[2][0] = 0;
	a[2][1] = 0;
	a[2][2] = 1;
}

/* Returns the index of the integer element within the given bounds of an array
 * Returns -1 if not found
 */
int findInArray(int arr[], int elem, int lower, int upper)
{
	for (int i = lower; i < upper; i++)
	{
		if (arr[i] == elem) return i;
	}
	return -1;
	// TODO: binary search
}

/*===============================================================================================================================================================================
 * 			RANKING (CORNERS)
 ===============================================================================================================================================================================*/

uint64_t RC::GetPDBSizeCorner() const
{
	uint64_t answer[] = {1, 24, 504, 9072, 136080, 1632960, 14696640, 88179840, 88179840};
	return 2187; //answer[corners.size()];
}

uint64_t RC::GetPDBHashCorner(const RCState &s, int threadID) const
{
	// Initialize variables
	int puzzle[8];
	int dual[16]; 
	int cornerSize = corners.size();
	int lastPiece = 8-(int)corners.size();
	for (int x = 0; x < 8; x++)
		dual[s.indices[x+12]-12] = x;
	for (int x = 0; x < cornerSize; x++)
		puzzle[x] = dual[corners[x]];
	
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	int numEntriesLeft = 8;
//	for (unsigned int x = 0; x < cornerSize; x++)
//	{
//		hashVal += puzzle[x] * FactorialUpperK(numEntriesLeft-1, lastPiece);
//		
//		numEntriesLeft--;
//		for (unsigned y = x; y < cornerSize; y++)
//		{
//			if (puzzle[y] > puzzle[x])
//				puzzle[y]--;
//		}
//	}
	
	int limit = std::min((int)cornerSize, 7);
	for (int x = 0; x < limit; x++)
	{
		part2 = part2*3 + s.rotation[corners[x]];
	}
	
	return part2; //* FactorialUpperK(8, lastPiece)+hashVal;
}


void RC::GetStateFromPDBHashCorner(uint64_t hash, RCState &s, int threadID) const
{
	int lastPiece = 8-(int)corners.size();
	int cornerSize = corners.size();
	int puzzle[12];
	int dual[16];
	uint64_t hashVal = hash;
	
//	hash /= FactorialUpperK(8, lastPiece); // for rotations
//	hashVal = hashVal%FactorialUpperK(8, lastPiece); // for pieces
//	
//	int numEntriesLeft = lastPiece+1;
//	for (int x = corners.size()-1; x >= 0; x--)
//	{
//		puzzle[x] = hashVal % numEntriesLeft;
//		hashVal /= numEntriesLeft;
//		numEntriesLeft++;
//		for (int y = x+1; y < cornerSize; y++)
//		{
//			if (puzzle[y] >= puzzle[x])
//				puzzle[y]++;
//		}
//	}
	
	for (int x = 0; x < 8; x++)
	{
		s.indices[x+12] = 0xF;
		s.rotation[x+12] = x;
	}
	
	for (int x = 0; x < cornerSize; x++)
	{
		s.indices[puzzle[x]] = corners[x];
		dual[corners[x]] = puzzle[x];
	}
	
	int cnt = 0; 
	int limit = std::min((int)corners.size(), 7);
	for (int x = limit-1; x >= 0; x--)
	{
		s.rotation[corners[x]] = hash%3;
		cnt += hash%3;
		hash/=3;
	}
	if (corners.size() == 8)
		s.rotation[corners[7]] = (3-(cnt%3)); // 0->0 2->1 1->2
}

// KEEP
uint64_t RC::GetStateHashCorner(const RCState &s)
{
	uint64_t hashVal = 0;
	for (int x = 0; x < 7; x++)
	{
		hashVal = (hashVal<<1) + s.rotation[19-x];
	}
	return hashVal;
}

/*===============================================================================================================================================================================
 * 			RANKING (EDGES)
 ===============================================================================================================================================================================*/
//
//uint64_t RC::GetPDBSizeEdge() const
//{
//	// last tile is symmetric
//	uint64_t power2[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 2048};
//	int elts = (int)edges.size();
//	return FactorialUpperK(12, 12-elts)*power2[elts];
////	return mr1.GetMaxRank()*power2[elts];
//}
//
//uint64_t RC::GetPDBHashEdge(const RCState &s, int threadID) const
//{
//	int puzzle[12];
//	int dual[16]; // seamlessly handle 0xF entries (no cube)
//	int lastPiece = 12-(int)edges.size();
//	//std::cout << "!" << s << "\n";
//	for (int x = 0; x < 12; x++)
//		dual[s.GetCubeInLoc(x)] = x;
//	for (int x = 0; x < edges.size(); x++)
//		puzzle[x] = dual[edges[x]];
//	
//	uint64_t hashVal = 0;
//	uint64_t part2 = 0;
//	int numEntriesLeft = 12;
//	for (unsigned int x = 0; x < edges.size(); x++)
//	{
//		hashVal += puzzle[x]*FactorialUpperK(numEntriesLeft-1, lastPiece);
//		
//		numEntriesLeft--;
//		for (unsigned y = x; y < edges.size(); y++)
//		{
//			if (puzzle[y] > puzzle[x])
//				puzzle[y]--;
//		}
//	}
//	int limit = std::min((int)edges.size(), 11);
//	for (int x = 0; x < limit; x++)
//	{
//		part2 = part2*2+(s.GetCubeOrientation(edges[x])?1:0);
//		//part2 = part2*3+s.GetCubeOrientation(dual[corners[x]]);
//	}
//	return part2*FactorialUpperK(12, lastPiece)+hashVal;
//}
//
//void RC::GetStateFromPDBHashEdge(uint64_t hash, RCState &s, int threadID) const
//{
//	int lastPiece = 12-(int)edges.size();
//	int puzzle[12];
//	int dual[16];
//	uint64_t hashVal = hash;
//	hash /= FactorialUpperK(12, lastPiece); // for rotations
//	hashVal = hashVal%FactorialUpperK(12, lastPiece); // for pieces
//	
//	int numEntriesLeft = lastPiece+1;
//	for (int x = edges.size()-1; x >= 0; x--)
//	{
//		puzzle[x] = hashVal%numEntriesLeft;
//		hashVal /= numEntriesLeft;
//		numEntriesLeft++;
//		for (int y = x+1; y < edges.size(); y++)
//		{
//			if (puzzle[y] >= puzzle[x])
//				puzzle[y]++;
//		}
//	}
//	for (int x = 0; x < 12; x++)
//	{
//		s.SetCubeInLoc(x, 0xF);
//		s.SetCubeOrientation(x, 0);
//	}
//	
//	for (int x = 0; x < edges.size(); x++)
//	{
//		s.SetCubeInLoc(puzzle[x], edges[x]);
//		dual[edges[x]] = puzzle[x];
//	}
//	
//	int cnt = 0;
//	int limit = std::min((int)edges.size(), 11);
//	for (int x = limit-1; x >= 0; x--)
//	{
//		s.SetCubeOrientation(edges[x], hash%2);
//		//s.SetCubeOrientation(dual[corners[x]], hash%3);
//		cnt += hash%2;
//		hash/=2;
//	}
//	if (edges.size() == 12)
//		s.SetCubeOrientation(edges[11], cnt%2);
//}
//
//
//// KEEP
//uint64_t RC::GetStateHashEdge(const RCState &s)
//{
//	uint64_t hashVal = 0;
//	for (int x = 0; x < 11; x++)
//	{
//		hashVal = (hashVal<<1)+s.GetCubeOrientation(11-x);
//	}
//	return hashVal;
//}

uint64_t RC::FactorialUpperK(int n, int k) const
{
	const uint64_t result[9][9] = {
		{1}, // n = 0
		{1, 1}, // n = 1
		{2, 2, 1}, // n = 2
		{6, 6, 3, 1}, // n = 3
		{24, 24, 12, 4, 1}, // n = 4
		{120, 120, 60, 20, 5, 1}, // n = 5
		{720, 720, 360, 120, 30, 6, 1}, // n = 6
		{5040, 5040, 2520, 840, 210, 42, 7, 1}, // n = 7
		{40320, 40320, 20160, 6720, 1680, 336, 56, 8, 1}, // n = 8
	};
	return result[n][k];
}



// PDB Class

RCPDB::RCPDB(RC *e,
			 const std::array<bool, 12> &edgeRotations, const std::array<bool, 12> &edgeLocations,
			 const std::array<bool, 8> &cornerRotations, const std::array<bool, 8> &cornerLocations)
:PDBHeuristic(e)
{
	for (int x = 0; x < 12; x++)
	{
		if (edgeRotations.at(x))
			this->edgeRotations.push_back(x);
		if (edgeLocations.at(x))
			this->edgeLocations.push_back(x);
	}
	for (int x = 0; x < 8; x++)
	{
		if (cornerRotations.at(x))
			this->cornerRotations.push_back(x);
		if (cornerLocations.at(x))
			this->cornerLocations.push_back(x);
	}
}

uint64_t RCPDB::GetStateHash(const RCState &s) const
{
	assert(false); // not implemented currently
}

void RCPDB::GetStateFromHash(RCState &s, uint64_t hash) const
{
	assert(false);
}

uint64_t RCPDB::GetPDBSize() const
{
	return 	GetEdgeRotationSize()*GetEdgeLocationSize()*GetCornerRotationSize()*GetCornerLocationSize();
//	return 4478976; // 2^11 * 3^7
}

uint64_t RCPDB::GetPDBHash(const RCState &s, int threadID) const
{
	uint64_t hash = 0;
	hash = GetEdgeLocationHash(s); // edge loc
	hash *= GetEdgeRotationSize(); // make space for edge rot
	hash += GetEdgeRotationHash(s); // edge rot
	hash *= GetCornerLocationSize(); // make space for corner loc
	hash += GetCornerLocationHash(s); // corner loc
	hash *= GetCornerRotationSize(); // make space for corner rot
	hash += GetCornerRotationHash(s); // corner rot
//	std::cout << "Hashes: el:" << GetEdgeLocationHash(s) << " er:" << GetEdgeRotationHash(s) <<
//	" cl:" << GetCornerLocationHash(s) << " cr:" << GetCornerRotationHash(s) << "\n";
	return hash;
	
//	// NOTE: Assuming edges + corners for now
//	//	int indices[20]; //Index of cubie in position
//	//	int rotation[20]; //Rotation of cubie in position
//	uint64_t hash = 0;
//	for (int x = 0; x < 11; x++)
//		hash = 2*hash + s.rotation[x];
//	for (int x = 12; x < 19; x++)
//		hash = 3*hash + s.rotation[x];
//	return hash;
}

void RCPDB::GetStateFromPDBHash(uint64_t hash, RCState &s, int threadID) const
{
//	hash = GetEdgeLocationHash(s); // edge loc
//	hash *= GetEdgeRotationSize(); // make space for edge rot
//	hash += GetEdgeRotationHash(s); // edge rot
//	hash *= GetCornerLocationSize(); // make space for corner loc
//	hash += GetCornerLocationHash(s); // corner loc
//	hash *= GetCornerRotationSize(); // make space for corner rot
//	hash += GetCornerRotationHash(s); // corner rot


	GetStateFromCornerRotationHash(s, hash%GetCornerRotationSize());
//	std::cout << "unrank cr:" << hash%GetCornerRotationSize();
	hash /= GetCornerRotationSize();
	GetStateFromCornerLocationHash(s, hash%GetCornerLocationSize());
//	std::cout << " cl:" << hash%GetCornerLocationSize();
	hash /= GetCornerLocationSize();
	GetStateFromEdgeRotationHash(s, hash%GetEdgeRotationSize());
//	std::cout << " er:" << hash%GetEdgeRotationSize();
	hash /= GetEdgeRotationSize();
	GetStateFromEdgeLocationHash(s, hash);
//	std::cout << " el:" << hash << "\n";

//	int two = 0;
//	int three = 0;
//	for (int x = 18; x >= 12; x--)
//	{
//		s.rotation[x] = hash%3;
//		three += s.rotation[x];
//		hash /= 3;
//	}
//	// fill in s.rotation[19]
//	s.rotation[19] = (3-(two%3))%3;
//	for (int x = 10; x >= 0; x--)
//	{
//		s.rotation[x] = hash%2;
//		two += s.rotation[x];
//		hash /= 2;
//	}
//	// fill in s.rotation[11]
//	s.rotation[11] = two%2;
}

uint64_t RCPDB::GetEdgeRotationSize() const
{
	int limit = std::min((int)edgeRotations.size(), 11);
	return 1ull<<limit;
}

uint64_t RCPDB::GetEdgeLocationSize() const
{
	return FactorialUpperK(12, 12-(int)edgeLocations.size());
}

uint64_t RCPDB::GetCornerRotationSize() const
{
	uint64_t answer[] = {1, 3, 9, 27, 81, 243, 729, 2187, 6561, 19683};
	int limit = std::min((int)cornerRotations.size(), 7);
	return answer[limit];
}

uint64_t RCPDB::GetCornerLocationSize() const
{
	return FactorialUpperK(8, 8-(int)cornerLocations.size());
}

uint64_t RCPDB::GetEdgeRotationHash(const RCState &s) const
{
	int limit = std::min((int)edgeRotations.size(), 11);
	uint64_t hash = 0;
	for (int x = 0; x < limit; x++)
	{
		hash = hash*2+(s.rotation[edgeRotations[x]]?1:0);
		//part2 = part2*3+s.GetCubeOrientation(dual[corners[x]]);
	}
	return hash;
}

uint64_t RCPDB::GetEdgeLocationHash(const RCState &s) const
{
	if (edgeLocations.size() == 0)
		return 0;
	int puzzle[12];
	int dual[16]; // seamlessly handle 0xF entries (no cube)
	int lastPiece = 12-(int)edgeLocations.size();
	//std::cout << "!" << s << "\n";
	for (int x = 0; x < 12; x++)
		dual[s.indices[x]] = x;
	for (int x = 0; x < edgeLocations.size(); x++)
		puzzle[x] = dual[edgeLocations[x]];
	
	uint64_t hashVal = 0;
	int numEntriesLeft = 12;
	for (unsigned int x = 0; x < edgeLocations.size(); x++)
	{
		hashVal += puzzle[x]*FactorialUpperK(numEntriesLeft-1, lastPiece);
		
		numEntriesLeft--;
		for (unsigned y = x; y < edgeLocations.size(); y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	return hashVal;
	
}

uint64_t RCPDB::GetCornerRotationHash(const RCState &s) const
{
	int limit = std::min((int)cornerRotations.size(), 7);
	uint64_t hash = 0;
	for (int x = 0; x < limit; x++)
	{
		hash = hash*3+s.rotation[12+cornerRotations[x]];
		//part2 = part2*3+s.GetCubeOrientation(dual[corners[x]]);
	}
	return hash;
}

uint64_t RCPDB::GetCornerLocationHash(const RCState &s) const
{
	if (cornerLocations.size() == 0)
		return 0;
	// TODO: handle fewer according to pattern
	int puzzle[8];
	int dual[16]; // seamlessly handle 0xF entries (no cube)
	int cornerSize = (int)cornerLocations.size();
	int lastPiece = 8-(int)cornerLocations.size();
	for (int x = 0; x < 8; x++)
		dual[s.indices[12+x]-12] = x;
	for (int x = 0; x < cornerLocations.size(); x++)
		puzzle[x] = dual[cornerLocations[x]];
	
	uint64_t hashVal = 0;
	int numEntriesLeft = 8;
	for (unsigned int x = 0; x < cornerSize; x++)
	{
		hashVal += puzzle[x]*FactorialUpperK(numEntriesLeft-1, lastPiece);
		
		numEntriesLeft--;
		for (unsigned y = x; y < cornerLocations.size(); y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	return hashVal;
//	return part2*FactorialUpperK(8, lastPiece)+hashVal;
}

void RCPDB::GetStateFromEdgeRotationHash(RCState &s, uint64_t hash) const
{
	for (int x = 0; x < 12; x++)
	{
//		s.SetCubeInLoc(x, 0xF);
//		s.SetCubeOrientation(x, 0);
		s.rotation[x] = 0;
	}
	
	int cnt = 0;
	int limit = std::min((int)edgeRotations.size(), 11);
	for (int x = limit-1; x >= 0; x--)
	{
		s.rotation[edgeRotations[x]] = hash%2;
//		s.SetCubeOrientation(edgeRotations[x], hash%2);
		//s.SetCubeOrientation(dual[corners[x]], hash%3);
		cnt += hash%2;
		hash/=2;
	}
	if (edgeRotations.size() == 12)
		s.rotation[edgeRotations[11]] = cnt%2;
//		s.SetCubeOrientation(edges[11], cnt%2);

}

void RCPDB::GetStateFromEdgeLocationHash(RCState &s, uint64_t hash) const
{
	int lastPiece = 12-(int)edgeLocations.size();
	int puzzle[12];
	int dual[16];
	uint64_t hashVal = hash;
	hash /= FactorialUpperK(12, lastPiece); // for rotations
	hashVal = hashVal%FactorialUpperK(12, lastPiece); // for pieces
	
	int numEntriesLeft = lastPiece+1;
	for (int x = (int) edgeLocations.size()-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < edgeLocations.size(); y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}
	for (int x = 0; x < 12; x++)
	{
		s.indices[x] = 0xF;
//		s.SetCubeInLoc(x, 0xF);
//		s.SetCubeOrientation(x, 0);
	}
	
	for (int x = 0; x < edgeLocations.size(); x++)
	{
		s.indices[puzzle[x]] = edgeLocations[x];
		//s.SetCubeInLoc(puzzle[x], edgeLocations[x]);
		dual[edgeLocations[x]] = puzzle[x];
	}
	
}

void RCPDB::GetStateFromCornerRotationHash(RCState &s, uint64_t hash) const
{
	for (int x = 0; x < 8; x++)
	{
		s.rotation[12+x] = 0;
	}
	int cnt = 0;
	int limit = std::min((int)cornerRotations.size(), 7);
	for (int x = limit-1; x >= 0; x--)
	{
		s.rotation[12+cornerRotations[x]] = hash%3;
//		s.SetCubeOrientation(cornerRotations[x], hash%3);
		cnt += hash%3;
		hash/=3;
	}
	if (cornerRotations.size() == 8)
	{
		s.rotation[12+cornerRotations[7]] = (3-(cnt%3))%3; // 0->0 2->1 1->2
		cnt += s.rotation[12+cornerRotations[7]];
	}
	assert((cnt%3) == 0);
}

void RCPDB::GetStateFromCornerLocationHash(RCState &s, uint64_t hash) const
{
	int lastPiece = 8-(int)cornerLocations.size();
	int cornerSize = (int)cornerLocations.size();
	int puzzle[12];
//	int dual[16];
	uint64_t hashVal = hash;
	hash /= FactorialUpperK(8, lastPiece); // for rotations
	hashVal = hashVal%FactorialUpperK(8, lastPiece); // for pieces
	
	int numEntriesLeft = lastPiece+1;
	for (int x = (int)cornerLocations.size()-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < cornerSize; y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}
	for (int x = 0; x < 8; x++)
	{
		s.indices[12+x] = 0xF;
//		s.SetCubeInLoc(x, 0xF);
//		s.SetCubeOrientation(x, 0);
	}
	
	for (int x = 0; x < cornerSize; x++)
	{
		s.indices[12+puzzle[x]] = 12+cornerLocations[x];
//		s.SetCubeInLoc(puzzle[x], cornerLocations[x]);
//		dual[cornerLocations[x]] = puzzle[x];
	}
}

uint64_t RCPDB::FactorialUpperK(int n, int k) const
{
	const uint64_t result[13][13] = {
		{1}, // n = 0
		{1, 1}, // n = 1
		{2, 2, 1}, // n = 2
		{6, 6, 3, 1}, // n = 3
		{24, 24, 12, 4, 1}, // n = 4
		{120, 120, 60, 20, 5, 1}, // n = 5
		{720, 720, 360, 120, 30, 6, 1}, // n = 6
		{5040, 5040, 2520, 840, 210, 42, 7, 1}, // n = 7
		{40320, 40320, 20160, 6720, 1680, 336, 56, 8, 1}, // n = 8
		{362880, 362880, 181440, 60480, 15120, 3024, 504, 72, 9, 1}, // n = 9
		{3628800, 3628800, 1814400, 604800, 151200, 30240, 5040, 720, 90, 10, 1}, // n = 10
		{39916800, 39916800, 19958400, 6652800, 1663200, 332640, 55440, 7920, 990, 110, 11, 1}, // n = 11
		{479001600, 479001600, 239500800, 79833600, 19958400, 3991680, 665280, 95040, 11880, 1320, 132, 12, 1} // n = 12
	};
	return result[n][k];
}

