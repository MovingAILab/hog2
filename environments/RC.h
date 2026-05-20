//
//  RC.h (Alternate Rubik's Cube implementation)
//
//  Created by Nathan Sturtevant on 7/9/21.
//

#ifndef __RC__
#define __RC__

#include <iostream>
#include <stdint.h>
#include <unordered_map>
#include <vector>
#include <array>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"

/*===============================================================================================================================================================================
 * 			CUBIE CLASS
 ===============================================================================================================================================================================*/

/* Contains all data necessary to DRAW a cubie
 * Can draw itself
 */

class Cubie
{
public:
	//Changing Variables
	std::vector<Graphics::point> basePoints;
	std::vector<std::vector<Graphics::point>> baseFacePoints;
	std::vector<Graphics::point> points;
	std::vector<std::vector<Graphics::point>> facePoints;
	rgbColor faceCols [6];
	int index;
	int RCindex = -1; //TEMP
	bool facesShown [6];
	int blackFaceToReset = -1;
	int faceInPos [6] = {0,1,2,3,4,5};

	//Constants
	const float sideLen = 0.2f;
	const float faceSize = 0.85f;
	const rgbColor outCol = rgbColor(0.f, 0.f, 0.f);
	const rgbColor inCol = rgbColor(0.04f, 0.04f, 0.06f);
	const std::vector<std::vector<float>> projection =
	{
		{1, 0, 0},
		{0, 1, 0}
	};
	const int pointsOnFace [6][4] =
	{
		{0, 1, 5, 4},
		{0, 1, 2, 3},
		{3, 0, 4, 7},
		{4, 5, 6, 7},
		{1, 2, 6, 5},
		{2, 3, 7, 6}
	};
	const rgbColor faceColors [6] =
	{
		rgbColor(255.f, 255.f, 255.f),	//	sf::Color::White,
		rgbColor(0.f, 0.f, 255.f),		//	sf::Color::Blue,
		rgbColor(255.f, 0.f, 0.f),		//	sf::Color::Red,
		rgbColor(0.f, 255.f, 0.f),		//	sf::Color::Green,
		Colors::orange, //rgbColor(255.f, 89.f, 0.f),		//	orangeCol,
		Colors::yellow //rgbColor(255.f, 240.f, 31.f),	//	sf::Color::Yellow,
	};
	const int faceOrderByAxis [3][4] =
	{
		{0,1,5,3},
		{1,2,3,4},
		{0,4,5,2}
	};
	const int facesShowing [20][3] =
	{
		// Visible faces for a given cubie position
		/* (With blue at front and white at top}
		 * 0 = Top
		 * 1 = Front
		 * 2 = Left
		 * 3 = Right
		 * 4 = Back
		 * 5 = Bottom
		 * -1 = N/A
		 */
			
		// Also represents the colors present on a given cubie index
		/* 0 = White... 5 = Yellow
		 * For corners, the colors are listed in a CLOCKWISE order from the face of index 0/5
		 * The face
		 */
			
		// Edges
		{0, 2, -1}, //0
		{1, 2, -1}, //1
		{0, 1, -1}, //2
		{1, 4, -1}, //3
		{0, 4, -1}, //4
		{3, 4, -1}, //5
		{0, 3, -1}, //6
		{3, 2, -1}, //7
		{5, 2, -1}, //8
		{5, 1, -1}, //9
		{5, 4, -1}, //10
		{5, 3, -1}, //11
		// Corners
		{0, 1, 2}, //12
		{0, 4, 1}, //13
		{0, 3, 4}, //14
		{0, 2, 3}, //15
		{5, 2, 1}, //16
		{5, 1, 4}, //17
		{5, 4, 3}, //18
		{5, 3, 2}  //19
	};
	
	void Initialize(int ind);
	void Initialize(int RCpos, int RCind, int RCrot);
	void Draw(Graphics::Display &display) const;
	void DrawFace(Graphics::Display &display, int index) const;
	void RotateRelative(const float angle[3]);
	void RotateBase(float angle [3]);
	void RotateFacePos(bool clockwise, int axis); 
	void ResetToBase();
	void ResetVisibleFace();
	void SetFacePositionVisible(bool toggle, int position);
	void PrintData();
	//Cubie & operator=(const Cubie & rhs);
};

/*===============================================================================================================================================================================
 * 			RCSTATE CLASS
 ===============================================================================================================================================================================*/

class RCState
{
public:
	RCState()
	{
		Reset();
	}
	void Reset()
	{
		// Set all values to default
		for (int i = 0; i < 20; i++)
		{
			indices[i] = i;
			rotation[i] = 0;
		}
	}
	// 12 edges followed by 8 corners
	int indices[20]; //Index of cubie in position //uint8_t
	int rotation[20]; //Rotation of cubie in position
	
//const int edgesOnFace[6][4] =
//{
//	{0, 6, 4, 2},  //0
//	{2, 3, 9, 1},  //1
//	{0, 1, 8, 7},  //2
//	{6, 7, 11, 5}, //3
//	{4, 5, 10, 3}, //4
//	{9, 10, 11, 8} //5
//};
//	int cornersOnFace[6][4] = 
//	{
//		{4 +11, 3 +11, 2 +11, 1 +11}, //0
//		{1 +11, 2 +11, 6 +11, 5 +11}, //1
//		{4 +11, 1 +11, 5 +11, 8 +11}, //2
//		{3 +11, 4 +11, 8 +11, 7 +11}, //3
//		{2 +11, 3 +11, 7 +11, 6 +11}, //4
//		{5 +11, 6 +11, 7 +11, 8 +11}  //5
//	};
//const int cornersOnFace[6][4] =
//{
//	{15, 14, 13, 12}, //0
//	{16, 12, 13, 17}, //1
//	{15, 12, 16, 19}, //2
//	{15, 19, 18, 14}, //3
//	{13, 14, 18, 17}, //4
//	{16, 17, 18, 19}  //5
//};
	
	// Move to RC Class
	void RotateFace(int move);
	void RotateEdges(int move);
	void RotateCorners(int move);
	void SwapPositions(int p1, int p2);
	void ShiftPositionsCW(const int (&arr)[4]);
	void ShiftPositionsCCW(const int (&arr)[4]);
	//
	
	void PrintState();
	
	// TODO: Remove
	void RotateFace(int face, int move);
	void RotateEdges(int face, int move);
	void RotateCorners(int face, int move);
	void ShiftPositions(const int (&arr)[4], bool forward);
};

typedef int RCAction;

static std::ostream &operator<<(std::ostream &out, RCState &tmp)
{
	for (int x = 0; x < 20; x++)
		out << tmp.rotation[x] << " : ";
	return out;
	
}

// Can't write these until data structures are defined
static bool operator==(const RCState &l1, const RCState &l2)
{
	for (int x = 0; x < 20; x++)
	{
		if (l1.indices[x] != l2.indices[x])
			return false;
		if (l1.rotation[x] != l2.rotation[x])
			return false;
	}
	return true;
	//return false;
	//return (l1.corner == l2.corner) && (l1.edge == l2.edge);
}
//
//static bool operator!=(const RCState &l1, const RCState &l2)
//{
//	return !(l1 == l2);
//}
//
//static std::ostream &operator<<(std::ostream &out, const RCState &s)
//{
//	out << "{" << s.edge << ", " << s.corner << "}";
//	return out;
//}

//void drawCube(Graphics::Display &display, int xPos, int yPos, int zPos, float tempAngle);

/*===============================================================================================================================================================================
 * 			RC CLASS
 ===============================================================================================================================================================================*/

class RC : public SearchEnvironment<RCState, RCAction>
{
public:
	// RELATED TO HASH FUNCTION
	std::vector<int> corners;
	std::vector<int> edges;
	
	
	//Constants TODO: MOVE TO BOTTOM
	const int fromFaceToCenter[6] =
	// Modifier: e.g. cubies on face 0 simply add 3 to their own position to get the position 
	// of the layer of cubies to get the position of the cubie 1 towards the center of the cubie
	{
		3, 9, 1, -9, -1, -3
	};
	const int faceBlackUnderside[6] =
	{
		5, 3, 4, 1, 2, 0
	};
	const float piOver2 = 1.57079632679;
	const float turnSpd = 0.04;
	const int edgeOrder [4] = {1, 5, 7, 3};
	const int cornerOrder [4] = {0, 2, 8, 6};
	const int convertStatePos [20] = 
	{	// Converts from RCState cubie indices to RC indices
		9, // 0
		3, // 1
		1, // 2
		5, // 3
		11,// 4
		22,// 5
		18,// 6
		20,// 7
		14,// 8
		7, // 9
		16,// 10
		24,// 11
		0, // 12
		2, // 13
		19,// 14
		17,// 15
		6, // 16
		8, // 17
		25,// 18
		23 // 19
	};
	
	//Cubie Object items
	mutable Cubie cubies [26];
	float rotationTotal [3];
	int cubiesOnFace [6][9]; //Face, position
	int cubieInPos[26];
	int faceTurning;
	int notInFaceTurning[6][17];
	bool rotating = false; 
	bool rotatingFaceBehind = false;
	float rotProgress = 0;
	float turnArr[3];
	float interpArr[3];
	bool passiveRot = true;
	
	void DrawCubies(Graphics::Display &display) const;
	void DrawCubiesRotating(Graphics::Display &display) const;
	void RotateCubies(float add[3]);
	void RotateFace(int face, int move);
	void InterpFaceRot(float progress);
	void TestUpdate();

	RC()
	{
		// HASH FUNCTION RELATED
		corners.resize(8);
		for (int i = 0; i < 8; i++)
		{
			corners[i] = i;
		}
		edges.resize(12);
		for (int i = 0; i < 12; i++)
		{
			edges[i] = i;
		}
		
		pruneSuccessors = false;
		for (int i = 0; i < 26; i++)
		{
			//std::cout << cubies[i].a << '\n';
			cubies[i].Initialize(i);
			cubieInPos[i] = i;
		}
		for (int i = 0; i < 3; i++)
		{
			rotationTotal[i] = 0.f;
		}
		// Initialize cubiesOnFace
		for (int i = 0; i < 9; i++)
		{
			int in = (i/3)*-9 + (i%3) + 18; in -= in/14;
			cubiesOnFace[0][i] = in;
			in = (i/3)*9 + (i%3) + 6; in -= in/14;
			cubiesOnFace[5][i] = in;
			
			in = (i/3)*3 + (i%3)*-9 + 18; in -= in/14;
			cubiesOnFace[2][i] = in;
			in = (i/3)*3 + (i%3)*9 + 2; in -= in/14;
			cubiesOnFace[4][i] = in;
			
			cubiesOnFace[1][i] = i;
			in = (i/3)*3 - (i%3) + 20; in -= in/14;
			cubiesOnFace[3][i] = in;
		}
		
		//Initialize notInFaceTurning
		for (int i = 0; i < 6; i++)
		{
			int j = 0;
			for (int k =  0; k < 26; k++)
			{
				bool add = true;
				for (int l = 0; l < 9; l++)
				{
					if (cubiesOnFace[i][l] == k)
					{
						//Don't add it to the list
						add = false;
						break;
					}
				}
				if (add)
				{
					notInFaceTurning[i][j] = k; 
					j++;
				}
			}
		}
	}

	std::string GetName() { return "RC"; }
	void SetPruneSuccessors(bool val) { pruneSuccessors = val; history.resize(0); }
	void GetSuccessors(const RCState &nodeID, std::vector<RCState> &neighbors) const;
	void GetActions(const RCState &nodeID, std::vector<RCAction> &actions) const;
	void GetPrunedActions(const RCState &nodeID, RCAction lastAction, std::vector<RCAction> &actions) const;
	RCAction GetAction(const RCState &s1, const RCState &s2) const;
	void ApplyAction(RCState &s, RCAction a) const;
	void UndoAction(RCState &s, RCAction a) const;

	void GetNextState(const RCState &, RCAction , RCState &) const;
	
	bool InvertAction(RCAction &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const RCState &node1, const RCState &node2) const;
	double HCost(const RCState &node1, const RCState &node2, double parentHCost) const;
	int Edge12PDBDist(const RCState &s);
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	double HCost(const RCState &node) const;
	
	double GCost(const RCState &node1, const RCState &node2) const { return 1.0; }
	double GCost(const RCState &node, const RCAction &act) const { return 1.0; }
	bool GoalTest(const RCState &node, const RCState &goal) const;
	
	/** Goal Test if the goal is stored **/
	bool GoalTest(const RCState &node) const;
	
	uint64_t GetStateHash(const RCState &node) const;

	uint64_t GetActionHash(RCAction act) const { return act; }
	void GetStateFromHash(uint64_t hash, RCState &node) const;
	
//	void OpenGLDraw() const;
//	void OpenGLDraw(const RCState&) const;
//	void OpenGLDrawCorners(const RCState&) const;
//	void OpenGLDrawEdges(const RCState&) const;
//	void OpenGLDrawEdgeDual(const RCState&) const;
//	void OpenGLDrawCenters() const;
//	void OpenGLDrawCubeBackground() const;
//	/** Draw the transition at some percentage 0...1 between two states */
//	void OpenGLDraw(const RCState&, const RCState&, float) const;
//	void OpenGLDraw(const RCState&, const RCAction&) const;
	
	void Draw(Graphics::Display &display, const RCState&) const;
	
//	void OpenGLDrawCube(int cube) const;
	void SetFaceColor(int face) const;
	mutable std::vector<RCAction> history;
	
	bool pruneSuccessors;
	
	// HASH FUNCTION
	// CORNERS
	uint64_t GetPDBSizeCorner() const;
	uint64_t GetPDBHashCorner(const RCState &s, int threadID) const;
	void GetStateFromPDBHashCorner(uint64_t hash, RCState &s, int threadID) const;
	uint64_t GetStateHashCorner(const RCState &s);

	// EDGES
//	uint64_t GetPDBSizeEdge() const;
//	uint64_t GetPDBHashEdge(const RCState &s, int threadID) const;
//	void GetStateFromPDBHashEdge(uint64_t hash, RCState &s, int threadID) const;
//	uint64_t GetStateHashEdge(const RCState &s);
	
	uint64_t FactorialUpperK(int n, int k) const;
};

class RCPDB : public PDBHeuristic<RCState, RCAction, RC, RCState, 4> {
public:
	RCPDB(RC *e,
		  const std::array<bool, 12> &edgeRotations, const std::array<bool, 12> &edgeLocations,
		  const std::array<bool, 8> &cornerRotations, const std::array<bool, 8> &cornerLocations);
	uint64_t GetStateHash(const RCState &s) const;
	void GetStateFromHash(RCState &s, uint64_t hash) const;
	uint64_t GetPDBSize() const;
	uint64_t GetPDBHash(const RCState &s, int threadID = 0) const;
	uint64_t GetAbstractHash(const RCState &s, int threadID = 0) const { return GetPDBHash(s); }
	void GetStateFromPDBHash(uint64_t hash, RCState &s, int threadID = 0) const;
	RCState GetStateFromAbstractState(RCState &s) const { return s; }

//	void OpenGLDraw() const
//	{}
	
	//	const char *GetName();
	bool Load(const char *prefix) { return false; }
	void Save(const char *prefix) {}
	bool Load(FILE *f){ return false; }
	void Save(FILE *f){}
	std::string GetFileName(const char *prefix) {return "";}
private:
	uint64_t FactorialUpperK(int n, int k) const;

	uint64_t GetEdgeRotationSize() const;
	uint64_t GetEdgeLocationSize() const;
	uint64_t GetCornerRotationSize() const;
	uint64_t GetCornerLocationSize() const;
	uint64_t GetEdgeRotationHash(const RCState &s) const;
	uint64_t GetEdgeLocationHash(const RCState &s) const;
	uint64_t GetCornerRotationHash(const RCState &s) const;
	uint64_t GetCornerLocationHash(const RCState &s) const;
	void GetStateFromEdgeRotationHash(RCState &s, uint64_t hash) const;
	void GetStateFromEdgeLocationHash(RCState &s, uint64_t hash) const;
	void GetStateFromCornerRotationHash(RCState &s, uint64_t hash) const;
	void GetStateFromCornerLocationHash(RCState &s, uint64_t hash) const;

	std::vector<int> edgeRotations;
	std::vector<int> edgeLocations;
	std::vector<int> cornerRotations;
	std::vector<int> cornerLocations;
};

#endif /* defined(__hog2_glut__RubiksCube__) */
