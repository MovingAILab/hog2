//
//  SnakeBird.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/26/20.
//  Copyright © 2020 University of Denver. All rights reserved.
//

#ifndef SnakeBird_h
#define SnakeBird_h

#include <stdio.h>
#include "SearchEnvironment.h"
#include <array>

namespace SnakeBird {

// 64 bits can hold
//
//
// world is 20x16 (WxH) = 320
// world is any x/y, as long as x*y < 512
// 1 snake = [5 bits, 4 bits] 9 bits head, 2 bits (4 dir) per segment
// 3 snakes = 27 bits + length

// uint64_t blockFruit
// block: 9 bits each, max 4(?) = 36 bits
// fruit: up to 28 - one bit each
const uint64_t locationMask = 0x1FF;
const uint64_t snakeLenMask = 0x1F;
const uint64_t fruitMask = 0x1F;
const uint64_t snakeHeadMask = 0x7;
const uint64_t snakeBodyMask = 0x3;
const uint64_t kOne = 0x1;
const int kDead = 510;
const int kInGoal = 511;
const uint8_t kNothingPushed = 0xFF;
//const int kMaxPushedObjects = 3;
const int codeSize = 2;

enum snakeDir : uint8_t {
	kLeft=0x0, kRight=0x1, kUp=0x2, kDown=0x3, kNoDirection=0x4
};
struct SnakeBirdState {
	uint64_t snakeBodies; // up to 32 in length
	uint64_t snakeHeads; // up to 4. 3bits (num snakes) + 9 bits head + 5 bits start location of next. (max coordinate) head means snake left level
	uint64_t locBlockFruit; // 4*9bit blocks; 28 fruit

	SnakeBirdState() :snakeBodies(0), snakeHeads(0), locBlockFruit(0) {}
	void Reset() { snakeBodies = snakeHeads = locBlockFruit = 0; }
	bool operator==(const SnakeBirdState &s) const {
		return (s.snakeBodies==snakeBodies)&&(s.snakeHeads==snakeHeads)&&(s.locBlockFruit==locBlockFruit);
	}
	int GetNumSnakes() const { return snakeHeads&snakeHeadMask; }
	void SetNumSnakes(int count) { snakeHeads = (snakeHeads&(~snakeHeadMask))|(count&snakeHeadMask); }
	bool IsDead(int whichSnake) const
	{ return GetSnakeHeadLoc(whichSnake) == kDead; }
	bool IsInPlay(int whichSnake) const
	{ return GetSnakeHeadLoc(whichSnake) != kDead && GetSnakeHeadLoc(whichSnake) != kInGoal; }
	int GetSnakeHeadLoc(int whichSnake) const
	{ return (snakeHeads>>(3+whichSnake*14))&0x1FF;}
	void SetSnakeHeadLoc(int whichSnake, int loc)
	{ snakeHeads &= ~(locationMask<<(3+whichSnake*14));
		snakeHeads |= ((loc&locationMask)<<(3+whichSnake*14)); }
	int GetSnakeBodyEnd(int whichSnake) const
	{ return (whichSnake<0)?0:(snakeHeads>>(3+9*(whichSnake+1)+5*whichSnake))&snakeLenMask; }
	void SetSnakeBodyEnd(int whichSnake, int endOffset)
	{ snakeHeads &= ~(snakeLenMask<<(3+9*(whichSnake+1)+5*whichSnake)); snakeHeads |= ((endOffset&snakeLenMask)<<(3+9*(whichSnake+1)+5*whichSnake)); }
	
	void SetSnakeLength(int whichSnake, int len)
	{ //SetSnakeBodyEnd(whichSnake, GetSnakeBodyEnd(whichSnake-1)+len-1);
		uint64_t oldBody[4]; // max 4 snakes
		int oldLen[4];
		for (int x = 0; x < GetNumSnakes(); x++)
		{
			oldBody[x] = GetBodyBits(x);
			oldLen[x] = GetSnakeLength(x);
		}
		oldLen[whichSnake] = len;
		oldBody[whichSnake] &= ((1<<len)-1);
		//SetSnakeBodyEnd(whichSnake, GetSnakeBodyEnd(whichSnake-1)+len-1);
		for (int x = whichSnake; x < GetNumSnakes(); x++)
			SetSnakeBodyEnd(x, GetSnakeBodyEnd(x-1)+oldLen[x]-1);
		for (int x = whichSnake; x < GetNumSnakes(); x++)
			SetBodyBits(x, oldBody[x]);
	}
	int GetSnakeLength(int whichSnake) const
	{ return GetSnakeBodyEnd(whichSnake)-GetSnakeBodyEnd(whichSnake-1)+1; }

	snakeDir GetSnakeDir(int whichSnake, int segment) const
	{ return static_cast<snakeDir>((snakeBodies>>(2*segment+2*GetSnakeBodyEnd(whichSnake-1)))&snakeBodyMask); }
	void SetSnakeDir(int whichSnake, int segment, snakeDir dir)
	{
		snakeBodies &= ~(snakeBodyMask<<(2*segment+2*GetSnakeBodyEnd(whichSnake-1)));
		snakeBodies |= ((static_cast<uint64_t>(dir))<<(2*segment+2*GetSnakeBodyEnd(whichSnake-1)));
	}
	// add dir to beginning of snake
	void InsertSnakeDir(int whichSnake, snakeDir dir)
	{
		uint64_t mask = 1;
		mask <<= (GetSnakeBodyEnd(whichSnake)-GetSnakeBodyEnd(whichSnake-1))*2; // length in bits
		mask -= 1;
		mask <<= GetSnakeBodyEnd(whichSnake-1)*2;
		uint64_t oldSnake = snakeBodies&(mask);
		oldSnake<<=2;
		oldSnake |= dir<<(GetSnakeBodyEnd(whichSnake-1)*2);
		snakeBodies &= (~mask);
		snakeBodies |= (oldSnake&(mask));
	}
	// grow snake by adding dir
	void InsertSnakeHeadDir(int whichSnake, snakeDir dir)
	{
		int start = GetSnakeBodyEnd(whichSnake-1)*2;
		uint64_t mask = 1; // mask is the low bits to keep
		mask<<=start;
		mask -= 1;

		uint64_t a = (snakeBodies&mask);
		uint64_t b = (dir<<start);
		uint64_t c = ((snakeBodies&(~mask))<<(2));
		snakeBodies = a|b|c;
	}
	uint64_t GetBodyBits(int whichSnake) const
	{
		uint64_t mask = 1;
		mask <<= (GetSnakeBodyEnd(whichSnake)-GetSnakeBodyEnd(whichSnake-1))*2; // length in bits
		mask -= 1;
		uint64_t result = snakeBodies>>(GetSnakeBodyEnd(whichSnake-1)*2);
		return result&mask;
	}
	void SetBodyBits(int whichSnake, uint64_t bodyBits)
	{
		uint64_t mask = 1;
		mask <<= (GetSnakeBodyEnd(whichSnake)-GetSnakeBodyEnd(whichSnake-1))*2; // length in bits
		mask -= 1;
		mask <<= GetSnakeBodyEnd(whichSnake-1)*2;
		bodyBits <<= GetSnakeBodyEnd(whichSnake-1)*2;
		snakeBodies &= (~mask);
		snakeBodies |= bodyBits;
	}
	void MakeSnakeLonger(int whichSnake)
	{
		uint64_t old[4]; // max 4 snakes
		for (int x = 0; x < GetNumSnakes(); x++)
			old[x] = GetBodyBits(x);
		for (int x = whichSnake; x < GetNumSnakes(); x++)
			SetSnakeBodyEnd(x, GetSnakeBodyEnd(x)+1);
		for (int x = whichSnake; x < GetNumSnakes(); x++)
			SetBodyBits(x, old[x]);
	}
	void MakeSnakeLonger(int whichSnake, snakeDir addDir)
	{
		uint64_t old[4]; // max 4 snakes
		for (int x = whichSnake; x < GetNumSnakes(); x++)
			old[x] = GetBodyBits(x);
		old[whichSnake] |= ((2*(GetSnakeLength(whichSnake)+1))<<addDir);
		for (int x = whichSnake; x < GetNumSnakes(); x++)
			SetSnakeBodyEnd(x, GetSnakeBodyEnd(x)+1);
		for (int x = whichSnake; x < GetNumSnakes(); x++)
			SetBodyBits(x, old[x]);
	}
	int GetObjectLocation(int whichObstacle) const { return (locBlockFruit>>(9*whichObstacle))&locationMask; }
	void SetObjectLocation(int whichObstacle, int loc) //{ return (locBlockFruit>>(9*whichObstacle))&locationMask; }
	{ locBlockFruit &= ~(locationMask<<(whichObstacle*9)); locBlockFruit |= ((loc&locationMask)<<(whichObstacle*9)); }
	
	// Fruit is 0 if present 1 if not. Thus by default all fruit is present.
	bool GetFruitPresent(int which) const { return ((locBlockFruit>>(36+which))&0x1)==0; }
	void ToggleFruitPresent(int which) { locBlockFruit ^= (kOne<<(36+which));}
	bool KFruitEaten(int k) const
	{
		uint64_t mask = (kOne<<(k))-1;
		return ((locBlockFruit>>36)&mask)==(mask);
		
	}
};

struct SnakeBirdAction {
	SnakeBirdAction() :bird(0xF), direction(0xF), pushed(0) {}
	unsigned int bird : 4; // which bird
	unsigned int direction : 4; // which direction
	uint8_t pushed; // 8 booleans; supporting 4 snakes & 4 blocks
	bool operator==(const SnakeBirdAction &a) const
	{
		// TODO: need to compare if anything is being pushed
		return a.bird == bird && a.direction == direction;
	}
};

static std::ostream &operator<<(std::ostream &out, const SnakeBirdAction &a)
{
	out << a.bird << " ";
	switch (a.direction)
	{
		case kUp: out << "up"; break;
		case kDown: out << "down"; break;
		case kLeft: out << "left"; break;
		case kRight: out << "right"; break;
	}
	return out;
}

enum SnakeBirdAnimation : uint8_t {
	kMovement,
	kFall,
	kInitialTeleport,
	kTeleport,
	kWentInGoal,
	kFellInGoal,
	kDoneAnimation,
	kPauseWhenDead,
	kNeedsInitialization
};

struct SnakeBirdAnimationStep {
	SnakeBirdAnimationStep() { Reset(); }
	void Reset() { anim = kNeedsInitialization; animationDuration = 0; teleportCount = 0;}
	SnakeBirdAnimation anim;
	SnakeBirdAction a;
	double animationDuration;
	int teleportCount;
};

/*
 Level files are text
 * Terrain types in text:
 * . Empty
 * G Ground
 * X Spikes
 * O Portal
 * E Exit
 * F Fruit
 * 1 [1-4] Parts of object
 * < Snake heads. Body is on the open side, eg <===
 * >
 * ^
 * V
 * a-z [snake body in order, all letters must be unique in level]
 */

const uint8_t kCanEnterMask = 0x80;
const uint8_t kGroundMask = 0x40;
const uint8_t kSnakeMask = 0x10;
const uint8_t kBlockMask = 0x20;

enum SnakeBirdWorldObject : uint8_t {
	// can always enter, but may have secondary effects
	kEmpty  = 0x80, // = 128
	kFruit  = 0x81,
	kExit   = 0x82,
	kPortal1= 0x83,
	kPortal2= 0x84,
	kPortal, //figure out where the portals go

	// cannot ever enter
	kGround = 0x40, // = 64
	kSpikes = 0x41, // = 65

	// for pushing
	kBlock1 = 0x20,
	kBlock2 = 0x21,
	kBlock3 = 0x22,
	kBlock4 = 0x23,

	// other snakes
	kSnake1 = 0x10,
	kSnake2 = 0x11,
	kSnake3 = 0x12,
	kSnake4 = 0x13,

	// Draw nothing
	kNothing= 0x0,
};

enum TeleportResult {
	kNoTeleport,
	kTeleportSuccess,
	kTeleportToExit
};

class SnakeBird : public SearchEnvironment<SnakeBirdState, SnakeBirdAction> {
public:
	SnakeBird(int width = 20, int height = 16);
	void Reset();
	void BiggerMapHeight();
	void BiggerMapWidth();
	void SmallerMapHeight();
	void SmallerMapWidth();
	bool Load(const char *filename);
	bool Save(const char *filename);
	std::string EncodeLevel() const;
	bool DecodeLevel(const std::string &);
	void BeginEditing();
	void EndEditing();

	SnakeBirdState GetStart() const;
	void SetStart(const SnakeBirdState &);
	void AddSnake(int x, int y, const std::vector<snakeDir> &body);
	void AddSnakeHead(int x, int y, int whichSnake);
	snakeDir GetAddingDirection(int x, int y, int endX, int endY);
	void AddSnakeBody(int x, int y, int whichSnake);
	
	void RemoveSnake(int x, int y, int o, int whichSnake);
	void SetGroundType(int x, int y, SnakeBirdWorldObject o);
	void RemoveBlock(int x, int y);
	int GetNumPortals();
	SnakeBirdWorldObject GetGroundType(int x, int y) const;
	SnakeBirdWorldObject GetRenderedGroundType(const SnakeBirdState &s, int x, int y);
	int GetWidth() const { return width; }
	int GetHeight() const { return height; }

	void GetSuccessors(const SnakeBirdState &nodeID, std::vector<SnakeBirdState> &neighbors) const;
	void GetActions(const SnakeBirdState &nodeID, std::vector<SnakeBirdAction> &actions) const;
	bool LivingState(const SnakeBirdState &s) const { return Render(s); }
	size_t GetNumFruit() { return fruit.size(); }
	//SnakeBirdAction GetAction(const SnakeBirdState &s1, const SnakeBirdState &s2) const;
	void ApplyAction(SnakeBirdState &s, SnakeBirdAction a) const;
	/* Applys the next portion of the action - not done until returns true. */
	bool ApplyPartialAction(SnakeBirdState &s, SnakeBirdAction a, SnakeBirdAnimationStep &step) const;
	bool Legal(SnakeBirdState &s, SnakeBirdAction a);
	// Cannot undo actions
	void UndoAction(SnakeBirdState &s, SnakeBirdAction a) const
	{ assert(false); }
	
	void GetNextState(const SnakeBirdState &s1, SnakeBirdAction a, SnakeBirdState &s2) const
	{
		s2 = s1;
		ApplyAction(s2, a);
	};
	
	bool InvertAction(SnakeBirdAction &a) const { return false; }
	
	/** Stores the goal for use by single-state HCost. **/
	void StoreGoal(SnakeBirdState &s)
	{ bValidSearchGoal = true; searchGoal = s; }
	
	/** Clears the goal from memory. **/
	void ClearGoal()
	{ bValidSearchGoal = false; }
	
	/** Returns true if the goal is stored and false otherwise. **/
	bool IsGoalStored() const
	{ return bValidSearchGoal; }
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const SnakeBirdState &node1, const SnakeBirdState &node2) const { return 0; }
	
	double GCost(const SnakeBirdState &node1, const SnakeBirdState &node2) const { return 1;}
	double GCost(const SnakeBirdState &node, const SnakeBirdAction &act) const { return 1;}
	bool GoalTest(const SnakeBirdState &node, const SnakeBirdState &goal) const { return GoalTest(node); }
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const SnakeBirdState &node) const
	{ for (int x = 0; x < node.GetNumSnakes(); x++) if (node.GetSnakeHeadLoc(x) != kInGoal) return false; return true; }
		
	uint64_t GetActionHash(SnakeBirdAction act) const {return (act.bird<<2)|act.direction;}
	uint64_t GetStateHash(const SnakeBirdState &node) const
	{
		//return node.snakeHeads^(node.snakeBodies>>11)^(node.snakeBodies<<17)^node.locBlockFruit;
		return node.snakeHeads^(node.snakeBodies)^node.locBlockFruit;
	}

	void OpenGLDraw() const {}
	void OpenGLDraw(const SnakeBirdState&) const {}
	/** Draw the transition at some percentage 0...1 between two states */
	void OpenGLDraw(const SnakeBirdState&, const SnakeBirdState&, float) const {}
	void OpenGLDraw(const SnakeBirdState&, const SnakeBirdAction&) const {};
	void GLLabelState(const SnakeBirdState&, const char *) const {} // draw label over state
	void GLDrawLine(const SnakeBirdState &x, const SnakeBirdState &y) const {}
	void GLDrawPath(const std::vector<SnakeBirdState> &x) const {}
	

	void Draw(Graphics::Display &display) const;
	void DrawObjects(Graphics::Display &display, double time = 0) const;
	void DrawObject(Graphics::Display &display, int x, int y, SnakeBirdWorldObject o, double time = 0) const;
	void Draw(Graphics::Display &display, int x, int y, float width = 1.0) const;
	void Draw(Graphics::Display &display, double time) const;
	void Draw(Graphics::Display &display, const SnakeBirdState&) const;
	void Draw(Graphics::Display &display, const SnakeBirdState&, int active) const;
	void Draw(Graphics::Display &display, const SnakeBirdState&, int active, double globalTime) const;
	void Draw(Graphics::Display &display, const SnakeBirdState&, const SnakeBirdState&,
			  int active, double percentComplete, double globalTime) const;
	void DrawLine(Graphics::Display &display, const SnakeBirdState &x, const SnakeBirdState &y, float width = 1.0) const;
	void DrawLabel(Graphics::Display &display, int x, int y, const char *str);
	void DrawSmallLabel(Graphics::Display &display, int x, int y, const char *str);

	// Allows us to draw text overlay
	float GetRadius() const;
	bool GetPointFromCoordinate(Graphics::point p, int &x, int &y);
private:
	std::string Code(int) const;
	int DeCode(const std::string &s, size_t offset) const;
	void SetGroundType(int index, SnakeBirdWorldObject o);
	bool Render(const SnakeBirdState &s) const;
	bool CanPush(const SnakeBirdState &s, int snake, SnakeBirdWorldObject obj, snakeDir dir,
				 SnakeBirdAction &a) const;
	bool IsOnSpikes(const SnakeBirdState &s, int which) const;
	
	// Apply Move helper functions
	// check if snakebirds can teleport - return true if one did
	TeleportResult HandleTeleports(SnakeBirdState &s, SnakeBirdAction &a,
								   snakeDir lastAction, snakeDir opposite, SnakeBirdAnimationStep step) const;
	SnakeBirdAnimation DoFirstMovement(const SnakeBirdAction &a, int offset, snakeDir opposite, SnakeBirdState &s) const;
	// returns true if a snake fell
	SnakeBirdAnimation DoFall(SnakeBirdAction &a, SnakeBirdState &s) const;

	
	int GetFruitOffset(int index) const;
public:
	int GetIndex(int x, int y) const;
	int GetX(int index) const;
	int GetY(int index) const;
private:
	int Distance(int index1, int index2);
	Graphics::point GetCenter(int x, int y) const;

	void DrawSnakeEnteringGoal(Graphics::Display &display, const SnakeBirdState &s,
							   int snake, bool isActive, double percentComplete) const;
	void DrawTranslatingSnake(Graphics::Display &display, const SnakeBirdState &old, const SnakeBirdState &s,
							  int snake, bool isActive, double percentComplete) const;
	void DrawMovingSnake(Graphics::Display &display, const SnakeBirdState &old, const SnakeBirdState &s,
						 int snake, bool isActive, double percentComplete) const;
	void DrawSnakeSegment(Graphics::Display &display, Graphics::point p, const rgbColor &color, bool head, bool tail, bool awake, snakeDir dirFrom, snakeDir dirTo, int whichSnake, bool isDead) const;

	// Member variables
	int width, height;

	std::array<SnakeBirdWorldObject, 512> world; // static world
	mutable std::array<SnakeBirdWorldObject, 512> render;
	
	std::vector<int> fruit;

	std::array<std::vector<int>, 4> objects; // offsets from base location
	std::array<bool, 4> objectFullyConnected;
	int portal1Loc, portal2Loc;
	int exitLoc;
	SnakeBirdState startState;
	bool editing;
//	int lastSnake;
	//	std::array<
};

}

namespace std {
template <>
struct hash<SnakeBird::SnakeBirdState>
{
	std::size_t operator()(const SnakeBird::SnakeBirdState &k) const
	{
		return k.snakeHeads^(k.snakeBodies)^k.locBlockFruit;
	}
};
	
}


#endif /* SnakeBird_h */
