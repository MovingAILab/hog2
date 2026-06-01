//
//  Hexagon.h
//  Hexagon
//
//  Created by Nathan Sturtevant on 11/13/21.
//  Copyright © 2021 MovingAI. All rights reserved.
//

#ifndef Hexagon_h
#define Hexagon_h

#include <cstdio>
#include <vector>
#include <array>
#include <map>
#include "SearchEnvironment.h"
#include "FPUtil.h"
#include "NBitArray.h"
#include <algorithm>
#include <thread>
#include <functional>
#include <vector>
#include <mutex>

using namespace std;

enum tPieceName {
	kHexagon = 0,
	kButterfly = 1,
	kElbow = 2,
	kLine = 3,
	kMountains = 4,
	kWrench = 5,
	kTriangle = 6,
	kHook = 7,
	kTrapezoid = 8,
	kSnake = 9
};

enum tFlipType {
	kCanFlip = 0,
	kSide1 = 1,
	kSide2 = 2,
    kHoles = 3
};

const int numPieces = 10;
const string pieceNames[numPieces] =
{
	"Hexagon",
	"Butterfly",
	"Elbow",
	"Line",
	"Mountains",
	"Wrench",
	"Triangle",
	"Hook",
	"Trapezoid",
	"Snake"
};

struct HexagonAction
{
	unsigned int piece : 8; // Support up to 256 pieces for analysis
	unsigned int location : 8; // Each piece can be in up to 256(?) different locations
	// Up to 324 (54x6) locations
//	int location : 6; // 54 possible locations
//	int rotation : 3; // 3 rotations x 2 flips = 6
};

class HexagonState
{
public:
	HexagonState()
	:state(60) {}
	NBitArray<4> state;
    vector<int> constraints;
    float entropy;
    uint64_t dots;
    int forbiddenPiece;
};

static bool operator==(const HexagonState &l1, const HexagonState &l2)
{
//    bool equal = true;
    
    for(int i =0; i<l1.state.Size(); i++){
        if(l1.state.Get(i) != l2.state.Get(i)
//
         && l1.state.Get(i) != 15 && l2.state.Get(i) != 15 && !((l1.state.Get(i) == 8 && l2.state.Get(i) == 9) || (l1.state.Get(i) == 9 && l2.state.Get(i) == 8))){
//            printf("%d %d %d\n", i, l1.state.Get(i), l2.state.Get(i));
            return false;
        }
    }
    
    return true;//l1.state == l2.state;
}

static ostream &operator<<(ostream &out, const HexagonAction &a)
{
	return out;
}


class HexagonEnvironment;

// (Old) code for loading and displaying files
class Hexagon : public SearchEnvironment<HexagonState, HexagonAction>
{
public:
	Hexagon();
	~Hexagon();
	void LoadPuzzle(const char *, HexagonState &s);
	void LoadSolution(const char *, HexagonState &s);
	void GetSuccessors(const HexagonState &nodeID, vector<HexagonState> &neighbors) const;
	void GetActions(const HexagonState &nodeID, vector<HexagonAction> &actions) const;

    
	HexagonAction GetAction(const HexagonState &s1, const HexagonState &s2) const;
	void ApplyAction(HexagonState &s, HexagonAction a) const;
	
	void GetNextState(const HexagonState &, HexagonAction , HexagonState &) const;
	bool InvertAction(HexagonAction &a) const;
	
	void RotateCW(HexagonState &s) const;
	
	/** Goal Test if the goal is stored **/
	bool GoalTest(const HexagonState &node) const;
	
	double HCost(const HexagonState &node1, const HexagonState &node2) const { return 0; }
	double GCost(const HexagonState &node1, const HexagonState &node2) const { return 1; }
	double GCost(const HexagonState &node, const HexagonAction &act) const { return 1; }
	bool GoalTest(const HexagonState &node, const HexagonState &goal) const
	{ return GoalTest(node); }

	uint64_t GetStateHash(const HexagonState &node) const;
	uint64_t GetActionHash(HexagonAction act) const;
		
	void Draw(Graphics::Display &display) const;
	/** Draws available pieces and constraints */
	void DrawSetup(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const HexagonState&) const;
private:
	friend HexagonEnvironment;
	void Load(const char *, HexagonState &s, bool solution);
	void GetCorners(int x, int y, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const;
	bool GetBorder(int x, int y, int xoff, int yoff, Graphics::point &p1, Graphics::point &p2) const;
	bool Valid(int x, int y) const;
	HexagonState solution;
	vector<rgbColor> pieceColors;
    vector<int> diagPieces;
    vector<int> notDiagPieces;
	vector<int> noFlipPieces;
	vector<int> notTouchPieces;
	vector<int> touchPieces;
};
//bits(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53)
class HexagonSearchState
{
public:
	HexagonSearchState()
	:bits(0), cnt(0), dots(0)
	{
	}
	void Reset()
    {
        bits = 0;
        cnt = 0;
//        dots = 0;
    }
    uint64_t bits, dots;
    array<bool, 121> edgeAdjacencies, cornerAdjacencies;
    vector<int> constraints, allConstraints, addedInitPieces;
    vector<uint64_t> emptySpaces;
	int cnt, index, initState = -1, forbiddenPiece;
    float entropy, entropyWithAddedPieces;
	array<HexagonAction, 12> state;
//	NBitArray<4> state;
};

static bool operator==(const HexagonAction &l1, const HexagonAction &l2)
{
	return l1.piece == l2.piece && l1.location == l2.location;
}

static bool operator==(const HexagonSearchState &l1, const HexagonSearchState &l2)
{
	if (l1.bits != l2.bits || l1.cnt != l2.cnt)
    {
//        cout << "Not Equal\n";
        return false;
    }
	for (int x = 0; x < l1.cnt; x++)
	{
		bool found = false;
		for (int y = 0; y < l1.cnt; y++) // needed if we have multiples of the same object
		{
			if (l1.state[x] == l2.state[y])
			{
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}
//    cout << "Equal! \n";
	return true;
}


// efficient search implementation
class HexagonEnvironment : public SearchEnvironment<HexagonSearchState, HexagonAction>
{
public:
	HexagonEnvironment();
	~HexagonEnvironment();
	void SetPieces(const vector<tPieceName> &pieces);
    vector<int> GetPieces();
	void SetFlippable(const array<tFlipType, numPieces> &flips);
	
	void GetSuccessors(const HexagonSearchState &nodeID, vector<HexagonSearchState> &neighbors) const;
	void GetActions(const HexagonSearchState &nodeID, vector<HexagonAction> &actions) const;
    void GetActions2(const HexagonSearchState &nodeID, vector<HexagonAction> &actions) const;
    void FullAnalysis(vector<HexagonSearchState> goals, vector<vector<HexagonSearchState>> &selectedPuzzles, int categories);
    void ConstraintSpaceSearch(vector<HexagonSearchState> goals);
//    void ConstraintSpaceSearchParallel(vector<HexagonSearchState> goals, vector<int> pieces, map<uint64_t, int> interestingPatterns, int THRESHOLD, uint64_t numPatterns, int threadNum, int totalThreads);
    void ConstraintSpaceSearchParallel(vector<HexagonSearchState> goals, vector<double> &interestingPatterns, int THRESHOLD, uint64_t numPatterns, int threadNum, int totalThreads);
    void FilterGoalsUsingPattern(vector<HexagonSearchState> goals, vector<double> &interestingPatterns, int THRESHOLD, uint64_t numPatterns, int threadNum, int totalThreads);
    
    void ColorConstraintSpaceSearchParallel(vector<HexagonSearchState> goals, vector<double> &interestingPatterns, int THRESHOLD, uint64_t numPatterns, int numColors, int threadNum, int totalThreads);

    void AddPiecesToInitState(HexagonSearchState &hss, int pieces, bool chooseEasiest);

    void GenerateColorRules(vector<HexagonSearchState> &goals, vector<double> &interestingPatterns, int THRESHOLD, uint64_t numPatterns, int numColors, int threadNum, int totalThreads, vector<vector<HexagonSearchState>> &selectedSolutions);
    void GenerateInitialStates(int numColors, vector<vector<HexagonSearchState>> &selectedSolutions, int categories) const;
    // @param[in] nb_elements : size of your for loop
    /// @param[in] functor(start, end) :
    /// your function processing a sub chunk of the for loop.
    /// "start" is the first index to process (included) until the index "end"
    /// (excluded)
    /// @code
    ///     for(int i = start; i < end; ++i)
    ///         computation(i);
    /// @endcode
    /// @param use_threads : enable / disable threads.
    ///
    ///
    static
    void parallel_for(unsigned nb_elements,
                      function<void (int start, int end)> functor,
                      bool use_threads = true)
    {
        // -------
        unsigned nb_threads_hint = thread::hardware_concurrency();
        unsigned nb_threads = nb_threads_hint == 0 ? 8 : (nb_threads_hint);

        unsigned batch_size = nb_elements / nb_threads;
        unsigned batch_remainder = nb_elements % nb_threads;

        vector< thread > my_threads(nb_threads);

        if( use_threads )
        {
            // Multithread execution
            for(unsigned i = 0; i < nb_threads; ++i)
            {
                int start = i * batch_size;
                my_threads[i] = thread(functor, start, start+batch_size);
            }
        }
        else
        {
            // Single thread execution (for easy debugging)
            for(unsigned i = 0; i < nb_threads; ++i){
                int start = i * batch_size;
                functor( start, start+batch_size );
            }
        }

        // Deform the elements left
        int start = nb_threads * batch_size;
        functor( start, start+batch_remainder);

        // Wait for the other thread to finish their task
        if( use_threads )
            for_each(my_threads.begin(), my_threads.end(), mem_fn(&thread::join));
    }
    
	HexagonAction GetAction(const HexagonSearchState &s1, const HexagonSearchState &s2) const;
	void ApplyAction(HexagonSearchState &s, HexagonAction a) const;
	void UndoAction(HexagonSearchState &s, HexagonAction a) const;
    
    void BuildAdjacencies(HexagonSearchState &goal);

	void GetNextState(const HexagonSearchState &, HexagonAction , HexagonSearchState &) const;
	bool InvertAction(HexagonAction &a) const;
	
	void RotateCW(HexagonSearchState &s) const;
	HexagonAction RotateCW(HexagonAction a) const;
	void Flip(HexagonSearchState &s) const;
	HexagonAction Flip(HexagonAction a) const;
    
    void ConvertToHexagonState(HexagonSearchState &hss, HexagonState &hs, bool fill = false);
    HexagonSearchState GetInitState(HexagonSearchState &hss, vector<int> *addedInitPieces = nullptr);
    float GetEntropy(HexagonSearchState &s, short mode = 0b111) const;
    void GetInferenceActions(const HexagonSearchState &s, vector<HexagonAction> &actions, short mode) const;

    void GetEmptySpaces(HexagonSearchState &s) const;
    
    bool GoalValidHoles(HexagonSearchState goal, uint64_t pattern) const;
    
    bool SizeOfEmtpyRegionRule(const HexagonSearchState &s) const;
    bool PiecesAreComposedOfTrapezoidsRule(const HexagonSearchState &s) const;
    bool PieceThatFitsTheSpaceIsNotAvailableRule(const HexagonSearchState &s) const;
    uint64_t LocationCanOnlyFitACertainPieceReqRule(const HexagonSearchState &s) const;
    int PieceCanOnlyGoInOnePlaceReqRule(const HexagonSearchState &s) const;
    
    uint64_t BitsFromArray(vector<int> a) const;
    string FancyPrintBoard(uint64_t bits, uint64_t holes = 0) const;
    void BuildLocationTable();//;int[] &bits, vector<int[]> &table)
    void BuildHolesTable();//;int[] &bits, vector<int[]> &table)
    void ConvertToSearchState();
    string PrintHexagonState(HexagonState &hs);

//    void ConvertToHexagonState(HexagonSearchState *s)
	/** Goal Test if the goal is stored **/
	bool GoalTest(const HexagonSearchState &node) const;
	
	double HCost(const HexagonSearchState &node1, const HexagonSearchState &node2) const { return 0; }
	double GCost(const HexagonSearchState &node1, const HexagonSearchState &node2) const { return 1; }
	double GCost(const HexagonSearchState &node, const HexagonAction &act) const { return 1; }
	bool GoalTest(const HexagonSearchState &node, const HexagonSearchState &goal) const
	{ return GoalTest(node); }

	uint64_t GetStateHash(const HexagonSearchState &node) const;
	uint64_t GetActionHash(HexagonAction act) const;
	
	/** Prints out the triangles used for this piece in HOG2 coordinates */
	void GeneratePieceCoordinates(tPieceName p);
	/** Prints out the outer coorsinates of the board*/
	void GenerateBoardBorder();
	
	void Draw(Graphics::Display &display) const;
	/** Draws available pieces and constraints */
	void DrawSetup(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const HexagonSearchState&) const;
private:
	void BuildRotationTable();
	void BuildFlipTable();
	void IndexToXY(int index, int &x, int &y) const;
	void GetCorners(int x, int y, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const;
	bool GetBorder(int x, int y, int xoff, int yoff, Graphics::point &p1, Graphics::point &p2) const;
	bool Valid(int x, int y) const;
	vector<rgbColor> pieceColors;
	vector<int> pieces;
	array<tFlipType, numPieces> flippable;
	Hexagon hex;
    int rotate30Map[numPieces][192];//14*6*2+1];
	int flipMap[numPieces][192];//[14*6*2+1];
    mutex patternLock;
};

#endif /* Hexagon_h */
