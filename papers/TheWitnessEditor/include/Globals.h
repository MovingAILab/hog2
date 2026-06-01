//
//  Globals.h
//  The Witness Editor
//
//  Created by Samarium on 2023-06-29.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_GLOBALS_H
#define THE_WITNESS_EDITOR_INCLUDE_GLOBALS_H

#include "Witness.h"
#include "WitnessInferenceRule.h"
#include "WitnessPuzzleEntropy.h"
#include "PuzzleEntropy.h"

constexpr int puzzleWidth = 3;
constexpr int puzzleHeight = 3;
const int minSolutions = 5000;
constexpr double inf = std::numeric_limits<double>::max();
extern unsigned long currBoard;
extern Witness<puzzleWidth, puzzleHeight> witness;
extern WitnessPuzzleEntropy<puzzleWidth, puzzleHeight> entropy;
extern Witness<puzzleWidth, puzzleHeight> editor;
extern InteractiveWitnessState<puzzleWidth, puzzleHeight> iws;
extern std::vector<Witness<puzzleWidth, puzzleHeight>> best;
extern std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
extern std::vector<size_t> currentSolutionIndices;
extern size_t gNumSolutions;
extern bool solved;
extern EntropyInfo gEntropy;
extern AdversaryEntropyInfo gAdvEntropy;
extern unsigned gLookahead;
extern bool gWithReplacement;
extern unsigned gSuggestedLocation;
extern unsigned gSolutionIndex;

struct RegionConstraintItem {
    WitnessRegionConstraint constraint;
    Graphics::point c;
    float radius{};
};

extern std::vector<RegionConstraintItem> gRegionConstraintItems;

struct PathConstraintItem {
    WitnessPathConstraintType constraint{};
    Graphics::point c;
    float radius{};
};

extern std::vector<PathConstraintItem> gPathConstraintItems;

struct TetrisItem {
    int parameter{};
    Graphics::point c;
    float radius{};
};

extern std::vector<TetrisItem> gTetrisPieces;

struct ColorItem {
    rgbColor color;
    Graphics::point c;
    float radius{};
};

extern std::vector<ColorItem> gProvidedColors;

extern int gSelectedEditorItem;
extern unsigned gSelectedTetrisItem;
extern bool recording;
extern bool parallel;
extern bool drawEditor;
extern int selectTetrisPiece;
extern Graphics::point cursor;
extern int cursorViewport;
extern bool gUseRelativeEntropy;
extern std::unordered_map<int,
        std::function<ActionType(const SearchEnvironment<WitnessState<puzzleWidth, puzzleHeight>, WitnessAction> &,
                                 WitnessState<puzzleWidth, puzzleHeight> &, const WitnessAction &)>> gInferenceRules;

inline auto GetCurrentEntropy(const Witness<puzzleWidth, puzzleHeight> &env)
{
    return entropy.SetRelative(gUseRelativeEntropy).Calculate(env, iws.ws, gLookahead, std::nullopt);
}

inline auto GetCurrentAdvEntropy(const Witness<puzzleWidth, puzzleHeight> &env)
{
    return iws.ws.path.empty() ?
        entropy.CalculateAdversaryEntropy(env, iws.ws, gLookahead) :
        entropy.CalculatePartialAdversaryEntropy(env, iws.ws, gLookahead);
}

inline void UpdateEntropy(const Witness<puzzleWidth, puzzleHeight> &env)
{
    gEntropy = GetCurrentEntropy(env);
    gEntropy.value = entropy.CalculateTotalSolutionInformation(env, iws.ws);
    gAdvEntropy = GetCurrentAdvEntropy(env);
}

#endif /* THE_WITNESS_EDITOR_INCLUDE_GLOBALS_H */
