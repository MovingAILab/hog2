/*
 *  $Id: Driver.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Junwen Shen on 06/29/23.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */
#include "Driver.h"

#include "Globals.h"
#include "Puzzles.h"
#include "SolutionUtil.h"

bool recording = false;
bool parallel = false;
// 2x2 + 5 (interesting)
// 2x5 + 5 [201/200][149][137/139][82][64][50]
// 3x4 + 5 [481/471][463!][399!][374! - 373][260][150][146][126]
unsigned long currBoard = 0;

Witness<puzzleWidth, puzzleHeight> witness;
WitnessPuzzleEntropy<puzzleWidth, puzzleHeight> entropy;
InteractiveWitnessState<puzzleWidth, puzzleHeight> iws;
std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
std::vector<SolutionTreeNode> solutionTree;
auto gInferenceRules = witnessInferenceRules<puzzleWidth, puzzleHeight>;

std::vector<TetrisItem> gTetrisPieces = {};

static void InitTetrisPieces()
{
    for (auto i = 1; i <= 24; ++i)
    {
        auto f = static_cast<float>(i);
        if (i <= 9)
        {
            Graphics::point p = {-1.0f + f * 0.2f, -0.75f};
            gTetrisPieces.push_back({i, p, 0.1f});
        }
        else if (i <= 18)
        {
            Graphics::point p = {-1.0f + (f - 9) * 0.2f, -0.55f};
            gTetrisPieces.push_back({i, p, 0.1f});
        }
        else
        {
            Graphics::point p = {-1.0f + (f - 18) * 0.2f, -0.35f};
            gTetrisPieces.push_back({i, p, 0.1f});
        }
    }
}

static void temp()
{
//    witness.AddSeparationConstraint(0, 0, Colors::blue);
//    witness.AddSeparationConstraint(2, 0, Colors::blue);
//    witness.AddSeparationConstraint(1, 1, Colors::orange);
//    witness.AddSeparationConstraint(3, 1, Colors::orange);
//    witness.AddSeparationConstraint(0, 2, Colors::orange);
//    witness.AddSeparationConstraint(3, 2, Colors::blue);
//    witness.AddSeparationConstraint(1, 3, Colors::blue);
//    witness.AddSeparationConstraint(2, 3, Colors::orange);
//    witness.AddSeparationConstraint(1, 2, Colors::lightblue);
//    witness.AddSeparationConstraint(2, 2, Colors::black);
//    witness.AddSeparationConstraint(3, 2, Colors::lightblue);
//    witness.AddMustCrossConstraint(true, 1, 2);
//    witness.AddMustCrossConstraint(false, 1, 2);
//    witness.AddMustCrossConstraint(true, 3, 3);
    witness.AddTriangleConstraint(1, 1, 3);
}

static void InitPuzzle()
{
    entropy.ruleSet.SetRules(gInferenceRules);
    gUseRelativeEntropy = false;
    entropy.SetRelative(gUseRelativeEntropy).SetBase2(true);
    witness.SetStart(2, 0);
//    witness.SetGoal(5, 0);
    GetAllSolutions(witness, allSolutions);
//    k87fxsr();
    temp();
    BuildTree(witness, allSolutions, solutionTree);
//    PrintTree(solutionTree);
    UpdateSolutionIndices();
    std::sort(currentSolutionIndices.begin(), currentSolutionIndices.end(), [&](size_t a, size_t b) {
        return entropy
            .SetRelative(gUseRelativeEntropy)
            .Calculate(witness, allSolutions[a], gLookahead, std::nullopt)
            .value >
               entropy
               .SetRelative(gUseRelativeEntropy)
               .Calculate(witness, allSolutions[b], gLookahead, std::nullopt).value;
    });
    gNumSolutions = currentSolutionIndices.size();
    UpdateEntropy(witness);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
    InstallKeyboardHandler(WitnessKeyboardHandler, "Solve", "Solve current board", kAnyModifier, 'v');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Test", "Test constraints", kAnyModifier, 't');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Record", "Record a movie", kAnyModifier, 'r');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Save", "Save current puzzle as svg", kAnyModifier, 's');
#ifdef __EMSCRIPTEN__
    InstallKeyboardHandler(WitnessKeyboardHandler, "Print", "Print The Windmill Json representations", kAnyModifier, 'w');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Load", "Load The Windmill Json representations", kAnyModifier, 'l');
#endif
    InstallKeyboardHandler(WitnessKeyboardHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Prev Board", "Jump to next found board.", kAnyModifier, '[');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Next Board", "Jump to prev found board", kAnyModifier, ']');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Prev 100 Board", "Jump to next 100 found board.", kAnyModifier, '{');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Next 100 Board", "Jump to prev 100 found board", kAnyModifier, '}');

    InstallKeyboardHandler(WitnessKeyboardHandler, "Editor", "Open editor", kAnyModifier, 'e');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Selection", "Open Tetris pieces panel", kAnyModifier, 'x');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Calculate", "Calculate entropy", kAnyModifier, 'c');

    InstallKeyboardHandler(WitnessKeyboardHandler, "Toggle All Rules", "Enable/Disable Rules", kAnyModifier, '0');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Toggle SPR", "Enable/Disable SeparationRule", kAnyModifier, '1');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Toggle PCR", "Enable/Disable PathConstraintRule", kAnyModifier, '2');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Toggle ISTR", "Enable/Disable InsideSolutionTreeRule", kAnyModifier, '3');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Toggle RCR", "Enable/Disable RegionCompletionRule", kAnyModifier, '4');
    InstallKeyboardHandler(WitnessKeyboardHandler, "Toggle APR", "Enable/Disable AlongThePathRule", kAnyModifier, '5');

    InstallCommandLineHandler(WitnessCLHandler, "-run", "-run", "Runs pre-set experiments.");
    InstallCommandLineHandler(WitnessCLHandler, "-test", "-test", "Basic test with MD heuristic");

    InstallWindowHandler(WitnessWindowHandler);
    InstallMouseClickHandler(WitnessClickHandler, static_cast<tMouseEventType>(kMouseMove | kMouseUp | kMouseDown | kMouseDrag));
}

int main(int argc, char *argv[])
{
#ifndef __EMSCRIPTEN__
    setTextBufferVisibility(false);
#endif
    InitTetrisPieces();
    InitPuzzle();
    InstallHandlers();
    RunHOGGUI(argc, argv, 1280, 640);
    return 0;
}
