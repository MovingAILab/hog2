#include "Driver.h"
#include "ExamineUtil.h"
#include "FileUtil.h"
#include "Globals.h"
#include "SolutionUtil.h"
#include "SVGUtil.h"

unsigned gSolutionIndex = 0;

static auto editorAvailable = std::array{ 'e', 'w', 'x', 'v', 'r' };

void WitnessKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
    if (drawEditor && !std::isdigit(key) &&
        std::find(editorAvailable.cbegin(), editorAvailable.cend(), key) == editorAvailable.cend())
        return;
    switch (key)
    {
    case 't':
        ParallelExamine(5);
        //			ExamineMustCross(numRequiredPieces);
        //			w.ClearTetrisConstraints();
        //			ExamineTetris(4);
        //			ExamineMustCrossAndRegions(numRequiredPieces, numSeparationPieces);
        //			ExamineMustCrossAnd3Regions(numRequiredPieces, numSeparationPieces);
        //			ExamineTriangles(6);
        // ExamineTriangles(puzzleHeight*puzzleWidth);
        //			ExamineRegionsAndStars(0);
        break;
    case 'v':
    {
        if (!currentSolutionIndices.empty())
        {
            iws.ws = allSolutions[currentSolutionIndices[(gSolutionIndex++) % currentSolutionIndices.size()]];
            iws.currState = InteractiveWitnessState<puzzleWidth, puzzleHeight>::kWaitingRestart;
            solved = true;
        }
        break;
    }
    case 's':
    {
        auto path = std::filesystem::temp_directory_path().string() + std::string("editor.svg");
        Graphics::Display &display = GetContext(windowID)->display;
        MakeSVG(display, path.c_str(), 640, 640, 0);
        std::cout << "Saved to " << path << std::endl;
        break;
    }
    case 'w':
    {
        submitTextToBuffer(std::string(witness).c_str());
        break;
    }
    case 'l':
    {
#ifdef __EMSCRIPTEN__
        std::stringstream ss(getTextBuffer());
        auto w = Witness<puzzleWidth, puzzleHeight>();
        ss >> w;
        witness = w;
        allSolutions.clear();
        GetAllSolutions(witness, allSolutions);
        BuildTree(witness, allSolutions, solutionTree);
        UpdateSolutionIndices();
#endif
        break;
    }
    case 'r':
    {
        iws.Reset();
        solved = false;
        break;
    }
    case '\t':
        break;
    case '[':
        if (!best.empty())
        {
            currBoard = (currBoard + (int) (best.size()) - 1) % best.size();
            Load(currBoard);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case ']':
        if (!best.empty())
        {
            currBoard = (currBoard + 1) % best.size();
            Load(currBoard); //, numRequiredPieces, numSeparationPieces);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case '{':
        if (!best.empty())
        {
            currBoard = (currBoard + (int) (100 * best.size()) - 100) % best.size();
            Load(currBoard);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case '}':
        if (!best.empty())
        {
            currBoard = (currBoard + 100) % best.size();
            Load(currBoard); //, numRequiredPieces, numSeparationPieces);
            printf("%lu of %lu\n", currBoard + 1, best.size());
        }
        break;
    case 'o':
    {
        if (iws.ws.path.empty())
        {
            iws.ws.path.emplace_back(0, 0);
            iws.ws.path.emplace_back(0, 1);
            iws.ws.path.emplace_back(1, 1);
        }
        else
        {
            iws.Reset();
        }
        break;
    }
    case 'e':
    { // open editor
        if (!drawEditor)
        {
            drawEditor = true;
            iws.Reset();
            solved = false;
            UpdateSolutionIndices();
            MoveViewport(windowID, 1, {0.0f, -1.0f, 1.0f, 1.0f});
        }
        else
        {
            iws.Reset();
            solved = false;
            MoveViewport(windowID, 1, {1.0f, -1.0f, 2.0f, 1.0f});
            MoveViewport(windowID, 2, {1.0f, 0.0f, 2.0f, 2.0f});
            drawEditor = false;
            gSelectedEditorItem = -1;
        }
        break;
    }
    case 'x':
    {
        if (drawEditor)
            (selectTetrisPiece != 0) ? MoveViewport(windowID, 2, {0.0f, 0.0f, 1.0f, 2.0f}) :
                MoveViewport(windowID, 2, {1.0f, 0.0f, 2.0f, 2.0f});
        break;
    }
    case 'c':
    {
        auto e = GetCurrentEntropy(witness);
        std::cout << "Entropy of the current state: ("
            << ((e.value == inf) ? "inf" :to_string_with_precision(e.value, 2))
            << ", " << e.depth << ")" << std::endl;
        auto ae = GetCurrentAdvEntropy(witness);
        std::cout << "Dead-end Entropy of the current state: ("
            << ((ae.value == inf) ? "inf" : to_string_with_precision(ae.value, 2))
            << ", " << ae.depth << ")" << std::endl;
        if (!solutionTree.empty())
        {
            auto &actions = *witness.actionCache.getItem();
            witness.GetActionSequence(iws.ws, actions);
            int index = 0;
            for(auto i = 1; i < actions.size(); ++i)
            {
                const auto &action = actions[i];
                index = solutionTree[index].children[static_cast<unsigned>(action)];
                std::cout << action << ", " << index << std::endl;
                if (index == -1)
                {
                    std::cout << "not in the tree" << std::endl;
                    break;
                }
            }
            witness.actionCache.returnItem(&actions);
        }
        break;
    }
    case '0':
    {
        if (entropy.ruleSet.disabled.empty())
        {
            entropy.ruleSet.DisableAllRules();
            std::cout << "All inference rules are disabled." << std::endl;
        }
        else
        {
            entropy.ruleSet.EnableAllRules();
            std::cout << "All inference rules are enabled." << std::endl;
        }
        UpdateEntropy(witness);
        break;
    }
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    {
        int index = std::stoi(&key) - 1;
        std::cout << static_cast<WitnessInferenceRule>(index);
        if (!entropy.ruleSet.DisableRule(index))
        {
            (void) entropy.ruleSet.EnableRule(index);
            std::cout << " enabled" << std::endl;
        }
        else std::cout << " disabled" << std::endl;
        UpdateEntropy(witness);
        break;
    }
    default:
        break;
    }
}
