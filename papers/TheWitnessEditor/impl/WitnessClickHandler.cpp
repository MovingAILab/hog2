#include "Driver.h"
#include "Globals.h"
#include "SolutionUtil.h"

Graphics::point cursor = Graphics::point{};
int cursorViewport = 0;
int selectTetrisPiece = 0;
bool solved = false;
bool gWithReplacement = false;
unsigned gSuggestedLocation = std::numeric_limits<unsigned>::max();

static double MaximizedEntropy(const WitnessRegionConstraint &constraint)
{
    double ret = -1.0;
    if (gWithReplacement)
    {
        for (auto i = 0; i < witness.regionConstraintLocations.size(); ++i)
        {
            int x = witness.GetRegionFromX(i);
            int y = witness.GetRegionFromY(i);
            if (constraint == witness.GetRegionConstraint(x, y))
            {
                witness.RemoveRegionConstraint(x, y);
                double e = GetCurrentEntropy(witness).value;
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.AddRegionConstraint(x, y, constraint);
            }
            else
            {
                auto c = WitnessRegionConstraint(editor.GetRegionConstraint(x, y));
                witness.AddRegionConstraint(x, y, constraint);
                double e = GetCurrentEntropy(witness).value;
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.RemoveRegionConstraint(x, y);
                witness.AddRegionConstraint(x, y, c);
            }
        }
    }
    else
    {
        for (auto i = 0; i < witness.regionConstraintLocations.size(); ++i)
        {
            int x = witness.GetRegionFromX(i);
            int y = witness.GetRegionFromY(i);
            if (witness.GetRegionConstraint(x, y).type == kNoRegionConstraint)
            {
                witness.AddRegionConstraint(x, y, constraint);
                double e = GetCurrentEntropy(witness).value;
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.RemoveRegionConstraint(x, y);
            }
        }
    }
    if (ret == -1)
        gSuggestedLocation = std::numeric_limits<unsigned>::max();
    return ret;
}

static double MaximizedEntropy(const WitnessPathConstraintType &constraint)
{
    double ret = -1.0;
    if (gWithReplacement)
    {
        for (auto i = 0; i < witness.pathConstraintLocations.size() - 1; ++i)
        {
            if (i == puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                continue;
            if (constraint == witness.pathConstraints[i])
            {
                witness.pathConstraints[i] = kNoPathConstraint;
                double e = GetCurrentEntropy(witness).value;
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.pathConstraints[i] = constraint;
            }
            else
            {
                auto p = witness.pathConstraints[i];
                witness.pathConstraints[i] = constraint;
                double e = GetCurrentEntropy(witness).value;
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.pathConstraints[i] = p;
            }
        }
    }
    else
    {
        for (auto i = 0; i < witness.pathConstraintLocations.size() - 1; ++i)
        {
            if (i == puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                continue;
            if (witness.pathConstraints[i] == kNoPathConstraint)
            {
                witness.pathConstraints[i] = constraint;
                double e = GetCurrentEntropy(witness).value;
                if (e > ret && e != inf)
                {
                    ret = e;
                    gSuggestedLocation = i;
                }
                witness.pathConstraints[i] = kNoPathConstraint;
            }
        }
    }
    if (ret == -1)
        gSuggestedLocation = std::numeric_limits<unsigned>::max();
    return ret;
}

bool WitnessClickHandler(unsigned long windowID, int viewport, int /*x*/, int /*y*/, point3d p, tButtonType,
                         tMouseEventType event)
{
    switch (viewport)
    {
    case 0:
    {
        if (!drawEditor)
        {
            if (event == kMouseUp && witness.Click(p, iws))
            {
                if (witness.GoalTest(iws.ws))
                {
                    std::cout << "Solved!" << std::endl;
                    solved = true;
                }
                else
                {
                    std::cout << "Invalid solution" << std::endl;
                    iws.Reset();
                }
            }
            if (event == kMouseMove)
                witness.Move(p, iws);
        }
        else
        {
            if (event == kMouseDown)
            {
                if (gSelectedEditorItem != -1 && viewport == cursorViewport)
                {
                    if (gSelectedEditorItem < gRegionConstraintItems.size())
                    {
                        for (auto i = 0; i < witness.regionConstraintLocations.size(); ++i)
                        {
                            if (PointInRect(p, witness.regionConstraintLocations[i].second))
                            {
                                auto [x, y] = witness.GetRegionXYFromIndex(i);
                                const auto &constraint = gRegionConstraintItems[gSelectedEditorItem].constraint;
                                if (constraint == witness.GetRegionConstraint(x, y))
                                    witness.RemoveRegionConstraint(x, y);
                                else
                                    witness.AddRegionConstraint(x, y, constraint);
                                double e = MaximizedEntropy(constraint);
                                std::cout << "location: " << i << std::endl;
                                std::cout << "max entropy: "
                                    << ((e == inf) ? "inf" : to_string_with_precision(e, 2)) << std::endl;
                                break;
                            }
                        }
                    }
                    else 
                    {
                        for (auto i = 0; i < witness.pathConstraintLocations.size() - 1; ++i) {
                            if (PointInRect(p, witness.pathConstraintLocations[i].second) &&
                                i != puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight) {
                                WitnessPathConstraintType constraint =
                                        gPathConstraintItems[gSelectedEditorItem - gRegionConstraintItems.size()]
                                                .constraint;
                                if (constraint == editor.pathConstraints[i])
                                    witness.pathConstraints[i] = kNoPathConstraint;
                                else
                                    witness.pathConstraints[i] = constraint;
                                double e = MaximizedEntropy(constraint);
                                std::cout << "location: " << i << std::endl;
                                std::cout << "max entropy: "
                                          << ((e == inf) ? "inf" : to_string_with_precision(e, 2)) << std::endl;
                                break;
                            }
                        }
                    }
                    UpdateSolutionIndices();
                }
            }
        }
        break;
    }
    case 1:
    {
        if (event == kMouseDown && viewport == cursorViewport)
        {
            bool selected = false;
            for (auto i = 0; i < gRegionConstraintItems.size(); ++i)
            {
                const auto &[constraint, c, radius] = gRegionConstraintItems[i];
                if (PointInRect(p, {c, radius}))
                {
                    gSelectedEditorItem = static_cast<int>(i);
                    if (gSelectedEditorItem == 2)
                        selectTetrisPiece = 1;
                    else if (gSelectedEditorItem == 3)
                        selectTetrisPiece = 2;
                    else
                        selectTetrisPiece = 0;
                    WitnessKeyboardHandler(windowID, kAnyModifier, 'x');
                    selected = true;
                    double e = MaximizedEntropy(constraint);
                    std::cout << "max entropy: "
                        << ((e == inf) ? "inf" : to_string_with_precision(e, 2)) << std::endl;
                    break;
                }
            }
            for (auto i = 0; i < gPathConstraintItems.size(); ++i)
            {
                const auto &[constraint, c, radius] = gPathConstraintItems[i];
                if (PointInRect(p, {c, radius}))
                {
                    gSelectedEditorItem = static_cast<int>(i + gRegionConstraintItems.size());
                    selected = true;
                    double e = MaximizedEntropy(constraint);
                    std::cout << "max entropy: "
                        << ((e == inf) ? "inf" : to_string_with_precision(e, 2)) << std::endl;
                    break;
                }
            }
            bool selectColor = false;
            for (const auto &[color, c, radius]: gProvidedColors)
            {
                if (PointInRect(p, {c, radius}))
                {
                    selectColor = true;
                    gRegionConstraintItems[0].constraint.color = color;
                    gRegionConstraintItems[1].constraint.color = color;
                    break;
                }
            }
            if (!selected && !selectColor)
            {
                gSelectedEditorItem = -1;
            }
            if (PointInRect(p, Graphics::rect{0.49, -0.22, 0.7, -0.14}))
            {
                witness.Reset();
                allSolutions.clear();
                GetAllSolutions(witness, allSolutions);
                UpdateSolutionIndices();
                UpdateEntropy(witness);
            }
            if (PointInRect(p, Graphics::rect{-0.33, -0.10, -0.225, -0.04}))
            {
                gUseRelativeEntropy ^= true;
                UpdateEntropy(witness);
            }
            if (PointInRect(p, Graphics::rect{0.26, -0.10, 0.31, -0.03}))
            {
                switch (gLookahead) {
                    case 0:
                        gLookahead = 1;
                        break;
                    case 1:
                        gLookahead = 2;
                        break;
                    default:
                        gLookahead = 0;
                        break;
                }
                UpdateEntropy(witness);
            }
            if (PointInRect(p, Graphics::rect{0.7, -0.10, 0.805, -0.03}))
            {
                gWithReplacement ^= true;
            }
            std::cout << "Entropy: ("
                << ((gEntropy.value == inf) ? "inf" : to_string_with_precision(gEntropy.value, 2))
                << ", " << gEntropy.depth << ")" << std::endl;
            std::cout << "Adv Entropy: ("
                << ((gAdvEntropy.value == inf) ? "inf" : to_string_with_precision(gAdvEntropy.value, 2))
                << ", " << gAdvEntropy.depth << ")" << std::endl;
        }
        break;
    }
    case 2:
    {
        if (event == kMouseDown)
        {
            for (const auto &[parameter, c, radius]: gTetrisPieces)
            {
                if (PointInRect(p, {c, radius}))
                {
                    gSelectedTetrisItem = parameter;
                    std::cout << "Selected Tetris: " << gSelectedTetrisItem << std::endl;
                    if (selectTetrisPiece == 1)
                        gRegionConstraintItems[2].constraint.parameter = parameter;
                    else
                        gRegionConstraintItems[3].constraint.parameter = parameter;
                    selectTetrisPiece = 0;
                    WitnessKeyboardHandler(windowID, kAnyModifier, 'x');
                }
            }
        }
    }
    default:
        break;
    }
    cursor = p;
    cursorViewport = viewport;
    if (event == kMouseDrag) // ignore movement with mouse button down
        return false;

    // Don't need any other mouse support
    return true;
}
