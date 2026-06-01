#include "Driver.h"
#include "SolutionUtil.h"

std::vector<RegionConstraintItem> gRegionConstraintItems = {
    {{kSeparation, 0, Colors::cyan}, Graphics::point{-0.75, -0.7}, 0.1},
    {{kStar, 0, Colors::cyan}, Graphics::point{-0.5, -0.7},  0.1},
    {{kTetris, 10, {0.862745098f, 0.6549019608f, 0.0f}}, Graphics::point{-0.25, -0.7}, 0.05},
    {{kNegativeTetris, 10, {0.2196078431f, 0.3607843137f, 0.8705882353f}}, Graphics::point{-0.05, -0.7}, 0.05},
    {{kTriangle, 1, Colors::orange}, Graphics::point{-0.75, -0.5}, 0.05},
    {{kTriangle, 2, Colors::orange}, Graphics::point{-0.5, -0.5}, 0.05},
    {{kTriangle, 3, Colors::orange}, Graphics::point{-0.2, -0.5}, 0.05},
};

std::vector<PathConstraintItem> gPathConstraintItems = {
    {kNoPathConstraint, Graphics::point{0.1, -0.5}, 0.075},
    {kMustCross, Graphics::point{0.3, -0.5}, 0.075},
    {kCannotCross, Graphics::point{0.5, -0.5}, 0.075}
};

bool drawEditor = false;
int gSelectedEditorItem = -1;
unsigned gSelectedTetrisItem = 0;
static Graphics::point gLastPosition = Graphics::point{-1, -1};
std::vector<size_t> currentSolutionIndices = {};
size_t gNumSolutions = 0;
EntropyInfo gEntropy{};
AdversaryEntropyInfo gAdvEntropy{};
bool gUseRelativeEntropy = true;
unsigned gLookahead = 0;

std::vector<ColorItem> gProvidedColors = {
    {Colors::black, Graphics::point{-0.75, -0.2}, 0.1},
    {Colors::cyan, Graphics::point{-0.6, -0.2}, 0.1},
    {Colors::magenta, Graphics::point{-0.45, -0.2}, 0.1},
    {Colors::yellow, Graphics::point{-0.3, -0.2}, 0.1},
    {Colors::red, Graphics::point{-0.15, -0.2}, 0.1},
    {Colors::green, Graphics::point{0.0, -0.2}, 0.1},
    {Colors::lightblue, Graphics::point{0.15, -0.2}, 0.1},
    {Colors::orange, Graphics::point{0.3, -0.2}, 0.1},
};

Witness<puzzleWidth, puzzleHeight> editor;

static inline size_t GetNumSolutions()
{
    return std::accumulate(allSolutions.begin(), allSolutions.end(), 0,
                           [&](const size_t &sum, const auto &solution){
        return sum + static_cast<size_t>(editor.GoalTest(solution));
    });
}

static void DrawGameViewport(unsigned long windowID)
{
    Graphics::Display &display = GetContext(windowID)->display;
    if (!drawEditor)
    {
        witness.Draw(display);
        iws.IncrementTime();
        witness.Draw(display, iws);
//        if (solved)
//            display.DrawText("Solved!", Graphics::point{1, 1}, Colors::black, 0.075,
//                             Graphics::textAlignRight, Graphics::textBaselineBottom);
    }
    else
    {
        editor.Draw(display);
        editor.Draw(display, iws.ws);
        display.DrawText("# of solutions: ", Graphics::point{0.75, -0.95}, Colors::black, 0.075,
                         Graphics::textAlignRight, Graphics::textBaselineTop);
        display.DrawText((gUseRelativeEntropy ? "ReMUSE: " : "MUSE: "),
                         Graphics::point{0.7, 0.9}, Colors::black, 0.075,
                         Graphics::textAlignRight, Graphics::textBaselineBottom);
        display.DrawText("MUAE: ",
                         Graphics::point{0.7, 1}, Colors::black, 0.075,
                         Graphics::textAlignRight, Graphics::textBaselineBottom);
        display.DrawText("Suggestion: ", Graphics::point{-1, 1}, Colors::black, 0.075,
                         Graphics::textAlignLeft, Graphics::textBaselineBottom);
        if (gSelectedEditorItem != -1 && cursorViewport == 0)
        {
            if (gSelectedEditorItem < gRegionConstraintItems.size())
            {
                bool cursorInPuzzle = false;
                const auto& constraint = gRegionConstraintItems[gSelectedEditorItem].constraint;
                for (auto i = 0; i < editor.regionConstraintLocations.size(); ++i)
                {
                    const auto &[position, rect] = editor.regionConstraintLocations[i];
                    if (PointInRect(cursor, rect))
                    {
                        auto [x, y] = editor.GetRegionXYFromIndex(i);
                        if (position != gLastPosition)
                        {
                            (constraint == editor.GetRegionConstraint(x, y)) ?
                                editor.RemoveRegionConstraint(x, y) :
                                editor.AddRegionConstraint(x, y, constraint);
                            gNumSolutions = GetNumSolutions();
                            UpdateEntropy(editor);
                        }
                        display.FrameRect(rect, (gNumSolutions > 0) ? Colors::gray : Colors::red, 0.01);
                        editor.DrawRegionConstraint(display, constraint, position);
                        gLastPosition = position;
                        cursorInPuzzle = true;
                        break;
                    }
                }
                if (!cursorInPuzzle)
                    editor.DrawRegionConstraint(display, constraint, cursor);
                if (gSuggestedLocation != std::numeric_limits<unsigned>::max())
                {
                    auto [x, y] = editor.GetRegionXYFromIndex(static_cast<int>(gSuggestedLocation));
                    display.DrawText((std::to_string(x) + ", " + std::to_string(y)).c_str(),
                                     Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                     Graphics::textAlignLeft, Graphics::textBaselineBottom);
                    if (editor.GetRegionConstraint(x, y).type == kNoRegionConstraint)
                        display.FrameRect(editor.regionConstraintLocations[gSuggestedLocation].second,
                                          Colors::green, 0.01);
                }
                else display.DrawText("None", Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                      Graphics::textAlignLeft, Graphics::textBaselineBottom);
            }
            else
            {
                for (auto i = 0; i < editor.pathConstraintLocations.size() - 1; ++i)
                {
                    const auto &[position, rect] = editor.pathConstraintLocations[i];
                    if (PointInRect(cursor, rect) &&
                        i != puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                    {
                        const auto &constraint =
                                gPathConstraintItems[gSelectedEditorItem - gRegionConstraintItems.size()]
                                        .constraint;
                        if (position != gLastPosition) {
                            editor.pathConstraints[i] = (constraint == editor.pathConstraints[i]) ?
                                                        kNoPathConstraint : constraint;
                            gNumSolutions = GetNumSolutions();
                            UpdateEntropy(editor);
                        }
                        display.FrameRect(rect, (gNumSolutions > 0) ? Colors::gray : Colors::red, 0.01);
                        gLastPosition = position;
                        break;
                    }
                }
                if (gSuggestedLocation != std::numeric_limits<unsigned>::max())
                {
                    const auto &[t, x, y] = editor.GetPathLocation(static_cast<int>(gSuggestedLocation));
                    display.DrawText((((t == 0) ? "horizontal, " : (t == 1) ? "vertical, " : "vertex, ") +
                                      std::to_string(x) + ", " +
                                      std::to_string(y)).c_str(),
                                     Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                     Graphics::textAlignLeft, Graphics::textBaselineBottom);
                    display.FrameRect(editor.pathConstraintLocations[gSuggestedLocation].second,
                                      Colors::green, 0.01);
                }
                else display.DrawText("None", Graphics::point{-0.57, 1}, Colors::black, 0.075,
                                      Graphics::textAlignLeft, Graphics::textBaselineBottom);
            }
            display.DrawText(std::to_string(gNumSolutions).c_str(), Graphics::point{0.915, -0.95},
                             Colors::black, 0.075, Graphics::textAlignRight, Graphics::textBaselineTop);
        }
        else
        {
            gLastPosition = Graphics::point{-1, -1};
            display.DrawText(std::to_string(currentSolutionIndices.size()).c_str(), Graphics::point{0.915, -0.95},
                             Colors::black, 0.075, Graphics::textAlignRight, Graphics::textBaselineTop);
        }
        if ((currentSolutionIndices.empty() || gNumSolutions == 0) && gEntropy.value != inf)
            UpdateEntropy(witness);
        display.DrawText((gEntropy.value != inf) ? to_string_with_precision(gEntropy.value, 2).c_str() : "inf",
                         Graphics::point{0.9, 0.9}, Colors::black, 0.075,
                         Graphics::textAlignRight, Graphics::textBaselineBottom);
        display.DrawText((gAdvEntropy.value != inf) ? to_string_with_precision(gAdvEntropy.value, 2).c_str() : "inf",
                         Graphics::point{0.9, 1}, Colors::black, 0.075,
                         Graphics::textAlignRight, Graphics::textBaselineBottom);
    }
}

static void FrameLightgrayRect(Graphics::Display& display, int viewport, const Graphics::rect& rect)
{
    if (cursorViewport == viewport && PointInRect(cursor, rect))
        display.FrameRect(rect, Colors::lightgray, 0.01);
}

static void DrawEditorViewport(unsigned long windowID)
{
    if (!drawEditor)
        return;
    Graphics::Display &display = GetContext(windowID)->display;

    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::gray);

    display.DrawText("Select a constraint", Graphics::point{-0.8, -0.83}, Colors::black, 0.05);
    for (auto i = 0; i < gRegionConstraintItems.size(); ++i)
    {
        const auto &[constraint, c, radius] = gRegionConstraintItems[i];
        editor.DrawRegionConstraint(display, constraint, c);
        if (i == gSelectedEditorItem)
        {
            if (i < gRegionConstraintItems.size() - 2)
                display.FrameRect({c, radius + 0.01f}, Colors::white, 0.01);
            else if (i == gRegionConstraintItems.size() - 2)
                display.FrameRect({c.x - radius - 0.03f, c.y - radius - 0.01f,
                                   c.x + radius + 0.03f, c.y + radius + 0.01f},
                                  Colors::white, 0.01);
            else
                display.FrameRect({c.x - radius - 0.06f, c.y - radius - 0.01f,
                                   c.x + radius + 0.06f, c.y + radius + 0.01f},
                                  Colors::white, 0.01);
        }
        else if (cursorViewport == 1 && PointInRect(cursor, {c, radius + 0.01f}))
        {
            if (i < gRegionConstraintItems.size() - 2)
                FrameLightgrayRect(display, 1, {c, radius + 0.01f});
            else if (i == gRegionConstraintItems.size() - 2)
                FrameLightgrayRect(display, 1, {c.x - radius - 0.03f, c.y - radius - 0.01f,
                                                c.x + radius + 0.03f, c.y + radius + 0.01f});
            else FrameLightgrayRect(display, 1, {c.x - radius - 0.06f, c.y - radius - 0.01f,
                                                 c.x + radius + 0.06f, c.y + radius + 0.01f});
        }
    }
    for (auto i = 0; i < gPathConstraintItems.size(); ++i)
    {
        const auto &[constraint, c, radius] = gPathConstraintItems[i];
        if (i == gSelectedEditorItem - gRegionConstraintItems.size())
            display.FillRect({c, radius + 0.01f}, Colors::green);
        else if (cursorViewport == 1 && PointInRect(cursor, {c, radius + 0.01f}))
            display.FillRect({c, radius + 0.01f}, Colors::lightgray);
        else
            display.FillRect({c, radius + 0.01f}, Colors::white);
        display.FillRect({c.x - radius, c.y - 0.025f, c.x + radius, c.y + 0.025f}, witness.drawColor);
        if (constraint == kCannotCross)
            display.FillRect({c, 0.025}, witness.backColor);
        else if (constraint == kMustCross)
            display.FillNGon(c, 0.025, 6, 30, witness.backColor);
    }
    display.DrawText("Select a color", Graphics::point{-0.8, -0.3}, Colors::black, 0.05);
    Graphics::rect rc;
    for (const auto &[color, c, _]: gProvidedColors)
    {
        rc = {c, 0.05f};
        display.FillRect(rc, color);
        FrameLightgrayRect(display, 1, rc);
    }
    display.DrawText("Clear All", Graphics::point{0.5, -0.15}, Colors::black, 0.05);
    rc = {0.49, -0.22, 0.7, -0.14};
    FrameLightgrayRect(display, 1, rc);
    display.DrawText("Use relative entropy: ", Graphics::point{-0.8, -0.04}, Colors::black, 0.05);
    display.DrawText((gUseRelativeEntropy) ? "true" : "false", Graphics::point{-0.32, -0.04},
                     (gUseRelativeEntropy) ? Colors::lightgreen : Colors::lightred, 0.05);
    rc = (gUseRelativeEntropy) ? Graphics::rect{-0.33, -0.10, -0.225, -0.03} :
            Graphics::rect{-0.33, -0.10, -0.21, -0.03};
    FrameLightgrayRect(display, 1, rc);
    display.DrawText("Lookahead steps: ", Graphics::point{-0.15, -0.04}, Colors::black, 0.05);
    display.DrawText(std::to_string(gLookahead).c_str(), Graphics::point{0.27, -0.04}, Colors::black, 0.05);
    rc = {0.26, -0.10, 0.31, -0.03};
    FrameLightgrayRect(display, 1, rc);
    display.DrawText("Replacement: ", Graphics::point{0.38, -0.04}, Colors::black, 0.05);
    display.DrawText((gWithReplacement) ? "true" : "false", Graphics::point{0.71, -0.04},
                     (gWithReplacement) ? Colors::lightgreen : Colors::lightred, 0.05);
    rc = (gWithReplacement) ? Graphics::rect{0.7, -0.10, 0.805, -0.03} : Graphics::rect{0.7, -0.10, 0.82, -0.03};
    FrameLightgrayRect(display, 1, rc);
}

static void DrawTetrisPiecesViewport(unsigned long windowID)
{
    if (!drawEditor)
        return;
    Graphics::Display &display = GetContext(windowID)->display;
    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::bluegray);
    for (const auto &[parameter, c, radius]: gTetrisPieces)
    {
        WitnessRegionConstraint constraint = {
            .type = (selectTetrisPiece == 2) ? kNegativeTetris : kTetris,
            .parameter = parameter,
            .color = Colors::white
        };
        editor.DrawRegionConstraint(display, constraint, c);
        if (cursorViewport == 2 && PointInRect(cursor, {c, radius}))
            FrameLightgrayRect(display, 2, {c, radius});
    }
}

void WitnessFrameHandler(unsigned long windowID, unsigned int viewport, void * /* data */)
{
    switch (viewport)
    {
        case 0:
            DrawGameViewport(windowID);
            break;
        case 1:
            editor = witness;
            DrawEditorViewport(windowID);
            break;
        case 2:
            DrawTetrisPiecesViewport(windowID);
            break;
        default:
            break;
    }
}
