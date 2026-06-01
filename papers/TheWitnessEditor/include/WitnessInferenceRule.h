//
//  WitnessInferenceRule.h
//  The Witness Editor
//
//  Created by Samarium on 2023-07-31.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H
#define THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H

#include "PuzzleInferenceRule.h"
#include "Witness.h"

struct SolutionTreeNode {
    WitnessAction action = kStart;
    std::array<int, kWitnessActionCount> children{};
    explicit SolutionTreeNode(WitnessAction action): action(action)
    {
        children.fill(-1);
    }
};

extern std::vector<SolutionTreeNode> solutionTree;

inline int InsertNode(std::vector<SolutionTreeNode> &tree, int parentIndex, WitnessAction action)
{
    auto index = static_cast<int>(tree.size());
    tree.emplace_back(action);
    if (parentIndex != -1)
        tree[parentIndex].children[static_cast<unsigned>(action)] = index;
    return index;
}

inline void PrintTree(const std::vector<SolutionTreeNode> &tree)
{
    for (auto i = 0; i < tree.size(); ++i)
    {
        std::cout << "Node " << i << ": Action = " << tree[i].action << ", Children = ";
        for (auto childIndex: tree[i].children)
        {
            if (childIndex != -1)
                std::cout << childIndex << " ";
        }
        std::cout << std::endl;
    }
}

template<int width, int height>
void BuildTree(const Witness<width, height> &puzzle,
               const std::vector<WitnessState<width, height>> solutions,
               std::vector<SolutionTreeNode> &tree)
{
    tree.reserve(std::numeric_limits<uint16_t>::max());
    Witness<width, height> w;
    w.pathConstraints = puzzle.pathConstraints;
//    std::for_each(w.pathConstraints.begin(), w.pathConstraints.end(), [](auto &contraint) {
//        if (contraint == kMustCross)
//            contraint = kNoPathConstraint;
//    });
    auto &actions = *w.actionCache.getItem();
    InsertNode(tree, -1, kStart);
    for (const auto &solution: solutions)
    {
        if (!w.GoalTest(solution))
            continue;
        int index = 0;
        w.GetActionSequence(solution, actions);
        for (auto i = 1; i < actions.size(); ++i)
        {
            const auto &action = actions[i];
            if (int child = tree[index].children[static_cast<int>(action)];
                child == -1)
                index = InsertNode(tree, index, action);
            else
                index = child;
        }
        actions.clear();
    }
    w.actionCache.returnItem(&actions);
}

static inline bool CompareSC(const WitnessRegionConstraint &a, const WitnessRegionConstraint &b)
{
    return a.type == kSeparation && b.type == kSeparation && a != b;
}

template<int width, int height>
ActionType SeparationRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                          const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto &witness = dynamic_cast<const Witness<width, height> &>(env);
    const auto &[currX, currY] = state.path.back();
    switch (action)
    {
        case kUp:
        {
            if (currX > 0 && currX < width
                && CompareSC(witness.GetRegionConstraint(currX - 1, currY),
                             witness.GetRegionConstraint(currX, currY)))
                return MUST_TAKE;
            break;
        }
        case kRight:
        {
            if (currY > 0 && currY < height
                && CompareSC(witness.GetRegionConstraint(currX, currY),
                             witness.GetRegionConstraint(currX, currY - 1)))
                return MUST_TAKE;
            break;
        }
        case kDown:
        {
            if (currX > 0 && currX < width
                && CompareSC(witness.GetRegionConstraint(currX - 1, currY - 1),
                             witness.GetRegionConstraint(currX, currY - 1)))
                return MUST_TAKE;
            break;
        }
        case kLeft:
        {
            if (currY > 0 && currY < height
                && CompareSC(witness.GetRegionConstraint(currX - 1, currY - 1),
                             witness.GetRegionConstraint(currX - 1, currY)))
                return MUST_TAKE;
            break;
        }
        default:
            break;
    }
    if (witness.constraintCount[kStar] > 0)
    {
        witness.GetLeftRightRegions(state);
        for (const auto &[color, regions]: witness.colorMap)
        {
            if (regions.size() == 2)
            {
                int r1 = regions[0];
                int r2 = regions[1];
                auto &rc1 = witness.GetRegionConstraint(r1);
                auto &rc2 = witness.GetRegionConstraint(r2);
                if ((rc1.type == kStar || rc2.type == kStar) &&
                    ((std::find(state.lhs.begin(), state.lhs.end(), r1) != state.lhs.end() &&
                     std::find(state.rhs.begin(), state.rhs.end(), r2) != state.rhs.end()) ||
                    (std::find(state.lhs.begin(), state.lhs.end(), r2) != state.lhs.end() &&
                     std::find(state.rhs.begin(), state.rhs.end(), r1) != state.rhs.end())))
                    return CANNOT_TAKE;
            }
        }
    }
    return UNKNOWN;
}

template<int width, int height>
ActionType PathConstraintRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                              const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto &witness = dynamic_cast<const Witness<width, height> &>(env);
    const auto &[currX, currY] = state.path.back();
    switch (action)
    {
        case kUp:
        {
            if (witness.GetMustCrossConstraint(false, currX, currY))
                return MUST_TAKE;
            if (witness.GetCannotCrossConstraint(false, currX, currY))
                return CANNOT_TAKE;
            break;
        }
        case kRight:
        {
            if (witness.GetMustCrossConstraint(true, currX, currY))
                return MUST_TAKE;
            if (witness.GetCannotCrossConstraint(true, currX, currY))
                return CANNOT_TAKE;
            break;
        }
        case kDown:
        {
            if (witness.GetMustCrossConstraint(false, currX, currY - 1))
                return MUST_TAKE;
            if (witness.GetCannotCrossConstraint(false, currX, currY - 1))
                return CANNOT_TAKE;
            break;
        }
        case kLeft:
        {
            if (witness.GetMustCrossConstraint(true, currX - 1, currY))
                return MUST_TAKE;
            if (witness.GetCannotCrossConstraint(true, currX - 1, currY))
                return CANNOT_TAKE;
            break;
        }
        default:
            break;
    }
    return UNKNOWN;
}

template<int width, int height>
ActionType TowardsGoalRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                           const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    if (action == kEnd)
        return MUST_TAKE;
    if (!state.HitTheWall() || state.IsAlongTheWall())
        return UNKNOWN;
    const auto &witness = dynamic_cast<const Witness<width, height> &>(env);
    if (witness.start.size() > 1 || witness.goal.size() > 1)
        return UNKNOWN;
    std::pair<int, int> start = witness.start[0];
    if (start.first != 0 && start.second != 0)
        return UNKNOWN;
    std::pair<int, int> goal = witness.goal[0];
    const auto &[currX, currY] = state.path.back();
    if (currX == 0 || currX == width) // vertical wall
    {
        if (currY < goal.second) // should choose kUP
        {
            if (start.second > currY && start.second <= goal.second) // unless the start is in the middle
                return (action == kDown) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kUp) ? MUST_TAKE : CANNOT_TAKE;
        }
        else // should choose kDown
        {
            if (start.second < currY && start.second >= goal.second)
                return (action == kUp) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kDown) ? MUST_TAKE : CANNOT_TAKE;
        }
    }
    else // horizontal wall
    {
        if (currX < goal.first) // should choose kRight
        {
            if (start.first > currX && start.first <= goal.first)
                return (action == kLeft) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kRight) ? MUST_TAKE : CANNOT_TAKE;
        }
        else // should choose kLeft
        {
            if (start.first < currX && start.first >= goal.first)
                return (action == kRight) ? MUST_TAKE : CANNOT_TAKE;
            return (action == kLeft) ? MUST_TAKE : CANNOT_TAKE;
        }
    }
}

template<int width, int height>
ActionType InsideSolutionTreeRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                                  const WitnessState<width, height> &state, const WitnessAction &action)
{
    if (action == kStart || action == kEnd)
        return MUST_TAKE;
    if (solutionTree.empty())
        return UNKNOWN;
    const auto &witness = dynamic_cast<const Witness<width, height> &>(env);
    auto &actions = *witness.actionCache.getItem();
    actions.clear();
    witness.GetActionSequence(state, actions);
    actions.emplace_back(action);
    int index = 0;
    for(auto i = 1; i < actions.size(); ++i)
    {
        index = solutionTree[index].children[static_cast<unsigned>(actions[i])];
        if (index == -1)
        {
            witness.actionCache.returnItem(&actions);
            return CANNOT_TAKE;
        }
    }
    witness.actionCache.returnItem(&actions);
    return UNKNOWN;
}

template<int width, int height>
ActionType RegionCompletionRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                                WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto &witness = dynamic_cast<const Witness<width, height> &>(env);
    witness.ApplyAction(state, action);
    bool regionSatisfied = true;
    const auto &head = state.path.back();
    if ((state.HitTheWall() && !state.IsAlongTheWall()) ||
        (std::find(witness.goal.cbegin(), witness.goal.cend(), head) == witness.goal.cend() &&
         witness.goalMap[witness.GetPathIndex(head)] != 0))
        regionSatisfied = witness.RegionTest(state);
    witness.UndoAction(state, action);
    return regionSatisfied ? UNKNOWN : CANNOT_TAKE;
}

template<int width, int height>
ActionType AlongThePathRule(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &env,
                            WitnessState<width, height> &state, const WitnessAction &action)
{
    if (state.path.empty())
        return UNKNOWN;
    const auto &witness = dynamic_cast<const Witness<width, height> &>(env);
    witness.ApplyAction(state, action);
    bool satisfied = witness.PathTest(state);
    witness.UndoAction(state, action);
    return satisfied ? UNKNOWN : CANNOT_TAKE;
}

enum WitnessInferenceRule {
    kSeparationRule,
    kPathConstraintRule,
    kInsideSolutionTreeRule,
    kRegionCompletionRule,
    kAlongThePathRule,
    kInferenceRuleCount [[maybe_unused]]
};

inline std::ostream &operator<<(std::ostream &os, WitnessInferenceRule wir)
{
    switch (wir)
    {
        case kSeparationRule:
            return os << "SeparationRule";
        case kPathConstraintRule:
            return os << "PathConstraintRule";
        case kInsideSolutionTreeRule:
            return os << "InsideSolutionTreeRule";
        case kRegionCompletionRule:
            return os << "RegionCompletionRule";
        case kAlongThePathRule:
            return os << "AlongThePathRule";
        default:
            return os;
    }
}

template<int width, int height>
std::unordered_map<int,
        std::function<ActionType(const SearchEnvironment<WitnessState<width, height>, WitnessAction> &,
                                 WitnessState<width, height> &, const WitnessAction &)>>
witnessInferenceRules =
{
    { kSeparationRule, std::function(SeparationRule<width, height>) },
    { kPathConstraintRule, std::function(PathConstraintRule<width, height>) },
    { kInsideSolutionTreeRule, std::function(InsideSolutionTreeRule<width, height>) },
    { kRegionCompletionRule, std::function(RegionCompletionRule<width, height>) },
    { kAlongThePathRule, std::function(AlongThePathRule<width, height>) },
};

#endif /* THE_WITNESS_EDITOR_INCLUDE_WITNESS_INFERENCE_RULE_H */
