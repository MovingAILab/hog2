//
//  SolutionUtil.h
//  The Witness Editor
//
//  Created by Samarium on 2023-06-29.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_SOLUTION_UTIL_H
#define THE_WITNESS_EDITOR_INCLUDE_SOLUTION_UTIL_H

#include "Globals.h"
#include "Timer.h"

template<int width, int height>
int CountSolutions(const Witness<width, height> &wp,
    const std::vector<WitnessState<width, height>> &solutions,
                   int &len, uint64_t limit)
{
    int count = 0;
    for (const auto &i: solutions)
    {
        if (wp.GoalTest(i))
        {
            len = (int)i.path.size();
            count++;
        }
        if (count > limit) break;
    }
    return count;
}

template<int width, int height>
int CountSolutions(const Witness<width, height> &w,
    const std::vector<WitnessState<width, height>> &allSolutions,
    std::vector<int> &solutions,
    const std::vector<int> &forbidden, int &len, uint64_t limit)
{
    solutions.resize(0);
    int count = 0;
    for (auto x: forbidden)
    {
        if (w.GoalTest(allSolutions[x]))
            return 0;
    }
    for (auto x = 0; x < allSolutions.size(); ++x)
    {
        if (w.GoalTest(allSolutions[x]))
        {
            len = static_cast<int>(allSolutions[x].path.size());
            count++;
            solutions.push_back(x);
        }
        if (count > limit) 
            break;
    }
    return count;
}

template<int width, int height>
void DFS(const Witness<width, height> &w, WitnessState<width, height> &s,
         std::vector<WitnessState<width, height>> &puzzles)
{
    std::vector<WitnessAction> acts;

    if (w.GoalTest(s))
    {
        puzzles.push_back(s);
        return;
    }

    w.GetActions(s, acts);
    for (auto &a: acts)
    {
        w.ApplyAction(s, a);
        DFS(w, s, puzzles);
        w.UndoAction(s, a);
    }
}

template<int width, int height>
void GetAllSolutions(const Witness<width, height> &w,
                     std::vector<WitnessState<width, height>> &puzzles)
{
    WitnessState<width, height> s;
    Timer t;
    t.StartTimer();
    DFS(w, s, puzzles);
    t.EndTimer();
    // printf("%lu solutions found in %1.2fs\n", puzzles.size(), t.GetElapsedTime());
}

template<int width, int height>
void GetAllSolutions(std::vector<WitnessState<width, height>> &puzzles)
{
    Witness<width, height> w;
    GetAllSolutions(w, puzzles);
}

void GetAllSolutions();
void UpdateSolutionIndices();

#endif /* THE_WITNESS_EDITOR_INCLUDE_SOLUTION_UTIL_H */
