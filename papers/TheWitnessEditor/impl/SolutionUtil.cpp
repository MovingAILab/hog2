#include "SolutionUtil.h"

void GetAllSolutions()
{
    std::vector<WitnessState<2, 2>> p1;
    GetAllSolutions(p1);
    std::vector<WitnessState<2, 3>> p2;
    GetAllSolutions(p2);
    std::vector<WitnessState<3, 3>> p3;
    GetAllSolutions(p3);
    std::vector<WitnessState<3, 4>> p4;
    GetAllSolutions(p4);
    std::vector<WitnessState<4, 4>> p5;
    GetAllSolutions(p5);
    std::vector<WitnessState<4, 5>> p6;
    GetAllSolutions(p6);
    std::vector<WitnessState<5, 5>> p7;
    GetAllSolutions(p7);
}

void UpdateSolutionIndices()
{
    currentSolutionIndices.clear();
    for (auto i = 0; i < allSolutions.size(); ++i)
    {
        if (witness.GoalTest(allSolutions[i]))
            currentSolutionIndices.emplace_back(i);
    }
    if (solved)
    {
        if (currentSolutionIndices.empty())
            iws.Reset();
        else
        {
            gSolutionIndex = 0;
            iws.ws = allSolutions[currentSolutionIndices[0]];
        }
    }
    UpdateEntropy(witness);
}
