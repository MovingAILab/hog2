//
//  PuzzleEntropy.hpp
//  The Witness Editor
//
//  Created by Samarium on 2023-07-21.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef HOG2_GENERIC_PUZZLE_ENTROPY_H
#define HOG2_GENERIC_PUZZLE_ENTROPY_H

#include <iterator>
#include <numeric>
#include <optional>
#include <vector>
#include "PuzzleInferenceRule.h"
#include "SearchEnvironment.h"
#include "vectorCache.h"

struct EntropyInfo
{
    double value;
    unsigned depth;
};

template<class State, class Action>
class Entropy
{
protected:
    static constexpr double inf = std::numeric_limits<double>::max();
    mutable vectorCache<Action> actCache;
    mutable vectorCache<EntropyInfo> entropyInfoCache;
    mutable vectorCache<double> doubleCache;
    bool isRelative = false;
    bool base2 = false;

    void Softmin(const std::vector<double> &vars, std::vector<double> &ret) const
    {
        const double sum = std::accumulate(vars.cbegin(), vars.cend(), 0.0,
            [&](const double r, const double i) {
                return r + (base2 ? std::exp2(-i) : std::exp(-i));
        });
        ret.reserve(vars.size());
        std::transform(vars.cbegin(), vars.cend(), std::back_inserter(ret),
            [&](const double i) {
                return (base2 ? std::exp2(-i) : std::exp(-i)) / sum;
        });
    }

    static double KlDivergence(const std::vector<double> &p, const std::vector<double> &q)
    {
        if (p.size() != q.size())
            throw std::invalid_argument("Input vectors must be of the same size.");
        double divergence = 0.0;
        for (auto i = 0; i < p.size(); ++i)
        {
            if (p[i] != 0.0 && q[i] != 0.0)
                divergence += p[i] * std::log2(p[i] / q[i]);
        }
        return divergence;
    }

    double ImmediateEntropy(const std::vector<Action> &actions,
                            const std::vector<double> &childEntropy,
                            std::optional<Action> prevAction) const
    {
        const auto size = actions.size();
        if (!isRelative)
            return std::log2(static_cast<double>(size));
        auto &childDist = *doubleCache.getItem();
        Softmin(childEntropy, childDist);
        auto &expectedDist = *doubleCache.getItem();
        expectedDist.resize(size);
        std::generate(expectedDist.begin(), expectedDist.end(), [
            size = static_cast<double>(size)
        ]() { return 1 / size; });
        if (size > 1 && prevAction.has_value() &&
            std::find(actions.cbegin(), actions.cend(), prevAction.value()) != actions.cend())
        {
            for (auto i = 0; i < size; ++i)
            {
                if (actions[i] == prevAction.value())
                    expectedDist[i] += probShift;
                else
                    expectedDist[i] -= probShift / (size - 1);
            }
        }
        const auto kl = KlDivergence(childDist, expectedDist);
        doubleCache.returnItem(&childDist);
        doubleCache.returnItem(&expectedDist);
        return kl;
    }

public:
    PuzzleInferenceRuleSet<State, Action> ruleSet;
    double probShift = 0.0;

    virtual ~Entropy() = default;

    auto& SetRelative(bool val)
    {
        this->isRelative = val;
        return *this;
    }

    auto& SetShift(double val)
    {
        this->probShift = val;
        return *this;
    }

    auto& SetBase2(bool val)
    {
        this->base2 = val;
        return *this;
    }

    virtual EntropyInfo Calculate(const SearchEnvironment<State, Action> &env, State &state,
                                  unsigned lookahead, std::optional<Action> prevAction) const
    {
        if (env.GoalTest(state))
            return { 0.0, 0 };
        auto &actions = *actCache.getItem();
        env.GetActions(state, actions);
        ruleSet.FilterActions(env, state, actions);
        if (actions.empty())
        {
            actCache.returnItem(&actions);
            return { inf, 0 };
        }
        auto &children = *entropyInfoCache.getItem();
        for (const auto &action: actions)
        {
            env.ApplyAction(state, action);
            children.emplace_back(Calculate(env, state, lookahead, action));
            env.UndoAction(state, action);
        }
        // TODO: apply look ahead
        EntropyInfo entropyInfo{};
        if (std::all_of(children.begin(), children.end(), [](const auto &info) {
            return info.value == 0;
        }))
            entropyInfo = { 0.0, children[0].depth + 1 };
        else if (std::all_of(children.begin(), children.end(), [](const auto &info) {
            return info.value == inf;
        }))
            entropyInfo = { inf, children[0].depth + 1 };
        else
        {
            auto &minChild = *std::min_element(children.begin(), children.end(),
                                               [](const auto &info1, const auto &info2) {
                return info1.value < info2.value;
            });
            auto &childrenEntropy = *doubleCache.getItem();
            childrenEntropy.reserve(children.size());
            std::transform(children.cbegin(), children.cend(), std::back_inserter(childrenEntropy),
                           [](const auto &info) { return info.value; });
            entropyInfo = { minChild.value + ImmediateEntropy(actions, childrenEntropy, prevAction),
                            minChild.depth + 1 };
            doubleCache.returnItem(&childrenEntropy);
        }
        entropyInfoCache.returnItem(&children);
        actCache.returnItem(&actions);
        return entropyInfo;
    }
};

#endif /* HOG2_GENERIC_PUZZLE_ENTROPY_H */
