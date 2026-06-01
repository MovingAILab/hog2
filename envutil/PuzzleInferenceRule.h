//
//  WitnessPuzzleInferenceRule.h
//  The Witness Editor
//
//  Created by Samarium on 2023-07-26.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef HOG2_ENV_UTIL_PUZZLE_INFERENCE_RULE_H
#define HOG2_ENV_UTIL_PUZZLE_INFERENCE_RULE_H

#include <algorithm>
#include <functional>
#include <iterator>
#include <unordered_map>
#include <vector>

#include "SearchEnvironment.h"

enum ActionType
{
    CANNOT_TAKE,
    MUST_TAKE,
    UNKNOWN,
    INVALID
};

template<class State, class Action>
class PuzzleInferenceRuleSet {
    using InferenceRule = std::function<ActionType(const SearchEnvironment<State, Action>&,
            State&, const Action&)>;

public:
    mutable std::unordered_map<Action, ActionType> logics = {};

    virtual ~PuzzleInferenceRuleSet() = default;

    virtual void UpdateActionLogics(const SearchEnvironment<State, Action> &env,
                                    State &state, const std::vector<Action> &actions) const
    {
        logics.clear();
        std::transform(actions.begin(), actions.end(), std::inserter(logics, logics.end()),
                       [](const Action &action) { return std::make_pair(action, UNKNOWN); });
        std::for_each(inferenceRules.begin(), inferenceRules.end(), [&](const auto &r) {
            if (std::find(disabled.begin(), disabled.end(), r.first) != disabled.end())
                return;
            std::for_each(logics.begin(), logics.end(), [&](auto &logic) {
                ActionType t = r.second(env, state, logic.first);
                if (t == UNKNOWN)
                    return;
                if (logic.second != UNKNOWN && logic.second != t)
                    logic.second = INVALID;
                else logic.second = t;
            });
        });
    }

    std::unordered_map<int, InferenceRule> inferenceRules;
    std::vector<int> disabled;

    virtual void SetRules(const std::unordered_map<int, InferenceRule> &rules)
    {
        inferenceRules = rules;
        disabled.reserve(rules.size());
    }

    virtual bool EnableRule(const int which)
    {
        if (const auto it = std::find(disabled.begin(), disabled.end(), which);
            it != disabled.end())
        {
            disabled.erase(it);
            return true;
        }
        return false;
    }

    virtual bool DisableRule(const int which)
    {
        if (std::find(disabled.begin(), disabled.end(), which) == disabled.end())
        {
            disabled.push_back(which);
            return true;
        }
        return false;
    }

    virtual void EnableAllRules()
    {
        disabled.clear();
    }

    virtual void DisableAllRules()
    {
        disabled.clear();
        std::transform(inferenceRules.begin(), inferenceRules.end(), std::inserter(disabled, disabled.end()),
                       [](const auto &r) { return r.first; });
    }

    virtual void FilterActions(const SearchEnvironment<State, Action> &env, State &state,
                               std::vector<Action> &actions) const
    {
        UpdateActionLogics(env, state, actions);
        if (std::count_if(logics.begin(), logics.end(), [](const auto &logic) {
            return logic.second == MUST_TAKE;
        }) > 1 || std::count_if(logics.begin(), logics.end(), [](const auto &logic) {
            return logic.second == INVALID;
        }) > 0)
        {
            actions.clear();
            return;
        }
        for (auto it = actions.begin(); it != actions.end(); )
        {
            Action &action = *it;
            switch (logics[action]) {
                case MUST_TAKE:
                {
                    actions.clear();
                    actions.push_back(action);
                    return;
                }
                case CANNOT_TAKE:
                {
                    it = actions.erase(it);
                    break;
                }
                default:
                    ++it;
            }
        }
    }
};

#endif /* HOG2_ENV_UTIL_PUZZLE_INFERENCE_RULE_H */
