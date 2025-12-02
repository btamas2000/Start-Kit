#pragma once
#include "SharedEnv.h"
#include "ActionModel.h"
#include "cluster_heuristics.h"
#include <vector>
#include <unordered_map>

namespace ClusterPIBTPlanner
{
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env);

    // Agent priority tracking
    extern std::vector<double> priorities;
    extern std::vector<int> agent_ids;
    
    // Planning state
    extern std::vector<int> prev_decision;
    extern std::vector<int> decision;
    extern std::vector<bool> occupied;
    extern std::vector<State> prev_states;
    extern std::vector<State> next_states;
    
    // Goal tracking
    extern std::vector<int> agent_goals;
    extern std::vector<int> dummy_goals;
    
    // Decision tracking for turning actions
    struct DecisionRecord {
        int location;
        bool done;
        DecisionRecord() : location(-1), done(true) {}
        DecisionRecord(int loc, bool d) : location(loc), done(d) {}
    };
    extern std::vector<DecisionRecord> decided;
    extern std::vector<bool> checked;
    
    // Helper functions
    bool executePIBT(int agent_id, int higher_priority_agent, SharedEnvironment* env);
    int getHeuristic(int from_loc, int to_loc);
    std::vector<int> getNeighbors(int location, SharedEnvironment* env);
    bool validateMove(int from_loc, int to_loc, SharedEnvironment* env);
    Action computeAction(const State& prev, int next_loc, SharedEnvironment* env);
    bool moveCheck(int agent_id, std::vector<Action>& actions, SharedEnvironment* env);
    
} // namespace ClusterPIBTPlanner
