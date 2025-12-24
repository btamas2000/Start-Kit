#include "cluster_pibt.h"

// My custom PIBT implementation using the cluster-based approach
// Pipeline:
// 1. Initialize necessary data structures
// 2. Assign priorities (base priority + other factors)
// 3. Sort agents by priority
// 4. For each agent in order of priority, select the best action
// 5. If conflicts arise, replan for the conflicted agents
namespace ClusterPIBTPlanner{
    // Necessary data structures and variables can be defined here
    std::vector<int> priorities;
    std::mt19937 rng; // Random number generator
    std::vector<State> previous_states;
    std::vector<State> next_states;
    std::vector<int> dummy_goals; // In case nothing is assigned to the agent

    void assignPriorities(SharedEnvironment* env) {
        // Base priority: deadlines
        std::vector<int> 
    }

    void initialize(int preprocess_time_limit, SharedEnvironment* env) {
        // Initialization code here
        priorities.resize(env->getNumAgents(), 0);
        previous_states.resize(env->getNumAgents());
        next_states.resize(env->getNumAgents());
        dummy_goals.resize(env->getNumAgents(), -1); // Assuming -1 indicates no goal
        rng.seed(0);
    }

    void plan(int time_limit,std::vector<Action> & actions,  SharedEnvironment* env){
        
    }
}