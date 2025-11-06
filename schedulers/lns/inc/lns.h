#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>

namespace LNSScheduler
{
    // Structure to represent a solution
    struct Solution {
        std::vector<int> agent_to_task; // agent_id -> task_id (-1 if no task)
        std::unordered_map<int, int> task_to_agent; // task_id -> agent_id
        double cost;
        
        Solution(int num_agents) : agent_to_task(num_agents, -1), cost(0.0) {}
    };
    
    // Calculate the cost/quality of a solution
    double evaluateSolution(const Solution& solution, SharedEnvironment* env);
    
    // Calculate cost for a single agent's task assignment
    double calculateAgentCost(int agent_id, int task_id, SharedEnvironment* env);
    
    // Generate initial solution using greedy heuristic
    Solution generateInitialSolution(const std::unordered_set<int>& free_agents,
                                     const std::unordered_set<int>& free_tasks,
                                     SharedEnvironment* env);
    
    // Destroy operators: remove some task assignments
    void destroyRandom(Solution& solution, int num_to_remove, 
                      std::unordered_set<int>& removed_tasks,
                      std::mt19937& rng);
    
    void destroyWorst(Solution& solution, int num_to_remove,
                     std::unordered_set<int>& removed_tasks,
                     SharedEnvironment* env);
    
    void destroyRelated(Solution& solution, int num_to_remove,
                       std::unordered_set<int>& removed_tasks,
                       SharedEnvironment* env,
                       std::mt19937& rng);
    
    // Repair operator: reassign removed tasks
    void repairGreedy(Solution& solution, 
                     const std::unordered_set<int>& removed_tasks,
                     const std::unordered_set<int>& free_agents,
                     SharedEnvironment* env);
    
    void repairRegret(Solution& solution,
                     const std::unordered_set<int>& removed_tasks,
                     const std::unordered_set<int>& free_agents,
                     SharedEnvironment* env);
    
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);
    
    void schedule_plan(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env);
    
} // namespace LNSScheduler