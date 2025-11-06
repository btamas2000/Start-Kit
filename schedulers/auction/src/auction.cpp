#include "auction.h"
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <queue>
#include <chrono>
#include <cmath>

// Forward declare the heuristic function from default planner
namespace DefaultPlanner {
    int get_h(SharedEnvironment* env, int source, int target);
    void init_heuristics(SharedEnvironment* env);
}

namespace AuctionScheduler
{
    // Helper function to calculate Manhattan distance between two locations
    int getManhattanDistance(SharedEnvironment* env, int loc1, int loc2) {
        int row1 = loc1 / env->cols;
        int col1 = loc1 % env->cols;
        int row2 = loc2 / env->cols;
        int col2 = loc2 % env->cols;
        return std::abs(row1 - row2) + std::abs(col1 - col2);
    }

    // Calculate the cost for an agent to execute a task
    double calculateTaskCost(int agent_id, int task_id, 
                            const std::vector<int>& current_schedule,
                            SharedEnvironment* env) {
        // Get agent's current location
        int agent_location = env->curr_states[agent_id].location;
        
        // Get the task
        const Task& task = env->task_pool.at(task_id);
        
        // Calculate makespan: time to complete all errands in the task
        int makespan = 0;
        int current_loc = agent_location;
        
        for (int loc : task.locations) {
            // Use the heuristic function for more accurate distance
            int dist = DefaultPlanner::get_h(env, current_loc, loc);
            makespan += dist;
            current_loc = loc;
        }
        
        // Base cost is the makespan
        double cost = static_cast<double>(makespan);
        
        // Workload penalty: agents with more tasks should bid higher
        double workload_penalty = current_schedule.size() * 5.0;
        cost += workload_penalty;
        
        // Deadline urgency: prioritize tasks with tighter deadlines
        if (task.t_deadline > 0) {
            int time_remaining = task.t_revealed + task.t_deadline - env->curr_timestep;
            int estimated_completion = env->curr_timestep + makespan;
            
            // If task would miss deadline, add heavy penalty
            if (estimated_completion > task.t_revealed + task.t_deadline) {
                cost += 10000.0; // Very high penalty for missing deadline
            }
            // If deadline is tight, add urgency bonus (lower cost)
            else if (time_remaining < makespan * 1.5) {
                cost -= 50.0; // Encourage taking urgent tasks
            }
        }
        
        // Task complexity penalty: tasks with more locations cost more
        double complexity_penalty = task.locations.size() * 2.0;
        cost += complexity_penalty;
        
        return cost;
    }

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env) {
        // Initialize heuristics for distance calculations
        DefaultPlanner::init_heuristics(env);
    }

    void schedule_plan(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
        auto start_time = std::chrono::steady_clock::now();
        
        // Get problem dimensions from environment
        int num_agents = env->num_of_agents;
        
        // Track free agents and free tasks like the default scheduler
        std::unordered_set<int> free_agents(env->new_freeagents.begin(), env->new_freeagents.end());
        std::unordered_set<int> free_tasks(env->new_tasks.begin(), env->new_tasks.end());
        
        // Initialize data structures for auction
        std::vector<std::vector<int>> agent_schedules(num_agents);
        std::unordered_map<int, int> task_to_agent;
        std::unordered_map<int, double> task_prices;
        
        // Initialize all task prices to 0
        for (int task_id : free_tasks) {
            task_prices[task_id] = 0.0;
        }
        
        // Auction parameters
        const int MAX_ITERATIONS = 1000;
        const double EPSILON = 0.1; // Bid increment for price updates
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            // Check time limit (leave 10% buffer)
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count();
            if (elapsed > time_limit * 0.9) break;
            
            bool changes_made = false;
            
            // Bidding phase: each agent bids on their most profitable task
            for (int agent_id : free_agents) {
                // Skip if agent already has a task assigned
                if (env->curr_task_schedule[agent_id] != -1) continue;
                
                double best_value = -std::numeric_limits<double>::infinity();
                double second_best_value = -std::numeric_limits<double>::infinity();
                int best_task = -1;
                
                // Find best and second-best task for this agent
                for (int task_id : free_tasks) {
                    double cost = calculateTaskCost(agent_id, task_id, 
                                                   agent_schedules[agent_id], env);
                    double value = -cost - task_prices[task_id]; // Profit = -cost - price
                    
                    if (value > best_value) {
                        second_best_value = best_value;
                        best_value = value;
                        best_task = task_id;
                    } else if (value > second_best_value) {
                        second_best_value = value;
                    }
                }
                
                // If agent found a profitable task
                if (best_task != -1 && best_value > -std::numeric_limits<double>::infinity()) {
                    // Calculate bid increment
                    double bid_increment = (best_value - second_best_value) + EPSILON;
                    
                    // Check if this agent should win the task
                    bool should_assign = false;
                    
                    if (task_to_agent.find(best_task) == task_to_agent.end()) {
                        // Task is unassigned
                        should_assign = true;
                    } else {
                        // Task is assigned to another agent - check if this bid is better
                        int current_agent = task_to_agent[best_task];
                        if (current_agent != agent_id) {
                            double current_cost = calculateTaskCost(current_agent, best_task,
                                                                   agent_schedules[current_agent], env);
                            double this_cost = calculateTaskCost(agent_id, best_task,
                                                                agent_schedules[agent_id], env);
                            if (this_cost < current_cost) {
                                should_assign = true;
                                // Remove from previous agent
                                auto& old_schedule = agent_schedules[current_agent];
                                old_schedule.erase(
                                    std::remove(old_schedule.begin(), old_schedule.end(), best_task),
                                    old_schedule.end());
                                // Previous agent becomes free again
                                free_agents.insert(current_agent);
                            }
                        }
                    }
                    
                    if (should_assign) {
                        // Assign task to this agent
                        agent_schedules[agent_id].push_back(best_task);
                        task_to_agent[best_task] = agent_id;
                        proposed_schedule[agent_id] = best_task;
                        
                        // Update task price
                        task_prices[best_task] += bid_increment;
                        
                        changes_made = true;
                    }
                }
            }
            
            // Convergence check
            if (!changes_made) {
                break;
            }
        }
        
        // Assign -1 to agents that didn't get tasks
        for (int agent_id = 0; agent_id < num_agents; ++agent_id) {
            if (proposed_schedule[agent_id] != -1) {
                // Already assigned
                continue;
            }
            if (env->curr_task_schedule[agent_id] == -1) {
                // Agent is free but got no task
                proposed_schedule[agent_id] = -1;
            }
        }
    }
}