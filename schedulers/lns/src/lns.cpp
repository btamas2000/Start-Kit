#include "lns.h"
#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>
#include <iostream>

// Forward declare the heuristic function from default planner
namespace DefaultPlanner {
    int get_h(SharedEnvironment* env, int source, int target);
    void init_heuristics(SharedEnvironment* env);
}

namespace LNSScheduler
{
    // Random number generator
    std::mt19937 rng;
    
    // Calculate cost for a single agent's task assignment
    double calculateAgentCost(int agent_id, int task_id, SharedEnvironment* env) {
        int agent_location = env->curr_states[agent_id].location;
        const Task& task = env->task_pool.at(task_id);
        
        // Calculate makespan: time to complete all errands
        int makespan = 0;
        int current_loc = agent_location;
        
        for (int loc : task.locations) {
            int dist = DefaultPlanner::get_h(env, current_loc, loc);
            makespan += dist;
            current_loc = loc;
        }
        
        double cost = static_cast<double>(makespan);
        
        // Deadline penalty
        if (task.t_deadline > 0) {
            int estimated_completion = env->curr_timestep + makespan;
            int deadline_time = task.t_revealed + task.t_deadline;
            
            if (estimated_completion > deadline_time) {
                // Heavy penalty for missing deadline
                int tardiness = estimated_completion - deadline_time;
                cost += 1000.0 * tardiness;
            } else {
                // Small penalty for tight deadlines (encourages early completion)
                int slack = deadline_time - estimated_completion;
                if (slack < 10) {
                    cost += 10.0 / (slack + 1);
                }
            }
        }
        
        // Task complexity
        cost += task.locations.size() * 1.5;
        
        return cost;
    }
    
    // Evaluate the total cost of a solution
    double evaluateSolution(const Solution& solution, SharedEnvironment* env) {
        double total_cost = 0.0;
        int num_assigned = 0;
        
        for (int agent_id = 0; agent_id < solution.agent_to_task.size(); ++agent_id) {
            int task_id = solution.agent_to_task[agent_id];
            if (task_id != -1) {
                total_cost += calculateAgentCost(agent_id, task_id, env);
                num_assigned++;
            }
        }
        
        // Bonus for assigning more tasks (encourage utilization)
        total_cost -= num_assigned * 0.5;
        
        return total_cost;
    }
    
    // Generate initial solution using greedy heuristic
    Solution generateInitialSolution(const std::unordered_set<int>& free_agents,
                                     const std::unordered_set<int>& free_tasks,
                                     SharedEnvironment* env) {
        Solution solution(env->num_of_agents);
        std::unordered_set<int> remaining_tasks = free_tasks;
        
        // Sort tasks by deadline urgency
        std::vector<int> task_list(free_tasks.begin(), free_tasks.end());
        std::sort(task_list.begin(), task_list.end(), [&](int a, int b) {
            const Task& task_a = env->task_pool.at(a);
            const Task& task_b = env->task_pool.at(b);
            
            // Prioritize tasks with deadlines
            if (task_a.t_deadline > 0 && task_b.t_deadline <= 0) return true;
            if (task_a.t_deadline <= 0 && task_b.t_deadline > 0) return false;
            
            if (task_a.t_deadline > 0 && task_b.t_deadline > 0) {
                int deadline_a = task_a.t_revealed + task_a.t_deadline;
                int deadline_b = task_b.t_revealed + task_b.t_deadline;
                return deadline_a < deadline_b;
            }
            
            return task_a.task_id < task_b.task_id;
        });
        
        // Greedy assignment: for each task, find best agent
        for (int task_id : task_list) {
            int best_agent = -1;
            double best_cost = std::numeric_limits<double>::infinity();
            
            for (int agent_id : free_agents) {
                // Skip if agent already assigned
                if (solution.agent_to_task[agent_id] != -1) continue;
                
                double cost = calculateAgentCost(agent_id, task_id, env);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_agent = agent_id;
                }
            }
            
            if (best_agent != -1) {
                solution.agent_to_task[best_agent] = task_id;
                solution.task_to_agent[task_id] = best_agent;
            }
        }
        
        solution.cost = evaluateSolution(solution, env);
        return solution;
    }
    
    // Destroy operator: randomly remove tasks
    void destroyRandom(Solution& solution, int num_to_remove,
                      std::unordered_set<int>& removed_tasks,
                      std::mt19937& rng) {
        removed_tasks.clear();
        
        // Collect all assigned tasks
        std::vector<int> assigned_agents;
        for (int i = 0; i < solution.agent_to_task.size(); ++i) {
            if (solution.agent_to_task[i] != -1) {
                assigned_agents.push_back(i);
            }
        }
        
        if (assigned_agents.empty()) return;
        
        // Randomly select tasks to remove
        std::shuffle(assigned_agents.begin(), assigned_agents.end(), rng);
        int to_remove = std::min(num_to_remove, (int)assigned_agents.size());
        
        for (int i = 0; i < to_remove; ++i) {
            int agent_id = assigned_agents[i];
            int task_id = solution.agent_to_task[agent_id];
            
            removed_tasks.insert(task_id);
            solution.agent_to_task[agent_id] = -1;
            solution.task_to_agent.erase(task_id);
        }
    }
    
    // Destroy operator: remove worst (most costly) assignments
    void destroyWorst(Solution& solution, int num_to_remove,
                     std::unordered_set<int>& removed_tasks,
                     SharedEnvironment* env) {
        removed_tasks.clear();
        
        // Calculate cost for each assignment
        std::vector<std::pair<double, int>> agent_costs; // (cost, agent_id)
        
        for (int agent_id = 0; agent_id < solution.agent_to_task.size(); ++agent_id) {
            int task_id = solution.agent_to_task[agent_id];
            if (task_id != -1) {
                double cost = calculateAgentCost(agent_id, task_id, env);
                agent_costs.push_back({cost, agent_id});
            }
        }
        
        if (agent_costs.empty()) return;
        
        // Sort by cost (descending)
        std::sort(agent_costs.begin(), agent_costs.end(), 
                 [](const auto& a, const auto& b) { return a.first > b.first; });
        
        int to_remove = std::min(num_to_remove, (int)agent_costs.size());
        
        for (int i = 0; i < to_remove; ++i) {
            int agent_id = agent_costs[i].second;
            int task_id = solution.agent_to_task[agent_id];
            
            removed_tasks.insert(task_id);
            solution.agent_to_task[agent_id] = -1;
            solution.task_to_agent.erase(task_id);
        }
    }
    
    // Destroy operator: remove related tasks (tasks assigned to nearby agents)
    void destroyRelated(Solution& solution, int num_to_remove,
                       std::unordered_set<int>& removed_tasks,
                       SharedEnvironment* env,
                       std::mt19937& rng) {
        removed_tasks.clear();
        
        // Find all assigned agents
        std::vector<int> assigned_agents;
        for (int i = 0; i < solution.agent_to_task.size(); ++i) {
            if (solution.agent_to_task[i] != -1) {
                assigned_agents.push_back(i);
            }
        }
        
        if (assigned_agents.empty()) return;
        
        // Pick a random seed agent
        std::uniform_int_distribution<> dist(0, assigned_agents.size() - 1);
        int seed_agent = assigned_agents[dist(rng)];
        int seed_location = env->curr_states[seed_agent].location;
        
        // Calculate distances from seed agent to all other agents
        std::vector<std::pair<int, int>> distances; // (distance, agent_id)
        for (int agent_id : assigned_agents) {
            int agent_loc = env->curr_states[agent_id].location;
            int dist = DefaultPlanner::get_h(env, seed_location, agent_loc);
            distances.push_back({dist, agent_id});
        }
        
        // Sort by distance (ascending)
        std::sort(distances.begin(), distances.end());
        
        int to_remove = std::min(num_to_remove, (int)distances.size());
        
        for (int i = 0; i < to_remove; ++i) {
            int agent_id = distances[i].second;
            int task_id = solution.agent_to_task[agent_id];
            
            removed_tasks.insert(task_id);
            solution.agent_to_task[agent_id] = -1;
            solution.task_to_agent.erase(task_id);
        }
    }
    
    // Repair operator: greedy insertion of removed tasks
    void repairGreedy(Solution& solution,
                     const std::unordered_set<int>& removed_tasks,
                     const std::unordered_set<int>& free_agents,
                     SharedEnvironment* env) {
        // For each removed task, find the best available agent
        for (int task_id : removed_tasks) {
            int best_agent = -1;
            double best_cost = std::numeric_limits<double>::infinity();
            
            for (int agent_id : free_agents) {
                // Skip if agent already assigned
                if (solution.agent_to_task[agent_id] != -1) continue;
                
                double cost = calculateAgentCost(agent_id, task_id, env);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_agent = agent_id;
                }
            }
            
            if (best_agent != -1) {
                solution.agent_to_task[best_agent] = task_id;
                solution.task_to_agent[task_id] = best_agent;
            }
        }
    }
    
    // Repair operator: regret-based insertion
    void repairRegret(Solution& solution,
                     const std::unordered_set<int>& removed_tasks,
                     const std::unordered_set<int>& free_agents,
                     SharedEnvironment* env) {
        std::unordered_set<int> remaining_tasks = removed_tasks;
        
        while (!remaining_tasks.empty()) {
            int best_task = -1;
            int best_agent = -1;
            double max_regret = -std::numeric_limits<double>::infinity();
            
            // For each task, calculate regret
            for (int task_id : remaining_tasks) {
                // Find best and second-best agent for this task
                double best_cost = std::numeric_limits<double>::infinity();
                double second_best_cost = std::numeric_limits<double>::infinity();
                int task_best_agent = -1;
                
                for (int agent_id : free_agents) {
                    if (solution.agent_to_task[agent_id] != -1) continue;
                    
                    double cost = calculateAgentCost(agent_id, task_id, env);
                    
                    if (cost < best_cost) {
                        second_best_cost = best_cost;
                        best_cost = cost;
                        task_best_agent = agent_id;
                    } else if (cost < second_best_cost) {
                        second_best_cost = cost;
                    }
                }
                
                // Regret is the difference between best and second-best
                double regret = second_best_cost - best_cost;
                
                if (regret > max_regret && task_best_agent != -1) {
                    max_regret = regret;
                    best_task = task_id;
                    best_agent = task_best_agent;
                }
            }
            
            if (best_task != -1 && best_agent != -1) {
                solution.agent_to_task[best_agent] = best_task;
                solution.task_to_agent[best_task] = best_agent;
                remaining_tasks.erase(best_task);
            } else {
                break; // No more valid assignments
            }
        }
    }
    
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env) {
        // Initialize heuristics for distance calculations
        DefaultPlanner::init_heuristics(env);
        
        // Seed random number generator
        rng.seed(42);
    }
    
    void schedule_plan(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
        auto start_time = std::chrono::steady_clock::now();
        
        // Track free agents and free tasks
        std::unordered_set<int> free_agents(env->new_freeagents.begin(), 
                                            env->new_freeagents.end());
        std::unordered_set<int> free_tasks(env->new_tasks.begin(), 
                                          env->new_tasks.end());
        
        // Generate initial solution
        Solution best_solution = generateInitialSolution(free_agents, free_tasks, env);
        Solution current_solution = best_solution;
        
        // LNS parameters
        const int MAX_ITERATIONS = 1000;
        const double DESTROY_RATE_MIN = 0.1;
        const double DESTROY_RATE_MAX = 0.4;
        int iteration = 0;
        int iterations_without_improvement = 0;
        
        // Adaptive destroy rate
        double destroy_rate = 0.2;
        
        while (iteration < MAX_ITERATIONS) {
            // Check time limit
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count();
            if (elapsed > time_limit * 0.9) break;
            
            // Calculate number of tasks to destroy
            int num_assigned = 0;
            for (int task_id : current_solution.agent_to_task) {
                if (task_id != -1) num_assigned++;
            }
            
            int num_to_remove = std::max(1, (int)(num_assigned * destroy_rate));
            
            // Copy current solution
            Solution new_solution = current_solution;
            std::unordered_set<int> removed_tasks;
            
            // Select destroy operator (randomly)
            std::uniform_int_distribution<> destroy_choice(0, 2);
            int destroy_op = destroy_choice(rng);
            
            if (destroy_op == 0) {
                destroyRandom(new_solution, num_to_remove, removed_tasks, rng);
            } else if (destroy_op == 1) {
                destroyWorst(new_solution, num_to_remove, removed_tasks, env);
            } else {
                destroyRelated(new_solution, num_to_remove, removed_tasks, env, rng);
            }
            
            // Select repair operator (randomly)
            std::uniform_int_distribution<> repair_choice(0, 1);
            int repair_op = repair_choice(rng);
            
            if (repair_op == 0) {
                repairGreedy(new_solution, removed_tasks, free_agents, env);
            } else {
                repairRegret(new_solution, removed_tasks, free_agents, env);
            }
            
            // Evaluate new solution
            new_solution.cost = evaluateSolution(new_solution, env);
            
            // Acceptance criterion (with simulated annealing)
            double temperature = 100.0 * std::exp(-iteration / 100.0);
            double delta = new_solution.cost - current_solution.cost;
            
            std::uniform_real_distribution<> prob_dist(0.0, 1.0);
            double acceptance_prob = std::exp(-delta / temperature);
            
            if (delta < 0 || prob_dist(rng) < acceptance_prob) {
                current_solution = new_solution;
                
                // Update best solution
                if (new_solution.cost < best_solution.cost) {
                    best_solution = new_solution;
                    iterations_without_improvement = 0;
                    
                    // Reduce destroy rate when improving
                    destroy_rate = std::max(DESTROY_RATE_MIN, destroy_rate * 0.95);
                } else {
                    iterations_without_improvement++;
                }
            } else {
                iterations_without_improvement++;
            }
            
            // Adaptive destroy rate
            if (iterations_without_improvement > 50) {
                destroy_rate = std::min(DESTROY_RATE_MAX, destroy_rate * 1.1);
                iterations_without_improvement = 0;
            }
            
            iteration++;
        }
        
        // Convert best solution to proposed_schedule format
        for (int agent_id = 0; agent_id < best_solution.agent_to_task.size(); ++agent_id) {
            proposed_schedule[agent_id] = best_solution.agent_to_task[agent_id];
        }
    }
    
} // namespace LNSScheduler