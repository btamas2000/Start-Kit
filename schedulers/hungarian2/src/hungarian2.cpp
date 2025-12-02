#include "hungarian2.h"
#include "SharedEnv.h"
#include "cluster_heuristics.h"
#include <vector>
#include <algorithm>

static const int LARGE_COST = 10000000; // avoid INT_MAX arithmetic issues

// Estimate when a busy agent will finish their current task using cluster heuristics
static int estimate_agent_finish_time(SharedEnvironment* env, int agent_id, int current_timestep)
{
    // If agent has no task assigned, they're already free
    if (env->curr_task_schedule[agent_id] == -1) {
        return current_timestep;
    }
    
    int task_id = env->curr_task_schedule[agent_id];
    const auto& task = env->task_pool[task_id];
    
    // Calculate remaining work based on idx_next_loc (current progress)
    int c_loc = env->curr_states.at(agent_id).location;
    int remaining_distance = 0;
    
    // Get cluster heuristics singleton
    clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
    
    // Only compute distance for remaining locations (from idx_next_loc onwards)
    for (int i = task.idx_next_loc; i < (int)task.locations.size(); ++i) {
        int loc = task.locations[i];
        int dist = clusterHeuristics.getHeuristicDistance(c_loc, loc);
        if (dist >= 0) {
            remaining_distance += dist;
        }
        c_loc = loc;
    }
    
    return current_timestep + remaining_distance;
}

// Compute basic distance-based cost using cluster heuristics
// For busy agents: includes wait time until free + travel distance
// For free agents: just travel distance
static int compute_basic_cost(SharedEnvironment* env, int agent_id, int task_id, int current_timestep)
{
    clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
    
    bool is_busy = (env->curr_task_schedule[agent_id] != -1);
    int start_time = current_timestep;
    int start_loc = env->curr_states.at(agent_id).location;
    
    // For busy agents, compute wait time and starting location
    if (is_busy) {
        int finish_time = estimate_agent_finish_time(env, agent_id, current_timestep);
        int wait_time = finish_time - current_timestep;
        start_time = finish_time;
        
        // Agent will be at the final location of their current task when they finish
        int current_task_id = env->curr_task_schedule[agent_id];
        const auto& current_task = env->task_pool[current_task_id];
        if (!current_task.locations.empty()) {
            start_loc = current_task.locations.back();
        }
    }
    
    // Compute travel distance for the new task
    int c_loc = start_loc;
    int travel_distance = 0;
    const auto& task = env->task_pool[task_id];
    for (int loc : task.locations) {
        int dist = clusterHeuristics.getHeuristicDistance(c_loc, loc);
        if (dist >= 0) {
            travel_distance += dist;
        }
        c_loc = loc;
    }
    
    // For busy agents, include wait time in cost
    int base_cost = (start_time - current_timestep) + travel_distance;
    
    return base_cost;
}

// Apply deadline-aware cost modification
// Takes a cost and applies penalties/bonuses based on deadline urgency
static int apply_deadline_cost(int base_cost, SharedEnvironment* env, int agent_id, 
                               int task_id, int current_timestep)
{
    // Start with base cost scaled up for precision
    int modified_cost = base_cost * 1000;
    
    const auto& task = env->task_pool[task_id];
    if (task.t_deadline > 0) {
        int deadline_absolute = task.t_revealed + task.t_deadline;
        const int DEADLINE_CRITICAL_THRESHOLD = 15; // timesteps before deadline to consider critical
        
        // Calculate when the task would be completed
        bool is_busy = (env->curr_task_schedule[agent_id] != -1);
        int start_time = current_timestep;
        if (is_busy) {
            start_time = estimate_agent_finish_time(env, agent_id, current_timestep);
        }
        int estimated_completion = start_time + base_cost;
        
        // Apply deadline-based scaling
        if (estimated_completion > deadline_absolute - DEADLINE_CRITICAL_THRESHOLD 
            && estimated_completion <= deadline_absolute) {
            // Tight deadline - higher priority (moderate discount)
            modified_cost = modified_cost / 100;
        } else if (estimated_completion < deadline_absolute) {
            // Comfortable deadline - lower priority (strong discount)
            modified_cost = modified_cost / 1000;
        }
        // If estimated_completion > deadline_absolute, keep base cost (deadline will be missed)
    }
    
    return modified_cost;
}

// Build cost matrix without dummy padding (returns n x m matrix)
static std::vector<std::vector<int>> build_cost_matrix(SharedEnvironment* env, 
    const std::vector<int>& combined_agents_list, const std::vector<int>& free_tasks_list,
    int current_timestep)
{
    int n = (int)combined_agents_list.size();
    int m = (int)free_tasks_list.size();

    // Build n x m matrix (no dummy padding yet)
    std::vector<std::vector<int>> cost_matrix(n, std::vector<int>(m, LARGE_COST));

    // Compute costs with deadline penalties applied
    for (int i = 0; i < n; i++) {
        int agent_id = combined_agents_list[i];
        
        for (int j = 0; j < m; j++) {
            int task_id = free_tasks_list[j];
            
            // Compute basic distance cost
            int basic_cost = compute_basic_cost(env, agent_id, task_id, current_timestep);
            
            // Apply deadline penalty
            int final_cost = apply_deadline_cost(basic_cost, env, agent_id, task_id, current_timestep);
            
            cost_matrix[i][j] = final_cost;
        }
    }

    return cost_matrix;
}

// Add dummy rows/columns to make the matrix square for Hungarian algorithm
static std::vector<std::vector<int>> pad_to_square(const std::vector<std::vector<int>>& cost_matrix)
{
    if (cost_matrix.empty()) return cost_matrix;
    
    int n = (int)cost_matrix.size();
    int m = (int)cost_matrix[0].size();
    int max_dim = std::max(n, m);
    
    std::vector<std::vector<int>> padded(max_dim, std::vector<int>(max_dim, LARGE_COST));
    
    // Copy original costs
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            padded[i][j] = cost_matrix[i][j];
        }
    }
    
    return padded;
}

void HungarianScheduler2::hun_schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // Initialize cluster heuristics with the map data
    clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
    clusterHeuristics.initialize(env->map, env->rows, env->cols, env);
}

// Persistent agent and task tracking (like default scheduler)
static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;

// Lookahead settings for considering busy agents
static const int LOOKAHEAD_TIMESTEPS = 10; // Consider agents finishing within this many timesteps
static const double DEFER_THRESHOLD = 0.4; // Defer if busy agent cost is this much better (40% improvement)

// Hungarian algorithm implementation (defined before hun_schedule_plan to avoid forward declaration)
static std::vector<int> HungarianAlgorithm(const std::vector<std::vector<int>>& cost_matrix)
{
    int n = (int)cost_matrix.size();
    int m = n; // square matrix
    std::vector<int> assignment(n, -1);

    const int INF = LARGE_COST;
    std::vector<int> u(n + 1), v(m + 1), p(m + 1), way(m + 1);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<int> minv(m + 1, INF);
        std::vector<bool> used(m + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], delta = INF, j1;
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    int cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            assignment[p[j] - 1] = j - 1;
        }
    }
    return assignment;
}

void HungarianScheduler2::hun_schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    // Maintain persistent free agent/task tracking (like default scheduler)
    // This allows us to track agents/tasks across multiple timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // Build ordered lists from the persistent sets for Hungarian algorithm
    std::vector<int> free_agents_list(free_agents.begin(), free_agents.end());
    std::vector<int> free_tasks_list(free_tasks.begin(), free_tasks.end());

    int n = (int)free_agents_list.size();
    int m = (int)free_tasks_list.size();
    if (n == 0 || m == 0) {
        // nothing to assign
        for (int aid : free_agents_list) proposed_schedule[aid] = -1;
        return;
    }

    // Get current timestep for deadline calculations
    int current_timestep = env->curr_timestep;

    // STEP 1: Identify near-finish agents (busy agents that will finish soon)
    // These agents are currently working on a task but will be free within LOOKAHEAD_TIMESTEPS
    std::unordered_set<int> near_finish_agents;
    for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id) {
        // Skip agents that are already free
        if (free_agents.find(agent_id) != free_agents.end()) {
            continue;
        }
        
        // Check if this busy agent will finish within the lookahead window
        int finish_time = estimate_agent_finish_time(env, agent_id, current_timestep);
        int time_until_free = finish_time - current_timestep;
        
        if (time_until_free > 0 && time_until_free <= LOOKAHEAD_TIMESTEPS) {
            near_finish_agents.insert(agent_id);
        }
    }

    // STEP 2: Combine free agents and near-finish agents for Hungarian assignment
    // This allows the algorithm to consider both immediately available agents
    // and agents that will soon become available
    std::vector<int> combined_agents_list = free_agents_list;
    for (int agent_id : near_finish_agents) {
        combined_agents_list.push_back(agent_id);
    }

    // STEP 3: Pre-filter tasks where busy agents have significant improvement
    // Collect tasks where busy agents would do significantly better - these should be deferred
    // and kept available for when those agents actually become free
    std::unordered_set<int> tasks_for_assignment_set;
    
    if (!near_finish_agents.empty()) {
        // Build a temporary cost matrix to evaluate which tasks near-finish agents would want
        auto temp_cost_matrix = build_cost_matrix(env, combined_agents_list, free_tasks_list, current_timestep);
        
        // Start with all tasks as candidates
        for (int task_id : free_tasks) {
            tasks_for_assignment_set.insert(task_id);
        }
        
        // For each near-finish agent, find which task they'd prefer
        for (int agent_id : near_finish_agents) {
            // Find this agent's index in combined list
            int agent_idx = -1;
            for (int i = 0; i < (int)combined_agents_list.size(); ++i) {
                if (combined_agents_list[i] == agent_id) {
                    agent_idx = i;
                    break;
                }
            }
            
            if (agent_idx == -1) continue;
            
            // Find the best task for this busy agent
            int best_task_idx = -1;
            int best_cost = LARGE_COST;
            for (int j = 0; j < m; ++j) {
                if (temp_cost_matrix[agent_idx][j] < best_cost) {
                    best_cost = temp_cost_matrix[agent_idx][j];
                    best_task_idx = j;
                }
            }
            
            if (best_task_idx >= 0 && best_cost < LARGE_COST) {
                int task_id = free_tasks_list[best_task_idx];
                
                // Check if this busy agent is significantly better than free agents
                int best_free_cost = LARGE_COST;
                for (int k = 0; k < (int)free_agents_list.size(); ++k) {
                    if (temp_cost_matrix[k][best_task_idx] < best_free_cost) {
                        best_free_cost = temp_cost_matrix[k][best_task_idx];
                    }
                }
                
                // If busy agent is significantly better, exclude this task from current assignment
                // This keeps the task available for when this agent becomes free
                if (best_free_cost < LARGE_COST) {
                    double improvement = (double)(best_free_cost - best_cost) / (double)best_free_cost;
                    
                    if (improvement > DEFER_THRESHOLD) {
                        // Exclude this task - it should wait for the busy agent
                        tasks_for_assignment_set.erase(task_id);
                    }
                }
            }
        }
    } else {
        // No near-finish agents, use all free tasks
        for (int task_id : free_tasks) {
            tasks_for_assignment_set.insert(task_id);
        }
    }
    
    // STEP 4: Build filtered lists for actual assignment
    // Only free agents and only tasks where there's no significant busy agent advantage
    std::vector<int> agents_for_assignment = free_agents_list;
    std::vector<int> tasks_for_assignment(tasks_for_assignment_set.begin(), tasks_for_assignment_set.end());
    
    // Update m to reflect filtered task count
    int filtered_m = (int)tasks_for_assignment.size();
    
    // STEP 5: Build cost matrix with only free agents and filtered tasks
    auto cost_matrix = build_cost_matrix(env, agents_for_assignment, tasks_for_assignment, current_timestep);
    
    // Pad to square matrix for Hungarian algorithm
    auto padded_matrix = pad_to_square(cost_matrix);
    auto assignment = HungarianAlgorithm(padded_matrix);

    // STEP 6: Process assignments for free agents only
    int assignment_n = (int)agents_for_assignment.size();
    
    for (int i = 0; i < assignment_n; ++i) {
        int task_index = (i < (int)assignment.size()) ? assignment[i] : -1;
        if (task_index >= 0 && task_index < filtered_m) {
            int agent_id = agents_for_assignment[i];
            int task_id = tasks_for_assignment[task_index];
            
            // Set the assignment and remove from persistent tracking
            proposed_schedule[agent_id] = task_id;
            free_agents.erase(agent_id);
            free_tasks.erase(task_id);
        } else {
            // No assignment for this agent
            int agent_id = agents_for_assignment[i];
            proposed_schedule[agent_id] = -1;
            // Keep agent in free_agents set for next round
        }
    }
}
