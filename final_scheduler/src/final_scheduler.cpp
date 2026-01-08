#include "final_scheduler.h"
#include "SharedEnv.h"
#include "final_utilities/inc/dynamic.h"
#include "final_utilities/inc/preprocess.h"
#include <cmath>
#include <algorithm>
#include <iostream>

static const int LARGE_COST = 10000000; // avoid INT_MAX arithmetic issues

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;

// --- Helper for Distance Calculation ---
static int get_heuristic_distance(SharedEnvironment* env, int loc1, int loc2) {
    auto& pre = PreprocessingPipeline::PreprocessPipeline::getInstance();
    
    // Fallback if map not loaded or locs invalid
    if (loc1 < 0 || loc2 < 0 || loc1 >= env->rows * env->cols || loc2 >= env->rows * env->cols) {
        return std::abs(loc1 / env->cols - loc2 / env->cols) + 
               std::abs(loc1 % env->cols - loc2 % env->cols);
    }

    // Note: These accessors (getClusterId, getInterClusterHeuristics) 
    // must be exposed in preprocess.h as discussed
    int c1 = pre.getClusterId(loc1);
    int c2 = pre.getClusterId(loc2);
    
    // Intra-cluster or Error (Fallback to Manhattan)
    if (c1 == -1 || c2 == -1 || c1 == c2) {
        int r1 = loc1 / env->cols, c_1 = loc1 % env->cols;
        int r2 = loc2 / env->cols, c_2 = loc2 % env->cols;
        return std::abs(r1 - r2) + std::abs(c_1 - c_2);
    }
    
    // Inter-cluster Heuristic Lookup
    const auto& matrix = pre.getInterClusterHeuristics()[c1][c2];
    
    // Find min cost among all portal connections between these two clusters
    int min_d = LARGE_COST;
    // The matrix is [Start_Portal_Idx][End_Portal_Idx]
    for(const auto& row : matrix) {
        for(const auto& path : row) {
            if(path.total_timesteps > 0 && path.total_timesteps < min_d) 
                min_d = path.total_timesteps;
        }
    }
    
    // If no path found in lookup, fallback to Manhattan * 2 (penalty)
    if (min_d == LARGE_COST) {
        int r1 = loc1 / env->cols, c_1 = loc1 % env->cols;
        int r2 = loc2 / env->cols, c_2 = loc2 % env->cols;
        return (std::abs(r1 - r2) + std::abs(c_1 - c_2));
    }

    return min_d;
}

// Estimate when a busy agent will finish their current task using cluster heuristics
static int estimate_agent_finish_time(SharedEnvironment* env, const DynamicData::Agent& agent, int current_timestep)
{
    if (agent.isFree()) {
        return current_timestep;
    }

    // 1. Calculate remaining time in the Low-Level Window (Windowed A*)
    int remaining_window = 0;
    int loc_at_window_end = agent.currentLocation;

    if (!agent.plan.empty() && agent.current_plan_index < (int)agent.plan.size()) {
        remaining_window = (int)agent.plan.size() - agent.current_plan_index;
        // The location where the agent will be at the end of its current specific plan
        loc_at_window_end = agent.plan.back().location;
    }

    // 2. If agent has no high-level goal, they finish at end of window
    if (agent.goalLocations.empty()) {
        return current_timestep + remaining_window;
    }

    // 3. Calculate heuristic distance from Window End -> Final Goal
    int final_goal = agent.goalLocations.front();
    
    // Don't calculate if already there
    if (loc_at_window_end == final_goal) {
        return current_timestep + remaining_window;
    }

    int heuristic_rem = get_heuristic_distance(env, loc_at_window_end, final_goal);
    
    return current_timestep + remaining_window + heuristic_rem;
}

// Compute basic distance-based cost using cluster heuristics
// For busy agents: includes wait time until free + travel distance
// For free agents: just travel distance
static int compute_cost(SharedEnvironment* env, const DynamicData::Agent& agent, int task_id, int current_timestep)
{
    // 1. Estimate when agent can START this new task
    int start_time = estimate_agent_finish_time(env, agent, current_timestep);
    int wait_time = start_time - current_timestep;

    // 2. Determine where the agent will be when it starts
    int agent_loc_when_free = agent.currentLocation;
    if (!agent.isFree() && !agent.goalLocations.empty()) {
        agent_loc_when_free = agent.goalLocations.front();
    } else if (!agent.isFree() && !agent.plan.empty()) {
        agent_loc_when_free = agent.plan.back().location;
    }

    // 3. Distance from Agent Start -> Task Location
    // Assumption: env->tasks is accessible and task_id corresponds to index or map key
    // We assume Task struct has a 'location' field.
    int task_loc = env->tasks[task_id].location;
    int travel_time = get_heuristic_distance(env, agent_loc_when_free, task_loc);
    
    return wait_time + travel_time;
}

// Apply deadline-aware cost modification
// Takes a cost and applies penalties/bonuses based on deadline urgency
static int apply_deadline_cost(int base_cost, SharedEnvironment* env, const DynamicData::Agent& agent, 
                               int task_id, int current_timestep)
{
    int deadline = env->tasks[task_id].deadline;
    
    int urgency_cost = (deadline - current_timestep);
    return base_cost + urgency_cost;
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
        const auto& agent = dynEnv.getAgent(agent_id);
        
        for (int j = 0; j < m; j++) {
            int task_id = free_tasks_list[j];
            
            // Compute basic distance cost
            int basic_cost = compute_cost(env, agent, task_id, current_timestep);
            
            // Apply deadline penalty
            int final_cost = apply_deadline_cost(basic_cost, env, agent, task_id, current_timestep);
            
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

void HungarianScheduler3::hun_schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // Initialize preprocessing pipeline and dynamic environment
    PreprocessPipeline::PreprocessPipeline& preprocessing = PreprocessPipeline::PreprocessPipeline::getInstance();

    if (!preprocessing.isInitialized()) {
        preprocessing.initialize(env->map, env->rows, env->cols);
    }
    
    // Initialize Dynamic Environment (passing the preprocessing singleton)
    DynamicData::DynamicEnvironment& dynEnv = DynamicData::DynamicEnvironment::getInstance();
    dynEnv.initialize(env->rows * env->cols, DynamicData::PLANNING_HORIZON);
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

void HungarianScheduler3::hun_schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    DynamicData::DynamicEnvironment& dynEnv = DynamicData::DynamicEnvironment::getInstance();

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // Build ordered lists from the persistent sets for Hungarian algorithm
    std::vector<int> free_agents_list(free_agents.begin(), free_agents.end());
    std::vector<int> free_tasks_list(free_tasks.begin(), free_tasks.end());

    // Get current timestep
    int current_timestep = env->curr_timestep;

    // STEP 1: Identify near-finish agents
    std::unordered_set<int> near_finish_agents;
    for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id) {
        if (free_agents.find(agent_id) != free_agents.end()) continue;
        
        int finish_time = estimate_agent_finish_time(env, dynEnv.getAgent(agent_id), current_timestep);
        int time_until_free = finish_time - current_timestep;
        
        if (time_until_free > 0 && time_until_free <= LOOKAHEAD_TIMESTEPS) {
            near_finish_agents.insert(agent_id);
        }
    }

    // STEP 2: Combine lists
    std::vector<int> free_agents_list(free_agents.begin(), free_agents.end());
    std::vector<int> free_tasks_list(free_tasks.begin(), free_tasks.end());
    
    std::vector<int> combined_agents_list = free_agents_list;
    for (int agent_id : near_finish_agents) {
        combined_agents_list.push_back(agent_id);
    }
    
    // STEP 3: Deferral Logic (Reserve tasks for incoming super-agents)
    std::unordered_set<int> tasks_for_assignment_set;
    for (int task_id : free_tasks) tasks_for_assignment_set.insert(task_id);

    if (!near_finish_agents.empty() && !free_tasks_list.empty()) {
        auto temp_cost_matrix = build_cost_matrix(env, combined_agents_list, free_tasks_list, current_timestep);
        
        for (int agent_id : near_finish_agents) {
            // Find agent index in combined list
            int agent_idx = -1;
            for(int i=0; i<(int)combined_agents_list.size(); ++i) {
                if(combined_agents_list[i] == agent_id) { agent_idx = i; break; }
            }
            if (agent_idx == -1) continue;

            // Find best task for this busy agent
            int best_task_idx = -1; 
            int best_cost = LARGE_COST;
            for(int j=0; j<(int)free_tasks_list.size(); ++j) {
                if(temp_cost_matrix[agent_idx][j] < best_cost) {
                    best_cost = temp_cost_matrix[agent_idx][j];
                    best_task_idx = j;
                }
            }

            // Check if free agents are terrible at this task
            if (best_task_idx != -1 && best_cost < LARGE_COST) {
                int best_free_cost = LARGE_COST;
                for(int k=0; k<(int)free_agents_list.size(); ++k) {
                    if(temp_cost_matrix[k][best_task_idx] < best_free_cost) {
                        best_free_cost = temp_cost_matrix[k][best_task_idx];
                    }
                }

                // If busy agent is significantly better, remove task from pool
                if (best_free_cost < LARGE_COST) {
                    double improvement = (double)(best_free_cost - best_cost) / (double)best_free_cost;
                    if (improvement > DEFER_THRESHOLD) {
                        tasks_for_assignment_set.erase(free_tasks_list[best_task_idx]);
                    }
                }
            }
        }
    }

    // STEP 4: Final Assignment
    std::vector<int> agents_for_assignment = free_agents_list;
    std::vector<int> tasks_for_assignment(tasks_for_assignment_set.begin(), tasks_for_assignment_set.end());

    if (agents_for_assignment.empty() || tasks_for_assignment.empty()) {
        return; // Nothing to assign
    }

    auto cost_matrix = build_cost_matrix(env, agents_for_assignment, tasks_for_assignment, current_timestep);
    auto padded_matrix = pad_to_square(cost_matrix);
    auto assignment = HungarianAlgorithm(padded_matrix);

    // STEP 5: Apply Results
    int assign_limit = std::min((int)assignment.size(), (int)agents_for_assignment.size());
    for(int i=0; i < assign_limit; ++i) {
        int task_idx = assignment[i];
        if (task_idx >= 0 && task_idx < (int)tasks_for_assignment.size()) {
            int agent_id = agents_for_assignment[i];
            int task_id = tasks_for_assignment[task_idx];

            if (cost_matrix[i][task_idx] < LARGE_COST) {
                proposed_schedule[agent_id] = task_id;

                dynEnv.getAgent(agent_id).assignTask(env->tasks[task_id]);
                
                // Update persistent sets
                free_agents.erase(agent_id);
                free_tasks.erase(task_id);
            }
        }
    }
}