#include "cluster_pibt.h"

namespace ClusterPIBTPlanner{

    // Global data structures for PIBT planner
    std::vector<int> decision;          // Location -> agent ID mapping
    std::vector<int> prev_decision;     // Previous location -> agent ID mapping
    std::vector<double> p;              // Agent priorities
    std::vector<State> prev_states;     // Previous states for each agent
    std::vector<State> next_states;     // Next states to be computed
    std::vector<int> ids;               // Agent IDs for priority ordering
    std::vector<double> p_copy;         // Copy of initial priorities
    std::vector<bool> occupied;         // Location occupation status
    std::vector<int> tasks;             // Current task/goal for each agent
    std::vector<int> dummy_goals;       // Initial positions as fallback goals
    std::mt19937 mt_rand;               // Random number generator
    SharedEnvironment* global_env;      // Store environment reference
    
    /**
     * @brief Assign deadline-aware priorities to agents
     * 
     * @param env shared environment object
     * 
     * Assigns priorities based on deadline urgency:
     * - Tight deadlines (critical): High priority (base + large bonus)
     * - Manageable deadlines (comfortable): Medium priority (base + moderate bonus)
     * - Likely missed or no deadline: Low priority (base only)
     */
    void assign_deadline_priorities(SharedEnvironment* env) {
        clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
        
        const int DEADLINE_CRITICAL_THRESHOLD = 15; // timesteps before deadline to consider critical
        const double HIGH_PRIORITY_BONUS = 10.0;     // bonus for tight deadlines
        const double MEDIUM_PRIORITY_BONUS = 5.0;    // bonus for manageable deadlines
        
        for (int i = 0; i < env->num_of_agents; i++) {
            // Start with base priority (from p_copy which is the original random priority)
            double base_priority = p_copy[i];
            
            // Check if agent has a task assigned
            if (env->goal_locations[i].empty()) {
                // No goal - keep base priority
                p[i] = base_priority;
                continue;
            }
            
            // Get current task location
            int goal_loc = env->goal_locations[i].front().first;
            int current_loc = env->curr_states[i].location;
            
            // Estimate travel distance using cluster heuristics
            int estimated_distance = clusterHeuristics.getHeuristicDistance(current_loc, goal_loc);
            if (estimated_distance < 0) {
                // Fallback to Manhattan distance
                int curr_x = current_loc / global_env->cols;
                int curr_y = current_loc % global_env->cols;
                int goal_x = goal_loc / global_env->cols;
                int goal_y = goal_loc % global_env->cols;
                estimated_distance = abs(curr_x - goal_x) + abs(curr_y - goal_y);
            }
            
            // Check if there's a deadline for this goal
            bool has_deadline = false;
            int deadline_absolute = -1;
            
            // Find the task associated with this goal to check deadline
            for (int task_id = 0; task_id < env->task_pool.size(); task_id++) {
                const auto& task = env->task_pool[task_id];
                if (task.agent_assigned == i && task.t_deadline > 0) {
                    has_deadline = true;
                    deadline_absolute = task.t_revealed + task.t_deadline;
                    break;
                }
            }
            
            if (!has_deadline) {
                // No deadline - keep base priority
                p[i] = base_priority;
                continue;
            }
            
            // Calculate estimated completion time
            int estimated_completion = env->curr_timestep + estimated_distance;
            int time_until_deadline = deadline_absolute - env->curr_timestep;
            int slack = time_until_deadline - estimated_distance;
            
            // Assign priority based on deadline urgency
            if (slack < DEADLINE_CRITICAL_THRESHOLD && estimated_completion <= deadline_absolute) {
                // Tight deadline - HIGH priority
                p[i] = base_priority + HIGH_PRIORITY_BONUS;
            } else if (estimated_completion <= deadline_absolute) {
                // Manageable deadline - MEDIUM priority
                p[i] = base_priority + MEDIUM_PRIORITY_BONUS;
            } else {
                // Likely to miss deadline - LOW priority (base only)
                p[i] = base_priority;
            }
        }
    }
    
    /**
     * @brief Get cluster-based heuristic distance from location to goal
     * 
     * @param ai agent ID
     * @param target target location
     * @return heuristic distance estimate, or Manhattan distance as fallback
     * 
     * Uses the ClusterHeuristics singleton to compute distances through the cluster graph.
     * Falls back to Manhattan distance if cluster heuristics are unavailable.
     */
    int get_cluster_h(int ai, int target) {
        clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
        
        if (!clusterHeuristics.isInitialized()) {
            // Fallback to Manhattan distance if not initialized
            int target_x = target / global_env->cols;
            int target_y = target % global_env->cols;
            int goal_x = tasks[ai] / global_env->cols;
            int goal_y = tasks[ai] % global_env->cols;
            return abs(target_x - goal_x) + abs(target_y - goal_y);
        }
        
        // Get cluster-based heuristic distance
        int heuristic = clusterHeuristics.getHeuristicDistance(target, tasks[ai]);
        
        // If cluster path not found, fall back to Manhattan distance
        if (heuristic < 0) {
            int target_x = target / global_env->cols;
            int target_y = target % global_env->cols;
            int goal_x = tasks[ai] / global_env->cols;
            int goal_y = tasks[ai] % global_env->cols;
            return abs(target_x - goal_x) + abs(target_y - goal_y);
        }
        
        return heuristic;
    }
    
    /**
     * @brief Initialize the cluster PIBT planner
     * 
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     * 
     * Initializes the cluster heuristics singleton and PIBT data structures
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
        // Initialize basic data structures
        assert(env->num_of_agents != 0);
        
        p.resize(env->num_of_agents);
        decision.resize(env->map.size(), -1);
        prev_decision.resize(env->map.size(), -1);
        prev_states.resize(env->num_of_agents);
        next_states.resize(env->num_of_agents);
        occupied.resize(env->map.size(), false);
        ids.resize(env->num_of_agents);
        tasks.resize(env->num_of_agents);
        dummy_goals.resize(env->num_of_agents);
        
        for (int i = 0; i < ids.size(); i++) {
            ids[i] = i;
        }
        
        // Store environment reference for global access
        global_env = env;
        
        // Initialize cluster heuristics singleton
        clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
        clusterHeuristics.initialize(env->map, env->rows, env->cols, env);
        
        // Initialize random number generator and assign initial priorities
        mt_rand.seed(0);
        srand(0);
        
        std::shuffle(ids.begin(), ids.end(), mt_rand);
        for (int i = 0; i < ids.size(); i++) {
            p[ids[i]] = static_cast<double>(ids.size() - i) / static_cast<double>(ids.size() + 1);
        }
        p_copy = p;
        
        return;
    }

    void plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env){
        
    }
}