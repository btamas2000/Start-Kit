#include "hungarian.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "TrajLNS.h"
#include "search.h"
#include "flow.h"
#include "profiler.h"
#include <unordered_set>
#include <algorithm>

static const int LARGE_COST = 10000000; // avoid INT_MAX arithmetic issues
static const double CONGESTION_WEIGHT = 2.0; // how much to weight congestion vs distance
static const double DEADLINE_WEIGHT = 10.0; // gentle penalty for deadline urgency (not too aggressive)
static const int DEADLINE_CRITICAL_THRESHOLD = 15; // timesteps before deadline to consider critical

// Number of promising assignments to evaluate with flow-aware A* per agent
// Higher values provide better congestion avoidance but increase computation time
// - TOP_K = 1: Fastest, only checks closest task, good for sparse environments
// - TOP_K = 3: Balanced, checks 3 closest tasks, good for moderate congestion
// - TOP_K = 5+: Thorough but slower, better for dense environments with heavy congestion
static const int TOP_K = 1; // Reduced to 1 for speed, focus on deadline prioritization

// Store TrajLNS instance as static to persist between calls
static DefaultPlanner::TrajLNS trajLNS;
static DefaultPlanner::MemoryPool searchMem;
static bool initialized = false;

// Persistent agent and task tracking (like default scheduler)
static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;

// Lookahead settings for considering busy agents
static const int LOOKAHEAD_TIMESTEPS = 5; // Consider agents finishing within this many timesteps
static const double DEFER_THRESHOLD = 0.05; // Defer if busy agent cost is this much better (5% improvement)

// Profiler instances for this module
static custom_utils::profiler::Profiler g_profiler;
static custom_utils::profiler::ModuleProfiler hung_prof(g_profiler, "hungarian");

void HungarianScheduler::hun_schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    DefaultPlanner::init_heuristics(env);
    // Initialize TrajLNS with the environment
    if (!initialized) {
        new (&trajLNS) DefaultPlanner::TrajLNS(env, DefaultPlanner::global_heuristictable, DefaultPlanner::global_neighbors);
        trajLNS.init_mem();
        searchMem.init(env->map.size());
        initialized = true;
    }
    //g_profiler.report();
}

// Compute cost considering both distance and congestion using flow-aware A*
static int compute_flow_aware_cost(SharedEnvironment* env, int agent_id, int task_id,
                                 DefaultPlanner::TrajLNS& lns, DefaultPlanner::MemoryPool& mem)
{
    auto t_start = std::chrono::steady_clock::now();
    // module-level profiler
    auto flow_timer = hung_prof.scoped("flow.total");

    int makespan = 0;
    int c_loc = env->curr_states.at(agent_id).location;
    DefaultPlanner::Traj path;

    // For each location in the task, run flow-aware A* to get there
    for (int loc : env->task_pool[task_id].locations) {
        auto& ht = DefaultPlanner::global_heuristictable[loc];
        if (ht.empty()) {
            DefaultPlanner::init_heuristic(ht, env, loc);
        }
        path.clear();
        // time the A* call specifically
        auto astar_timer = hung_prof.scoped("flow.astar");
        auto goal_node = DefaultPlanner::astar(env, lns.flow, ht, path, mem, c_loc, loc, &lns.neighbors);
        (void)astar_timer; // ensure timer lives until after astar

        makespan += goal_node.op_flow + goal_node.all_vertex_flow; // Add congestion costs
        makespan += path.size() - 1; // Add path length
        c_loc = loc;
    }

    (void)flow_timer; // ensure scoped timer records when function exits
    return makespan;
}

// Compute basic distance-based cost without flow awareness
static int compute_basic_cost(SharedEnvironment* env, int agent_id, int task_id)
{
    auto basic_timer = hung_prof.scoped("basic");
    int makespan = 0;
    int c_loc = env->curr_states.at(agent_id).location;
    for (int loc : env->task_pool[task_id].locations) {
        makespan += DefaultPlanner::get_h(env, c_loc, loc);
        c_loc = loc;
    }
    (void)basic_timer;
    return makespan;
}

// Estimate when a busy agent will finish their current task
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
    
    // Only compute distance for remaining locations (from idx_next_loc onwards)
    for (int i = task.idx_next_loc; i < (int)task.locations.size(); ++i) {
        int loc = task.locations[i];
        remaining_distance += DefaultPlanner::get_h(env, c_loc, loc);
        c_loc = loc;
    }
    
    return current_timestep + remaining_distance;
}

// Compute cost for a busy agent with deadline awareness
// Includes: wait time until free + travel distance + deadline penalties
static int compute_busy_agent_cost_with_deadline(SharedEnvironment* env, int agent_id, int task_id, int current_timestep)
{
    int finish_time = estimate_agent_finish_time(env, agent_id, current_timestep);
    int wait_time = finish_time - current_timestep;
    
    // Compute distance from where agent will be when free to the new task
    // Use the final location of their current task (more accurate than current location)
    int current_task_id = env->curr_task_schedule[agent_id];
    int start_loc = env->curr_states.at(agent_id).location;
    
    if (current_task_id != -1) {
        const auto& current_task = env->task_pool[current_task_id];
        // Agent will be at the last location of their current task when they finish
        if (!current_task.locations.empty()) {
            start_loc = current_task.locations.back();
        }
    }
    
    // Compute distance from final location of current task to new task
    int c_loc = start_loc;
    int task_distance = 0;
    const auto& new_task = env->task_pool[task_id];
    for (int loc : new_task.locations) {
        task_distance += DefaultPlanner::get_h(env, c_loc, loc);
        c_loc = loc;
    }
    
    int base_cost = wait_time + task_distance;
    
    // Apply deadline penalty (same logic as compute_deadline_aware_cost)
    int makespan = base_cost * 1000;
    const auto& task = env->task_pool[task_id];
    if (task.t_deadline > 0) {
        int deadline_absolute = task.t_revealed + task.t_deadline;
        // Agent will start this task at finish_time, not current_timestep
        int estimated_completion = finish_time + task_distance;
        
        if (estimated_completion > deadline_absolute - DEADLINE_CRITICAL_THRESHOLD && estimated_completion <= deadline_absolute) {
            // Tight deadline - gentle linear penalty
            makespan = makespan / 100;
        } else if (estimated_completion < deadline_absolute) {
            makespan = makespan / 1000;
        }
    }
    
    return makespan;
}

// Compute cost with deadline urgency factored in (gentle approach)
static int compute_deadline_aware_cost(SharedEnvironment* env, int agent_id, int task_id, int current_timestep)
{
    auto deadline_timer = hung_prof.scoped("deadline");
    int makespan = compute_basic_cost(env, agent_id, task_id) * 1000;
    
    // Add gentle deadline penalty if task has a deadline
    const auto& task = env->task_pool[task_id];
    if (task.t_deadline > 0) {
        int deadline_absolute = task.t_revealed + task.t_deadline;
        int estimated_completion = current_timestep + makespan;
        
        if (estimated_completion > deadline_absolute - DEADLINE_CRITICAL_THRESHOLD && estimated_completion <= deadline_absolute) {
            // Tight deadline - gentle linear penalty
            makespan = makespan / 100; //
        } else if (estimated_completion < deadline_absolute) {
            makespan = makespan / 1000;
        }
    }
    
    (void)deadline_timer;
    return makespan;
}

// Store promising assignments for later flow-aware evaluation
struct Assignment {
    int agent_idx;
    int task_idx;
    int basic_cost;
    bool operator<(const Assignment& other) const {
        return basic_cost < other.basic_cost;
    }
};

static std::vector<std::vector<int>> build_cost_matrix(SharedEnvironment* env, 
    const std::vector<int>& combined_agents_list, const std::vector<int>& free_tasks_list,
    DefaultPlanner::TrajLNS& lns, DefaultPlanner::MemoryPool& mem, int current_timestep)
{
    int n = (int)combined_agents_list.size();
    int m = (int)free_tasks_list.size();
    int max_dim = std::max(n, m);
    
    // Use smaller TOP_K if we have few tasks to avoid unnecessary computation
    int effective_top_k = std::min(TOP_K, m);

    std::vector<std::vector<int>> cost_matrix(max_dim, std::vector<int>(max_dim, LARGE_COST));
    std::vector<std::vector<Assignment>> promising(n); // Store promising assignments per agent

    // First pass: compute deadline-aware costs and find promising assignments
    for (int i = 0; i < max_dim; i++) {
        if (i >= n) {
            // dummy agent
            for (int j = 0; j < max_dim; j++) cost_matrix[i][j] = LARGE_COST;
        } else {
            int agent_id = combined_agents_list[i];
            
            // Check if this agent is busy (not free)
            bool is_busy = (env->curr_task_schedule[agent_id] != -1);
            
            for (int j = 0; j < max_dim; j++) {
                if (j >= m) {
                    cost_matrix[i][j] = LARGE_COST; // dummy task
                } else {
                    int task_id = free_tasks_list[j];
                    
                    int cost;
                    if (is_busy) {
                        // For busy (near-finish) agents, use busy agent cost with deadline awareness
                        cost = compute_busy_agent_cost_with_deadline(env, agent_id, task_id, current_timestep);
                    } else {
                        // For free agents, use standard deadline-aware cost
                        cost = compute_deadline_aware_cost(env, agent_id, task_id, current_timestep);
                    }
                    
                    cost_matrix[i][j] = cost;
                    promising[i].push_back({i, j, cost});
                }
            }
            // Sort and keep top K promising assignments for this agent
            std::sort(promising[i].begin(), promising[i].end());
            if (promising[i].size() > effective_top_k) {
                promising[i].resize(effective_top_k);
            }
        }
    }

    // Second pass: optionally refine with flow-aware costs for very promising assignments
    for (int i = 0; i < n; i++) {
        for (const auto& p : promising[i]) {
            int agent_id = combined_agents_list[p.agent_idx];
            int task_id = free_tasks_list[p.task_idx];
            // Update cost with flow-aware computation only for promising assignments
            cost_matrix[p.agent_idx][p.task_idx] = compute_flow_aware_cost(env, agent_id, task_id, lns, mem);
        }
    }

    return cost_matrix;
}

std::vector<int> HungarianAlgorithm(const std::vector<std::vector<int>>& cost_matrix)
{
    auto hunalg_timer = hung_prof.scoped("hunalg");
    int n = (int)cost_matrix.size();
    int m = n; // square matrix
    std::vector<int> assignment(n, -1);

    const int INF = LARGE_COST;
    std::vector<int> u(n + 1), v(m + 1), p(m + 1), way(m + 1);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        std::vector<int> minv(m + 1, INF);
        std::vector<char> used(m + 1, false);
        int j0 = 0;
        do {
            used[j0] = true;
            int i0 = p[j0];
            int j1 = 0;
            int delta = INF;
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
    (void)hunalg_timer;
    return assignment;
}

void HungarianScheduler::hun_schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // reset profiler at start of scheduling round
    g_profiler.reset();

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
        g_profiler.report();
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
        auto temp_cost_matrix = build_cost_matrix(env, combined_agents_list, free_tasks_list, trajLNS, searchMem, current_timestep);
        
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
    auto cost_matrix = build_cost_matrix(env, agents_for_assignment, tasks_for_assignment, trajLNS, searchMem, current_timestep);
    auto assignment = HungarianAlgorithm(cost_matrix);

    // STEP 6: Process assignments for free agents only
    int assignment_n = (int)agents_for_assignment.size();
    
    for (int i = 0; i < assignment_n; ++i) {
        int task_index = (i < (int)assignment.size()) ? assignment[i] : -1;
        if (task_index >= 0 && task_index < filtered_m) {
            int agent_id = agents_for_assignment[i];
            int task_id = tasks_for_assignment[task_index];
            
            // This is a free agent - assign the task normally
            // Add paths to flow tracking
            int c_loc = env->curr_states.at(agent_id).location;
            for (int loc : env->task_pool[task_id].locations) {
                DefaultPlanner::Traj path;
                auto& ht = DefaultPlanner::global_heuristictable[loc];
                if (ht.empty()) {
                    DefaultPlanner::init_heuristic(ht, env, loc);
                }
                DefaultPlanner::astar(env, trajLNS.flow, ht, path, searchMem, c_loc, loc, &DefaultPlanner::global_neighbors);
                
                // Try to find an unused traj slot in trajLNS.trajs to reuse
                int tmp_agent = -1;
                for (int ai = 0; ai < (int)trajLNS.trajs.size(); ++ai) {
                    if (trajLNS.trajs[ai].empty()) { tmp_agent = ai; break; }
                }

                if (tmp_agent != -1) {
                    // temporarly store and call existing add_traj helper
                    auto saved = trajLNS.trajs[tmp_agent];
                    trajLNS.trajs[tmp_agent] = path;
                    DefaultPlanner::add_traj(trajLNS, tmp_agent);
                    trajLNS.trajs[tmp_agent] = saved;
                } else {
                    // fallback: directly update flows (same logic as add_traj)
                    if (path.size() > 1) {
                        trajLNS.soc += (int)path.size() - 1;
                        for (size_t j = 1; j < path.size(); ++j) {
                            int loc = path[j];
                            int prev_loc = path[j-1];
                            int diff = loc - prev_loc;
                            int d = DefaultPlanner::get_d(diff, trajLNS.env);
                            trajLNS.flow[prev_loc].d[d] += 1;
                        }
                    }
                }
                c_loc = loc;
            }
            
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

    g_profiler.report();
}