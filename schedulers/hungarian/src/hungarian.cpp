#include "hungarian.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "TrajLNS.h"
#include "search.h"
#include "flow.h"
#include "profiler.h"

static const int LARGE_COST = 1000000000; // avoid INT_MAX arithmetic issues
static const double CONGESTION_WEIGHT = 2.0; // how much to weight congestion vs distance
static const double DEADLINE_WEIGHT = 500.0; // penalty per timestep of deadline urgency
static const int DEADLINE_CRITICAL_THRESHOLD = 50; // timesteps before deadline to consider critical

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
        auto goal_node = DefaultPlanner::astar(env, lns.flow, ht, path, mem, c_loc, loc, &DefaultPlanner::global_neighbors);
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

// Compute cost with deadline urgency factored in
static int compute_deadline_aware_cost(SharedEnvironment* env, int agent_id, int task_id, int current_timestep)
{
    auto deadline_timer = hung_prof.scoped("deadline");
    int makespan = compute_basic_cost(env, agent_id, task_id);
    
    // Add deadline penalty if task has a deadline
    const auto& task = env->task_pool[task_id];
    if (task.t_deadline > 0) {
        int deadline_absolute = task.t_revealed + task.t_deadline;
        int estimated_completion = current_timestep + makespan;
        int slack = deadline_absolute - estimated_completion;
        
        // Penalize tasks that might miss deadline or have tight deadlines
        if (slack < 0) {
            // Already going to miss deadline - very high penalty
            makespan += LARGE_COST / 2;
        } else if (slack < DEADLINE_CRITICAL_THRESHOLD) {
            // Tight deadline - add penalty inversely proportional to slack
            int urgency_penalty = (int)(DEADLINE_WEIGHT * (DEADLINE_CRITICAL_THRESHOLD - slack));
            makespan += urgency_penalty;
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
    const std::vector<int>& free_agents, const std::vector<int>& free_tasks,
    DefaultPlanner::TrajLNS& lns, DefaultPlanner::MemoryPool& mem, int current_timestep)
{
    int n = (int)free_agents.size();
    int m = (int)free_tasks.size();
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
            int agent_id = free_agents[i];
            for (int j = 0; j < max_dim; j++) {
                if (j >= m) {
                    cost_matrix[i][j] = LARGE_COST; // dummy task
                } else {
                    int task_id = free_tasks[j];
                    // Use deadline-aware cost instead of basic cost
                    int deadline_cost = compute_deadline_aware_cost(env, agent_id, task_id, current_timestep);
                    cost_matrix[i][j] = deadline_cost;
                    promising[i].push_back({i, j, deadline_cost});
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
    // (Currently disabled to prioritize speed and deadline awareness over congestion)
    /*
    for (int i = 0; i < n; i++) {
        for (const auto& p : promising[i]) {
            int agent_id = free_agents[p.agent_idx];
            int task_id = free_tasks[p.task_idx];
            // Update cost with flow-aware computation only for promising assignments
            cost_matrix[p.agent_idx][p.task_idx] = compute_flow_aware_cost(env, agent_id, task_id, lns, mem);
        }
    }
    */

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

    // Build ordered lists from env's containers so indices map deterministically
    std::vector<int> free_agents_list(env->new_freeagents.begin(), env->new_freeagents.end());
    std::vector<int> free_tasks_list(env->new_tasks.begin(), env->new_tasks.end());

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

    // Get cost matrix using deadline-aware costs
    auto cost_matrix = build_cost_matrix(env, free_agents_list, free_tasks_list, trajLNS, searchMem, current_timestep);
    auto assignment = HungarianAlgorithm(cost_matrix);

    // Update flow information for assigned paths and map assignments
    for (int i = 0; i < n; ++i) {
        int task_index = (i < (int)assignment.size()) ? assignment[i] : -1;
        if (task_index >= 0 && task_index < m) {
            int agent_id = free_agents_list[i];
            int task_id = free_tasks_list[task_index];
            
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
            
            // Set the assignment
            proposed_schedule[agent_id] = task_id;
        } else {
            int agent_id = free_agents_list[i];
            proposed_schedule[agent_id] = -1;
        }
    }

    g_profiler.report();
}