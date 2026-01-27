#include "final_scheduler.h"
#include "SharedEnv.h"
#include "preprocess.h"
#include <cmath>
#include <algorithm>
#include <iostream>

static const int LARGE_COST = 10000000; // avoid INT_MAX arithmetic issues
static const bool DEADLINE_AWARE_SCHEDULING = false; // enable deadline aware scheduling
static const int TIGHT_DEADLINE_THRESHOLD = 10; // if naive expectation is within this threshold of deadline, consider it tight

// distance calculation between two locations using heuristics
static int get_heuristic_distance(int loc1, int loc2, int rows, int cols) {
    int c1 = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getClusterId(loc1);
    int c2 = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getClusterId(loc2);

    if (c1 == -1 || c2 == -1) {
        std::cout << "Error: Location not in any cluster in get_heuristic_distance." << std::endl;
        return LARGE_COST; // unreachable
    }

    int a1 = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getAreaId(loc1);
    int a2 = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getAreaId(loc2);

    if (a1 != a2) {
        std::cout << "Error: Locations not in the same area in get_heuristic_distance." << std::endl;
        return LARGE_COST; // unreachable
    }

    if (c1 == c2) {
        // manhattan distance
        return (std::abs((loc1 / cols) - (loc2 / cols)) + std::abs((loc1 % cols) - (loc2 % cols)));
    }

    // different clusters, use inter-cluster heuristics + local BFS in start cluster and goal cluster

    int total_distance = -1;

    // get all combinations of portals in start and goal clusters
    // find minimum distance combination

    const std::vector<PreprocessingPipeline::Cluster>& clusters = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getClusters();
    const PreprocessingPipeline::Cluster& start_cluster = clusters[c1];
    const PreprocessingPipeline::Cluster& goal_cluster = clusters[c2];

    const std::vector<PreprocessingPipeline::Portal>& portals = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortals();

    std::vector<int> estimates;

    for (int i = start_cluster.portal_begin; i <= start_cluster.portal_end; i++) {
        const PreprocessingPipeline::Portal& start_portal = portals[i];
        // chose middle of portal cells as entry point
        int start_portal_loc = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortalCells()[start_portal.cells_begin + start_portal.size / 2];
        int dist_to_start_portal = (std::abs((loc1 / cols) - (start_portal_loc / cols)) + std::abs((loc1 % cols) - (start_portal_loc % cols)));
        for (int j = goal_cluster.portal_begin; j <= goal_cluster.portal_end; j++) {
            const PreprocessingPipeline::Portal& goal_portal = portals[j];
            int goal_portal_loc = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortalCells()[goal_portal.cells_begin + goal_portal.size / 2];
            int dist_from_goal_portal = (std::abs((goal_portal_loc / cols) - (loc2 / cols)) + std::abs((goal_portal_loc % cols) - (loc2 % cols)));
            // get inter-cluster heuristic between start_portal and goal_portal
            int inter_cluster_heuristic = 0;
            if (start_portal.id != goal_portal.id) {
                // distance from start_portal to goal_portal: (goal_portal.id * total_portals + start_portal.id)
                int index = goal_portal.id * portals.size() + start_portal.id;
                inter_cluster_heuristic = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getInterClusterHeuristics()[index];
            }

            int total_estimate = dist_to_start_portal + inter_cluster_heuristic + dist_from_goal_portal;
            estimates.push_back(total_estimate);
        }
    }

    for (int d : estimates) {
        if (d >= 0) {
            if (total_distance == -1 || d < total_distance) {
                total_distance = d;
            }
        }
    }

    return total_distance;
}

// estimate when a busy agent will finish their current task using cluster heuristics
static int estimate_agent_finish_time(DynamicData::Agent& agent, int current_timestep) {
    // if agent is free, return current time
    int task_id = agent.getAssignedTaskID();
    if (task_id == -1) return current_timestep;

    // otherwise check if agent has planned path to final goal of assigned task, if yes, return finish time
    // if not planned, return a large time in the future

    SharedEnvironment* shared_env = DynamicData::DynamicEnvironment::getInstance().getSharedEnvironment();

    if (shared_env->task_pool[task_id].locations.size() - 1 == shared_env->task_pool[task_id].idx_next_loc) { // next is final goal
        int goal_loc = shared_env->task_pool[task_id].locations.back();

        std::vector<DynamicData::LowLevelStep>& ll_plan = agent.getLowLevelPlan();

        if (ll_plan.empty()) {
            return LARGE_COST; // no plan
        }

        for (int i = 0; i < ll_plan.size(); i++) {
            if (ll_plan[i].location == goal_loc) { // found plan to final goal
                return ll_plan[i].t;
            }
        }

        return LARGE_COST; // no plan to final goal found

    } else {
        return LARGE_COST; // not yet planned to final goal
    }
}

// compute cost for assigning agent to task
static int compute_cost(int start_loc,  SharedEnvironment* shared_env, int task_id, int current_timestep) {
    std::vector<int> goal_locs = shared_env->task_pool[task_id].locations;

    int cost = 0;
    int start_loc_copy = start_loc;

    for (int loc : goal_locs) {
        int dist = get_heuristic_distance(start_loc_copy, loc, shared_env->rows, shared_env->cols);
        if (dist >= LARGE_COST) {
            return LARGE_COST;
        }
        cost += dist;
        start_loc_copy = loc;
    }

    return cost;
}

static int apply_deadline_modifier(int task_id, int base_cost, SharedEnvironment* shared_env) {
    if (!DEADLINE_AWARE_SCHEDULING) {
        return base_cost;
    }

    if (base_cost >= LARGE_COST) { // unreachable
        return base_cost;
    }

    const Task& task = shared_env->task_pool[task_id];
    if (task.t_deadline < 0) { // no deadline
        return base_cost;
    }

    if (base_cost < task.t_deadline - TIGHT_DEADLINE_THRESHOLD) { // comfortably within deadline
        return base_cost * 100; // deprioritize
    }

    if (base_cost < task.t_deadline && base_cost >= task.t_deadline - TIGHT_DEADLINE_THRESHOLD) { // meets deadline but tightly
        return base_cost * 10; // slight deprioritization
    }

    return base_cost; // slacking
}

// build cost matrix
static std::vector<std::vector<int>> build_cost_matrix(std::vector<int> agents, std::vector<int> delays, std::vector<int> starts, std::vector<int>& tasks, SharedEnvironment* shared_env, int current_timestep) {
    int n = (int)agents.size();
    int m = (int)tasks.size();

    std::vector<std::vector<int>> cost_matrix(n, std::vector<int>(m, LARGE_COST));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            int cost = compute_cost(starts[i], shared_env, tasks[j], current_timestep);
            int final_cost = cost;
            if (DEADLINE_AWARE_SCHEDULING) {
                final_cost = apply_deadline_modifier(tasks[j], cost, shared_env);
            }
            cost_matrix[i][j] = final_cost + delays[i]; // add delay cost for busy agents
        }
    }

    return cost_matrix;
}

// add dummy rows/columns to make the matrix square for Hungarian algorithm
static std::vector<std::vector<int>> pad_to_square(const std::vector<std::vector<int>>& cost_matrix) {
    if (cost_matrix.empty()) return cost_matrix;

    int n = (int)cost_matrix.size();
    int m = (int)cost_matrix[0].size();
    int max_dim = std::max(n, m);

    std::vector<std::vector<int>> padded(max_dim, std::vector<int>(max_dim, LARGE_COST));

    // copy original costs
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            padded[i][j] = cost_matrix[i][j];
        }
    }

    return padded;
}

void HungarianScheduler3::hun_schedule_initialize(int preprocess_time_limit, SharedEnvironment* env) {
    DynamicData::DynamicEnvironment::getInstance().initialize(env);
}

// Lookahead settings for considering busy agents
static const int LOOKAHEAD_TIMESTEPS = 10; // consider agents finishing within this many timesteps (based on estimated finish times)
static const double DEFER_THRESHOLD = 0.4; // reserve tasks when busy agents are this much better than free agents

// Hungarian algorithm implementation
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

void HungarianScheduler3::hun_schedule_plan(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env) {
    // std::cout << "HungarianScheduler3: Scheduling at timestep " << env->curr_timestep << "\n";

    DynamicData::DynamicEnvironment::getInstance().initializeLiveData(); // if its not the first timestep, this will return immediately
    DynamicData::DynamicEnvironment::getInstance().advanceTimeStep(); // advance internal time and update agents (if first timestep, this will do nothing)

    // 1. prepare data

    std::vector<DynamicData::Agent>& agents = DynamicData::DynamicEnvironment::getInstance().getAgents();

    for (int i = 0; i < env->num_of_agents; ++i) {
        proposed_schedule[i] = -1; // default no assignment
    }

    DynamicData::DynamicEnvironment::getInstance().updateFreeAgents(env->new_freeagents);
    DynamicData::DynamicEnvironment::getInstance().updateTaskPool(env->new_tasks);

    std::unordered_set<int> free_agents = DynamicData::DynamicEnvironment::getInstance().getFreeAgents();
    std::unordered_set<int> task_pool = DynamicData::DynamicEnvironment::getInstance().getTaskPool();
    std::vector<int> free_agents_list(free_agents.begin(), free_agents.end());
    std::vector<int> task_pool_list(task_pool.begin(), task_pool.end());

    int n = (int)free_agents_list.size();
    int m = (int)task_pool_list.size();
    if (n == 0 || m == 0) return; // ??? is this right for the simulation? not sure

    std::vector<int> start_positions;
    std::vector<int> delays;

    for (int agent_id : free_agents_list) {
        start_positions.push_back(DynamicData::DynamicEnvironment::getInstance().getAgent(agent_id).getCurrentLocation());
        delays.push_back(0); // free agents have no delay
    }

    // Steps:
    // 2. collect busy agents that will finish soon

    std::vector<int> near_finish_agents;
    std::vector<int> near_finish_starts;
    std::vector<int> near_finish_delays;

    for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id) {
        if (free_agents.find(agent_id) != free_agents.end()) {
            continue;
        }

        int finish_time = estimate_agent_finish_time(agents[agent_id], env->curr_timestep);
        int time_until_free = finish_time - env->curr_timestep;
        
        if (time_until_free > 0 && time_until_free <= LOOKAHEAD_TIMESTEPS) {
            near_finish_agents.push_back(agent_id);
            near_finish_starts.push_back(env->task_pool[agents[agent_id].getAssignedTaskID()].locations.back());
            near_finish_delays.push_back(time_until_free);
        }
    }

    // 2. combine free agents and near-finish agents

    std::vector<int> combined_agents_list = free_agents_list;
    for (int agent_id : near_finish_agents) {
        combined_agents_list.push_back(agent_id);
    }
    std::vector<int> combined_starts = start_positions;
    for (int start_loc : near_finish_starts) {
        combined_starts.push_back(start_loc);
    }
    std::vector<int> combined_delays = delays;
    for (int delay : near_finish_delays) {
        combined_delays.push_back(delay);
    }

    // 3. run Hungarian algorithm

    auto cost_matrix = build_cost_matrix(combined_agents_list, combined_delays, combined_starts, task_pool_list, env, env->curr_timestep);

    auto padded_matrix = pad_to_square(cost_matrix);

    auto assignment = HungarianAlgorithm(padded_matrix);

    // 4. set proposed schedule based on assignment

    struct Assignment {
        int agent_index;
        int task_index;
        int cost;
    };

    std::vector<Assignment> assignments_made;

    for (int i = 0; i < assignment.size(); ++i) {
        if (i < free_agents_list.size()) {
            // free agent
            auto it = std::find(task_pool_list.begin(), task_pool_list.end(), assignment[i]);
            int task_index = -1;
            if (it != task_pool_list.end()) {
                task_index = it - task_pool_list.begin();
            }
            if (task_index != -1 && cost_matrix[i][task_index] < LARGE_COST) {
                assignments_made.push_back({i, task_index, cost_matrix[i][task_index]});
                proposed_schedule[free_agents_list[i]] = task_pool_list[task_index];
                agents[free_agents_list[i]].assignTask(assignment[i]);
                DynamicData::DynamicEnvironment::getInstance().removeAgentFromFreeSet(free_agents_list[i]);
                DynamicData::DynamicEnvironment::getInstance().removeTaskFromPool(assignment[i]);
            } else {
                assignments_made.push_back({i, -1, LARGE_COST});
                proposed_schedule[i] = -1;
            }
        }
    }

    // 5. assign priorities based on costs
    // lower cost assignments get higher priority

    std::sort(assignments_made.begin(), assignments_made.end(), [](const Assignment& a, const Assignment& b) {
        if (a.cost < LARGE_COST && b.cost < LARGE_COST) {
            return a.cost < b.cost;
        } else if (a.cost < LARGE_COST) {
            return true;
        } else if (b.cost < LARGE_COST) {
            return false;
        } else {
            return false;
        }
    });

    for (int i = 0; i < assignments_made.size(); ++i) {
        int prio = DynamicData::DynamicEnvironment::getInstance().getGlobalNextPriority();
        agents[assignments_made[i].agent_index].setPriority(prio);
        DynamicData::DynamicEnvironment::getInstance().setGlobalNextPriority(prio + 1);
    }

    // debug
    // std::cout << "HungarianScheduler3: Scheduling Complete.\n";
    // for (int i = 0; i < proposed_schedule.size(); ++i) {
    //     if (proposed_schedule[i] != -1) {
    //         std::cout << "  Agent " << i << " assigned to Task " << proposed_schedule[i] << "\n";
    //         for (int loc : env->task_pool[proposed_schedule[i]].locations) {
    //             std::cout << "    Location: " << loc << "\n";
    //         }
    //     } else {
    //         std::cout << "  Agent " << i << " not assigned\n";
    //     }
    // }
}