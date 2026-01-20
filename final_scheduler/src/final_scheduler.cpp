#include "final_scheduler.h"
#include "SharedEnv.h"
#include "preprocess.h"
#include <cmath>
#include <algorithm>
#include <iostream>

static const int LARGE_COST = 10000000; // avoid INT_MAX arithmetic issues

// Distance calculation between two locations using heuristic table
static int get_heuristic_distance(int loc1, int loc2, int rows, int cols) {
    auto& prep = PreprocessingPipeline::PreprocessPipeline::getInstance();

    int c1 = prep.getClusterId(loc1);
    int c2 = prep.getClusterId(loc2);

    if (c1 == -1 || c2 == -1) {
        return LARGE_COST; // unreachable
    }

    int a1 = prep.getAreaId(loc1);
    int a2 = prep.getAreaId(loc2);

    if (a1 != a2) {
        return LARGE_COST; // unreachable
    }

    if (c1 == c2) {
        // manhattan
        return (std::abs((loc1 / cols) - (loc2 / cols)) + std::abs((loc1 % cols) - (loc2 % cols)));
    }

    // different clusters, use inter-cluster heuristics + local BFS in start cluster and goal cluster

    int total_distance = -1;

    // get all combinations of portals in start and goal clusters
    // find minimum distance combination

    const std::vector<PreprocessingPipeline::Cluster>& clusters = prep.getClusters();
    const PreprocessingPipeline::Cluster& start_cluster = clusters[c1];
    const PreprocessingPipeline::Cluster& goal_cluster = clusters[c2];

    const std::vector<PreprocessingPipeline::Portal>& portals = prep.getPortals();

    std::vector<int> estimates;

    for (int i = start_cluster.portal_begin; i <= start_cluster.portal_end; i++) {
        const PreprocessingPipeline::Portal& start_portal = portals[i];
        // chose middle of portal cells as entry point
        int start_portal_loc = prep.getPortalCells()[start_portal.cells_begin + start_portal.size / 2];
        int dist_to_start_portal = (std::abs((loc1 / cols) - (start_portal_loc / cols)) + std::abs((loc1 % cols) - (start_portal_loc % cols)));
        for (int j = goal_cluster.portal_begin; j <= goal_cluster.portal_end; j++) {
            const PreprocessingPipeline::Portal& goal_portal = portals[j];
            int goal_portal_loc = prep.getPortalCells()[goal_portal.cells_begin + goal_portal.size / 2];
            int dist_from_goal_portal = (std::abs((goal_portal_loc / cols) - (loc2 / cols)) + std::abs((goal_portal_loc % cols) - (loc2 % cols)));
            // get inter-cluster heuristic between start_portal and goal_portal
            int inter_cluster_heuristic = 0;
            if (start_portal.id != goal_portal.id) {
                // distance from start_portal to goal_portal: (goal_portal.id * total_portals + start_portal.id)
                int index = goal_portal.id * portals.size() + start_portal.id;
                inter_cluster_heuristic = prep.getInterClusterHeuristics()[index];
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

// Estimate when a busy agent will finish their current task using cluster heuristics
static int estimate_agent_finish_time(const DynamicData::Agent& agent, int current_timestep) {
    // if agent is free, return current time
    int task_id = agent.getAssignedTaskId();
    if (task_id == -1) return current_timestep;

    // otherwise check if agent has planned path to final goal of assigned task, if yes, return finish time
    // if not planned, return a large time in the future

    SharedEnvironment* shared_env = DynamicData::DynamicEnvironment::getInstance().getSharedEnvironment();

    if (shared_env->task_pool[task_id].locations.size() - 1 == shared_env->task_pool[task_id].idx_next_loc) { // next is final goal
        int goal_loc = shared_env->task_pool[task_id].locations.back();

        const std::vector<DynamicData::LowLevelStep>& ll_plan = agent.getLowLevelPlan();

        if (ll_plan.empty()) {
            return LARGE_COST; // no plan
        }

        for (int i = 0; i < ll_plan.size(); i++) {
            if (ll_plan[i].location == goal_loc) { // found plan to final goal
                return ll_plan[i].timestep;
            }
        }

        return LARGE_COST; // no plan to final goal found

    } else {
        return LARGE_COST; // not yet planned to final goal
    }
}

// Compute cost for assigning agent to task
static int compute_cost(int start_loc,  SharedEnvironment* shared_env, int task_id, int current_timestep) {
    std::vector<int> goal_locs = shared_env->task_pool[task_id].locations;

    int cost = 0;

    for (int loc : goal_locs) {
        int dist = get_heuristic_distance(start_loc, loc, shared_env->rows, shared_env->cols);
        if (dist >= LARGE_COST) {
            return LARGE_COST;
        }
        cost += dist;
        start_loc = loc;
    }

    return cost;
}

// Build cost matrix
static std::vector<std::vector<int>> build_cost_matrix(std::vector<int> agents, std::vector<int> delays, std::vector<int> starts, std::vector<int>& tasks, SharedEnvironment* shared_env, int current_timestep) {
    int n = (int)agents.size();
    int m = (int)tasks.size();

    std::vector<std::vector<int>> cost_matrix(n, std::vector<int>(m, LARGE_COST));

    for (int i = 0; i < n; i++) {
        int start_loc = starts[i];
        for (int j = 0; j < m; j++) {
            int cost = compute_cost(start_loc, shared_env, tasks[j], current_timestep);
            cost_matrix[i][j] = cost + delays[i]; // add delay cost for busy agents
            j++;
        }
    }

    return cost_matrix;
}

// Add dummy rows/columns to make the matrix square for Hungarian algorithm
static std::vector<std::vector<int>> pad_to_square(const std::vector<std::vector<int>>& cost_matrix) {
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

void HungarianScheduler3::hun_schedule_initialize(int preprocess_time_limit, SharedEnvironment* env) {
    DynamicData::DynamicEnvironment::getInstance().initialize(env);
}

// Lookahead settings for considering busy agents
static const int LOOKAHEAD_TIMESTEPS = 10; // consider agents finishing within this many timesteps (based on estimated finish times)
static const double DEFER_THRESHOLD = 0.4; // reserve tasks when busy agents are this much better than free agents

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

void HungarianScheduler3::hun_schedule_plan(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env) {
    auto& dyn_env = DynamicData::DynamicEnvironment::getInstance();

    dyn_env.initializeLiveData(); // if its not the first timestep, this will return immediately
    dyn_env.advanceTimeStep(); // advance internal time and update agents (if first timestep, this will do nothing)

    std::cout << "HungarianScheduler3: Scheduling at timestep " << env->curr_timestep << "\n";

    // 1. prepare data

    std::vector<DynamicData::Agent>& agents = dyn_env.getAgents();

    for (int i = 0; i < env->num_of_agents; ++i) {
        proposed_schedule[i] = -1; // default no assignment
    }

    std::unordered_set<int> free_agents = dyn_env.getFreeAgents();
    std::unordered_set<int> task_pool = dyn_env.getTaskPool();

    for (int i = 0; i < env->new_freeagents.size(); ++i) {
        free_agents.insert(env->new_freeagents[i]);
    }

    for (int i = 0; i < env->new_tasks.size(); ++i) {
        task_pool.insert(env->new_tasks[i]);
    }

    std::vector<int> free_agents_list(free_agents.begin(), free_agents.end());
    std::vector<int> task_pool_list(task_pool.begin(), task_pool.end());

    int n = (int)free_agents_list.size();
    int m = (int)task_pool_list.size();
    if (n == 0 || m == 0) return;

    std::vector<int> start_positions;
    std::vector<int> delays;

    for (int agent_id : free_agents_list) {
        start_positions.push_back(dyn_env.getAgent(agent_id).getCurrentLocation());
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
            near_finish_starts.push_back(env->task_pool[agents[agent_id].getAssignedTaskId()].locations.back());
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

    // 3. if busy agents are significantly better for some tasks, defer those tasks

    std::vector<int> tasks_for_assignment = task_pool_list; // start with all tasks

    if (!near_finish_agents.empty()) {
        // Build a temporary cost matrix to evaluate which tasks near-finish agents would want
        auto temp_cost_matrix = build_cost_matrix(combined_agents_list, combined_starts, combined_delays, task_pool_list, env, env->curr_timestep);
        
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

            // find the best task for this busy agent where it can do significantly better than free agents
            // if not the best cost among busy agents though, dont reserve
            std::vector<float> improvements(m, 0.0f); // improvement over other agents
            std::vector<int> best_costs(m, 0);

            for (int j = 0; j < m; ++j) {
                int busy_agent_cost = temp_cost_matrix[agent_idx][j];

                // find best cost among other agents
                for (int k = 0; k < (int)combined_agents_list.size(); ++k) {
                    if (k == agent_idx) continue;
                    int other_cost = temp_cost_matrix[k][j];
                    if (other_cost < LARGE_COST) {
                        float improvement = (float)(other_cost - busy_agent_cost) / (float)other_cost;
                        if (improvement > improvements[j]) {
                            improvements[j] = improvement;
                        }
                    }
                }
            }
            
            // Find the best task for this busy agent
            // int best_task_idx = -1;
            // int best_cost = LARGE_COST;
            // for (int j = 0; j < m; ++j) {
            //     if (temp_cost_matrix[agent_idx][j] < best_cost) {
            //         best_cost = temp_cost_matrix[agent_idx][j];
            //         best_task_idx = j;
            //     }
            // }
            
            // if (best_task_idx >= 0 && best_cost < LARGE_COST) {
            //     int task_id = task_pool_list[best_task_idx];
                
            //     // Check if this busy agent is significantly better than free agents
            //     int best_free_cost = LARGE_COST;
            //     for (int k = 0; k < (int)free_agents_list.size(); ++k) {
            //         if (temp_cost_matrix[k][best_task_idx] < best_free_cost) {
            //             best_free_cost = temp_cost_matrix[k][best_task_idx];
            //         }
            //     }
                
            //     // If busy agent is significantly better, exclude this task from current assignment
            //     // This keeps the task available for when this agent becomes free
            //     if (best_free_cost < LARGE_COST) {
            //         double improvement = (double)(best_free_cost - best_cost) / (double)best_free_cost;
                    
            //         if (improvement > DEFER_THRESHOLD) {
            //             // Exclude this task - it should wait for the busy agent
            //             std::cerr << "Task" << task_id << " deferred for busy agent " << agent_idx << "\n";
            //             tasks_for_assignment_set.erase(task_id);
            //         }
            //     }
            // }
        }
    } else {
        // No near-finish agents, use all free tasks
        for (int task_id : task_pool) {
            tasks_for_assignment_set.insert(task_id);
        }
    }

    // 4. remove deferred tasks task pool

    std::vector<int> agents_for_assignment = free_agents_list;
    std::vector<int> tasks_for_assignment(tasks_for_assignment_set.begin(), tasks_for_assignment_set.end());
    int filtered_m = (int)tasks_for_assignment.size();

    // 5. rebuild cost matrix with only free agents and non-deferred tasks

    std::vector<DynamicData::Agent> agents_for_assignment_vec;
    for (int agent_id : agents_for_assignment) {
        agents_for_assignment_vec.push_back(agents[agent_id]);
    }

    auto cost_matrix = build_cost_matrix(agents_for_assignment_vec, tasks_for_assignment_set, env, env->curr_timestep);

    // 6. run Hungarian algorithm and assign tasks to free agents only

    auto padded_matrix = pad_to_square(cost_matrix);
    auto assignment = HungarianAlgorithm(padded_matrix);

    int assignment_n = (int)agents_for_assignment.size();

    // 7. set proposed schedule based on assignment

    int global_prio = DynamicData::DynamicEnvironment::getInstance().getGlobalNextPriority();

    for (int i = 0; i < assignment_n; ++i) {
        int task_index = (i < (int)assignment.size()) ? assignment[i] : -1;
        if (task_index >= 0 && task_index < filtered_m) {
            int agent_id = agents_for_assignment[i];
            int task_id = tasks_for_assignment[task_index];

            agents[agent_id].setAssignedTaskID(task_id);
            agents[agent_id].setPriority(global_prio);
            DynamicData::DynamicEnvironment::getInstance().incrementGlobalNextPriority();
            
            // Set the assignment and remove from persistent tracking
            proposed_schedule[agent_id] = task_id;

            free_agents.erase(agent_id);
            task_pool.erase(task_id);
            
        } else {
            // No assignment for this agent
            int agent_id = agents_for_assignment[i];
            proposed_schedule[agent_id] = -1;
            // Keep agent in free_agents set for next round
        }
    }
}