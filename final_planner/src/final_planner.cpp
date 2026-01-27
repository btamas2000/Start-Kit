#include "final_planner.h"
#include "high_level_planner.h"
#include "low_level_planner.h"
#include <queue>


void MyPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    DynamicData::DynamicEnvironment::getInstance().initialize(env);
}

void updateCongestionTrackerWithPlan(std::vector<DynamicData::HighLevelStep> plan) {
    const std::vector<PreprocessingPipeline::Portal>& portals = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortals();
    DynamicData::CongestionTracker& congestion_tracker = DynamicData::DynamicEnvironment::getInstance().getCongestionTracker();
    const std::vector<int>& cluster_map = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getClusterMap();
    int current_time = DynamicData::DynamicEnvironment::getInstance().getCurrentTime();

    if (plan.size() == 0) {
        return;
    } else if (plan.size() == 1) {
        if (plan[0].type == DynamicData::WaypointType::PORTAL) {
            const PreprocessingPipeline::Portal& portal = portals[plan[0].id];
            congestion_tracker.addClusterUsage(
                portal.from,
                current_time,
                plan[0].arrival_time_est
            );
        } else if (plan[0].type == DynamicData::WaypointType::LOCATION) {
            int cluster_id = cluster_map[plan[0].id];
            congestion_tracker.addClusterUsage(
                cluster_id,
                current_time,
                plan[0].arrival_time_est
            );
        }
    } else {
        for (int i = 0; i < static_cast<int>(plan.size()); i++) {
            if (i == 0) {
                // first step
                if (plan[i].type == DynamicData::WaypointType::PORTAL) {
                    const PreprocessingPipeline::Portal& portal = portals[plan[i].id];
                    congestion_tracker.addClusterUsage(
                        portal.from,
                        current_time,
                        plan[i].arrival_time_est
                    );
                } else if (plan[i].type == DynamicData::WaypointType::LOCATION) {
                    int cluster_id = cluster_map[plan[i].id];
                    congestion_tracker.addClusterUsage(
                        cluster_id,
                        current_time,
                        plan[i].arrival_time_est
                    );
                }
            } else {
                if (plan[i - 1].type == DynamicData::WaypointType::PORTAL && plan[i].type == DynamicData::WaypointType::PORTAL) {
                    // check if opposite portals
                    
                    const PreprocessingPipeline::Portal& from_portal = portals[plan[i - 1].id];
                    const PreprocessingPipeline::Portal& to_portal = portals[plan[i].id];

                    if (from_portal.opposite_portal_id == to_portal.id) {
                        congestion_tracker.addPortalUsage(
                            from_portal.id,
                            plan[i - 1].arrival_time_est
                        );
                    } else {
                        int cluster_id = portals[plan[i].id].from;
                        congestion_tracker.addClusterUsage(
                            cluster_id,
                            plan[i - 1].arrival_time_est,
                            plan[i].arrival_time_est
                        );
                    }
                } else if (plan[i - 1].type == DynamicData::WaypointType::LOCATION && plan[i].type == DynamicData::WaypointType::PORTAL) {
                    // location to portal
                    int cluster_id = portals[plan[i].id].from;
                    congestion_tracker.addClusterUsage(
                        cluster_id,
                        plan[i - 1].arrival_time_est,
                        plan[i].arrival_time_est
                    );
                } else if (plan[i - 1].type == DynamicData::WaypointType::PORTAL && plan[i].type == DynamicData::WaypointType::LOCATION) {
                    // portal to location
                    int cluster_id = portals[plan[i - 1].id].from;
                    congestion_tracker.addClusterUsage(
                        cluster_id,
                        plan[i - 1].arrival_time_est,
                        plan[i].arrival_time_est
                    );
                } else if (plan[i - 1].type == DynamicData::WaypointType::LOCATION && plan[i].type == DynamicData::WaypointType::LOCATION) {
                    // location to location
                    int cluster_id = cluster_map[plan[i - 1].id];
                    congestion_tracker.addClusterUsage(
                        cluster_id,
                        plan[i - 1].arrival_time_est,
                        plan[i].arrival_time_est
                    );
                }
            }
        }
    }
}

bool goalReachedInPlan(const std::vector<DynamicData::LowLevelStep>& plan, int task_id, SharedEnvironment* env) {
    if (env->task_pool[task_id].locations.size() - 1 == env->task_pool[task_id].idx_next_loc) { // next is final goal
        if (plan.empty()) {
            return false; // no plan
        }
        int goal_loc = env->task_pool[task_id].locations.back();
        for (const auto& step : plan) {
            if (step.location == goal_loc) { // found plan to final goal
                return true;
            }
        }
    }
    return false; // no plan to final goal found
}

void MyPlanner::plan(int time_limit, std::vector<Action> & actions,  SharedEnvironment* env) {

    TimePoint start_time = std::chrono::steady_clock::now();
    auto duration_limit = std::chrono::milliseconds(time_limit);

    auto cutoff_time = start_time + std::chrono::duration_cast<std::chrono::milliseconds>(duration_limit * 0.9);

    std::cout << "MyPlanner: Planning at timestep " << env->curr_timestep << "\n";
    
    actions.clear();
    actions.resize(env->num_of_agents, Action::W); // default to Wait

    if (!DynamicData::DynamicEnvironment::getInstance().isLiveDataInitialized()) {
        DynamicData::DynamicEnvironment::getInstance().initializeLiveData();
    }

    std::vector<DynamicData::Agent>& agents = DynamicData::DynamicEnvironment::getInstance().getAgents();
    std::vector<DynamicData::Agent> agents_copy = agents; // make a copy to avoid sorting original

    std::sort(agents_copy.begin(), agents_copy.end(), 
              [](const DynamicData::Agent& a, const DynamicData::Agent& b) {
                  return a.getPriority() < b.getPriority();
              });

    DynamicData::LowLevelPlanner ll_planner;
    DynamicData::HighLevelPlanner hl_planner;

    for (const auto& agent : agents_copy) {
        if (std::chrono::steady_clock::now() > cutoff_time) {
            std::cout << "MyPlanner: Time limit reached during planning.\n";
            break; // Time limit reached
        }

        int agent_id = agent.getAgentID();

        if (agent.isHighLevelReplanNeeded() || agent.isLowLevelReplanNeeded()) {
            // plan both
            std::vector<DynamicData::HighLevelStep> hl_plan = hl_planner.planHighLevelPath(agent_id);
            if (hl_plan.empty()) {
                std::cout << "MyPlanner: High-level planning failed for agent " << agent_id << "\n";
                continue;
            }

            agents[agent_id].setHighLevelPlan(hl_plan);
            agents[agent_id].setHighLevelReplanNeeded(false);

            // update congestion tracker
            updateCongestionTrackerWithPlan(hl_plan);

            std::pair<bool, std::vector<DynamicData::LowLevelStep>> ll_result = ll_planner.planLowLevelPath(agent_id);
            if (!ll_result.first) {
                std::cout << "MyPlanner: Low-level planning failed for agent " << agent_id << "\n";
                continue;
            }

            std::cout << "MyPlanner: Low-level planning succeeded for agent " << agent_id << " with plan length " << ll_result.second.size() << "\n";

            DynamicData::ReservationTable& reservation_table = DynamicData::DynamicEnvironment::getInstance().getReservationTable();

            std::vector<DynamicData::LowLevelStep>& old_ll_plan = agents[agent_id].getLowLevelPlan();
            // remove old path reservation
            int ll_step_index = agents[agent_id].getLLStepIndex();
            reservation_table.releasePath(std::vector<DynamicData::LowLevelStep>(old_ll_plan.begin() + ll_step_index, old_ll_plan.end()));

            agents[agent_id].setLowLevelPlan(ll_result.second);
            agents[agent_id].setLowLevelReplanNeeded(false);

            // reserve path in reservation table
            reservation_table.reservePath(ll_result.second);
        }

        // set action for current timestep
        std::pair<bool, DynamicData::LowLevelStep> current_step = agents[agent_id].getCurrentLowLevelStep();
        if (current_step.first) {
            actions[agent_id] = current_step.second.nextAction;
        } else {
            std::cout << "MyPlanner: No current low-level step for agent " << agent_id << "\n";
            actions[agent_id] = Action::W; // no valid step, wait
        }

        // check if ll plan needs extension
        int ll_step_index = agents[agent_id].getLLStepIndex();
        int ll_planned_until = agents[agent_id].getLLPlannedUntil();
        int extend_threshold = (ll_planned_until / DynamicData::PLANNING_HORIZON) * DynamicData::PLANNING_HORIZON + DynamicData::PLANNING_HORIZON - 1;
        bool goal_reached = goalReachedInPlan(agents[agent_id].getLowLevelPlan(), agents[agent_id].getAssignedTaskID(), env);
        if (!goal_reached && ll_planned_until - ll_step_index < DynamicData::FILL_PLAN_AFTER) {
            std::vector<DynamicData::LowLevelStep>& ll_plan = agents[agent_id].getLowLevelPlan();
            DynamicData::LowLevelStep& last_step = ll_plan[ll_planned_until];

            std::pair<bool, std::vector<DynamicData::LowLevelStep>> ll_extension_result = ll_planner.extendLowLevelPath(agent_id, last_step, extend_threshold);

            if (ll_extension_result.first) {
                // remove old path reservation with last step as start
                DynamicData::ReservationTable& reservation_table = DynamicData::DynamicEnvironment::getInstance().getReservationTable();
                reservation_table.releasePath(std::vector<DynamicData::LowLevelStep>(ll_plan.begin() + ll_planned_until, ll_plan.end()));

                // modify last planned element data
                last_step = DynamicData::LowLevelStep(
                    ll_extension_result.second[0].t,
                    ll_extension_result.second[0].location,
                    ll_extension_result.second[0].orientation,
                    ll_extension_result.second[0].nextAction,
                    ll_extension_result.second[0].placeholder_step,
                    ll_extension_result.second[0].hl_step_index
                );
                agents[agent_id].extendLowLevelPlan(std::vector<DynamicData::LowLevelStep>(ll_extension_result.second.begin() + 1, ll_extension_result.second.end()), false);

                // reserve new path in reservation table
                reservation_table.reservePath(ll_extension_result.second);
            } else {
                std::cout << "MyPlanner: Low-level plan extension failed for agent " << agent_id << "\n";
                agents[agent_id].setLowLevelReplanNeeded(true);
            }
        }
    }

    // check if moves are valid
    // if not, set replanning flags and revert to wait

    bool valid = false;

    std::vector<int> prev_state(env->rows * env->cols, -2);
    std::vector<int> next_state(env->rows * env->cols, -2);

    for (int i = 0; i < env->rows * env->cols; i++) {
        if (env->map[i] != 1) { // not obstacle
            prev_state[i] = -1;
            next_state[i] = -1;
        }
    }

    for (int i = 0; i < env->num_of_agents; i++) {
        int loc = env->curr_states[i].location;
        prev_state[loc] = i;
    }

    std::unordered_set<int> reverted_agents;

    for (int i = 0; i < env->num_of_agents; i++) {
        int loc = env->curr_states[i].location;
        Action action = actions[i];
        int next_loc = loc;

        if (action == Action::FW) {
            // move forward
            int orientation = env->curr_states[i].orientation;
            if (orientation == 0) {
                // east
                next_loc = loc + 1;
            } else if (orientation == 1) {
                // south
                next_loc = loc + env->cols;
            } else if (orientation == 2) {
                // west
                next_loc = loc - 1;
            } else {
                // north
                next_loc = loc - env->cols;
            }
        } else {
            // other actions do not change location
            next_loc = loc;
        }

        if (next_state[next_loc] == -1) {
            // free to move
            next_state[next_loc] = i;
        } else {
            // conflict detected
            // higher priority agent keeps move, lower priority agents revert to wait
            int prio1 = agents[i].getPriority();
            int prio2 = agents[next_state[next_loc]].getPriority();
            if (prio1 < prio2) {
                // agent i has higher priority, revert other agent
                reverted_agents.insert(next_state[next_loc]);
                next_state[next_loc] = i;
            } else {
                // agent i has lower priority, revert self
                reverted_agents.insert(i);
            }
        }
    }

    valid = (reverted_agents.size() == 0);

    while(!valid) {
        std::unordered_set<int> new_reverted_agents;
        for (int agent_id : reverted_agents) {
            actions[agent_id] = Action::W;
            agents[agent_id].setLowLevelReplanNeeded(true);
            agents[agent_id].setHighLevelReplanNeeded(true);
            // check if revertion causes conflicts for other agents
            if (next_state[env->curr_states[agent_id].location] != -1) {
                int other_agent_id = next_state[env->curr_states[agent_id].location];
                new_reverted_agents.insert(other_agent_id);
            }
            next_state[env->curr_states[agent_id].location] = agent_id;
        }
        reverted_agents = new_reverted_agents;
        valid = (reverted_agents.size() == 0);
    }

    std::cout << "MyPlanner: Debugging Reservation Table at timestep " << env->curr_timestep << ":\n";
    DynamicData::DynamicEnvironment::getInstance().getReservationTable().printTablesAtTimestep(env->curr_timestep);

    return;
}