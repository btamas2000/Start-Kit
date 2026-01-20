#include "final_planner.h"
#include "high_level_planner.h"
#include "low_level_planner.h"
#include <queue>


void MyPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    // Initialize preprocessing
    DynamicData::DynamicEnvironment::getInstance().initialize(env);
}

void updateCongestionTrackerWithPlan(std::vector<HighLevelStep> plan) {
    std::vector<PreprocessingPipeline::Portal>& portals = DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortals();
    DynamicData::CongestionTracker& congestion_tracker = DynamicData::DynamicEnvironment::getInstance().getCongestionTracker();

    for (int i = 0; i < static_cast<int>(hl_plan.size()); i++) {
        if (i > 0) {
            if (hl_plan[i - 1].type == DynamicData::WaypointType::PORTAL && hl_plan[i].type == DynamicData::WaypointType::PORTAL) {
                // check if opposite portals
                
                PreprocessingPipeline::Portal& from_portal = portals[hl_plan[i - 1].id];
                PreprocessingPipeline::Portal& to_portal = portals[hl_plan[i].id];

                if (from_portal.opposite_portal_id == to_portal.id) {
                    congestion_tracker.addPortalUsage(
                        from_portal.id,
                        hl_plan[i].arrival_time
                    );
                } else {
                    int cluster_id = portals[hl_plan[i].id].from;
                    congestion_tracker.addClusterUsage(
                        cluster_id,
                        hl_plan[i - 1].arrival_time,
                        hl_plan[i].arrival_time
                    );
                }
            } else if (hl_plan[i - 1].type == DynamicData::WaypointType::LOCATION && hl_plan[i].type == DynamicData::WaypointType::PORTAL) {
                // location to portal
                int cluster_id = portals[hl_plan[i].id].from;
                congestion_tracker.addClusterUsage(
                    cluster_id,
                    hl_plan[i - 1].arrival_time,
                    hl_plan[i].arrival_time
                );
            } else if (hl_plan[i - 1].type == DynamicData::WaypointType::PORTAL && hl_plan[i].type == DynamicData::WaypointType::LOCATION) {
                // portal to location
                int cluster_id = portals[hl_plan[i - 1].id].from;
                congestion_tracker.addClusterUsage(
                    cluster_id,
                    hl_plan[i - 1].arrival_time,
                    hl_plan[i].arrival_time
                );
            }
        }
    }
}

void MyPlanner::plan(int time_limit, std::vector<Action> & actions,  SharedEnvironment* env) {

    TimePoint start_time = std::chrono::steady_clock::now();
    TimePoint end_time = start_time + std::chrono::milliseconds(time_limit);

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
        if (std::chrono::steady_clock::now() > end_time * 0.9) {
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

            agents[agent_id].setLowLevelPlan(ll_result.second);
            agents[agent_id].setLowLevelReplanNeeded(false);

            // reserve path in reservation table
            DynamicData::ReservationTable& reservation_table = DynamicData::DynamicEnvironment::getInstance().getReservationTable();
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
        if (ll_planned_until - ll_step_index < DynamicData::FILL_PLAN_AFTER) {
            std::vector<DynamicData::LowLevelStep>& ll_plan = agents[agent_id].getLowLevelPlan();
            LowLevelStep& last_step = ll_plan[ll_planned_until];

            std::pair<bool, std::vector<DynamicData::LowLevelStep>> ll_extension_result = ll_planner.extendLowLevelPath(agent_id, last_step, extend_threshold);

            if (ll_extension_result.first) {
                // remove old path reservation with last step as start
                DynamicData::ReservationTable& reservation_table = DynamicData::DynamicEnvironment::getInstance().getReservationTable();
                reservation_table.releasePath(std::vector<DynamicData::LowLevelStep>(ll_plan.begin() + ll_planned_until, ll_plan.end()));

                // modify last planned element data
                last_step = LowLevelStep(
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

    

    return;
}