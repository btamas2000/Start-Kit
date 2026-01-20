#include "low_level_planner.h"
#include <iostream>
#include <queue>
#include <cmath>
#include <algorithm>

namespace DynamicData {
    LowLevelPlanner::LowLevelPlanner() {
        node_pool_.clear();
    }

    void LowLevelPlanner::clearNodePool() {
        for (auto node : node_pool_) {
            delete node;
        }
        node_pool_.clear();
    }

    float LowLevelPlanner::manhattanDistance(int r1, int c1, int r2, int c2) {
        return static_cast<float>(std::abs(r1 - r2) + std::abs(c1 - c2));
    }

    float LowLevelPlanner::getGScore(LLNode* from_node, int to_location, int to_orientation) {
        DynamicEnvironment& dyn_env = DynamicEnvironment::getInstance();
        SharedEnvironment* shared_env = dyn_env.getSharedEnvironment();

        // basic cost is previous g score + 1 for moving to next location (applied later)
        float g_score = from_node->g_score;

        // apply penalty if there is one

        bool soft_reserved = dyn_env.getReservationTable().isCellSoftReserved(to_location, from_node->timestep + 1);

        if (soft_reserved) {
            g_score += (1.0f * SOFT_RESERVATION_PENALTY);
        } else {
            g_score += 1.0f;
        }

        return g_score;
    }

    float LowLevelPlanner::getHScore(int location, bool cluster_crossing, HighLevelStep& hl_step) {
        DynamicEnvironment& dyn_env = DynamicEnvironment::getInstance();
        SharedEnvironment* shared_env = dyn_env.getSharedEnvironment();

        if (hl_step.type == WaypointType::LOCATION) {
            // heuristic is manhattan distance to the location
            return manhattanDistance(
                location / shared_env->cols,
                location % shared_env->cols,
                hl_step.id / shared_env->cols,
                hl_step.id % shared_env->cols
            );
        } else if (hl_step.type == WaypointType::PORTAL && !cluster_crossing) {
            // use portal distance to portal
            std::vector<PreprocessingPipeline::Portal>& portals = dyn_env.getPreprocessing().getPortals();
            PreprocessingPipeline::Portal& portal = portals[hl_step.id];

            float distance = static_cast<float>(dyn_env.getPreprocessing().getPortalDistances()[
                portal.distances_index * shared_env->rows * shared_env->cols + location
            ]);

            return distance;
        } else if (hl_step.type == WaypointType::PORTAL && cluster_crossing) {
            if (dyn_env.getPreprocessing().getPortals()[hl_step.id].from == dyn_env.getPreprocessing().getClusterId(location)) {
                return 0.0f; // already at the portal
            } else {
                PreprocessingPipeline::Portal& opposite_portal = dyn_env.getPreprocessing().getPortals()[
                    dyn_env.getPreprocessing().getPortals()[hl_step.id].opposite_portal_id
                ];

                if (opposite_portal.from == PreprocessingPipeline::PreprocessPipeline::getInstance().getClusterId(location)) {
                    // use portal distance to opposite portal but increment by 1 to account for crossing
                    float distance = static_cast<float>(dyn_env.getPreprocessing().getPortalDistances()[
                        opposite_portal.distances_index * shared_env->rows * shared_env->cols + location
                    ]);

                    return distance + 1.0f;
                } else {
                    // should not happen
                    std::cout << "Error: Location not in either portal cluster in getHScore." << std::endl;
                    return LARGE_COST;
                }
            }
        } else {
            // should not happen
            std::cout << "Error: Invalid waypoint type in getHScore." << std::endl;
            return LARGE_COST;
        }
    }

    std::vector<LowLevelStep> LowLevelPlanner::reconstructPath(LLNode* goal_node) {
        std::vector<LowLevelStep> path;
        LLNode* current = goal_node;

        // careful with action logic as LowLevelStep stores the next action taken so we shift by one

        Action last_action = Action::NA;

        while (current != nullptr) {
            LowLevelStep step;
            step.location = current->location;
            step.orientation = current->orientation;
            step.timestep = current->timestep;
            step.action_taken = last_action;
            path.push_back(step);
            last_action = current->action_taken;
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    std::pair<bool, std::vector<LowLevelStep>> LowLevelPlanner::planLowLevelPath(int agent_id) {
        bool found = false;
        std::vector<LowLevelStep> path;

        clearNodePool();

        DynamicEnvironment& dyn_env = DynamicEnvironment::getInstance();
        Agent& agent = dyn_env.getAgents()[agent_id];
        SharedEnvironment* shared_env = dyn_env.getSharedEnvironment();

        if (agent.getHighLevelPlan().empty()) {
            std::cout << "Error: Agent " << agent_id << " has no high-level plan for low-level planning." << std::endl;
            return std::make_pair(false, path);
        }

        // A* search from current location to first high-level step

        std::priority_queue<LLNode*, std::vector<LLNode*>, LLNodeCompare> open_set;
        std::unordered_set<LLNode*, LLNodeHash, LLNodeEqual> closed_set;



        // create start node
        LLNode* start_node = new LLNode(
            agent.getCurrentLocation(),
            agent.getCurrentOrientation(),
            dyn_env.getCurrentTime(),
            0,
            0.0f,
            getHScore(agent.getCurrentLocation(), false, agent.getHighLevelPlan()[0]),
            nullptr,
            Action::NA
        );

        open_set.push(start_node);
        node_pool_.push_back(start_node);

        std::vector<HighLevelStep>& hl_plan = agent.getHighLevelPlan();

        int start_time = dyn_env.getCurrentTime();

        while (!open_set.empty()) {
            LLNode* current_node = open_set.top();
            open_set.pop();
            
            closed_set.insert(current_node);

            if (closed_set.find(current_node) != closed_set.end()) {
                continue; // already processed
            }

            // check if reached current high-level step goal

            int current_hl_index = current_node->hl_step_index;

            // skip to next relevant high-level step if already reached subgoals
            for (int i = current_hl_index; i < static_cast<int>(hl_plan.size()); i++) {
                if (hl_plan[i].type == WaypointType::LOCATION) {
                    if (current_node->location == hl_plan[i].id) {
                        current_hl_index += 1;
                    }
                } else if (hl_plan[i].type == WaypointType::PORTAL) {
                    if (hl_plan[i].id == dyn_env.getPreprocessing().getPortalMap()[current_node->location]) {
                        current_hl_index += 1;
                    }
                }
            }

            // check if agent has reached the final high-level step
            if (current_hl_index >= static_cast<int>(hl_plan.size())) {
                found = true;
                path = reconstructPath(current_node);
                break;
            }

            // check if reached the limit of planning time

            if (current_node->timestep - start_time >= PLANNING_HORIZON - 1) {
                // reached planning horizon
                found = true;
                path = reconstructPath(current_node);
                break;
            }

            // generate neighbors

            ReservationTable& reservation_table = dyn_env.getReservationTable();

            // possible actions: FW, CR, CCR, W
            // start check with W, CR and CCR as that can only be done if location in t+1 is not blocked

            Projections current_cell_projections = dyn_env.getPreprocessing().getProjectionsAtLocation(current_node->location);

            if (!reservation_table.isCellReserved(current_node->location, current_node->timestep + 1)) {
                // wait action

                if (!(current_node->orientation == 0 && current_cell_projections.projected_west) &&
                    !(current_node->orientation == 1 && current_cell_projections.projected_north) &&
                    !(current_node->orientation == 2 && current_cell_projections.projected_east) &&
                    !(current_node->orientation == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node = new LLNode(
                        current_node->location,
                        current_node->orientation,
                        current_node->timestep + 1,
                        current_hl_index,
                        getGScore(current_node, current_node->location, current_node->orientation),
                        getHScore(current_node->location, false, hl_plan[current_hl_index]),
                        current_node,
                        Action::W
                    );

                    if (closed_set.find(neighbor_node) == closed_set.end()) {
                        open_set.push(neighbor_node);
                        node_pool_.push_back(neighbor_node);
                    } else {
                        delete neighbor_node; // already in closed set, discard
                    }
                }

                // CR and CCR actions

                int new_orientation_cr = (current_node->orientation + 1) % 4;

                if (!(new_orientation_cr == 0 && current_cell_projections.projected_west) &&
                    !(new_orientation_cr == 1 && current_cell_projections.projected_north) &&
                    !(new_orientation_cr == 2 && current_cell_projections.projected_east) &&
                    !(new_orientation_cr == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node_cr = new LLNode(
                        current_node->location,
                        new_orientation_cr,
                        current_node->timestep + 1,
                        current_hl_index,
                        getGScore(current_node, current_node->location, new_orientation_cr),
                        getHScore(current_node->location, false, hl_plan[current_hl_index]),
                        current_node,
                        Action::CR
                    );

                    if (closed_set.find(neighbor_node_cr) == closed_set.end()) {
                        open_set.push(neighbor_node_cr);
                        node_pool_.push_back(neighbor_node_cr);
                    } else {
                        delete neighbor_node_cr; // already in closed set, discard
                    }
                }

                int new_orientation_ccr = (current_node->orientation + 3) % 4;
                if (!(new_orientation_ccr == 0 && current_cell_projections.projected_west) &&
                    !(new_orientation_ccr == 1 && current_cell_projections.projected_north) &&
                    !(new_orientation_ccr == 2 && current_cell_projections.projected_east) &&
                    !(new_orientation_ccr == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node_ccr = new LLNode(
                        current_node->location,
                        new_orientation_ccr,
                        current_node->timestep + 1,
                        current_hl_index,
                        getGScore(current_node, current_node->location, new_orientation_ccr),
                        getHScore(current_node->location, false, hl_plan[current_hl_index]),
                        current_node,
                        Action::CCR
                    );

                    if (closed_set.find(neighbor_node_ccr) == closed_set.end()) {
                        open_set.push(neighbor_node_ccr);
                        node_pool_.push_back(neighbor_node_ccr);
                    } else {
                        delete neighbor_node_ccr; // already in closed set, discard
                    }
                }
            }

            // FW action

            int cell_forward_location = -1;

            if (current_node->orientation == 0) {
                // east
                cell_forward_location = current_node->location + 1;
            } else if (current_node->orientation == 1) {
                // south
                cell_forward_location = current_node->location + shared_env->cols;
            } else if (current_node->orientation == 2) {
                // west
                cell_forward_location = current_node->location - 1;
            } else {
                // north
                cell_forward_location = current_node->location - shared_env->cols;
            }

            if (cell_forward_location >= 0 && cell_forward_location < shared_env->rows * shared_env->cols) {
                if (!reservation_table.isCellReserved(cell_forward_location, current_node->timestep + 1)) {
                    Projections forward_cell_projections = dyn_env.getPreprocessing().getProjectionsAtLocation(cell_forward_location);

                    if (!(current_node->orientation == 0 && forward_cell_projections.projected_west) &&
                        !(current_node->orientation == 1 && forward_cell_projections.projected_north) &&
                        !(current_node->orientation == 2 && forward_cell_projections.projected_east) &&
                        !(current_node->orientation == 3 && forward_cell_projections.projected_south)) {

                        LLNode* neighbor_node_fw = new LLNode(
                            cell_forward_location,
                            current_node->orientation,
                            current_node->timestep + 1,
                            current_hl_index,
                            getGScore(current_node, cell_forward_location, current_node->orientation),
                            getHScore(cell_forward_location, false, hl_plan[current_hl_index]),
                            current_node,
                            Action::FW
                        );

                        if (closed_set.find(neighbor_node_fw) == closed_set.end()) {
                            open_set.push(neighbor_node_fw);
                            node_pool_.push_back(neighbor_node_fw);
                        } else {
                            delete neighbor_node_fw; // already in closed set, discard
                        }
                    }
                }
            }
        }

        clearNodePool();
        return std::make_pair(found, path);
    }

    std::pair<bool, std::vector<LowLevelStep>> LowLevelPlanner::extendLowLevelPath(int agent_id, const LowLevelStep& last_step, int extend_until_timestep) {
        bool found = false;
        std::vector<LowLevelStep> path;

        clearNodePool();

        DynamicEnvironment& dyn_env = DynamicEnvironment::getInstance();
        Agent& agent = dyn_env.getAgents()[agent_id];
        SharedEnvironment* shared_env = dyn_env.getSharedEnvironment();

        if (agent.getHighLevelPlan().empty()) {
            std::cout << "Error: Agent " << agent_id << " has no high-level plan for low-level planning." << std::endl;
            return std::make_pair(false, path);
        }

        // A* search from last step to extend_until_timestep

        std::priority_queue<LLNode*, std::vector<LLNode*>, LLNodeCompare> open_set;
        std::unordered_set<LLNode*, LLNodeHash, LLNodeEqual> closed_set;
        std::vector<PreprocessingPipeline::Portal>& portals = dyn_env.getPreprocessing().getPortals();

        std::vector<HighLevelStep>& hl_plan = agent.getHighLevelPlan();
        bool cluster_crossing = false;
        if (last_step.hl_step_index > 0) {
            if (hl_plan[last_step.hl_step_index - 1].type == WaypointType::PORTAL &&
                hl_plan[last_step.hl_step_index].type == WaypointType::PORTAL) {
                    if (portals[hl_plan[last_step.hl_step_index - 1].id].opposite_portal_id == hl_plan[last_step.hl_step_index].id) {
                        cluster_crossing = true;
                    }
            }
        }

        // create start node
        LLNode* start_node = new LLNode(
            last_step.location,
            last_step.orientation,
            last_step.timestep,
            last_step.hl_step_index,
            0.0f,
            getHScore(last_step.location, cluster_crossing, hl_plan[last_step.hl_step_index]),
            nullptr,
            Action::NA
        );

        open_set.push(start_node);
        node_pool_.push_back(start_node);

        while (!open_set.empty()) {
            LLNode* current_node = open_set.top();
            open_set.pop();
            
            closed_set.insert(current_node);

            if (closed_set.find(current_node) != closed_set.end()) {
                continue; // already processed
            }

            int current_hl_index = current_node->hl_step_index;

            // skip to next relevant high-level step if already reached subgoals
            for (int i = current_hl_index; i < static_cast<int>(hl_plan.size()); i++) {
                if (hl_plan[i].type == WaypointType::LOCATION) {
                    if (current_node->location == hl_plan[i].id) {
                        current_hl_index += 1;
                    }
                } else if (hl_plan[i].type == WaypointType::PORTAL) {
                    if (hl_plan[i].id == dyn_env.getPreprocessing().getPortalMap()[current_node->location]) {
                        current_hl_index += 1;
                    }
                }
            }

            // check if agent has reached the final high-level step
            if (current_hl_index >= static_cast<int>(hl_plan.size())) {
                found = true;
                path = reconstructPath(current_node);
                break;
            }

            // check if reached the limit of planning time
            if (current_node->timestep >= extend_until_timestep) {
                // reached extend until timestep
                found = true;
                path = reconstructPath(current_node);
                break;
            }

            // generate neighbors

            ReservationTable& reservation_table = dyn_env.getReservationTable();

            // possible actions: FW, CR, CCR, W
            // start check with W, CR and CCR as that can only be done if location in t+1 is not blocked

            Projections current_cell_projections = dyn_env.getPreprocessing().getProjectionsAtLocation(current_node->location);

            if (!reservation_table.isCellReserved(current_node->location, current_node->timestep + 1)) {
                // wait action

                if (!(current_node->orientation == 0 && current_cell_projections.projected_west) &&
                    !(current_node->orientation == 1 && current_cell_projections.projected_north) &&
                    !(current_node->orientation == 2 && current_cell_projections.projected_east) &&
                    !(current_node->orientation == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node = new LLNode(
                        current_node->location,
                        current_node->orientation,
                        current_node->timestep + 1,
                        current_hl_index,
                        getGScore(current_node, current_node->location, current_node->orientation),
                        getHScore(current_node->location, false, hl_plan[current_hl_index]),
                        current_node,
                        Action::W
                    );

                    if (closed_set.find(neighbor_node) == closed_set.end()) {
                        open_set.push(neighbor_node);
                        node_pool_.push_back(neighbor_node);
                    } else {
                        delete neighbor_node; // already in closed set, discard
                    }
                }

                // CR and CCR actions

                int new_orientation_cr = (current_node->orientation + 1) % 4;

                if (!(new_orientation_cr == 0 && current_cell_projections.projected_west) &&
                    !(new_orientation_cr == 1 && current_cell_projections.projected_north) &&
                    !(new_orientation_cr == 2 && current_cell_projections.projected_east) &&
                    !(new_orientation_cr == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node_cr = new LLNode(
                        current_node->location,
                        new_orientation_cr,
                        current_node->timestep + 1,
                        current_hl_index,
                        getGScore(current_node, current_node->location, new_orientation_cr),
                        getHScore(current_node->location, false, hl_plan[current_hl_index]),
                        current_node,
                        Action::CR
                    );

                    if (closed_set.find(neighbor_node_cr) == closed_set.end()) {
                        open_set.push(neighbor_node_cr);
                        node_pool_.push_back(neighbor_node_cr);
                    } else {
                        delete neighbor_node_cr; // already in closed set, discard
                    }
                }

                int new_orientation_ccr = (current_node->orientation + 3) % 4;
                if (!(new_orientation_ccr == 0 && current_cell_projections.projected_west) &&
                    !(new_orientation_ccr == 1 && current_cell_projections.projected_north) &&
                    !(new_orientation_ccr == 2 && current_cell_projections.projected_east) &&
                    !(new_orientation_ccr == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node_ccr = new LLNode(
                        current_node->location,
                        new_orientation_ccr,
                        current_node->timestep + 1,
                        current_hl_index,
                        getGScore(current_node, current_node->location, new_orientation_ccr),
                        getHScore(current_node->location, false, hl_plan[current_hl_index]),
                        current_node,
                        Action::CCR
                    );

                    if (closed_set.find(neighbor_node_ccr) == closed_set.end()) {
                        open_set.push(neighbor_node_ccr);
                        node_pool_.push_back(neighbor_node_ccr);
                    } else {
                        delete neighbor_node_ccr; // already in closed set, discard
                    }
                }
            }

            // FW action

            int cell_forward_location = -1;

            if (current_node->orientation == 0) {
                // east
                cell_forward_location = current_node->location + 1;
            } else if (current_node->orientation == 1) {
                // south
                cell_forward_location = current_node->location + shared_env->cols;
            } else if (current_node->orientation == 2) {
                // west
                cell_forward_location = current_node->location - 1;
            } else {
                // north
                cell_forward_location = current_node->location - shared_env->cols;
            }

            if (cell_forward_location >= 0 && cell_forward_location < shared_env->rows * shared_env->cols) {
                if (!reservation_table.isCellReserved(cell_forward_location, current_node->timestep + 1)) {
                    Projections forward_cell_projections = dyn_env.getPreprocessing().getProjectionsAtLocation(cell_forward_location);

                    if (!(current_node->orientation == 0 && forward_cell_projections.projected_west) &&
                        !(current_node->orientation == 1 && forward_cell_projections.projected_north) &&
                        !(current_node->orientation == 2 && forward_cell_projections.projected_east) &&
                        !(current_node->orientation == 3 && forward_cell_projections.projected_south)) {

                        LLNode* neighbor_node_fw = new LLNode(
                            cell_forward_location,
                            current_node->orientation,
                            current_node->timestep + 1,
                            current_hl_index,
                            getGScore(current_node, cell_forward_location, current_node->orientation),
                            getHScore(cell_forward_location, false, hl_plan[current_hl_index]),
                            current_node,
                            Action::FW
                        );

                        if (closed_set.find(neighbor_node_fw) == closed_set.end()) {
                            open_set.push(neighbor_node_fw);
                            node_pool_.push_back(neighbor_node_fw);
                        } else {
                            delete neighbor_node_fw; // already in closed set, discard
                        }
                    }
                }
            }
        }

        clearNodePool();
        return std::make_pair(found, path);
    }
}