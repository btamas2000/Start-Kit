#include "low_level_planner.h"
#include <iostream>
#include <queue>
#include <cmath>
#include <algorithm>

namespace DynamicData {
    std::string action_tostring(Action action) {
        switch (action) {
            case Action::FW: return "FW";
            case Action::W: return "W";
            case Action::CR: return "CR";
            case Action::CCR: return "CCR";
            case Action::NA: return "NA";
            default: return "UNKNOWN";
        }
    }

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
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();

        // basic cost is previous g score + 1 for moving to next location (applied later)
        float g_score = from_node->g_score;

        // apply penalty if there is one

        bool soft_reserved = DynamicEnvironment::getInstance().getReservationTable().isCellSoftReserved(to_location, from_node->timestep + 1);

        if (soft_reserved) {
            g_score += (1.0f * SOFT_RESERVATION_PENALTY);
        } else {
            g_score += 1.0f;
        }

        return g_score;
    }

    float LowLevelPlanner::getHScore(int location, bool cluster_crossing, const HighLevelStep& hl_step) {
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();

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
            const std::vector<PreprocessingPipeline::Portal>& portals = DynamicEnvironment::getInstance().getPreprocessing().getPortals();
            const PreprocessingPipeline::Portal& portal = portals[hl_step.id];

            float distance = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getPortalDistances()[
                portal.distances_index * shared_env->rows * shared_env->cols + location
            ]);

            return distance;
        } else if (hl_step.type == WaypointType::PORTAL && cluster_crossing) {
            if (DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_step.id].from == DynamicEnvironment::getInstance().getPreprocessing().getClusterId(location)) {
                return 0.0f; // already at the portal
            } else {
                const PreprocessingPipeline::Portal& opposite_portal = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[
                    DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_step.id].opposite_portal_id
                ];

                if (opposite_portal.from == DynamicEnvironment::getInstance().getPreprocessing().getClusterId(location)) {
                    // use portal distance to opposite portal but increment by 1 to account for crossing
                    float distance = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getPortalDistances()[
                        opposite_portal.distances_index * shared_env->rows * shared_env->cols + location
                    ]);

                    return distance + 1.0f;
                } else {
                    // should not happen
                    std::cout << "Error: Location not in either portal cluster in getHScore." << std::endl;
                    return LL_LARGE_COST;
                }
            }
        } else {
            // should not happen
            std::cout << "Error: Invalid waypoint type in getHScore." << std::endl;
            return LL_LARGE_COST;
        }
    }

    std::vector<LowLevelStep> LowLevelPlanner::reconstructPath(LLNode* goal_node) {
        std::vector<LowLevelStep> path;
        LLNode* current = goal_node;

        // careful with action logic as LowLevelStep stores the next action taken so we shift by one

        Action last_action = Action::NA;

        while (current != nullptr) {
            LowLevelStep step(current->timestep, current->location, current->orientation, last_action, false, current->hl_step_index);
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

        Agent& agent = DynamicEnvironment::getInstance().getAgents()[agent_id];
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();
        const std::vector<int>& cluster_map = DynamicEnvironment::getInstance().getPreprocessing().getClusterMap();

        if (agent.getHighLevelPlan().empty()) {
            std::cout << "Error: Agent " << agent_id << " has no high-level plan for low-level planning." << std::endl;
            return std::make_pair(false, path);
        }

        // A* search from current location to first high-level step

        std::priority_queue<LLNode*, std::vector<LLNode*>, LLNodeCompare> open_set;
        std::unordered_set<LLNode*, LLNodeHash, LLNodeEqual> closed_set;

        const std::vector<HighLevelStep>& hl_plan = agent.getHighLevelPlan();

        // debug high level plan
        // std::cout << "LowLevelPlanner: High-level plan for agent " << agent_id << ":" << std::endl;
        // for (const auto& step : hl_plan) {
        //     std::cout << "  Step: Type=" << (step.type == WaypointType::LOCATION ? "LOCATION" : "PORTAL") << ", ID=" << step.id << ", ArrivalTime=" << step.arrival_time_est << std::endl;
        // }

        // calculate hl step index from start
        int initial_hl_index = 0;
        for (int i = 0; i < static_cast<int>(hl_plan.size()); i++) {
            if (hl_plan[i].type == WaypointType::LOCATION) {
                if (agent.getCurrentLocation() == hl_plan[i].id) {
                    initial_hl_index = i + 1;
                } else {
                    break;
                }
            } else if (hl_plan[i].type == WaypointType::PORTAL) {
                bool cell_on_portal = false;
                const PreprocessingPipeline::Portal& portal = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[i].id];
                for (int j = portal.cells_begin; j <= portal.cells_end; j++) {
                    if (agent.getCurrentLocation() == DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortalCells()[j]) {
                        cell_on_portal = true;
                        break;
                    }
                }
                if (cell_on_portal) {
                    initial_hl_index = i + 1;
                } else {
                    break;
                }
            }
        }

        // calculate cluster crossing info for first hl step
        bool initial_cluster_crossing = false;
        if (initial_hl_index > 0) {
            if (hl_plan[initial_hl_index - 1].type == WaypointType::PORTAL &&
                hl_plan[initial_hl_index].type == WaypointType::PORTAL) {
                    if (DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[initial_hl_index - 1].id].opposite_portal_id == hl_plan[initial_hl_index].id) {
                        initial_cluster_crossing = true;
                    }
            }
        }

        // create start node
        LLNode* start_node = new LLNode(
            agent.getCurrentLocation(),
            agent.getCurrentOrientation(),
            DynamicEnvironment::getInstance().getCurrentTime(),
            initial_hl_index,
            0.0f,
            getHScore(agent.getCurrentLocation(), initial_cluster_crossing, hl_plan[initial_hl_index]),
            nullptr,
            Action::NA
        );

        open_set.push(start_node);
        node_pool_.push_back(start_node);

        int start_time = DynamicEnvironment::getInstance().getCurrentTime();

        while (!open_set.empty()) {
            LLNode* current_node = open_set.top();
            open_set.pop();

            if (closed_set.find(current_node) != closed_set.end()) {
                continue; // already processed
            }
            
            closed_set.insert(current_node);

            // check if agent has reached the final high-level step
            if (current_node->hl_step_index >= static_cast<int>(hl_plan.size())) {
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

            bool cluster_crossing = false;
            if (current_node->hl_step_index > 0) {
                if (hl_plan[current_node->hl_step_index - 1].type == WaypointType::PORTAL &&
                    hl_plan[current_node->hl_step_index].type == WaypointType::PORTAL) {
                        if (DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[current_node->hl_step_index - 1].id].opposite_portal_id == hl_plan[current_node->hl_step_index].id) {
                            cluster_crossing = true;
                        }
                }
            }
            int allowed_cluster1 = -1;
            int allowed_cluster2 = -1;
            if (cluster_crossing) {
                allowed_cluster1 = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[current_node->hl_step_index - 1].id].from;
                allowed_cluster2 = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[current_node->hl_step_index - 1].id].to;
            } else {
                if (hl_plan[current_node->hl_step_index].type == WaypointType::LOCATION) {
                    allowed_cluster1 = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(hl_plan[current_node->hl_step_index].id);
                } else if (hl_plan[current_node->hl_step_index].type == WaypointType::PORTAL) {
                    allowed_cluster1 = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[current_node->hl_step_index].id].from;
                }
            }

            ReservationTable& reservation_table = DynamicEnvironment::getInstance().getReservationTable();

            // possible actions: FW, CR, CCR, W
            // start check with W, CR and CCR as that can only be done if location in t+1 is not blocked

            Projections current_cell_projections = reservation_table.getProjections(current_node->location, current_node->timestep + 1);

            if (reservation_table.isCellFree(current_node->location, current_node->timestep + 1) &&
                (cluster_map[current_node->location] == allowed_cluster1 ||
                 cluster_map[current_node->location] == allowed_cluster2)) {
                // wait action

                if (!(current_node->orientation == 0 && current_cell_projections.projected_west) &&
                    !(current_node->orientation == 1 && current_cell_projections.projected_north) &&
                    !(current_node->orientation == 2 && current_cell_projections.projected_east) &&
                    !(current_node->orientation == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node = new LLNode(
                        current_node->location,
                        current_node->orientation,
                        current_node->timestep + 1,
                        current_node->hl_step_index,
                        getGScore(current_node, current_node->location, current_node->orientation),
                        getHScore(current_node->location, cluster_crossing, hl_plan[current_node->hl_step_index]),
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
                        current_node->hl_step_index,
                        getGScore(current_node, current_node->location, new_orientation_cr),
                        getHScore(current_node->location, cluster_crossing, hl_plan[current_node->hl_step_index]),
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
                        current_node->hl_step_index,
                        getGScore(current_node, current_node->location, new_orientation_ccr),
                        getHScore(current_node->location, cluster_crossing, hl_plan[current_node->hl_step_index]),
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
                if (reservation_table.isCellFree(cell_forward_location, current_node->timestep + 1) &&
                    (cluster_map[cell_forward_location] == allowed_cluster1 ||
                     cluster_map[cell_forward_location] == allowed_cluster2)) {
                    Projections forward_cell_projections = reservation_table.getProjections(cell_forward_location, current_node->timestep + 1);

                    if (!(current_node->orientation == 0 && forward_cell_projections.projected_west) &&
                        !(current_node->orientation == 1 && forward_cell_projections.projected_north) &&
                        !(current_node->orientation == 2 && forward_cell_projections.projected_east) &&
                        !(current_node->orientation == 3 && forward_cell_projections.projected_south)) {

                        // check if next hl step is reached
                        int next_hl_index = current_node->hl_step_index;
                        if (next_hl_index < static_cast<int>(hl_plan.size())) {
                            if (hl_plan[next_hl_index].type == WaypointType::LOCATION) {
                                if (cell_forward_location == hl_plan[next_hl_index].id) {
                                    next_hl_index += 1;
                                }
                            } else if (hl_plan[next_hl_index].type == WaypointType::PORTAL) {
                                const PreprocessingPipeline::Portal& portal = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[next_hl_index].id];
                                for (int j = portal.cells_begin; j <= portal.cells_end; j++) {
                                    if (cell_forward_location == DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortalCells()[j]) {
                                        next_hl_index += 1;
                                        break;
                                    }
                                }
                            }
                        }

                        LLNode* neighbor_node_fw = new LLNode(
                            cell_forward_location,
                            current_node->orientation,
                            current_node->timestep + 1,
                            next_hl_index,
                            getGScore(current_node, cell_forward_location, current_node->orientation),
                            getHScore(cell_forward_location, cluster_crossing, hl_plan[next_hl_index]),
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

        // std::cout << "Path found: " << (found ? "YES" : "NO") << std::endl;
        // if (found) {
        //     std::cout << "Path length: " << path.size() << std::endl;
        //     for (const auto& step : path) {
        //         std::cout << "  Step: t=" << step.t << ", loc=" << step.location << ", orient=" << step.orientation << ", action=" << action_tostring(step.nextAction) << ", hl_index=" << step.hl_step_index << std::endl;
        //     }
        // }
        // std::cout << "Explored nodes: " << closed_set.size() << std::endl;

        clearNodePool();
        return std::make_pair(found, path);
    }

    std::pair<bool, std::vector<LowLevelStep>> LowLevelPlanner::extendLowLevelPath(int agent_id, const LowLevelStep& last_step, int extend_until_timestep) {
        bool found = false;
        std::vector<LowLevelStep> path;

        clearNodePool();

        Agent& agent = DynamicEnvironment::getInstance().getAgents()[agent_id];
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();
        const std::vector<int>& cluster_map = DynamicEnvironment::getInstance().getPreprocessing().getClusterMap();

        if (agent.getHighLevelPlan().empty()) {
            std::cout << "Error: Agent " << agent_id << " has no high-level plan for low-level planning." << std::endl;
            return std::make_pair(false, path);
        }

        // A* search from last step to extend_until_timestep

        std::priority_queue<LLNode*, std::vector<LLNode*>, LLNodeCompare> open_set;
        std::unordered_set<LLNode*, LLNodeHash, LLNodeEqual> closed_set;
        const std::vector<PreprocessingPipeline::Portal>& portals = DynamicEnvironment::getInstance().getPreprocessing().getPortals();

        const std::vector<HighLevelStep>& hl_plan = agent.getHighLevelPlan();
        bool initial_cluster_crossing = false;
        if (last_step.hl_step_index > 0) {
            if (hl_plan[last_step.hl_step_index - 1].type == WaypointType::PORTAL &&
                hl_plan[last_step.hl_step_index].type == WaypointType::PORTAL) {
                    if (portals[hl_plan[last_step.hl_step_index - 1].id].opposite_portal_id == hl_plan[last_step.hl_step_index].id) {
                        initial_cluster_crossing = true;
                    }
            }
        }

        // create start node
        LLNode* start_node = new LLNode(
            last_step.location,
            last_step.orientation,
            last_step.t,
            last_step.hl_step_index,
            0.0f,
            getHScore(last_step.location, initial_cluster_crossing, hl_plan[last_step.hl_step_index]),
            nullptr,
            Action::NA
        );

        // std::cout << "ExtendLowLevelPath: Start node created at location " << last_step.location << " with timestep " << last_step.t << " and hl_step_index " << last_step.hl_step_index << std::endl;

        open_set.push(start_node);
        node_pool_.push_back(start_node);

        int start_time = last_step.t;

        while (!open_set.empty()) {
            LLNode* current_node = open_set.top();
            open_set.pop();
            
            if (closed_set.find(current_node) != closed_set.end()) {
                continue; // already processed
            }

            closed_set.insert(current_node);

            // check if agent has reached the final high-level step
            if (current_node->hl_step_index >= static_cast<int>(hl_plan.size())) {
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

            int cluster_crossing = false;
            if (current_node->hl_step_index > 0) {
                if (hl_plan[current_node->hl_step_index - 1].type == WaypointType::PORTAL &&
                    hl_plan[current_node->hl_step_index].type == WaypointType::PORTAL) {
                        if (portals[hl_plan[current_node->hl_step_index - 1].id].opposite_portal_id == hl_plan[current_node->hl_step_index].id) {
                            cluster_crossing = true;
                        }
                }
            }
            int allowed_cluster1 = -1;
            int allowed_cluster2 = -1;
            if (cluster_crossing) {
                allowed_cluster1 = portals[hl_plan[current_node->hl_step_index - 1].id].from;
                allowed_cluster2 = portals[hl_plan[current_node->hl_step_index - 1].id].to;
            } else {
                if (hl_plan[current_node->hl_step_index].type == WaypointType::LOCATION) {
                    allowed_cluster1 = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(hl_plan[current_node->hl_step_index].id);
                } else if (hl_plan[current_node->hl_step_index].type == WaypointType::PORTAL) {
                    allowed_cluster1 = portals[hl_plan[current_node->hl_step_index].id].from;
                }
            }

            ReservationTable& reservation_table = DynamicEnvironment::getInstance().getReservationTable();

            // possible actions: FW, CR, CCR, W
            // start check with W, CR and CCR as that can only be done if location in t+1 is not blocked

            Projections current_cell_projections = reservation_table.getProjections(current_node->location, current_node->timestep + 1);

            if (reservation_table.isCellFree(current_node->location, current_node->timestep + 1) &&
                (cluster_map[current_node->location] == allowed_cluster1 ||
                 cluster_map[current_node->location] == allowed_cluster2)) {
                // wait action

                if (!(current_node->orientation == 0 && current_cell_projections.projected_west) &&
                    !(current_node->orientation == 1 && current_cell_projections.projected_north) &&
                    !(current_node->orientation == 2 && current_cell_projections.projected_east) &&
                    !(current_node->orientation == 3 && current_cell_projections.projected_south)) {

                    LLNode* neighbor_node = new LLNode(
                        current_node->location,
                        current_node->orientation,
                        current_node->timestep + 1,
                        current_node->hl_step_index,
                        getGScore(current_node, current_node->location, current_node->orientation),
                        getHScore(current_node->location, cluster_crossing, hl_plan[current_node->hl_step_index]),
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
                        current_node->hl_step_index,
                        getGScore(current_node, current_node->location, new_orientation_cr),
                        getHScore(current_node->location, cluster_crossing, hl_plan[current_node->hl_step_index]),
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
                        current_node->hl_step_index,
                        getGScore(current_node, current_node->location, new_orientation_ccr),
                        getHScore(current_node->location, cluster_crossing, hl_plan[current_node->hl_step_index]),
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
                if (reservation_table.isCellFree(cell_forward_location, current_node->timestep + 1) &&
                    (cluster_map[cell_forward_location] == allowed_cluster1 ||
                     cluster_map[cell_forward_location] == allowed_cluster2)) {
                    Projections forward_cell_projections = reservation_table.getProjections(cell_forward_location, current_node->timestep + 1);

                    if (!(current_node->orientation == 0 && forward_cell_projections.projected_west) &&
                        !(current_node->orientation == 1 && forward_cell_projections.projected_north) &&
                        !(current_node->orientation == 2 && forward_cell_projections.projected_east) &&
                        !(current_node->orientation == 3 && forward_cell_projections.projected_south)) {

                        // check if next hl step is reached
                        int next_hl_index = current_node->hl_step_index;
                        if (next_hl_index < static_cast<int>(hl_plan.size())) {
                            if (hl_plan[next_hl_index].type == WaypointType::LOCATION) {
                                if (cell_forward_location == hl_plan[next_hl_index].id) {
                                    next_hl_index += 1;
                                }
                            } else if (hl_plan[next_hl_index].type == WaypointType::PORTAL) {
                                const PreprocessingPipeline::Portal& portal = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[hl_plan[next_hl_index].id];
                                for (int j = portal.cells_begin; j <= portal.cells_end; j++) {
                                    if (cell_forward_location == DynamicData::DynamicEnvironment::getInstance().getPreprocessing().getPortalCells()[j]) {
                                        next_hl_index += 1;
                                        break;
                                    }
                                }
                            }
                        }

                        LLNode* neighbor_node_fw = new LLNode(
                            cell_forward_location,
                            current_node->orientation,
                            current_node->timestep + 1,
                            next_hl_index,
                            getGScore(current_node, cell_forward_location, current_node->orientation),
                            getHScore(cell_forward_location, cluster_crossing, hl_plan[next_hl_index]),
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