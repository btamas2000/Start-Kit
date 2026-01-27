#include "high_level_planner.h"
#include <iostream>
#include <algorithm>
#include <limits>

namespace DynamicData {
    HighLevelPlanner::HighLevelPlanner() {
        node_pool_.clear();
    }

    void HighLevelPlanner::clearNodePool() {
        for (auto node : node_pool_) {
            delete node;
        }
        node_pool_.clear();
    }

    float HighLevelPlanner::manhattanDistance(int r1, int c1, int r2, int c2) {
        return static_cast<float>(std::abs(r1 - r2) + std::abs(c1 - c2));
    }

    std::vector<std::pair<int, float>> HighLevelPlanner::getPortalDistancesFromLocation(int location_id) {
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();

        std::vector<std::pair<int, float>> portal_distances;

        int cluster_id = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(location_id);
        const PreprocessingPipeline::Cluster& cluster = DynamicEnvironment::getInstance().getPreprocessing().getClusters()[cluster_id];

        const std::vector<PreprocessingPipeline::Portal>& portals = DynamicEnvironment::getInstance().getPreprocessing().getPortals();

        for (int i = cluster.portal_begin; i <= cluster.portal_end; i++) {
            const PreprocessingPipeline::Portal& portal = portals[i];
            // compute distance from location to portal using precomputed portal distances
            float distance = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getPortalDistances()[
                portal.distances_index * shared_env->rows * shared_env->cols + location_id
            ]);
            portal_distances.emplace_back(portal.id, distance);
        }

        return portal_distances;
    }

    std::pair<float, int> HighLevelPlanner::getGScore(HLNode* from_node, int id, WaypointType type) {
        // return pair of (g_score, travel_time)

        CongestionTracker& congestion_tracker = DynamicEnvironment::getInstance().getCongestionTracker();
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();

        if (type == WaypointType::LOCATION && from_node->type == WaypointType::LOCATION) {
            // location to location
            // use manhattan distance
            // apply cluster penalty

            int cluster_id = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(id);

            float distance = manhattanDistance(
                from_node->id / shared_env->cols,
                from_node->id % shared_env->cols,
                id / shared_env->cols,
                id % shared_env->cols
            );

            float penalty = congestion_tracker.getClusterCongestion(
                cluster_id,
                from_node->arrival_time,
                from_node->arrival_time + distance
            );

            return std::make_pair(distance * (1.0f + penalty), static_cast<int>(distance));
            
        } else if (type == WaypointType::LOCATION && from_node->type == WaypointType::PORTAL) {
            // portal to location
            // use precomputed portal distances in reverse to estimate distance
            // apply cluster penalty

            float distance = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getPortalDistances()[
                DynamicEnvironment::getInstance().getPreprocessing().getPortals()[from_node->id].distances_index * shared_env->rows * shared_env->cols + id
            ]);

            int cluster_id = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(id);
            float penalty = congestion_tracker.getClusterCongestion(
                cluster_id,
                from_node->arrival_time,
                from_node->arrival_time + distance
            );

            return std::make_pair(distance * (1.0f + penalty), static_cast<int>(distance));
            
        } else if (type == WaypointType::PORTAL && from_node->type == WaypointType::LOCATION) {
            // location to portal
            // use precomputed portal distances to estimate distance
            // apply cluster penalty

            float distance = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getPortalDistances()[
                DynamicEnvironment::getInstance().getPreprocessing().getPortals()[id].distances_index * shared_env->rows * shared_env->cols + from_node->id
            ]);

            int cluster_id = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(from_node->id);

            float penalty = congestion_tracker.getClusterCongestion(
                cluster_id,
                from_node->arrival_time,
                from_node->arrival_time + distance
            );

            return std::make_pair(distance * (1.0f + penalty), static_cast<int>(distance));
            

        } else if (type == WaypointType::PORTAL && from_node->type == WaypointType::PORTAL) {
            // portal to portal
            // use inter-cluster heuristic to estimate distance (gives back intra-cluster distance if called on portals in same cluster)
            // if in same cluster, apply cluster penalty
            // else, apply portal penalty
            
            int cluster_from = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[from_node->id].from;
            int cluster_to = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[id].from;

            if (cluster_from == cluster_to) {
                // same cluster, use inter-cluster heuristic
                float distance = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getInterClusterHeuristics()[
                    from_node->id * DynamicEnvironment::getInstance().getPreprocessing().getPortals().size() + id
                ]);
                
                float cluster_congestion_value = congestion_tracker.getClusterCongestion(
                    cluster_from,
                    from_node->arrival_time,
                    from_node->arrival_time + distance
                );

                float penalty = 1.0f;
                int cluster_capacity = DynamicEnvironment::getInstance().getPreprocessing().getClusters()[cluster_from].size;

                if (cluster_congestion_value > static_cast<float>(cluster_capacity) * 0.5f) { // more than 50% capacity
                    penalty *= CLUSTER_PENALTY;
                }
                if (cluster_congestion_value > static_cast<float>(cluster_capacity) * 0.6f) { // more than 60% capacity
                    penalty *= CLUSTER_PENALTY;
                }
                if (cluster_congestion_value > static_cast<float>(cluster_capacity) * 0.7f) { // more than 70% capacity
                    penalty *= CLUSTER_PENALTY;
                }
                if (cluster_congestion_value > static_cast<float>(cluster_capacity) * 0.8f) { // more than 80% capacity
                    penalty *= CLUSTER_PENALTY;
                }

                return std::make_pair(distance * (1.0f * penalty), static_cast<int>(distance));

            } else {
                std::vector<float> opposing_flow_congestion;
                const PreprocessingPipeline::Portal& from_portal = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[from_node->id];
                const PreprocessingPipeline::Portal& to_portal = DynamicEnvironment::getInstance().getPreprocessing().getPortals()[id];

                if (from_portal.is_critical_this_side) {
                    if (from_portal.has_shared_area) {
                        // get shared areas
                        std::vector<PreprocessingPipeline::Portal> shared_portals;
                        for (int i = from_portal.shared_begin; i < from_portal.shared_end; i++) {
                            shared_portals.push_back(DynamicEnvironment::getInstance().getPreprocessing().getPortals()[i]);
                        }
                        for (const auto& sp : shared_portals) {
                            float opposite_usage = congestion_tracker.getPortalCongestion(
                                sp.opposite_portal_id,
                                from_node->arrival_time + 1
                            );
                            opposing_flow_congestion.push_back(opposite_usage);
                        }
                    } else {
                        float opposite_usage = congestion_tracker.getPortalCongestion(
                            from_portal.opposite_portal_id,
                            from_node->arrival_time + 1
                        );
                        opposing_flow_congestion.push_back(opposite_usage);
                    }
                } else if (from_portal.is_critical_other_side) {
                    if (to_portal.has_shared_area) {
                        // get shared areas
                        std::vector<PreprocessingPipeline::Portal> shared_portals;
                        for (int i = to_portal.shared_begin; i < to_portal.shared_end; i++) {
                            shared_portals.push_back(DynamicEnvironment::getInstance().getPreprocessing().getPortals()[i]);
                        }
                        for (const auto& sp : shared_portals) {
                            float opposite_usage = congestion_tracker.getPortalCongestion(
                                sp.id,
                                from_node->arrival_time + 1
                            );
                            opposing_flow_congestion.push_back(opposite_usage);
                        }
                    } else {
                        float usage = congestion_tracker.getPortalCongestion(
                            to_portal.id,
                            from_node->arrival_time + 1
                        );
                        opposing_flow_congestion.push_back(usage);
                    }
                }

                float penalty = 1.0f;

                for (const auto& ofc : opposing_flow_congestion) {
                    // since these penalties for critical portals only, apply more significant penalty
                    // if (any) opposing flow congestion value is more than 0.5f, apply portal penalty
                    if (ofc > 0.5f) {
                        penalty *= PORTAL_PENALTY;
                        break;
                    }
                }

                return std::make_pair(1.0f * (1.0f * penalty), 1);
            }
        } else {
            // should not reach here
            std::cout << "Error: Invalid waypoint type combination in getGScore." << std::endl;
            return std::make_pair(HL_LARGE_COST, -1);
        }
    }

    float HighLevelPlanner::getHScore(int from_id, WaypointType from_type, int to_id, WaypointType to_type) {
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();

        // get best heuristic estimate from the from node to the to node
        // only cases handled are those that can get called during planning
        if (from_type == WaypointType::LOCATION && to_type == WaypointType::LOCATION && from_id == to_id) {
            // probably called for goal
            return 0.0f;
            
        } else if (from_type == WaypointType::LOCATION && to_type == WaypointType::LOCATION && from_id != to_id) {
            // probably called for start to goal direct heuristic
            int from_cluster = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(from_id);
            int to_cluster = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(to_id);

            if (from_cluster == to_cluster) {
                // same cluster, use manhattan distance
                return manhattanDistance(
                    from_id / shared_env->cols,
                    from_id % shared_env->cols,
                    to_id / shared_env->cols,
                    to_id % shared_env->cols
                );
            } else {
                // for this get all distances from start to accessible portals,
                // then get all distances from goal to accessible portals,
                // after that, take the mininum of all the combinations

                std::vector<std::pair<int, float>> from_portal_distances = getPortalDistancesFromLocation(from_id);
                std::vector<std::pair<int, float>> to_portal_distances = getPortalDistancesFromLocation(to_id);

                float min_heuristic = -1.0f;

                for (const auto& fpd : from_portal_distances) {
                    int from_portal_id = fpd.first;
                    float distance_from_location = fpd.second;

                    for (const auto& tpd : to_portal_distances) {
                        int to_portal_id = tpd.first;
                        float distance_to_location = tpd.second;

                        float inter_cluster_heuristic = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getInterClusterHeuristics()[
                            to_portal_id * DynamicEnvironment::getInstance().getPreprocessing().getPortals().size() + from_portal_id
                        ]);

                        float total_heuristic = distance_from_location + inter_cluster_heuristic + distance_to_location;

                        if (min_heuristic < 0.0f || total_heuristic < min_heuristic) {
                            min_heuristic = total_heuristic;
                        }
                    }
                }

                return min_heuristic;
            }
        } else if (from_type == WaypointType::PORTAL && to_type == WaypointType::LOCATION) {
            // probably called for portal to goal heuristic
            // get all portals, get all distances to them, then get inter heuristic from the from portal to those portals, take min

            std::vector<std::pair<int, float>> portal_distances = getPortalDistancesFromLocation(to_id);

            float min_heuristic = -1.0f;
            for (const auto& pd : portal_distances) {
                int portal_id = pd.first;
                float distance_to_location = pd.second;

                float inter_cluster_heuristic = static_cast<float>(DynamicEnvironment::getInstance().getPreprocessing().getInterClusterHeuristics()[
                    portal_id * DynamicEnvironment::getInstance().getPreprocessing().getPortals().size() + from_id
                ]);

                float total_heuristic = inter_cluster_heuristic + distance_to_location;

                if (min_heuristic < 0.0f || total_heuristic < min_heuristic) {
                    min_heuristic = total_heuristic;
                }
            }

            return min_heuristic;

        } else {
            // other cases not handled
            std::cout << "Error: Invalid waypoint type combination in getHScore." << std::endl;
            return HL_LARGE_COST;
        }
    }

    std::vector<std::pair<int, WaypointType>> HighLevelPlanner::getNeighbors(HLNode* current_node, int goal_loc) {
        // if current node is location, neighbors are: all portals in the same cluster + goal location (if in same cluster)
        // if current node is portal, neighbors are: all portals in the same cluster, the opposite portal + goal location (if in same cluster)

        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();

        const std::vector<PreprocessingPipeline::Portal>& portals = DynamicEnvironment::getInstance().getPreprocessing().getPortals();

        std::vector<std::pair<int, WaypointType>> neighbors;

        if (current_node->type == WaypointType::LOCATION) {
            int cluster_id = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(current_node->id);
            const PreprocessingPipeline::Cluster& cluster = DynamicEnvironment::getInstance().getPreprocessing().getClusters()[cluster_id];

            // add all portals in the same cluster
            for (int i = cluster.portal_begin; i <= cluster.portal_end; i++) {
                const PreprocessingPipeline::Portal& portal = portals[i];
                neighbors.emplace_back(portal.id, WaypointType::PORTAL);
            }

            // if goal location is in same cluster, add it as neighbor
            int goal_cluster_id = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(goal_loc);
            if (goal_cluster_id == cluster_id) {
                neighbors.emplace_back(goal_loc, WaypointType::LOCATION);
            }

        } else if (current_node->type == WaypointType::PORTAL) {
            const PreprocessingPipeline::Portal& portal = portals[current_node->id];
            int cluster_id = portal.from;
            const PreprocessingPipeline::Cluster& cluster = DynamicEnvironment::getInstance().getPreprocessing().getClusters()[cluster_id];

            // add all portals in the same cluster
            for (int i = cluster.portal_begin; i <= cluster.portal_end; i++) {
                const PreprocessingPipeline::Portal& p = portals[i];
                if (p.id != current_node->id) { // avoid adding self
                    neighbors.emplace_back(p.id, WaypointType::PORTAL);
                }
            }

            // add opposite portal
            neighbors.emplace_back(portal.opposite_portal_id, WaypointType::PORTAL);

            // if goal location is in same cluster, add it as neighbor
            int goal_cluster_id = DynamicEnvironment::getInstance().getPreprocessing().getClusterId(goal_loc);
            if (goal_cluster_id == cluster_id) {
                neighbors.emplace_back(goal_loc, WaypointType::LOCATION);
            }
        }

        return neighbors;
    }

    std::vector<HighLevelStep> HighLevelPlanner::reconstructPath(HLNode* goal_node) {
        std::vector<HighLevelStep> path;
        HLNode* current = goal_node;

        while (current != nullptr) {
            HighLevelStep step(current->type, current->id, current->arrival_time);
            path.push_back(step);
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    std::pair<bool, std::vector<HighLevelStep>> HighLevelPlanner::findPath(int start_id, WaypointType start_type, int goal_id, WaypointType goal_type, int start_time) {
        // high level A* search from start to goal
        // returns pair of (found, path)

        bool found = false;
        std::vector<HighLevelStep> path;

        clearNodePool();

        // quick check if already at goal
        if (start_id == goal_id && start_type == goal_type) {
            found = true;
            HighLevelStep step(start_type, start_id, start_time);
            path.push_back(step);

            clearNodePool();
            return std::make_pair(found, path);
        }

        // open set
        std::priority_queue<HLNode*, std::vector<HLNode*>, HLNodeCompare> open_set;
        // closed set
        std::unordered_set<HLNode*, HLNodeHash, HLNodeEqual> closed_set;

        // create start node
        HLNode* start_node = new HLNode(
            start_type,
            start_id,
            start_time,
            0.0f,
            getHScore(start_id, start_type, goal_id, goal_type),
            nullptr
        );

        node_pool_.push_back(start_node);
        open_set.push(start_node);

        while (!open_set.empty()) {
            HLNode* current_node = open_set.top();
            open_set.pop();

            // check if current node is in closed set
            if (closed_set.find(current_node) != closed_set.end()) {
                continue; // already processed
            }

            closed_set.insert(current_node);

            // check if goal reached
            if (current_node->id == goal_id && current_node->type == goal_type) {
                found = true;
                path = reconstructPath(current_node);
                break;
            }

            std::vector<std::pair<int, WaypointType>> neighbors = getNeighbors(current_node, goal_id);

            for (const auto& neighbor : neighbors) {
                int neighbor_id = neighbor.first;
                WaypointType neighbor_type = neighbor.second;

                std::pair<float, int> g_score_pair = getGScore(current_node, neighbor_id, neighbor_type);
                float g_score = current_node->g_score + g_score_pair.first;
                int arrival_time = current_node->arrival_time + g_score_pair.second;

                HLNode* neighbor_node = new HLNode(
                    neighbor_type,
                    neighbor_id,
                    arrival_time,
                    g_score,
                    getHScore(neighbor_id, neighbor_type, goal_id, goal_type),
                    current_node
                );

                if (closed_set.find(neighbor_node) == closed_set.end()) {
                    open_set.push(neighbor_node);
                    node_pool_.push_back(neighbor_node);
                } else {
                    delete neighbor_node; // already in closed set, discard
                }
            }
        }

        clearNodePool();

        return std::make_pair(found, path);
    }

    std::vector<HighLevelStep> HighLevelPlanner::planHighLevelPath(int agent_id) {
        std::cout << "Planning high-level path for agent " << agent_id << "..." << std::endl;
        Agent& agent = DynamicEnvironment::getInstance().getAgents()[agent_id];
        SharedEnvironment* shared_env = DynamicEnvironment::getInstance().getSharedEnvironment();

        std::vector<HighLevelStep> plan;

        int task_id = agent.getAssignedTaskID();

        if (shared_env->task_pool[task_id].idx_next_loc < 0 || shared_env->task_pool[task_id].idx_next_loc >= static_cast<int>(shared_env->task_pool[task_id].locations.size())) {
            // no next location to go to
            std::cout << "Error: Agent " << agent_id << " has no next location for assigned task " << task_id << "." << std::endl;
            return plan;
        }

        int current_location = agent.getCurrentLocation();
        int current_time = DynamicEnvironment::getInstance().getCurrentTime();
        
        for (int i = shared_env->task_pool[task_id].idx_next_loc; i < static_cast<int>(shared_env->task_pool[task_id].locations.size()); i++) {
            int goal_location = shared_env->task_pool[task_id].locations[i];

            std::pair<bool, std::vector<HighLevelStep>> result = findPath(
                current_location,
                WaypointType::LOCATION,
                goal_location,
                WaypointType::LOCATION,
                current_time
            );

            if (!result.first) {
                // path not found
                std::cout << "Error: High-level path not found for agent " << agent_id << " from location " << current_location << " to location " << goal_location << " at timestep " << current_time << "." << std::endl;
                return std::vector<HighLevelStep>();
            }

            if (plan.empty()) {
                // first segment, add all steps
                plan.insert(plan.end(), result.second.begin(), result.second.end());
            } else {
                // subsequent segments, remove first step to avoid duplication
                result.second.erase(result.second.begin());
                plan.insert(plan.end(), result.second.begin(), result.second.end());
            }

            // update current location and time for next segment
            current_location = goal_location;
            current_time = plan.back().arrival_time_est;
        }

        // debug: print plan
        // std::cout << "High-level plan for agent " << agent_id << ":" << std::endl;
        // for (const auto& step : plan) {
        //     std::cout << "  Step: Type=" << (step.type == WaypointType::LOCATION ? "LOCATION" : "PORTAL") << ", ID=" << step.id << ", ArrivalTime=" << step.arrival_time_est << std::endl;
        // }

        clearNodePool();

        return plan;
    }
}