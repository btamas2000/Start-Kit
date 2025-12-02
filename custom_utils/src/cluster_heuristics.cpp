#include "../inc/cluster_heuristics.h"
#include <queue>
#include <limits>
#include <cmath>

namespace clustering {

// Singleton instance getter
ClusterHeuristics& ClusterHeuristics::getInstance() {
    static ClusterHeuristics instance;
    return instance;
}

// Private constructor
ClusterHeuristics::ClusterHeuristics() 
    : initialized_(false), pipeline_(nullptr), env_(nullptr) {
}

void ClusterHeuristics::initialize(const std::vector<int>& map, int rows, int cols, SharedEnvironment* env) {
    // Skip if already initialized
    if (initialized_) {
        return;
    }
    
    // Store reference to shared environment
    env_ = env;
    
    // Create clustering pipeline and run clustering
    pipeline_ = new ClusteringPipeline(map, rows, cols);
    pipeline_->runClustering();
    
    // Precompute cluster pair distances
    precomputeClusterDistances();
    
    // Initialize capacity scores for all clusters
    const auto& clusters = pipeline_->getClusters();
    for (const auto& cluster : clusters) {
        // Capacity is based on cluster size weighted by clearance quality
        // Higher maxima = more open space = higher capacity
        // Normalize maxima to [0, 1] range (assuming maxima is clearance value)
        float normalizedMaxima = std::min(cluster.maxima / 10.0f, 1.0f);
        float capacityScore = cluster.size * (0.5f + 0.5f * normalizedMaxima);
        
        clusterCongestion_[cluster.id] = ClusterCongestion(cluster.id, capacityScore);
    }
    
    initialized_ = true;
}

void ClusterHeuristics::precomputeClusterDistances() {
    if (!pipeline_) return;
    
    const auto& clusters = pipeline_->getClusters();
    
    // For each cluster, process its neighbors
    for (const auto& cluster : clusters) {
        for (int neighborID : cluster.neighbors) {
            // Only process each pair once (smaller ID first)
            if (cluster.id >= neighborID) continue;
            
            // Find boundary cells between these two clusters
            auto boundaryPairs = findBoundaryCells(cluster.id, neighborID);
            
            if (boundaryPairs.empty()) continue;
            
            // Find the best (shortest) path through boundary cells
            int minDistance = std::numeric_limits<int>::max();
            int bestEntryCell = -1;
            int bestExitCell = -1;
            
            for (const auto& [cellA, cellB] : boundaryPairs) {
                // Compute distance: cellA -> cellB
                int dist = computePathDistance(cellA, cellB);
                if (dist >= 0 && dist < minDistance) {
                    minDistance = dist;
                    bestEntryCell = cellA;
                    bestExitCell = cellB;
                }
            }
            
            // Store the result
            if (minDistance != std::numeric_limits<int>::max()) {
                ClusterPairDistance cpd;
                cpd.clusterA = cluster.id;
                cpd.clusterB = neighborID;
                cpd.entryCell = bestEntryCell;
                cpd.exitCell = bestExitCell;
                cpd.distance = minDistance;
                
                // Store with ordered pair as key
                clusterDistances_[{cluster.id, neighborID}] = cpd;
            }
        }
    }
}

std::vector<std::pair<int, int>> ClusterHeuristics::findBoundaryCells(int clusterA, int clusterB) const {
    if (!pipeline_) return {};
    
    const auto& clusters = pipeline_->getClusters();
    const auto& map = pipeline_->getMap();
    const auto& cellToCluster = pipeline_->getCellToClusterMap();
    int rows = pipeline_->getRows();
    int cols = pipeline_->getCols();
    
    std::vector<std::pair<int, int>> boundaryPairs;
    
    // Find cluster A
    const Cluster* cA = nullptr;
    for (const auto& c : clusters) {
        if (c.id == clusterA) {
            cA = &c;
            break;
        }
    }
    if (!cA) return {};
    
    // For each cell in cluster A
    for (int cellA : cA->cells) {
        int row = cellA / cols;
        int col = cellA % cols;
        
        // Check 4-connected neighbors
        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};
        
        for (int d = 0; d < 4; d++) {
            int newRow = row + dr[d];
            int newCol = col + dc[d];
            
            if (newRow < 0 || newRow >= rows || newCol < 0 || newCol >= cols) continue;
            
            int neighborLoc = newRow * cols + newCol;
            
            // Check if neighbor belongs to cluster B
            if (cellToCluster[neighborLoc] == clusterB && map[neighborLoc] == 0) {
                boundaryPairs.push_back({cellA, neighborLoc});
            }
        }
    }
    
    return boundaryPairs;
}

int ClusterHeuristics::computePathDistance(int startLoc, int goalLoc) const {
    if (!pipeline_) return -1;
    
    const auto& map = pipeline_->getMap();
    const auto& cellToCluster = pipeline_->getCellToClusterMap();
    int rows = pipeline_->getRows();
    int cols = pipeline_->getCols();
    
    // Get cluster IDs for start and goal to constrain BFS
    int clusterA = cellToCluster[startLoc];
    int clusterB = cellToCluster[goalLoc];
    
    // BFS to find shortest path, constrained to the two clusters
    std::queue<std::pair<int, int>> q; // (location, distance)
    std::vector<bool> visited(map.size(), false);
    
    q.push({startLoc, 0});
    visited[startLoc] = true;
    
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};
    
    while (!q.empty()) {
        auto [loc, dist] = q.front();
        q.pop();
        
        if (loc == goalLoc) {
            return dist;
        }
        
        int row = loc / cols;
        int col = loc % cols;
        
        for (int d = 0; d < 4; d++) {
            int newRow = row + dr[d];
            int newCol = col + dc[d];
            
            if (newRow < 0 || newRow >= rows || newCol < 0 || newCol >= cols) continue;
            
            int newLoc = newRow * cols + newCol;
            
            // Only traverse free cells that belong to either cluster A or B
            if (!visited[newLoc] && map[newLoc] == 0) {
                int cellCluster = cellToCluster[newLoc];
                if (cellCluster == clusterA || cellCluster == clusterB) {
                    visited[newLoc] = true;
                    q.push({newLoc, dist + 1});
                }
            }
        }
    }
    
    return -1; // No path found
}

int ClusterHeuristics::getClusterDistance(int clusterA, int clusterB) const {
    if (!initialized_) return -1;
    
    // Ensure ordered pair (smaller ID first)
    if (clusterA > clusterB) {
        std::swap(clusterA, clusterB);
    }
    
    auto it = clusterDistances_.find({clusterA, clusterB});
    if (it != clusterDistances_.end()) {
        return it->second.distance;
    }
    
    return -1; // Not neighbors or not found
}

int ClusterHeuristics::getClusterID(int loc) const {
    if (!initialized_ || !pipeline_) return -1;
    return pipeline_->getClusterID(loc);
}

int ClusterHeuristics::getHeuristicDistance(int startLoc, int goalLoc) const {
    if (!initialized_ || !pipeline_) return -1;
    
    // Get cluster IDs
    int startCluster = pipeline_->getClusterID(startLoc);
    int goalCluster = pipeline_->getClusterID(goalLoc);
    
    if (startCluster == -1 || goalCluster == -1) return -1;
    
    // Same cluster: direct BFS
    if (startCluster == goalCluster) {
        return computePathDistance(startLoc, goalLoc);
    }
    
    // Different clusters: compute distances to precomputed exit points
    
    // Step 1: Get distances from start location to exit points in start cluster
    auto startExitDistances = computeDistancesToExitPoints(startLoc, startCluster);
    if (startExitDistances.empty()) return -1;
    
    // Step 2: Get distances from entry points in goal cluster to goal location
    auto goalEntryDistances = computeDistancesToExitPoints(goalLoc, goalCluster);
    if (goalEntryDistances.empty()) return -1;
    
    // Step 3: Find shortest path through cluster graph incorporating start/goal distances
    int minDistance = findClusterPath(startCluster, goalCluster, startExitDistances, goalEntryDistances);
    
    return minDistance;
}

std::unordered_map<int, int> ClusterHeuristics::computeDistancesToExitPoints(int loc, int clusterID) const {
    if (!pipeline_) return {};
    
    const auto& map = pipeline_->getMap();
    const auto& cellToCluster = pipeline_->getCellToClusterMap();
    const auto& clusters = pipeline_->getClusters();
    int rows = pipeline_->getRows();
    int cols = pipeline_->getCols();
    
    // Find current cluster
    const Cluster* currentCluster = nullptr;
    for (const auto& c : clusters) {
        if (c.id == clusterID) {
            currentCluster = &c;
            break;
        }
    }
    if (!currentCluster) return {};
    
    // Collect all precomputed entry/exit points for this cluster's neighbors
    std::unordered_set<int> targetPoints;
    for (int neighborID : currentCluster->neighbors) {
        int cA = std::min(clusterID, neighborID);
        int cB = std::max(clusterID, neighborID);
        auto it = clusterDistances_.find({cA, cB});
        if (it != clusterDistances_.end()) {
            const auto& cpd = it->second;
            // Add the point in our cluster (either entry or exit depending on orientation)
            int pointInOurCluster = (cpd.clusterA == clusterID) ? cpd.entryCell : cpd.exitCell;
            targetPoints.insert(pointInOurCluster);
        }
    }
    
    if (targetPoints.empty()) return {};
    
    // BFS to compute distances to target points
    std::unordered_map<int, int> distances;
    std::queue<std::pair<int, int>> q;
    std::vector<bool> visited(map.size(), false);
    
    q.push({loc, 0});
    visited[loc] = true;
    
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};
    
    while (!q.empty() && distances.size() < targetPoints.size()) {
        auto [currentLoc, dist] = q.front();
        q.pop();
        
        // Check if this is a target point
        if (targetPoints.count(currentLoc)) {
            distances[currentLoc] = dist;
        }
        
        int row = currentLoc / cols;
        int col = currentLoc % cols;
        
        // Continue BFS within cluster
        for (int d = 0; d < 4; d++) {
            int newRow = row + dr[d];
            int newCol = col + dc[d];
            
            if (newRow < 0 || newRow >= rows || newCol < 0 || newCol >= cols) continue;
            
            int newLoc = newRow * cols + newCol;
            
            if (!visited[newLoc] && map[newLoc] == 0 && cellToCluster[newLoc] == clusterID) {
                visited[newLoc] = true;
                q.push({newLoc, dist + 1});
            }
        }
    }
    
    return distances;
}

int ClusterHeuristics::findClusterPath(int startCluster, int goalCluster, 
                                        const std::unordered_map<int, int>& startExitDistances,
                                        const std::unordered_map<int, int>& goalEntryDistances) const {
    if (!pipeline_) return -1;
    
    const auto& clusters = pipeline_->getClusters();
    
    // Dijkstra's algorithm on cluster graph
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq; // (distance, cluster)
    std::unordered_map<int, int> distances;
    
    // Initialize distances
    for (const auto& c : clusters) {
        distances[c.id] = std::numeric_limits<int>::max();
    }
    
    // Start from each exit point in start cluster
    for (const auto& [exitPoint, distToExit] : startExitDistances) {
        // Find which neighbor this exit point connects to
        const Cluster* startClusterObj = nullptr;
        for (const auto& c : clusters) {
            if (c.id == startCluster) {
                startClusterObj = &c;
                break;
            }
        }
        if (!startClusterObj) continue;
        
        for (int neighborID : startClusterObj->neighbors) {
            int cA = std::min(startCluster, neighborID);
            int cB = std::max(startCluster, neighborID);
            auto it = clusterDistances_.find({cA, cB});
            if (it == clusterDistances_.end()) continue;
            
            const auto& cpd = it->second;
            int pointInStartCluster = (cpd.clusterA == startCluster) ? cpd.entryCell : cpd.exitCell;
            
            if (pointInStartCluster == exitPoint) {
                // This exit point connects to this neighbor
                int newDist = distToExit + cpd.distance;
                if (newDist < distances[neighborID]) {
                    distances[neighborID] = newDist;
                    pq.push({newDist, neighborID});
                }
            }
        }
    }
    
    int minTotalDistance = std::numeric_limits<int>::max();
    
    while (!pq.empty()) {
        auto [currentDist, current] = pq.top();
        pq.pop();
        
        // Skip if we've already found a better path
        if (currentDist > distances[current]) continue;
        
        // Find current cluster
        const Cluster* currentCluster = nullptr;
        for (const auto& c : clusters) {
            if (c.id == current) {
                currentCluster = &c;
                break;
            }
        }
        
        if (!currentCluster) continue;
        
        // Explore neighbors
        for (int neighbor : currentCluster->neighbors) {
            int cA = std::min(current, neighbor);
            int cB = std::max(current, neighbor);
            auto it = clusterDistances_.find({cA, cB});
            
            if (it == clusterDistances_.end()) continue;
            
            const auto& cpd = it->second;
            int edgeWeight = cpd.distance;
            int newDist = distances[current] + edgeWeight;
            
            // Check if this neighbor is the goal cluster
            if (neighbor == goalCluster) {
                // We're crossing into the goal cluster
                // Find which entry point this connection uses
                int entryPointInGoal = (cpd.clusterA == goalCluster) ? cpd.entryCell : cpd.exitCell;
                
                // Check if we have distance from this entry point to goal
                auto goalIt = goalEntryDistances.find(entryPointInGoal);
                if (goalIt != goalEntryDistances.end()) {
                    int totalDist = newDist + goalIt->second;
                    minTotalDistance = std::min(minTotalDistance, totalDist);
                }
            } else {
                // Regular neighbor exploration
                if (newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    pq.push({newDist, neighbor});
                }
            }
        }
    }
    
    return (minTotalDistance == std::numeric_limits<int>::max()) ? -1 : minTotalDistance;
}

void ClusterHeuristics::setAgentPlan(const AgentPlan& plan, int currentTimestep) {
    if (!initialized_ || !env_ || plan.agentID < 0) return;
    
    // Remove old reservations if this agent already has a plan
    auto it = agentPlans_.find(plan.agentID);
    if (it != agentPlans_.end()) {
        // Remove from cluster congestion
        for (auto& [clusterID, congestion] : clusterCongestion_) {
            congestion.removeAgentReservations(plan.agentID);
        }
        // Remove from edge congestion
        for (auto& [edge, congestion] : edgeCongestion_) {
            congestion.removeAgentReservations(plan.agentID);
        }
    }
    
    // Store the new plan
    agentPlans_[plan.agentID] = plan;
    
    // Convert plan to time interval reservations
    convertPlanToReservations(plan, currentTimestep);
}

void ClusterHeuristics::convertPlanToReservations(const AgentPlan& plan, int currentTimestep) {
    if (plan.isEmpty()) return;
    
    // Assume unit speed (1 cell per timestep) for now
    // TODO: Get actual agent speed from SharedEnvironment if available
    const int speed = 1;
    
    int currentTime = currentTimestep;
    
    for (size_t i = 0; i < plan.path.size(); ++i) {
        const auto& step = plan.path[i];
        
        if (step.distanceEstimate <= 0) continue;
        
        // Calculate time interval for this cluster
        int duration = step.distanceEstimate / speed;
        if (duration == 0) duration = 1; // Minimum 1 timestep
        
        int startTime = currentTime;
        int endTime = currentTime + duration - 1;
        
        // Add reservation to cluster congestion
        auto& clusterCong = clusterCongestion_[step.clusterID];
        if (clusterCong.clusterID == -1) {
            clusterCong.clusterID = step.clusterID;
            // TODO: Initialize capacityScore if needed
        }
        clusterCong.addReservation(plan.agentID, startTime, endTime);
        
        // Add reservation to edge congestion if not the first step
        if (i > 0) {
            const auto& prevStep = plan.path[i - 1];
            int clusterA = std::min(prevStep.clusterID, step.clusterID);
            int clusterB = std::max(prevStep.clusterID, step.clusterID);
            
            auto edgeKey = std::make_pair(clusterA, clusterB);
            auto& edgeCong = edgeCongestion_[edgeKey];
            if (edgeCong.clusterA == -1) {
                edgeCong.clusterA = clusterA;
                edgeCong.clusterB = clusterB;
            }
            
            // Edge reservation spans the transition between clusters
            edgeCong.addReservation(plan.agentID, startTime, startTime);
        }
        
        // Advance time for next cluster
        currentTime = endTime + 1;
    }
}

float ClusterHeuristics::getClusterCongestionRatio(int clusterID, int timestep) const {
    auto it = clusterCongestion_.find(clusterID);
    if (it == clusterCongestion_.end()) return 0.0f;
    return it->second.getCongestionRatioAt(timestep);
}

int ClusterHeuristics::getClusterOccupancy(int clusterID, int timestep) const {
    auto it = clusterCongestion_.find(clusterID);
    if (it == clusterCongestion_.end()) return 0;
    return it->second.getOccupancyAt(timestep);
}

int ClusterHeuristics::getEdgeUsage(int clusterA, int clusterB, int timestep) const {
    int minCluster = std::min(clusterA, clusterB);
    int maxCluster = std::max(clusterA, clusterB);
    
    auto it = edgeCongestion_.find({minCluster, maxCluster});
    if (it == edgeCongestion_.end()) return 0;
    return it->second.getUsageAt(timestep);
}

float ClusterHeuristics::getClusterCapacity(int clusterID) const {
    auto it = clusterCongestion_.find(clusterID);
    if (it == clusterCongestion_.end()) return 0.0f;
    return it->second.capacityScore;
}

bool ClusterHeuristics::hasAgentPlan(int agentID) const {
    return agentPlans_.find(agentID) != agentPlans_.end();
}

const AgentPlan* ClusterHeuristics::getAgentPlan(int agentID) const {
    auto it = agentPlans_.find(agentID);
    if (it == agentPlans_.end()) return nullptr;
    return &(it->second);
}

void ClusterHeuristics::removeAgentPlan(int agentID) {
    // Remove from agent plans
    agentPlans_.erase(agentID);
    
    // Remove all reservations
    for (auto& [clusterID, congestion] : clusterCongestion_) {
        congestion.removeAgentReservations(agentID);
    }
    for (auto& [edge, congestion] : edgeCongestion_) {
        congestion.removeAgentReservations(agentID);
    }
}

void ClusterHeuristics::pruneExpiredReservations(int currentTimestep) {
    // Prune cluster reservations
    for (auto& [clusterID, congestion] : clusterCongestion_) {
        congestion.pruneExpiredReservations(currentTimestep);
    }
    
    // Prune edge reservations
    for (auto& [edge, congestion] : edgeCongestion_) {
        congestion.pruneExpiredReservations(currentTimestep);
    }
}

} // namespace clustering
