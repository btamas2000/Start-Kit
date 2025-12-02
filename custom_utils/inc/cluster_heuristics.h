#ifndef CLUSTER_HEURISTICS_H
#define CLUSTER_HEURISTICS_H

#include "clustering.h"
#include <unordered_map>
#include <queue>
#include <deque>
#include "SharedEnv.h"

namespace clustering {

// Structure to represent a single step in an agent's cluster-level path
struct ClusterTransition {
    int clusterID;
    int entryPoint;         // Cell location where agent enters cluster (-1 if starting cluster)
    int exitPoint;          // Cell location where agent exits cluster (-1 if this is final/goal cluster)
    bool isGoalLocation;    // True if agent has a goal to reach in this cluster
    int goalLocation;       // Actual goal cell location if isGoalLocation=true, -1 otherwise
    int distanceEstimate;   // Estimated distance to traverse through this cluster
    
    ClusterTransition() 
        : clusterID(-1), entryPoint(-1), exitPoint(-1), 
          isGoalLocation(false), goalLocation(-1),
          distanceEstimate(-1) {}
    
    ClusterTransition(int cluster, int entry, int exit, bool hasGoal = false, int goal = -1,
                     int distance = -1)
        : clusterID(cluster), entryPoint(entry), exitPoint(exit),
          isGoalLocation(hasGoal), goalLocation(goal),
          distanceEstimate(distance) {}
};

// Structure to store an agent's planned path through clusters
struct AgentPlan {
    int agentID;
    std::deque<ClusterTransition> path;  // Sequence of cluster transitions
    
    AgentPlan() : agentID(-1) {}
    explicit AgentPlan(int id) : agentID(id) {}
    
    // Helper methods
    bool isEmpty() const { return path.empty(); }
    size_t size() const { return path.size(); }
    const ClusterTransition& getCurrentStep() const { return path.front(); }
    const ClusterTransition& getStep(size_t index) const { return path[index]; }
    void advanceStep() { if (!path.empty()) path.pop_front(); }
    void addStep(const ClusterTransition& step) { path.push_back(step); }
    void clear() { path.clear(); }
    
    // Distance-related methods
    int getTotalDistanceEstimate() const {
        int total = 0;
        for (const auto& step : path) {
            if (step.distanceEstimate > 0) {
                total += step.distanceEstimate;
            }
        }
        return total;
    }
};

// Compact time interval for reservations
struct TimeInterval {
    int agentID;
    int startTime;
    int endTime;
    
    TimeInterval() : agentID(-1), startTime(-1), endTime(-1) {}
    
    TimeInterval(int agent, int start, int end) 
        : agentID(agent), startTime(start), endTime(end) {}
    
    // Check if timestep is within this interval
    bool contains(int timestep) const {
        return timestep >= startTime && timestep <= endTime;
    }
    
    // Check if two intervals overlap in time
    bool overlaps(const TimeInterval& other) const {
        return !(endTime < other.startTime || other.endTime < startTime);
    }
    
    // Get duration of this interval
    int duration() const {
        if (startTime < 0 || endTime < 0) return 0;
        return endTime - startTime + 1;
    }
};

// Per-cluster congestion tracking with time intervals
struct ClusterCongestion {
    int clusterID;
    float capacityScore;  // Precomputed from cluster.size and cluster.maxima
    std::vector<TimeInterval> reservedAt;  // Agents occupying this cluster over time
    
    ClusterCongestion() : clusterID(-1), capacityScore(0.0f) {}
    explicit ClusterCongestion(int id, float capacity = 0.0f) 
        : clusterID(id), capacityScore(capacity) {}
    
    // Query occupancy at specific timestep
    int getOccupancyAt(int timestep) const {
        int count = 0;
        for (const auto& interval : reservedAt) {
            if (interval.contains(timestep)) count++;
        }
        return count;
    }
    
    // Get congestion ratio at timestep
    float getCongestionRatioAt(int timestep) const {
        if (capacityScore <= 0.0f) return 1000.0f;
        return static_cast<float>(getOccupancyAt(timestep)) / capacityScore;
    }
    
    // Add a reservation for an agent
    void addReservation(int agentID, int startTime, int endTime) {
        reservedAt.emplace_back(agentID, startTime, endTime);
    }
    
    // Remove all reservations for a specific agent
    void removeAgentReservations(int agentID) {
        reservedAt.erase(
            std::remove_if(reservedAt.begin(), reservedAt.end(),
                [agentID](const TimeInterval& interval) {
                    return interval.agentID == agentID;
                }),
            reservedAt.end()
        );
    }
    
    // Clean up expired reservations (before given timestep)
    void pruneExpiredReservations(int currentTimestep) {
        reservedAt.erase(
            std::remove_if(reservedAt.begin(), reservedAt.end(),
                [currentTimestep](const TimeInterval& interval) {
                    return interval.endTime < currentTimestep;
                }),
            reservedAt.end()
        );
    }
};

// Per-edge congestion tracking with time intervals
struct EdgeCongestion {
    int clusterA;
    int clusterB;
    std::vector<TimeInterval> reservedAt;  // Agents using this edge over time
    
    EdgeCongestion() : clusterA(-1), clusterB(-1) {}
    EdgeCongestion(int a, int b) : clusterA(a), clusterB(b) {}
    
    // Query usage at specific timestep
    int getUsageAt(int timestep) const {
        int count = 0;
        for (const auto& interval : reservedAt) {
            if (interval.contains(timestep)) count++;
        }
        return count;
    }
    
    // Add a reservation for an agent
    void addReservation(int agentID, int startTime, int endTime) {
        reservedAt.emplace_back(agentID, startTime, endTime);
    }
    
    // Remove all reservations for a specific agent
    void removeAgentReservations(int agentID) {
        reservedAt.erase(
            std::remove_if(reservedAt.begin(), reservedAt.end(),
                [agentID](const TimeInterval& interval) {
                    return interval.agentID == agentID;
                }),
            reservedAt.end()
        );
    }
    
    // Clean up expired reservations
    void pruneExpiredReservations(int currentTimestep) {
        reservedAt.erase(
            std::remove_if(reservedAt.begin(), reservedAt.end(),
                [currentTimestep](const TimeInterval& interval) {
                    return interval.endTime < currentTimestep;
                }),
            reservedAt.end()
        );
    }
};

// Structure to store precomputed distance between cluster pair
struct ClusterPairDistance {
    int clusterA;
    int clusterB;
    int entryCell;  // Cell in clusterA at the boundary
    int exitCell;   // Cell in clusterB at the boundary
    int distance;
};

// Hash function for pair<int, int> to use in unordered_map
struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

// Singleton class for cluster-based heuristics
class ClusterHeuristics {
public:
    // Get singleton instance
    static ClusterHeuristics& getInstance();
    
    // Delete copy constructor and assignment operator
    ClusterHeuristics(const ClusterHeuristics&) = delete;
    ClusterHeuristics& operator=(const ClusterHeuristics&) = delete;
    
    // Initialize with map data - runs clustering internally
    void initialize(const std::vector<int>& map, int rows, int cols, SharedEnvironment* env);
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Get precomputed distance between two clusters
    // Returns -1 if clusters are not neighbors or if not found
    int getClusterDistance(int clusterA, int clusterB) const;
    
    // Get cluster ID for a given location
    int getClusterID(int loc) const;
    
    // Get the clustering pipeline (for access to cluster data)
    const ClusteringPipeline* getPipeline() const { return pipeline_; }
    
    // Compute heuristic distance from location A to location B using cluster distances
    // Returns -1 if no path exists
    int getHeuristicDistance(int startLoc, int goalLoc) const;
    
    // Plan management
    // Set or update an agent's plan - removes old reservations and adds new ones
    void setAgentPlan(const AgentPlan& plan, int currentTimestep);
    
    // Query methods
    // Get congestion ratio for a cluster at specific timestep
    float getClusterCongestionRatio(int clusterID, int timestep) const;
    
    // Get number of agents occupying a cluster at specific timestep
    int getClusterOccupancy(int clusterID, int timestep) const;
    
    // Get number of agents using an edge at specific timestep
    int getEdgeUsage(int clusterA, int clusterB, int timestep) const;
    
    // Get capacity score for a cluster
    float getClusterCapacity(int clusterID) const;
    
    // Check if an agent has a registered plan
    bool hasAgentPlan(int agentID) const;
    
    // Get an agent's plan (returns nullptr if not found)
    const AgentPlan* getAgentPlan(int agentID) const;
    
    // Remove an agent's plan and all reservations
    void removeAgentPlan(int agentID);
    
    // Clean up expired reservations before given timestep
    void pruneExpiredReservations(int currentTimestep);
    
private:
    // Private constructor for singleton
    ClusterHeuristics();
    ~ClusterHeuristics() = default;
    
    // Precompute distances between neighboring clusters
    void precomputeClusterDistances();
    
    // Helper: compute distance between two cells using BFS
    int computePathDistance(int startLoc, int goalLoc) const;
    
    // Helper: find boundary cells between two neighboring clusters
    std::vector<std::pair<int, int>> findBoundaryCells(int clusterA, int clusterB) const;
    
    // Helper: compute distances from a location to precomputed exit points in its cluster
    std::unordered_map<int, int> computeDistancesToExitPoints(int loc, int clusterID) const;
    
    // Helper: find shortest path between clusters using Dijkstra on cluster graph
    // Incorporates start and goal intra-cluster distances
    int findClusterPath(int startCluster, int goalCluster,
                        const std::unordered_map<int, int>& startExitDistances,
                        const std::unordered_map<int, int>& goalEntryDistances) const;
    
    // Helper: convert distance-based plan to time intervals and update congestion structures
    void convertPlanToReservations(const AgentPlan& plan, int currentTimestep);
    
    // Data members
    bool initialized_;
    ClusteringPipeline* pipeline_;
    SharedEnvironment* env_;  // Reference to shared environment for dynamic info
    
    // Storage for precomputed cluster pair distances
    // Key: (clusterA_id, clusterB_id) where clusterA_id < clusterB_id
    // Value: ClusterPairDistance with distance info
    std::unordered_map<std::pair<int, int>, ClusterPairDistance, PairHash> clusterDistances_;
    
    // Agent plans storage
    // Key: agent ID, Value: agent's planned cluster-level path
    std::unordered_map<int, AgentPlan> agentPlans_;
    
    // Congestion tracking
    // Key: cluster ID, Value: congestion info for that cluster
    std::unordered_map<int, ClusterCongestion> clusterCongestion_;
    
    // Edge usage tracking
    // Key: (clusterA_id, clusterB_id) where clusterA_id < clusterB_id, Value: edge congestion info
    std::unordered_map<std::pair<int, int>, EdgeCongestion, PairHash> edgeCongestion_;
};

} // namespace clustering

#endif // CLUSTER_HEURISTICS_H
