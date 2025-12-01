#ifndef CLUSTER_HEURISTICS_H
#define CLUSTER_HEURISTICS_H

#include "clustering.h"
#include <vector>
#include <unordered_map>
#include <memory>

namespace clustering {

// Singleton class for cluster-based heuristics
// Provides fast distance estimates and routing utilities using cluster abstraction
class ClusterHeuristics {
public:
    // Get singleton instance
    static ClusterHeuristics& getInstance();
    
    // Delete copy constructor and assignment operator
    ClusterHeuristics(const ClusterHeuristics&) = delete;
    ClusterHeuristics& operator=(const ClusterHeuristics&) = delete;
    
    // Initialize with clustering results
    void initialize(const ClusteringPipeline& pipeline);
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Core heuristic functions
    int getHeuristic(int fromLoc, int toLoc) const;
    int getIntraClusterDistance(int fromLoc, int toLoc, int clusterID) const;
    int getInterClusterDistance(int fromCluster, int toCluster) const;
    
    // Cluster query functions
    int getClusterID(int loc) const;
    const Cluster* getCluster(int clusterID) const;
    const std::vector<Cluster>& getClusters() const { return clusters_; }
    
    // Utility functions
    bool areInSameCluster(int loc1, int loc2) const;
    bool areNeighborClusters(int cluster1, int cluster2) const;
    int getClusterCentroid(int clusterID) const;
    
    // Distance transform access
    int getDistanceTransform(int loc) const;
    
    // Grid dimensions
    int getRows() const { return rows_; }
    int getCols() const { return cols_; }
    
private:
    // Private constructor for singleton
    ClusterHeuristics();
    ~ClusterHeuristics() = default;
    
    // Initialization helpers
    void computeClusterCentroids();
    void computeInterClusterDistances();
    void buildIntraClusterDistances();
    
    // Helper functions
    inline int getRow(int loc) const { return loc / cols_; }
    inline int getCol(int loc) const { return loc % cols_; }
    inline int getLoc(int row, int col) const { return row * cols_ + col; }
    int manhattanDistance(int loc1, int loc2) const;
    
    // Data members
    bool initialized_;
    int rows_;
    int cols_;
    
    std::vector<Cluster> clusters_;
    std::vector<int> cellToClusterMap_;
    std::vector<int> distances_;  // Distance transform
    
    // Precomputed data for fast queries
    std::vector<int> clusterCentroids_;  // Representative point per cluster
    std::unordered_map<int, std::unordered_map<int, int>> interClusterDistances_;  // cluster_id -> cluster_id -> distance
    
    // Optional: intra-cluster distance tables for small clusters
    std::unordered_map<int, std::vector<std::vector<int>>> intraClusterDistTables_;
};

} // namespace clustering

#endif // CLUSTER_HEURISTICS_H
