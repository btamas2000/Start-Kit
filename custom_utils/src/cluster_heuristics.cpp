#include "../inc/cluster_heuristics.h"
#include <algorithm>
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
    : initialized_(false), rows_(0), cols_(0) {
}

void ClusterHeuristics::initialize(const ClusteringPipeline& pipeline) {
    // Copy data from clustering pipeline
    rows_ = pipeline.getRows();
    cols_ = pipeline.getCols();
    clusters_ = pipeline.getClusters();
    cellToClusterMap_ = pipeline.getCellToClusterMap();
    distances_ = pipeline.getDistances();
    
    // Precompute heuristic data
    computeClusterCentroids();
    computeInterClusterDistances();
    // buildIntraClusterDistances(); // Optional, uncomment if needed
    
    initialized_ = true;
}

void ClusterHeuristics::computeClusterCentroids() {
    clusterCentroids_.resize(clusters_.size());
    
    for (const auto& cluster : clusters_) {
        if (cluster.cells.empty()) {
            clusterCentroids_[cluster.id] = -1;
            continue;
        }
        
        // Find cell closest to geometric center
        int sumRow = 0, sumCol = 0;
        for (int loc : cluster.cells) {
            sumRow += getRow(loc);
            sumCol += getCol(loc);
        }
        
        int centerRow = sumRow / cluster.cells.size();
        int centerCol = sumCol / cluster.cells.size();
        int centerLoc = getLoc(centerRow, centerCol);
        
        // Find actual cell in cluster closest to center
        int bestLoc = cluster.cells[0];
        int bestDist = manhattanDistance(bestLoc, centerLoc);
        
        for (int loc : cluster.cells) {
            int dist = manhattanDistance(loc, centerLoc);
            if (dist < bestDist) {
                bestDist = dist;
                bestLoc = loc;
            }
        }
        
        clusterCentroids_[cluster.id] = bestLoc;
    }
}

void ClusterHeuristics::computeInterClusterDistances() {
    // BFS from each cluster to compute abstract distances
    for (const auto& cluster : clusters_) {
        std::vector<int> clusterDist(clusters_.size(), -1);
        std::queue<int> q;
        
        clusterDist[cluster.id] = 0;
        q.push(cluster.id);
        
        while (!q.empty()) {
            int curr = q.front();
            q.pop();
            
            for (int neighbor : clusters_[curr].neighbors) {
                if (clusterDist[neighbor] == -1) {
                    clusterDist[neighbor] = clusterDist[curr] + 1;
                    q.push(neighbor);
                }
            }
        }
        
        // Store distances
        for (size_t i = 0; i < clusterDist.size(); i++) {
            if (clusterDist[i] != -1) {
                interClusterDistances_[cluster.id][i] = clusterDist[i];
            }
        }
    }
}

void ClusterHeuristics::buildIntraClusterDistances() {
    // Optional: Build full distance tables for small clusters
    // Only compute for clusters smaller than a threshold
    const int MAX_CLUSTER_SIZE = 100;
    
    for (const auto& cluster : clusters_) {
        if (cluster.size > MAX_CLUSTER_SIZE) continue;
        
        // Build distance table for this cluster
        std::vector<std::vector<int>> distTable(cluster.cells.size(), 
                                                  std::vector<int>(cluster.cells.size(), 0));
        
        for (size_t i = 0; i < cluster.cells.size(); i++) {
            for (size_t j = i + 1; j < cluster.cells.size(); j++) {
                int dist = manhattanDistance(cluster.cells[i], cluster.cells[j]);
                distTable[i][j] = dist;
                distTable[j][i] = dist;
            }
        }
        
        intraClusterDistTables_[cluster.id] = distTable;
    }
}

int ClusterHeuristics::getHeuristic(int fromLoc, int toLoc) const {
    if (!initialized_) return manhattanDistance(fromLoc, toLoc);
    
    int fromCluster = cellToClusterMap_[fromLoc];
    int toCluster = cellToClusterMap_[toLoc];
    
    // If in same cluster, use Manhattan distance (or intra-cluster table if available)
    if (fromCluster == toCluster) {
        return getIntraClusterDistance(fromLoc, toLoc, fromCluster);
    }
    
    // Different clusters: use abstract distance + boundary estimates
    int abstractDist = getInterClusterDistance(fromCluster, toCluster);
    
    // Add distances from locations to their cluster centroids
    int fromToCentroid = manhattanDistance(fromLoc, clusterCentroids_[fromCluster]);
    int toToCentroid = manhattanDistance(toLoc, clusterCentroids_[toCluster]);
    
    // Estimate based on cluster graph distance and local distances
    // This is admissible but not necessarily tight
    return fromToCentroid + abstractDist * 10 + toToCentroid;  // Scale abstract distance
}

int ClusterHeuristics::getIntraClusterDistance(int fromLoc, int toLoc, int clusterID) const {
    // Check if we have a precomputed table
    auto it = intraClusterDistTables_.find(clusterID);
    if (it != intraClusterDistTables_.end()) {
        // Find indices in cluster cells
        const auto& cluster = clusters_[clusterID];
        auto fromIt = std::find(cluster.cells.begin(), cluster.cells.end(), fromLoc);
        auto toIt = std::find(cluster.cells.begin(), cluster.cells.end(), toLoc);
        
        if (fromIt != cluster.cells.end() && toIt != cluster.cells.end()) {
            int fromIdx = std::distance(cluster.cells.begin(), fromIt);
            int toIdx = std::distance(cluster.cells.begin(), toIt);
            return it->second[fromIdx][toIdx];
        }
    }
    
    // Fallback to Manhattan distance
    return manhattanDistance(fromLoc, toLoc);
}

int ClusterHeuristics::getInterClusterDistance(int fromCluster, int toCluster) const {
    auto it = interClusterDistances_.find(fromCluster);
    if (it != interClusterDistances_.end()) {
        auto it2 = it->second.find(toCluster);
        if (it2 != it->second.end()) {
            return it2->second;
        }
    }
    return -1;  // No path
}

int ClusterHeuristics::getClusterID(int loc) const {
    if (!initialized_ || loc < 0 || loc >= static_cast<int>(cellToClusterMap_.size())) {
        return -1;
    }
    return cellToClusterMap_[loc];
}

const Cluster* ClusterHeuristics::getCluster(int clusterID) const {
    if (!initialized_ || clusterID < 0 || clusterID >= static_cast<int>(clusters_.size())) {
        return nullptr;
    }
    return &clusters_[clusterID];
}

bool ClusterHeuristics::areInSameCluster(int loc1, int loc2) const {
    if (!initialized_) return false;
    return cellToClusterMap_[loc1] == cellToClusterMap_[loc2];
}

bool ClusterHeuristics::areNeighborClusters(int cluster1, int cluster2) const {
    if (!initialized_ || cluster1 < 0 || cluster1 >= static_cast<int>(clusters_.size())) {
        return false;
    }
    
    const auto& neighbors = clusters_[cluster1].neighbors;
    return std::find(neighbors.begin(), neighbors.end(), cluster2) != neighbors.end();
}

int ClusterHeuristics::getClusterCentroid(int clusterID) const {
    if (!initialized_ || clusterID < 0 || clusterID >= static_cast<int>(clusterCentroids_.size())) {
        return -1;
    }
    return clusterCentroids_[clusterID];
}

int ClusterHeuristics::getDistanceTransform(int loc) const {
    if (!initialized_ || loc < 0 || loc >= static_cast<int>(distances_.size())) {
        return -1;
    }
    return distances_[loc];
}

int ClusterHeuristics::manhattanDistance(int loc1, int loc2) const {
    int r1 = getRow(loc1);
    int c1 = getCol(loc1);
    int r2 = getRow(loc2);
    int c2 = getCol(loc2);
    return std::abs(r1 - r2) + std::abs(c1 - c2);
}

} // namespace clustering
