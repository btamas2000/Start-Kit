#include "../inc/clustering.h"
#include <algorithm>
#include <queue>
#include <limits>
#include <iostream>

namespace clustering {

// Static member initialization - direction offsets for 4-connectivity
const int ClusteringPipeline::DR[4] = {-1, 1, 0, 0};
const int ClusteringPipeline::DC[4] = {0, 0, -1, 1};

ClusteringPipeline::ClusteringPipeline(const std::vector<int>& map, int rows, int cols)
    : rows_(rows), cols_(cols), map_(map) {
    distances_.resize(rows_ * cols_, -1);
    cellToClusterMap_.resize(rows_ * cols_, -1);
}

ClusteringPipeline::~ClusteringPipeline() {}

void ClusteringPipeline::computeDistanceTransform() {
    // Create padded map with obstacles on borders
    int paddedRows = rows_ + 2;
    int paddedCols = cols_ + 2;
    int paddedSize = paddedRows * paddedCols;
    
    std::vector<int> paddedMap(paddedSize, 1);  // Initialize with obstacles
    std::vector<int> paddedDists(paddedSize, -1);
    
    // Copy original map to padded version
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            int paddedLoc = (r + 1) * paddedCols + (c + 1);
            int origLoc = r * cols_ + c;
            paddedMap[paddedLoc] = map_[origLoc];
        }
    }
    
    // BFS from all obstacles
    std::queue<int> q;
    
    for (int loc = 0; loc < paddedSize; loc++) {
        if (paddedMap[loc] == 1) {  // Obstacle
            paddedDists[loc] = 0;
            q.push(loc);
        }
    }
    
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        
        int r = curr / paddedCols;
        int c = curr % paddedCols;
        
        for (int i = 0; i < 4; i++) {
            int nr = r + DR[i];
            int nc = c + DC[i];
            
            if (nr >= 0 && nr < paddedRows && nc >= 0 && nc < paddedCols) {
                int next = nr * paddedCols + nc;
                if (paddedDists[next] == -1) {
                    paddedDists[next] = paddedDists[curr] + 1;
                    q.push(next);
                }
            }
        }
    }
    
    // Extract original region back to 1D distances
    distances_.clear();
    distances_.resize(rows_ * cols_);
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            int origLoc = r * cols_ + c;
            int paddedLoc = (r + 1) * paddedCols + (c + 1);
            distances_[origLoc] = paddedDists[paddedLoc];
        }
    }
}

std::pair<int, int> ClusteringPipeline::findGlobalMinMax() const {
    int minVal = -1;
    int maxVal = -1;
    
    for (int dist : distances_) {
        if (minVal == -1 || dist < minVal) minVal = dist;
        if (maxVal == -1 || dist > maxVal) maxVal = dist;
    }
    
    return {minVal, maxVal};
}

std::vector<int> ClusteringPipeline::selectCellsAtDistance(int targetDistance) const {
    std::vector<int> cells;
    
    for (int loc = 0; loc < static_cast<int>(distances_.size()); loc++) {
        if (distances_[loc] == targetDistance) {
            cells.push_back(loc);
        }
    }
    
    return cells;
}

std::vector<std::pair<int, std::vector<int>>> 
ClusteringPipeline::findLocalMaximas(int threshold) {
    auto [minVal, maxVal] = findGlobalMinMax();
    std::vector<std::pair<int, std::vector<int>>> localMaximaPlateaus;
    std::unordered_set<int> plateauCells;
    std::unordered_set<int> falsePlateaus;
    
    for (int i = maxVal; i >= minVal; i--) {
        if (i < threshold) break;
        
        auto cellsI = selectCellsAtDistance(i);
        
        for (int loc : cellsI) {
            if (plateauCells.find(loc) != plateauCells.end()) continue;
            if (falsePlateaus.find(loc) != falsePlateaus.end()) continue;
            
            std::vector<int> potentialMaximumPlateau;
            std::unordered_set<int> plateauSet;
            bool falsePlateau = false;
            
            std::queue<int> q;
            q.push(loc);
            potentialMaximumPlateau.push_back(loc);
            plateauSet.insert(loc);
            
            while (!q.empty() && !falsePlateau) {
                int curr = q.front();
                q.pop();
                
                int r = getRow(curr);
                int c = getCol(curr);
                
                for (int d = 0; d < 4; d++) {
                    int nr = r + DR[d];
                    int nc = c + DC[d];
                    
                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next = getLoc(nr, nc);
                        
                        if (distances_[next] > distances_[curr]) {
                            falsePlateau = true;
                            break;
                        } else if (falsePlateaus.find(next) != falsePlateaus.end()) {
                            falsePlateau = true;
                            break;
                        } else if (distances_[next] == distances_[curr]) {
                            if (plateauSet.find(next) == plateauSet.end()) {
                                potentialMaximumPlateau.push_back(next);
                                plateauSet.insert(next);
                                q.push(next);
                            }
                        }
                    }
                }
            }
            
            if (!falsePlateau) {
                localMaximaPlateaus.push_back({i, potentialMaximumPlateau});
                for (int pLoc : potentialMaximumPlateau) {
                    plateauCells.insert(pLoc);
                }
            } else {
                for (int pLoc : potentialMaximumPlateau) {
                    falsePlateaus.insert(pLoc);
                }
            }
        }
    }
    
    return localMaximaPlateaus;
}

std::vector<std::pair<int, std::vector<int>>> ClusteringPipeline::joinDiagonalMaxima(
    const std::vector<std::pair<int, std::vector<int>>>& plateaus) {
    
    std::vector<std::pair<int, std::vector<int>>> joinedPlateaus;
    
    const int allDR[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
    const int allDC[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    std::vector<int> maximaMap(rows_ * cols_, -1);
    for (int i = 0; i < plateaus.size(); i++) {
        for (int loc : plateaus[i].second) {
            maximaMap[loc] = i;
        }
    }
    
    for (const auto& plateau : plateaus) {
        int maxima = plateau.first;
        std::vector<int> currentPlateauCells;
        std::unordered_set<int> visited;
        std::queue<int> q;
        for (int loc : plateau.second) {
            currentPlateauCells.push_back(loc);
            q.push(loc);
            visited.insert(loc);
        }
        while (!q.empty()) {
            int curr = q.front();
            q.pop();

            int r = getRow(curr);
            int c = getCol(curr);
            for (int d = 0; d < 8; d++) {
                
            }
        }
    }
    
    return joinedPlateaus;
}

std::vector<Cluster> ClusteringPipeline::formInitialClusters(
    const std::vector<std::pair<int, std::vector<int>>>& localMaxima,
    float dropFraction) {
    
    std::vector<int> clusterMap(rows_ * cols_, -1);
    
    struct QueueItem {
        int loc;
        int maxima;
        int maxExpansion;
    };
    
    std::queue<QueueItem> q;
    
    // Initialize with maxima plateaus
    for (size_t i = 0; i < localMaxima.size(); i++) {
        int maxima = localMaxima[i].first;
        for (int loc : localMaxima[i].second) {
            clusterMap[loc] = i;
            q.push({loc, maxima, maxima - 2});
        }
    }
    
    // Expand clusters
    while (!q.empty()) {
        auto [curr, maxima, maxExpansion] = q.front();
        q.pop();
        
        int r = getRow(curr);
        int c = getCol(curr);
        
        for (int i = 0; i < 4; i++) {
            int nr = r + DR[i];
            int nc = c + DC[i];
            
            if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                int next = getLoc(nr, nc);
                
                if (distances_[next] <= distances_[curr] && 
                    distances_[next] > 0 && 
                    distances_[next] >= dropFraction * maxima) {
                    if (clusterMap[next] == -1) {
                        clusterMap[next] = clusterMap[curr];
                        if (maxExpansion > 0) {
                            q.push({next, maxima, maxExpansion - 1});
                        }
                    }
                }
            }
        }
    }
    
    // Collect clusters
    std::vector<Cluster> clusters;
    for (size_t i = 0; i < localMaxima.size(); i++) {
        Cluster cluster;
        cluster.id = i;
        cluster.degree = 0;
        cluster.maxima = localMaxima[i].first;
        
        for (int loc = 0; loc < static_cast<int>(clusterMap.size()); loc++) {
            if (clusterMap[loc] == static_cast<int>(i)) {
                cluster.cells.push_back(loc);
            }
        }
        
        cluster.size = cluster.cells.size();
        clusters.push_back(cluster);
    }
    
    return clusters;
}

std::vector<Cluster> ClusteringPipeline::clusterDT1Cells(
    const std::vector<Cluster>& clusters, int minSize) {
    
    std::vector<Cluster> updatedClusters = clusters;
    int clusterId = updatedClusters.size();
    
    std::vector<int> dt1Map(rows_ * cols_, -1);
    
    // Mark non-DT1 cells
    for (int loc = 0; loc < static_cast<int>(distances_.size()); loc++) {
        if (distances_[loc] != 1) {
            dt1Map[loc] = -2;
        }
    }
    
    // Find DT1 connected components
    for (int loc = 0; loc < static_cast<int>(dt1Map.size()); loc++) {
        if (dt1Map[loc] == -1) {
            std::queue<int> q;
            std::vector<int> currentClusterCells;
            std::unordered_set<int> visited;
            
            q.push(loc);
            currentClusterCells.push_back(loc);
            visited.insert(loc);
            
            while (!q.empty()) {
                int curr = q.front();
                q.pop();
                
                int r = getRow(curr);
                int c = getCol(curr);
                
                for (int i = 0; i < 4; i++) {
                    int nr = r + DR[i];
                    int nc = c + DC[i];
                    
                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next = getLoc(nr, nc);
                        if (dt1Map[next] == -1 && visited.find(next) == visited.end()) {
                            currentClusterCells.push_back(next);
                            visited.insert(next);
                            q.push(next);
                        }
                    }
                }
            }
            
            if (static_cast<int>(currentClusterCells.size()) >= minSize) {
                for (int cell : currentClusterCells) {
                    dt1Map[cell] = clusterId;
                }
                
                Cluster cluster;
                cluster.id = clusterId;
                cluster.degree = 0;
                cluster.maxima = 1;
                cluster.cells = currentClusterCells;
                cluster.size = currentClusterCells.size();
                
                updatedClusters.push_back(cluster);
                clusterId++;
            } else {
                for (int cell : currentClusterCells) {
                    dt1Map[cell] = -2;
                }
            }
        }
    }
    
    return updatedClusters;
}

std::vector<Cluster> ClusteringPipeline::assignLeftoverCells(const std::vector<Cluster>& clusters) {
    std::vector<Cluster> updatedClusters = clusters;
    
    // Sort clusters by maxima
    std::vector<Cluster*> sortedClusters;
    for (auto& cluster : updatedClusters) {
        sortedClusters.push_back(&cluster);
    }
    std::sort(sortedClusters.begin(), sortedClusters.end(),
              [](const Cluster* a, const Cluster* b) { return a->maxima < b->maxima; });
    
    std::vector<int> assigningMap(rows_ * cols_, -1);
    
    struct QueueItem {
        int loc;
        int clusterId;
    };
    std::queue<QueueItem> q;
    
    // Initialize queue with existing cluster cells
    for (auto* cluster : sortedClusters) {
        for (int loc : cluster->cells) {
            assigningMap[loc] = cluster->id;
            q.push({loc, cluster->id});
        }
    }
    
    // Expand to unassigned cells
    while (!q.empty()) {
        auto [curr, clusterId] = q.front();
        q.pop();
        
        int r = getRow(curr);
        int c = getCol(curr);
        
        for (int i = 0; i < 4; i++) {
            int nr = r + DR[i];
            int nc = c + DC[i];
            
            if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                int next = getLoc(nr, nc);
                if (assigningMap[next] == -1 && distances_[next] > 0) {
                    assigningMap[next] = clusterId;
                    q.push({next, clusterId});
                }
            }
        }
    }
    
    // Rebuild cluster cell lists
    for (auto& cluster : updatedClusters) {
        cluster.cells.clear();
        for (int loc = 0; loc < static_cast<int>(assigningMap.size()); loc++) {
            if (assigningMap[loc] == cluster.id) {
                cluster.cells.push_back(loc);
            }
        }
        cluster.size = cluster.cells.size();
    }
    
    return updatedClusters;
}

std::vector<Cluster> ClusteringPipeline::updateClusterNeighbors(const std::vector<Cluster>& clusters) {
    std::vector<Cluster> updatedClusters = clusters;
    
    std::vector<int> clusterMap(rows_ * cols_, -1);
    
    for (const auto& cluster : updatedClusters) {
        for (int loc : cluster.cells) {
            clusterMap[loc] = cluster.id;
        }
    }
    
    for (auto& cluster : updatedClusters) {
        std::unordered_set<int> neighborSet;
        
        for (int loc : cluster.cells) {
            int r = getRow(loc);
            int c = getCol(loc);
            
            for (int i = 0; i < 4; i++) {
                int nr = r + DR[i];
                int nc = c + DC[i];
                
                if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                    int next = getLoc(nr, nc);
                    int neighborId = clusterMap[next];
                    if (neighborId != -1 && neighborId != cluster.id) {
                        neighborSet.insert(neighborId);
                    }
                }
            }
        }
        
        cluster.neighbors.clear();
        cluster.neighbors.assign(neighborSet.begin(), neighborSet.end());
        cluster.degree = cluster.neighbors.size();
    }
    
    return updatedClusters;
}

void ClusteringPipeline::buildCellToClusterMap() {
    // Reset map
    std::fill(cellToClusterMap_.begin(), cellToClusterMap_.end(), -1);
    
    // Populate cell-to-cluster mapping
    for (const auto& cluster : clusters_) {
        for (int loc : cluster.cells) {
            cellToClusterMap_[loc] = cluster.id;
        }
    }
}

void ClusteringPipeline::runClustering(int threshold, float dropFraction, int minCorridorSize) {
    // Step 1: Compute distance transform
    computeDistanceTransform();
    
    // Step 2: Find local maxima
    auto plateaus = findLocalMaximas(threshold);

    // Step 3: Join diagonally connected maxima cells in single plateaus
    plateaus = joinDiagonalMaxima(plateaus);
    
    // Step 4: Form initial clusters
    auto initialClusters = formInitialClusters(plateaus, dropFraction);
    
    // Step 5: Cluster DT=1 cells
    auto clustersWithDT1 = clusterDT1Cells(initialClusters, minCorridorSize);
    
    // Step 6: Assign leftover cells
    clusters_ = assignLeftoverCells(clustersWithDT1);
    
    // Step 7: Update cluster neighbors
    clusters_ = updateClusterNeighbors(clusters_);
    
    // Step 8: Build cell-to-cluster lookup map
    buildCellToClusterMap();
}

} // namespace clustering
