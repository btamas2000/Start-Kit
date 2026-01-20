#include "preprocess.h"
#include <math.h>
#include <algorithm>
#include <iostream>

namespace PreprocessingPipeline {
    void PreprocessPipeline::identifyDistinctAreas() {
        // Implementation for identifying distinct areas in the map

        std::vector<int> areas(rows_ * cols_, -1);

        int area_id = 0;

        for (int r = 0; r < rows_; ++r) {
            for (int c = 0; c < cols_; ++c) {
                int loc = r * cols_ + c;

                if (map_[loc] == 1) { // Obstacle
                    areas[loc] = -2;
                } else if (areas[loc] == -1) {
                    // New area found, perform flood fill to mark all connected cells
                    std::queue<int> q;
                    q.push(loc);
                    areas[loc] = area_id;

                    while (!q.empty()) {
                        int current = q.front();
                        q.pop();

                        int curr_r = current / cols_;
                        int curr_c = current % cols_;

                        // Check 4-connected neighbors
                        const int dr[] = {-1, 1, 0, 0};
                        const int dc[] = {0, 0, -1, 1};

                        for (int d = 0; d < 4; ++d) {
                            int new_r = curr_r + dr[d];
                            int new_c = curr_c + dc[d];

                            if (new_r >= 0 && new_r < rows_ && new_c >= 0 && new_c < cols_) {
                                int neighbor_loc = new_r * cols_ + new_c;

                                if (map_[neighbor_loc] == 0 && areas[neighbor_loc] == -1) {
                                    areas[neighbor_loc] = area_id;
                                    q.push(neighbor_loc);
                                }
                            }
                        }
                    }

                    area_id++;
                }
            }
        }

        areaMap_ = std::move(areas);
    }

    void PreprocessPipeline::computeDistanceTransform() {
        std::vector<int> distances(rows_ * cols_, -1);

        // Implementation for computing distance transform of the map

        int paddedRows = rows_ + 2;
        int paddedCols = cols_ + 2;
        int paddedSize = paddedRows * paddedCols;

        std::vector<int> paddedMap(paddedSize, 1);  // Initialize with obstacles
        std::vector<int> paddedDists(paddedSize, -1);

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

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        while (!q.empty()) {
            int curr = q.front();
            q.pop();
            
            int r = curr / paddedCols;
            int c = curr % paddedCols;
            
            for (int i = 0; i < 4; i++) {
                int nr = r + dr[i];
                int nc = c + dc[i];
                
                if (nr >= 0 && nr < paddedRows && nc >= 0 && nc < paddedCols) {
                    int next = nr * paddedCols + nc;
                    if (paddedDists[next] == -1) {
                        paddedDists[next] = paddedDists[curr] + 1;
                        q.push(next);
                    }
                }
            }
        }

        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                int origLoc = r * cols_ + c;
                int paddedLoc = (r + 1) * paddedCols + (c + 1);
                distances[origLoc] = paddedDists[paddedLoc];
            }
        }


        distanceTransform_ = std::move(distances);
    }

    // Helper
    std::pair<int, int> PreprocessPipeline::findGlobalMinMax() {
        int minVal = -1;
        int maxVal = -1;
        
        for (int dist : distanceTransform_) {
            if (minVal == -1 || dist < minVal) minVal = dist;
            if (maxVal == -1 || dist > maxVal) maxVal = dist;
        }
        
        return {minVal, maxVal};
    }

    // Helper
    std::vector<int> PreprocessPipeline::selectCellsAtDistance(int targetDistance) {
        std::vector<int> cells;
        
        for (int loc = 0; loc < static_cast<int>(distanceTransform_.size()); loc++) {
            if (distanceTransform_[loc] == targetDistance) {
                cells.push_back(loc);
            }
        }
        
        return cells;
    }

    void PreprocessPipeline::findLocalMaximas() {
        auto [minVal, maxVal] = findGlobalMinMax();
        std::vector<std::pair<int, std::vector<int>>> localMaximaPlateaus;
        std::unordered_set<int> plateauCells;
        std::unordered_set<int> falsePlateaus;
        
        for (int i = maxVal; i >= minVal; i--) {
            if (i < 2) break;
            
            auto cellsI = selectCellsAtDistance(i);
            
            for (int loc : cellsI) {
                if (plateauCells.find(loc) != plateauCells.end()) continue;
                if (falsePlateaus.find(loc) != falsePlateaus.end()) continue;
                
                std::vector<int> potentialMaximumPlateau;
                std::unordered_set<int> plateauSet;
                bool falsePlateau = false;
                
                const int dr[] = {-1, 1, 0, 0};
                const int dc[] = {0, 0, -1, 1};

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
                        int nr = r + dr[d];
                        int nc = c + dc[d];
                        
                        if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                            int next = getLoc(nr, nc);
                            
                            if (distanceTransform_[next] > distanceTransform_[curr]) {
                                falsePlateau = true;
                                break;
                            } else if (falsePlateaus.find(next) != falsePlateaus.end()) {
                                falsePlateau = true;
                                break;
                            } else if (distanceTransform_[next] == distanceTransform_[curr]) {
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

        localMaximaPlateaus_ = std::move(localMaximaPlateaus);
    }

    void PreprocessPipeline::mergeLocalMaximas() {
        // Implementation for merging local maximas that are diagonally connected
        // Merge if diagonal local maximas have same distance value

        std::vector<std::pair<int, std::vector<int>>> merged_plateaus;

        std::unordered_set<int> visited;
        std::unordered_set<int> all_maxima_cells;

        for (const auto& plateau : localMaximaPlateaus_) {
            for (int loc : plateau.second) {
                all_maxima_cells.insert(loc);
            }
        }

        const int dr[] = {-1, 1, -1, 1};
        const int dc[] = {-1, 1, 1, -1};

        for (const auto& local_maximas: localMaximaPlateaus_) {
            int maxima_value = local_maximas.first;
            std::vector<int> plateau = local_maximas.second;

            std::unordered_set<int> potential_merge;
            std::queue<int> q;

            for (int loc: plateau) {
                if (visited.find(loc) == visited.end()) {
                    potential_merge.insert(loc);
                    visited.insert(loc);
                    q.push(loc);
                }
            }

            while (!q.empty()) {
                int curr = q.front();
                q.pop();

                int r = getRow(curr);
                int c = getCol(curr);

                for (int d = 0; d < 4; d++) {
                    int nr = r + dr[d];
                    int nc = c + dc[d];

                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next = getLoc(nr, nc);

                        if (potential_merge.find(next) == potential_merge.end() && visited.find(next) == visited.end() && distanceTransform_[next] == maxima_value) {
                            potential_merge.insert(next);
                            visited.insert(next);
                            q.push(next);
                        }
                    }
                }
            }

            if (!potential_merge.empty()) {
                std::vector<int> merged_cells(potential_merge.begin(), potential_merge.end());
                merged_plateaus.push_back({maxima_value, merged_cells});
            }
        }

        localMaximaPlateaus_.clear();
        localMaximaPlateaus_ = std::move(merged_plateaus);
    }

    void PreprocessPipeline::formInitialClusters() {
        // Implementation for forming initial clusters from local maxima plateaus

        clusters_.clear();
        clusterCells_.clear();

        std::vector<int> clusterMap(rows_ * cols_, -1);
    
        struct QueueItem {
            int loc;
            int maxima;
        };
        
        std::queue<QueueItem> q;
        
        // Initialize with maxima plateaus
        for (size_t i = 0; i < localMaximaPlateaus_.size(); i++) {
            int maxima = localMaximaPlateaus_[i].first;
            for (int loc : localMaximaPlateaus_[i].second) {
                clusterMap[loc] = i;
                q.push({loc, maxima});
            }
        }

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        while (!q.empty()) {
            auto [curr, maxima] = q.front();
            q.pop();
            
            int r = getRow(curr);
            int c = getCol(curr);
            
            for (int i = 0; i < 4; i++) {
                int nr = r + dr[i];
                int nc = c + dc[i];
                
                if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                    int next = getLoc(nr, nc);
                    
                    if (distanceTransform_[next] <= distanceTransform_[curr] && distanceTransform_[next] > 0) {
                        if (clusterMap[next] == -1) {
                            clusterMap[next] = clusterMap[curr];
                            if (distanceTransform_[next] > 1) {
                                q.push({next, maxima});
                            }
                        }
                    }
                }
            }
        }
        
        // Collect clusters
        std::vector<Cluster> clusters;
        for (size_t i = 0; i < localMaximaPlateaus_.size(); i++) {
            Cluster cluster;
            cluster.id = i;
            cluster.area_id = areaMap_[localMaximaPlateaus_[i].second[0]];
            cluster.size = 0;
            cluster.maxima = localMaximaPlateaus_[i].first;
            // set cells and size when finalized
            // set rest to whatever
            cluster.cells_begin = -1;
            cluster.cells_end = -1;
            cluster.neighbor_begin = -1;
            cluster.neighbor_end = -1;
            cluster.portal_begin = -1;
            cluster.portal_end = -1;

            clusters.push_back(cluster);
        }

        clusterMap_ = std::move(clusterMap);
        clusters_ = std::move(clusters);
    }

    void PreprocessPipeline::clusterDT1Cells(int minSize) {
        // Implementation for clustering DT1 cells into existing clusters

        std::vector<Cluster> updatedClusters = clusters_;
        int clusterId = updatedClusters.size();
        std::vector<int> clusterMap = clusterMap_;

        std::unordered_set<int> clustered_cells;

        for (const int loc : clusterMap) {
            if (loc != -1) {
                clustered_cells.insert(loc);
            }
        }

        std::unordered_set<int> leftover_dt1_cells;

        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                int loc = getLoc(r, c);
                if (distanceTransform_[loc] == 1 && clustered_cells.find(loc) == clustered_cells.end()) {
                    leftover_dt1_cells.insert(loc);
                }
            }
        }

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        // Sort cells by core potential
        // Best case: dt=1 cell that has 3 neighbor cells (since it is dt=1, there cannot be 4)
        // Second best: dt=1 cell that has 2 neighbor cells
        // Worst case: dt=1 cell that has 1 neighbor cell

        std::vector<int> best_cores;
        std::vector<int> second_cores;
        std::vector<int> worst_cores;

        for (int loc : leftover_dt1_cells) {
            int r = getRow(loc);
            int c = getCol(loc);
            int neighbor_count = 0;

            for (int i = 0; i < 4; i++) {
                int nr = r + dr[i];
                int nc = c + dc[i];

                int next = getLoc(nr, nc);
                if (leftover_dt1_cells.find(next) != leftover_dt1_cells.end()) {
                    neighbor_count++;
                }
            }

            if (neighbor_count >= 3) {
                best_cores.push_back(loc);
            } else if (neighbor_count == 2) {
                second_cores.push_back(loc);
            } else {
                worst_cores.push_back(loc);
            }
        }

        std::vector<int> cores_in_order;
        cores_in_order.insert(cores_in_order.end(), best_cores.begin(), best_cores.end());
        cores_in_order.insert(cores_in_order.end(), second_cores.begin(), second_cores.end());
        cores_in_order.insert(cores_in_order.end(), worst_cores.begin(), worst_cores.end());

        std::unordered_set<int> visited;

        for (int loc : cores_in_order) {
            if (visited.find(loc) != visited.end()) continue;

            std::queue<int> q;
            q.push(loc);
            visited.insert(loc);
            std::vector<int> current_cells;
            current_cells.push_back(loc);

            while (!q.empty()) {
                int curr = q.front();
                q.pop();

                int r = getRow(curr);
                int c = getCol(curr);

                for (int i = 0; i < 4; i++) {
                    int nr = r + dr[i];
                    int nc = c + dc[i];

                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next = getLoc(nr, nc);
                        if (visited.find(next) == visited.end() && leftover_dt1_cells.find(next) != leftover_dt1_cells.end()) {
                            visited.insert(next);
                            q.push(next);
                            current_cells.push_back(next);
                        }
                    }
                }
            }

            if (static_cast<int>(current_cells.size()) >= minSize) {
                Cluster cluster;
                cluster.id = clusterId;
                cluster.area_id = areaMap_[current_cells[0]];
                cluster.size = 0;
                cluster.maxima = 1;
                cluster.cells_begin = -1;
                cluster.cells_end = -1;
                cluster.neighbor_begin = -1;
                cluster.neighbor_end = -1;
                cluster.portal_begin = -1;
                cluster.portal_end = -1;

                for (int cell : current_cells) {
                    clusterMap[cell] = clusterId;
                }

                updatedClusters.push_back(cluster);
                clusterId++;
            }
        }

        clusterMap_.clear();
        clusterMap_ = std::move(clusterMap);
        clusters_.clear();
        clusters_ = std::move(updatedClusters);
    }

    void PreprocessPipeline::assignLeftoverCells() {
        // Implementation for greedy assignment of leftover cells to nearest clusters

        std::vector<Cluster> updatedClusters = clusters_;
        clusters_.clear();

        std::vector<int> clusterMap = clusterMap_;
        clusterMap_.clear();

        std::vector<int> clusterCells;
        clusterCells_.clear();

        // Sort clusters by maxima ascending
        std::vector<Cluster> sortedClusters = updatedClusters;

        std::sort(sortedClusters.begin(), sortedClusters.end(), [](const Cluster& a, const Cluster& b) {
            return a.maxima < b.maxima;
        });

        struct QueueItem {
            int loc;
            int clusterId;
        };

        std::queue<QueueItem> q;

        for (int i = 0; i < clusterMap.size(); i++) {
            if (clusterMap[i] != -1) {
                q.push({i, clusterMap[i]});
            }
        }

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        while (!q.empty()) {
            auto [curr, clusterId] = q.front();
            q.pop();

            int r = getRow(curr);
            int c = getCol(curr);

            for (int i = 0; i < 4; i++) {
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                    int next = getLoc(nr, nc);
                    if (clusterMap[next] == -1 && distanceTransform_[next] > 0) {
                        clusterMap[next] = clusterId;
                        q.push({next, clusterId});
                    }
                }
            }
        }

        int c = 0;

        for (Cluster& cluster : updatedClusters) {
            std::vector<int> cluster_cells;
            for (int r = 0; r < rows_; r++) {
                for (int c = 0; c < cols_; c++) {
                    int loc = getLoc(r, c);
                    if (clusterMap[loc] == cluster.id) {
                        cluster_cells.push_back(loc);
                    }
                }
            }

            int size = 0;
            
            for (int cell : cluster_cells) {
                clusterCells.emplace_back(cell);
                size++;
            }

            cluster.size = size;
            cluster.cells_begin = c;
            cluster.cells_end = c + size - 1;
            c += size;
        }
        
        clusterMap_ = std::move(clusterMap);
        clusters_ = std::move(updatedClusters);
        clusterCells_ = std::move(clusterCells);
    }

    // Helper
    // template<typename T>
    // std::vector<std::vector<T>> split_evenly(const std::vector<T>& data, size_t max_chunk_size) {
    //     if (data.empty() || max_chunk_size == 0) {
    //         return {};
    //     }

    //     size_t n = data.size();

    //     size_t num_chunks = (n + max_chunk_size - 1) / max_chunk_size;

    //     size_t base_chunk_size = n / num_chunks;
    //     size_t remainder = n % num_chunks;

    //     std::vector<std::vector<T>> chunks;
    //     chunks.reserve(num_chunks);

    //     size_t start_index = 0;

    //     for (size_t i = 0; i < num_chunks; ++i) {
    //         size_t current_chunk_size = base_chunk_size + (i < remainder ? 1 : 0);
    //         std::vector<T> chunk(data.begin() + start_index, data.begin() + start_index + current_chunk_size);
    //         chunks.push_back(std::move(chunk));
    //         start_index += current_chunk_size;
    //     }

    //     return chunks;
    // }


    void PreprocessPipeline::updateNeighbors() {
        // Implementation for identifying neighbors of each cluster

        std::vector<Cluster> updatedClusters = clusters_;
        clusters_.clear();

        std::vector<int> clusterNeighbors;
        clusterNeighbors_.clear();

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        int c = 0;

        for (Cluster& cluster : updatedClusters) {
            std::unordered_set<int> neighborSet;

            for (int i = cluster.cells_begin; i <= cluster.cells_end; i++) {
                int loc = clusterCells_[i];
                int r = getRow(loc);
                int c = getCol(loc);

                for (int j = 0; j < 4; j++) {
                    int nr = r + dr[j];
                    int nc = c + dc[j];

                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next = getLoc(nr, nc);
                        int neighborId = clusterMap_[next];
                        if (neighborId != -1 && neighborId != cluster.id) {
                            neighborSet.insert(neighborId);
                        }
                    }
                }
            }

            for (int neighborId : neighborSet) {
                clusterNeighbors.emplace_back(neighborId);
            }

            cluster.neighbor_begin = c;
            cluster.neighbor_end = c + neighborSet.size() - 1;
            cluster.size = neighborSet.size();
            c += neighborSet.size();
        }

        clusters_ = std::move(updatedClusters);
        clusterNeighbors_ = std::move(clusterNeighbors);
    }


    void PreprocessPipeline::updatePortals() {
        // Implementation for identifying portal connections between clusters

        // Notes to myself:
        // If A and B are connected and both have only one portal to each other, then those two portals are opposites
        // if A has multiple portals to B, and B has multiple portals to A, then since by geometry of grid maps, cells cannot overlap, just check one cell from each portal to see which portal it connects to
        // Portals are bidirectional and also in pairs
        // A portal is critical if the minimum cell count on either side is 1
        // A portal has shared area if there exists at least one cell in the same cluster that is also part of another portal

        std::vector<Portal> portals;
        portals_.clear();

        portalMap_.clear();
        std::vector<int> portalMap(rows_ * cols_, -1);

        std::vector<int> portalCells;
        portalCells_.clear();

        std::vector<int> sharedAreaPortals;
        sharedAreaPortals_.clear();

        std::vector<Cluster> updatedClusters = clusters_;
        clusters_.clear();

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        int c = 0;
        int s = 0;

        for (Cluster& cluster : updatedClusters) {
            std::unordered_map<int, std::vector<int>> neighborPortals;

            for (int i = cluster.cells_begin; i <= cluster.cells_end; i++) {
                int loc = clusterCells_[i];
                int r = getRow(loc);
                int c = getCol(loc);

                for (int j = 0; j < 4; j++) {
                    int nr = r + dr[j];
                    int nc = c + dc[j];

                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next = getLoc(nr, nc);
                        int neighborId = clusterMap_[next];
                        if (neighborId != -1 && neighborId != cluster.id) {
                            neighborPortals[neighborId].push_back(loc);
                        }
                    }
                }
            }

            std::vector<std::pair<int, std::vector<int>>> portal_groups; // group together neighboring portal cells (as multiple portals can lead to the same neighbor)

            const int portal_dr[] = {-1, 1, 0, 0, -1, 1, -1, 1};
            const int portal_dc[] = {0, 0, -1, 1, -1, -1, 1, 1};

            for (const auto& [n_id, cells] : neighborPortals) {
                std::unordered_set<int> visited;
                std::unordered_set<int> valid_candidates(cells.begin(), cells.end());

                for (int start_cell : cells) {
                    if (visited.count(start_cell)) continue; // Already grouped

                    std::vector<int> group_cells;
                    std::queue<int> q;

                    q.push(start_cell);
                    visited.insert(start_cell);

                    while (!q.empty()) {
                        int curr = q.front();
                        q.pop();

                        group_cells.push_back(curr);

                        for (int i = 0; i < 8; ++i) {
                            int nr = getRow(curr) + portal_dr[i];
                            int nc = getCol(curr) + portal_dc[i];
                          
                            if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                                int neighbor_loc = getLoc(nr, nc);
                                if (!visited.count(neighbor_loc) && valid_candidates.count(neighbor_loc)) {
                                    visited.insert(neighbor_loc);
                                    q.push(neighbor_loc);
                                }
                            }
                        }
                    }

                    portal_groups.push_back({n_id, group_cells});
                }
            }

            int c_0 = c;

            for (const auto& [n_id, cells] : portal_groups) {
                Portal portal;
                portal.id = c;
                portal.from = cluster.id;
                portal.to = n_id;
                portal.size = cells.size();
                portal.is_critical_this_side = (cluster.size == 1);
                portal.is_critical_other_side = -1;
                portal.opposite_portal_id = -1;
                portal.shared_begin = -1;
                portal.shared_end = -1;
                portal.has_shared_area = false;
                portal.distances_index = -1;
                portal.intra_h_begin = -1;
                portal.intra_h_end = -1;
                portal.inter_h_begin = -1;
                portal.inter_h_end = -1;

                for (int cell : cells) {
                    portalMap[cell] = portal.id;
                    portalCells.emplace_back(cell);
                }
                portal.cells_begin = s;
                portal.cells_end = s + cells.size() - 1;
                s += cells.size();

                portals.push_back(portal);
                c++;
            }

            // account for no portals case
            if (c_0 == c) {
                cluster.portal_begin = -1;
                cluster.portal_end = -1;
            } else {
                cluster.portal_begin = c_0;
                cluster.portal_end = c - 1;
            }
        }

        c = 0;

        for (Portal& portal : portals) {
            // Check for shared area
            bool shared = false;
            std::unordered_set<int> shared_portals;

            int cluster_id = clusterMap_[portalCells[portal.cells_begin]];

            int p_begin = updatedClusters[cluster_id].portal_begin;
            int p_end = updatedClusters[cluster_id].portal_end;

            for (int i = p_begin; i <= p_end; i++) {
                if (i == portal.id) continue;
                Portal& other_portal = portals[i];
                std::unordered_set<int> other_cells_set;

                for (int j = other_portal.cells_begin; j <= other_portal.cells_end; j++) {
                    other_cells_set.insert(portalCells[j]);
                }

                for (int j = portal.cells_begin; j <= portal.cells_end; j++) {
                    if (other_cells_set.find(portalCells[j]) != other_cells_set.end()) {
                        shared = true;
                        shared_portals.insert(other_portal.id);
                        break;
                    }
                }
            }

            if (shared) {
                portal.has_shared_area = true;
                
                for (int sp_id : shared_portals) {
                    sharedAreaPortals.emplace_back(sp_id);
                }

                portal.shared_begin = c;
                portal.shared_end = c + shared_portals.size() - 1;
                c += shared_portals.size();
            }
        }

        for (Portal& portal : portals) {
            // Assign opposite portals
            if (portal.opposite_portal_id != -1) continue; // Already assigned

            for (int i = updatedClusters[portal.to].portal_begin; i <= updatedClusters[portal.to].portal_end; i++) {
                // check if this portal leads back to the original cluster
                // also check if at least one cell is adjecent to the original portal
                // as there can be multiple portals between two clusters and we need to find the correct opposite
                Portal& neighbor_portal = portals[i];

                if (neighbor_portal.to == portal.from) {
                    // check adjacency

                    bool adjacent = false;

                    std::unordered_set<int> neighbor_portal_cells;
                    for (int j = neighbor_portal.cells_begin; j <= neighbor_portal.cells_end; j++) {
                        neighbor_portal_cells.insert(portalCells[j]);
                    }

                    for (int j = portal.cells_begin; j <= portal.cells_end && !adjacent; j++) {
                        int p_cell = portalCells[j];
                        int r = getRow(p_cell);
                        int c = getCol(p_cell);

                        for (int d = 0; d < 4; d++) {
                            int nr = r + dr[d];
                            int nc = c + dc[d];

                            if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                                int next = getLoc(nr, nc);
                                
                                if (neighbor_portal_cells.find(next) != neighbor_portal_cells.end()) {
                                    adjacent = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (adjacent) {
                        portal.opposite_portal_id = neighbor_portal.id;
                        neighbor_portal.opposite_portal_id = portal.id;

                        if (portal.is_critical_this_side) {
                            neighbor_portal.is_critical_other_side = true;
                        }

                        if (neighbor_portal.is_critical_this_side) {
                            portal.is_critical_other_side = true;
                        }

                        break;
                    }
                }
            }
        }

        portals_ = std::move(portals);
        portalMap_ = std::move(portalMap);
        portalCells_ = std::move(portalCells);
        sharedAreaPortals_ = std::move(sharedAreaPortals);
        clusters_ = std::move(updatedClusters);

        // for (int i = 0; i < portals_.size(); i++) {
        //     std::cout << "Portal " << i << ": from cluster " << portals_[i].from << " to cluster " << portals_[i].to << ", size " << portals_[i].size << ", opposite portal " << portals_[i].opposite_portal_id << std::endl;
        // }
    }

    void PreprocessPipeline::computePortalDistances() {
        // Implementation for precomputing portal distances within clusters

        // For each portal, compute distances to all cells in the same cluster
        // Distance is 0 if cell is part of the portal
        // Use BFS for distance computation
        // For saving space, store distances of multiple portals in one slice of portalDistances_
        // A slice of portalDistances_ is all cells of the map where each cell has distance from a specific portal but only for cells in the same cluster
        // Size of portalDistances_ is rows_ * cols_ * global_max_portals_in_cluster

        portalDistances_.clear();

        std::vector<Portal> updatedPortals = portals_;
        portals_.clear();
        
        int max_portals_in_cluster = 0;
        for (const Cluster& cluster : clusters_) {
            int portal_count = cluster.portal_end - cluster.portal_begin + 1;
            if (portal_count > max_portals_in_cluster) {
                max_portals_in_cluster = portal_count;
            }
        }

        std::vector<int> portalDistances(rows_ * cols_ * max_portals_in_cluster, -1);

        for (int s = 0; s < max_portals_in_cluster; s++) {
            // select portals in slice s
            std::vector<int> portals_in_slice;

            for (const Cluster& cluster : clusters_) {
                // account for no portals case
                if (cluster.portal_begin == -1) continue;

                int portal_count = cluster.portal_end - cluster.portal_begin + 1;
                if (s < portal_count) {
                    portals_in_slice.push_back(cluster.portal_begin + s);
                }
            }

            // for each portal in this slice, compute distances in its cluster
            for (int i : portals_in_slice) {
                Portal& portal = updatedPortals[i];
                int cluster_id = portal.from;
                
                std::queue<int> q;

                for (int j = portal.cells_begin; j <= portal.cells_end; j++) {
                    int cell = portalCells_[j];
                    portalDistances[s * rows_ * cols_ + cell] = 0;
                    q.push(cell);
                }

                const int dr[] = {-1, 1, 0, 0};
                const int dc[] = {0, 0, -1, 1};

                while (!q.empty()) {
                    int curr = q.front();
                    q.pop();

                    int r = getRow(curr);
                    int c = getCol(curr);

                    for (int d = 0; d < 4; d++) {
                        int nr = r + dr[d];
                        int nc = c + dc[d];

                        if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                            int next = getLoc(nr, nc);
                            if (clusterMap_[next] == cluster_id && portalDistances[s * rows_ * cols_ + next] == -1) {
                                portalDistances[s * rows_ * cols_ + next] = portalDistances[s * rows_ * cols_ + curr] + 1;
                                q.push(next);
                            }
                        }
                    }
                }

                portal.distances_index = s;
            }
        }

        portals_ = std::move(updatedPortals);
        portalDistances_ = std::move(portalDistances);
    }

    void PreprocessPipeline::computeIntraClusterShortestPaths() {
        // Implementation for precomputing intra-cluster shortest paths between portals

        // for each portal, compute shortest paths to all other portals in the same cluster
        // using the same order for portal as in portals_, store in intraClusterHeuristics_
        // for every cluster, for every portal in that cluster, store the timesteps it takes to reach every other portal in that cluster
        // for space efficiency, only store the paths from a portal to other portals in the same cluster
        // that will result in, for every cluster, a space of its number of portals * (number of portals - 1) portal heuristic entries

        intraClusterHeuristics_.clear();
        std::vector<int> intraClusterHeuristics;

        std::vector<Portal> updatedPortals = portals_;
        portals_.clear();

        const int orientations[] = {0, 90, 180, 270};

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        // int c = 0;

        for (const Cluster& cluster : clusters_) {
            // account for no portals case
            if (cluster.portal_begin == -1) continue;

            int portal_count = cluster.portal_end - cluster.portal_begin + 1;

            // special case: only one portal in cluster -> no intra-cluster heuristics needed
            if (portal_count == 1) continue;

            // std::vector<int> cluster_portal_heuristics(portal_count * (portal_count - 1), -1);

            for (int i = cluster.portal_begin; i <= cluster.portal_end; i++) {
                Portal& portal = updatedPortals[i];

                portal.intra_h_begin = intraClusterHeuristics.size();

                for (int j = cluster.portal_begin; j <= cluster.portal_end; j++) {
                    if (i == j) continue;

                    const Portal& target_portal = updatedPortals[j];

                    int current_cell = portalCells_[(portal.cells_begin + portal.cells_end) / 2]; // Middle cell as representative

                    int search_orientation = -1;

                    int min_distance = 0;

                    bool found = false;

                    while(!found) {
                        int min_d = -1;
                        int min_total_d = -1;
                        int next_cell = -1;

                        for (int i = 0; i < 4; ++i) {
                            int nr = getRow(current_cell) + dr[i];
                            int nc = getCol(current_cell) + dc[i];

                            if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                                int neighbor_loc = getLoc(nr, nc);
                                if (clusterMap_[neighbor_loc] == cluster.id) {
                                    int d = portalDistances_[target_portal.distances_index * rows_ * cols_ + neighbor_loc];
                                    if (d == 0) {
                                        found = true;
                                        next_cell = neighbor_loc;
                                        break;
                                    }
                                    if (min_d == -1 || d < min_d) {
                                        min_d = d;
                                        next_cell = neighbor_loc;
                                    }
                                }
                            }
                        }

                        if (search_orientation == -1) {
                            min_distance += 1;
                            if (next_cell == current_cell - rows_) {
                                search_orientation = 0;
                            } else if (next_cell == current_cell + rows_) {
                                search_orientation = 180;
                            } else if (next_cell == current_cell - 1) {
                                search_orientation = 270;
                            } else if (next_cell == current_cell + 1) {
                                search_orientation = 90;
                            }
                        } else {
                            // rotation aware movement cost
                            // agent can rotate clockwise and counterclockwise too
                            int desired_orientation;
                            if (next_cell == current_cell - rows_) {
                                desired_orientation = 0;
                            } else if (next_cell == current_cell + rows_) {
                                desired_orientation = 180;
                            } else if (next_cell == current_cell - 1) {
                                desired_orientation = 270;
                            } else if (next_cell == current_cell + 1) {
                                desired_orientation = 90;
                            }

                            int rotation_cost = std::min(abs(desired_orientation - search_orientation), 360 - abs(desired_orientation - search_orientation)) / 90;
                            min_distance += (1 + rotation_cost);
                            search_orientation = desired_orientation;
                        }
                        current_cell = next_cell;
                    }

                    intraClusterHeuristics.emplace_back(min_distance);

                    // local vector index: (i - cluster.portal_begin) * (portal_count - 1) + (j - cluster.portal_begin - (j > i ? 1 : 0))
                    // cluster_portal_heuristics[(i - cluster.portal_begin) * (portal_count - 1) + (j - cluster.portal_begin - (j > i ? 1 : 0))] = min_distance;
                }

                portal.intra_h_end = intraClusterHeuristics.size() - 1;
            }
        }

        for (Portal& portal : updatedPortals) {
            std::cout << "Portal " << portal.id << " intra-cluster heuristic range: " << portal.intra_h_begin << " to " << portal.intra_h_end << std::endl;
        }

        portals_ = std::move(updatedPortals);
        intraClusterHeuristics_ = std::move(intraClusterHeuristics);
    }

    void PreprocessPipeline::computeInterClusterHeuristics() {
        // Implementation for precomputing inter-cluster heuristics between portals

        const int INF = 2000000000;

        interClusterHeuristics_.clear();

        std::vector<Portal> updatedPortals = portals_;
        portals_.clear();

        std::vector<int> interClusterHeuristics(updatedPortals.size() * updatedPortals.size(), -1);

        for (int i = 0; i < static_cast<int>(updatedPortals.size()); i++) {
            int row_offset = i * updatedPortals.size();

            std::vector<int> distances(updatedPortals.size(), INF);

            distances[i] = 0;

            std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

            pq.push({0, i});

            while(!pq.empty()) {
                int d = pq.top().first;
                int curr_portal_id = pq.top().second;
                pq.pop();

                if (d > distances[curr_portal_id]) continue;

                const Portal& curr_portal = updatedPortals[curr_portal_id];

                int opposite_portal_id = curr_portal.opposite_portal_id;
                int op_weight = 1;

                if (distances[opposite_portal_id] > distances[curr_portal_id] + op_weight) {
                    distances[opposite_portal_id] = distances[curr_portal_id] + op_weight;
                    pq.push({distances[opposite_portal_id], opposite_portal_id});
                }

                const Cluster& c = clusters_[curr_portal.from];
                int p_start = c.portal_begin;
                int p_end = c.portal_end;
                int portal_count = p_end - p_start + 1;

                if (portal_count > 1 && curr_portal.intra_h_begin != -1) {
                    int h_idx = curr_portal.intra_h_begin;

                    for (int j = p_start; j <= p_end; j++) {
                        if (j == curr_portal_id) continue;

                        int weight = intraClusterHeuristics_[h_idx];
                        h_idx++;

                        if (weight != -1) {
                            if (distances[j] > distances[curr_portal_id] + weight) {
                                distances[j] = distances[curr_portal_id] + weight;
                                pq.push({distances[j], j});
                            }
                        }
                    }
                }
            }

            for (int j = 0; j < static_cast<int>(updatedPortals.size()); j++) {
                if (distances[j] != INF) {
                    interClusterHeuristics[row_offset + j] = distances[j];
                } else {
                    interClusterHeuristics[row_offset + j] = -1;
                }
            }

        }

        interClusterHeuristics_ = std::move(interClusterHeuristics);

        std::cout << "Inter-cluster heuristics size: " << interClusterHeuristics_.size() << std::endl;

        for (Portal& portal : updatedPortals) {
            portal.inter_h_begin = portal.id * updatedPortals.size();
            portal.inter_h_end = portal.inter_h_begin + updatedPortals.size() - 1;
        }

        portals_ = std::move(updatedPortals);
    }

    void PreprocessPipeline::computeProjectionBase() {
        // Implementation for computing projection base for clusters

        projectionBase_.clear();
        std::vector<int> projectionBase(clusters_.size(), -1);

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                if (map_[getLoc(r, c)] == 0) {
                    int loc = getLoc(r, c);
                    projectionBase[loc] = 0;

                    int neighbor_cells = 0;

                    for (int i = 0; i < 4; i++) {
                        int nr = r + dr[i];
                        int nc = c + dc[i];

                        if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                            int neighbor_loc = getLoc(nr, nc);
                            if (map_[neighbor_loc] == 0) {
                                neighbor_cells++;
                            }
                        }
                    }

                    if (neighbor_cells <= 2) {
                        projectionBase[loc] = 1;
                    }
                }
            }
        }

        projectionBase_ = std::move(projectionBase);
    }

    void PreprocessPipeline::debugResults() {
        // output all results into txt files for debugging

        std::ofstream area_file(PREPROCESSING_DEBUG_PATH + "/debug_area_map.txt");
        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                int loc = getLoc(r, c);
                if (map_[loc] == 1) {
                    area_file << "@";
                } else {
                    area_file << areaMap_[loc];
                }
            }
            area_file << "\n";
        }

        area_file.close();

        std::ofstream dt_file(PREPROCESSING_DEBUG_PATH + "/debug_distance_transform.txt");
        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                int loc = getLoc(r, c);
                dt_file << distanceTransform_[loc];
            }
            dt_file << "\n";
        }
        dt_file.close();

        std::ofstream cluster_file(PREPROCESSING_DEBUG_PATH + "/debug_clusters.txt");

        for (const Cluster& cluster : clusters_) {
            cluster_file << "Cluster ID: " << cluster.id << "\n";
            cluster_file << "Area ID: " << cluster.area_id << "\n";
            cluster_file << "Size: " << cluster.size << "\n";
            cluster_file << "Maxima: " << cluster.maxima << "\n";
            cluster_file << "Cells: ";
            for (int i = cluster.cells_begin; i <= cluster.cells_end; i++) {
                cluster_file << clusterCells_[i] << " ";
            }
            cluster_file << "\nNeighbors: ";
            for (int i = cluster.neighbor_begin; i <= cluster.neighbor_end; i++) {
                cluster_file << clusterNeighbors_[i] << " ";
            }
            cluster_file << "\nPortals: ";
            for (int i = cluster.portal_begin; i <= cluster.portal_end; i++) {
                cluster_file << portals_[i].id << " ";
            }
            cluster_file << "\n\n";

            for (int r = 0; r < rows_; r++) {
                for (int c = 0; c < cols_; c++) {
                    int loc = getLoc(r, c);
                    if (clusterMap_[loc] == cluster.id) {
                        cluster_file << "C";
                    } else {
                        if (map_[loc] == 1) {
                            cluster_file << "@";
                        } else {
                            cluster_file << ".";
                        }
                    }
                }
                cluster_file << "\n";
            }

            cluster_file << "\n====================\n\n";
        }

        cluster_file.close();

        std::ofstream portal_file(PREPROCESSING_DEBUG_PATH + "/debug_portals.txt");

        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                int loc = getLoc(r, c);
                if (portalMap_[loc] == -1) {
                    if (map_[loc] == 1) { 
                        portal_file << "@";
                    } else {
                        portal_file << ".";
                    }
                } else {
                    portal_file << "P";
                }
            }
            portal_file << "\n";
        }

        cluster_file << "\n====================\n\n";

        for (const Portal& portal : portals_) {
            portal_file << "Portal ID: " << portal.id << "\n";
            portal_file << "From Cluster ID: " << portal.from << "\n";
            portal_file << "To Cluster ID: " << portal.to << "\n";
            portal_file << "Size: " << portal.size << "\n";
            portal_file << "Cells: ";
            for (int i = portal.cells_begin; i <= portal.cells_end; i++) {
                portal_file << portalCells_[i] << " ";
            }
            portal_file << "\nIs Critical This Side: " << portal.is_critical_this_side << "\n";
            portal_file << "Is Critical Other Side: " << portal.is_critical_other_side << "\n";
            portal_file << "Opposite Portal ID: " << portal.opposite_portal_id << "\n";
            portal_file << "Has Shared Area: " << portal.has_shared_area << "\n\n";
            if (portal.has_shared_area) {
                portal_file << "Shared Area Portals: ";
                for (int i = portal.shared_begin; i <= portal.shared_end; i++) {
                    portal_file << sharedAreaPortals_[i] << " ";
                }
                portal_file << "\n";
            }

            std::unordered_set<int> clusterCellsSet;
            for (int i = clusters_[portal.from].cells_begin; i <= clusters_[portal.from].cells_end; i++) {
                clusterCellsSet.insert(clusterCells_[i]);
            }

            std::vector<int> portalDistancesList(rows_ * cols_, -1);
            for (int i = 0; i < rows_ * cols_; i++) {
                if (clusterCellsSet.find(i) != clusterCellsSet.end()) {
                    portalDistancesList[i] = portalDistances_[portal.distances_index * rows_ * cols_ + i];
                }
            }

            for (int r = 0; r < rows_; r++) {
                for (int c = 0; c < cols_; c++) {
                    int loc = getLoc(r, c);
                    if (portalDistancesList[loc] == -1) {
                        if (map_[loc] == 1) {
                            portal_file << "@";
                        } else {
                            portal_file << ".";
                        }
                    } else {
                        portal_file << portalDistancesList[loc] % 10;
                    }
                }
                portal_file << "\n";
            }

            portal_file << "\n====================\n\n";

            if (portal.intra_h_begin != -1) {
                portal_file << "Intra-Cluster Heuristics:\n";
                for (int i = portal.intra_h_begin; i <= portal.intra_h_end; i++) {
                    portal_file << intraClusterHeuristics_[i] << " ";
                }
                portal_file << "\n\n";
            }

            if (portal.inter_h_begin != -1) {
                portal_file << "Inter-Cluster Heuristics:\n";
                for (int i = portal.inter_h_begin; i <= portal.inter_h_end; i++) {
                    portal_file << interClusterHeuristics_[i] << " ";
                }
                portal_file << "\n\n";
            }
        }

        portal_file.close();
    }

    void PreprocessPipeline::runPreprocessing() {
        // Main function to run the entire preprocessing pipeline

        if (preprocessing_done_) return;

        std::cout << "Starting preprocessing..." << std::endl;
        std::cout << "Identifying distinct areas..." << std::endl;
        identifyDistinctAreas();
        std::cout << "Computing distance transform..." << std::endl;
        computeDistanceTransform();
        std::cout << "Finding local maximas..." << std::endl;
        findLocalMaximas();
        std::cout << "Forming initial clusters..." << std::endl;
        formInitialClusters();
        std::cout << "Clustering DT1 cells..." << std::endl;
        clusterDT1Cells(MINIMUM_DT1_CLUSTER_SIZE);
        std::cout << "Assigning leftover cells..." << std::endl;
        assignLeftoverCells();
        std::cout << "Updating neighbors..." << std::endl;
        updateNeighbors();
        std::cout << "Updating portals..." << std::endl;
        updatePortals();
        std::cout << "Computing portal distances..." << std::endl;
        computePortalDistances();
        std::cout << "Computing intra-cluster shortest paths..." << std::endl;
        computeIntraClusterShortestPaths();
        std::cout << "Computing inter-cluster heuristics..." << std::endl;
        computeInterClusterHeuristics();
        std::cout << "Preprocessing completed." << std::endl;
        preprocessing_done_ = true;

        if (DEBUG_PREPROCESSING) {
            std::cout << "Debugging preprocessing results..." << std::endl;
            debugResults();
            std::cout << "Debugging completed." << std::endl;
        }
    }

    PreprocessPipeline::PreprocessPipeline() {
        initialized_ = false;
        preprocessing_done_ = false;

        map_.clear();
        rows_ = -1;
        cols_ = -1;

        areaMap_.clear();
        distanceTransform_.clear();
        localMaximaPlateaus_.clear();
        clusters_.clear();
        clusterMap_.clear();
        clusterCells_.clear();
        clusterNeighbors_.clear();
        portals_.clear();
        portalMap_.clear();
        portalCells_.clear();
        sharedAreaPortals_.clear();
        portalDistances_.clear();
        intraClusterHeuristics_.clear();
        interClusterHeuristics_.clear();
    }

    void PreprocessPipeline::initialize(const std::vector<int>& map, int rows, int cols) {
        if (initialized_) return;

        map_ = map;
        rows_ = rows;
        cols_ = cols;

        runPreprocessing();
        
        initialized_ = true;
    }
}