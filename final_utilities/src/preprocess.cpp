#include "preprocess.h"
#include <math.h>

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

        areaMap_ = areas;
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


        distanceTransform_ = distances;
    }

    // Helper
    std::pair<int, int> ClusteringPipeline::findGlobalMinMax() const {
        int minVal = -1;
        int maxVal = -1;
        
        for (int dist : distances_) {
            if (minVal == -1 || dist < minVal) minVal = dist;
            if (maxVal == -1 || dist > maxVal) maxVal = dist;
        }
        
        return {minVal, maxVal};
    }

    // Helper
    std::vector<int> ClusteringPipeline::selectCellsAtDistance(int targetDistance) const {
        std::vector<int> cells;
        
        for (int loc = 0; loc < static_cast<int>(distances_.size()); loc++) {
            if (distances_[loc] == targetDistance) {
                cells.push_back(loc);
            }
        }
        
        return cells;
    }

    void ClusteringPipeline::findLocalMaximas() {
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

        localMaximaPlateaus_ = localMaximaPlateaus;
    }

    void ClusteringPipeline::mergeLocalMaximas() {
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

                        if (potential_merge.find(next) == potential_merge.end() && visited.find(next) == visited.end() && distances_[next] == maxima_value) {
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

        return merged_plateaus;
    }

    void PreprocessPipeline::formInitialClusters() {
        // Implementation for forming initial clusters from local maxima plateaus

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
                    
                    if (distances_[next] <= distances_[curr] && distances_[next] > 0) {
                        if (clusterMap[next] == -1) {
                            clusterMap[next] = clusterMap[curr];
                            if (distances_[next] > 1) {
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
            cluster.degree = 0;
            cluster.maxima = localMaximaPlateaus_[i].first;
            
            for (int loc = 0; loc < static_cast<int>(clusterMap.size()); loc++) {
                if (clusterMap[loc] == static_cast<int>(i)) {
                    cluster.cells.push_back(loc);
                }
            }
            
            cluster.size = cluster.cells.size();
            clusters.push_back(cluster);
        }

        clusters_ = clusters;
    }

    void PreprocessPipeline::clusterDT1Cells(int minSize) {
        // Implementation for clustering DT1 cells into existing clusters

        std::vector<Cluster> updatedClusters = clusters_;
        int clusterId = updatedClusters.size();

        std::unordered_set<int> clustered_cells;

        for (const auto& cluster : updatedClusters) {
            for (int loc : cluster.cells) {
                clustered_cells.insert(loc);
            }
        }

        std::unordered_set<int> leftover_dt1_cells;

        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                int loc = getLoc(r, c);
                if (distances_[loc] == 1 && clustered_cells.find(loc) == clustered_cells.end()) {
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
                if (clustered_cells.find(next) != clustered_cells.end()) {
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
                cluster.degree = 0;
                cluster.maxima = 1;
                cluster.cells = current_cells;
                cluster.size = current_cells.size();

                updatedClusters.push_back(cluster);
                clusterId++;
            }
        }

        clusters_ = updatedClusters;
    }

    void PreprocessPipeline::assignLeftoverCells() {
        // Implementation for greedy assignment of leftover cells to nearest clusters

        std::vector<Cluster> updatedClusters = clusters_;

        // Sort clusters by maxima ascending
        std::sort(updatedClusters.begin(), updatedClusters.end(), [](const Cluster& a, const Cluster& b) {
            return a.maxima < b.maxima;
        });

        std::vector<int> assignmentMap(rows_ * cols_, -1);

        struct QueueItem {
            int loc;
            int clusterId;
        };

        std::queue<QueueItem> q;

        for (auto* cluster : sortedClusters) {
            for (int loc : cluster->cells) {
                assignmentMap[loc] = cluster->id;
                q.push({loc, cluster->id});
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
                    if (assignmentMap[next] == -1 && distances_[next] > 0) {
                        assignmentMap[next] = clusterId;
                        q.push({next, clusterId});
                    }
                }
            }
        }

        for (auto& cluster : updatedClusters) {
            std::vector<int> cluster_cells;
            for (int r = 0; r < rows_; r++) {
                for (int c = 0; c < cols_; c++) {
                    int loc = getLoc(r, c);
                    if (assignmentMap[loc] == cluster.id) {
                        cluster_cells.push_back(loc);
                    }
                }
            }
            cluster.cells = cluster_cells;
            cluster.size = cluster.cells.size();
        }
        
        clusters_ = updatedClusters;

        for (const auto& cluster : updatedClusters) {
            for (int loc : cluster.cells) {
                clusterMap_[loc] = cluster.id;
            }
        }
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

    void PreprocessPipeline::updateTopology() {
        // Implementation for updating cluster neighbor information

        std::vector<Cluster> updatedClusters = clusters_;

        std::vector<int> clusterMap(rows_ * cols_, -1);
        for (const auto& cluster : updatedClusters) {
            for (int loc : cluster.cells) {
                clusterMap[loc] = cluster.id;
            }
        }

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        struct PortalItem {
            int n_id;
            int from;
            int to;
        };

        int portal_id = 0;

        for (auto& cluster : updatedClusters) {
            std::unordered_set<int> neighborSet;
            std::unordered_set<PortalItem> portalSet;

            for (int loc : cluster.cells) {
                int r = getRow(loc);
                int c = getCol(loc);

                for (int i = 0; i < 4; i++) {
                    int nr = r + dr[i];
                    int nc = c + dc[i];

                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next = getLoc(nr, nc);
                        int neighborId = clusterMap[next];
                        if (neighborId != -1 && neighborId != cluster.id) {
                            neighborSet.insert(neighborId);
                            portalSet.insert({neighborId, loc, next});
                        }
                    }
                }
            }

            cluster.neighbors.clear();
            for (int neighborId : neighborSet) {
                cluster.neighbors.push_back(neighborId);
            }
            cluster.degree = cluster.neighbors.size();

            const int portal_dr[] = {-1, 1, 0, 0, -1, 1, -1, 1};
            const int portal_dc[] = {0, 0, -1, 1, -1, -1, 1, 1};

            std::vector<std::pair<int, std::vector<int>>> connected_portal_groups;

            // Group portals that are connected
            std::unordered_map<int, std::unordered_set<int>> portals_by_neighbor;

            for (const auto& p : portalSet) {
                portals_by_neighbor[p.n_id].insert(p.from);
            }

            for (auto& [n_id, from_cells] : portals_by_neighbor) {
    
                std::unordered_set<int> visited;

                for (int start_cell : from_cells) {
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
                            int nr = portal_dr[i];
                            int nc = portal_dc[i];
                          
                            if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                                int neighbor_loc = getLoc(nr, nc);
                                if (from_cells.count(neighbor_loc) && !visited.count(neighbor_loc)) {
                                    visited.insert(neighbor_loc);
                                    q.push(neighbor_loc);
                                }
                            }
                        }
                    }

                    connected_portal_groups.push_back({n_id, std::move(group_cells)});
                }
            }

            Portal portal;
            for (const auto& [n_id, cells] : connected_portal_groups) {
                portal.id = portal_id;
                portal.from_cluster_id = cluster.id;
                portal.to_cluster_id = n_id;
                portal.cells = cells;
                portal.is_critical = false; // Placeholder, set based on criteria
                portal.opposite_portal_id = -1; // Placeholder, set when both directions are known

                cluster.portals.push_back(&portal);
                portals_.push_back(&portal);
                portal_id++;
            }
        }

        clusters_ = updatedClusters;
    }

    void PreprocessPipeline::updatePortals() {
        // Implementation for identifying portal connections between clusters

        // Notes to myself:
        // If A and B are connected and both have only one portal to each other, then those two portals are opposites
        // if A has multiple portals to B, and B has multiple portals to A, then since by geometry of grid maps, cells cannot overlap, just check one cell from each portal to see which portal it connects to
        // Portals are bidirectional and also in pairs
        // A portal is critical if the minimum cell count on either side is 1

        for (auto& portal : portals_) {
            if (portal->opposite_portal_id != -1) continue; // Already assigned

            Cluster neighbor = clusters_[portal->to];

            std::vector<Portal*> candidate_opposites;

            for (auto* neighbor_portal : neighbor.portals) {
                if (neighbor_portal->to == portal->from) {
                    candidate_opposites.push_back(neighbor_portal);
                }
            }

            if (candidate_opposites.size() == 1) {
                portal->opposite_portal_id = candidate_opposites[0]->id;
                candidate_opposites[0]->opposite_portal_id = portal->id;
            } else {
                // Need to check cells
                for (auto* candidate : candidate_opposites) {
                    int neighbor_neighbor_id = candidate->to;
                    if (neighbor_neighbor_id == portal->from) {
                        portal->opposite_portal_id = candidate->id;
                        candidate->opposite_portal_id = portal->id;
                        break;
                    }
                }
            }

            // Check criticality
            Portal* opposite_portal = portals_[portal->opposite_portal_id];

            int from_size = portal->cells.size();
            int to_size = opposite_portal->cells.size();

            if (from_size == 1 || to_size == 1) {
                portal->is_critical = true;
                opposite_portal->is_critical = true;
            }
        }

        // 2 important edge cases to consider:
        // 1. when there is a portal from A to B and also from A to C and both portals are critical and share the same cell in A
        // 2. when there is a portal from A to B, from A to C and from A to D and all three portals are critical and share the same cell in A
        // In both cases, all portals involved should be marked as affected by each other
        // So if one portal is disabled temporarily, the others are also considered disabled

        // TODO: implement the above logic
    }

    void PreprocessPipeline::computePortalDistances() {
        // Implementation for precomputing portal distances within clusters

        // Every cell in the portal is 0 distance
        // Every other cell in the cluster is assigned the minimum distance to any portal cell

        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        for (Portal* portal : portals_) {
            int current_cluster_id = portal->from_cluster_id;

            std::vector<int> distances(clusters_[current_cluster_id].size, -1);
            std::queue<int> q;

            std::unordered_set<int> portal_cells_set(portal->cells.begin(), portal->cells.end());

            for (int i = 0; i < clusters_[current_cluster_id].size; i++) {
                int loc = clusters_[current_cluster_id].cells[i];
                if (portal_cells_set.find(loc) != portal_cells_set.end()) {
                    distances[i] = 0;
                    q.push(i);
                }
            }

            while (!q.empty()) {
                int curr_idx = q.front();
                q.pop();

                int curr_loc = clusters_[current_cluster_id].cells[curr_idx];
                int r = getRow(curr_loc);
                int c = getCol(curr_loc);

                for (int i = 0; i < 4; i++) {
                    int nr = r + dr[i];
                    int nc = c + dc[i];

                    if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                        int next_loc = getLoc(nr, nc);
                        auto it = std::find(clusters_[current_cluster_id].cells.begin(), clusters_[current_cluster_id].cells.end(), next_loc);
                        if (it != clusters_[current_cluster_id].cells.end()) {
                            int next_idx = std::distance(clusters_[current_cluster_id].cells.begin(), it);
                            if (distances[next_idx] == -1) {
                                distances[next_idx] = distances[curr_idx] + 1;
                                q.push(next_idx);
                            }
                        }
                    }
                }
            }

            PortalDistances portalDistances;
            portalDistances.p_id = portal->id;

            for (int i = 0; i < clusters_[current_cluster_id].size; i++) {
                if (distances[i] != -1) {
                    portalDistances.distances.push_back({clusters_[current_cluster_id].cells[i], distances[i]});
                }
            }

            portalDistances_.push_back(portalDistances);
        }
    }

    void PreprocessPipeline::computeIntraClusterShortestPaths() {
        // Implementation for precomputing intra-cluster shortest paths between portals

        // For each portal, compute shortest paths to all other portals in the same cluster

        intraClusterHeuristics_.resize(portals_.size());

        for (const Portal* portal : portals_) {
            int current_cluster_id = portal->from_cluster_id;

            for (const Portal* target_portal : clusters_[current_cluster_id].portals) {
                if (portal->id == target_portal->id) continue;

                PortalHeuristic path;
                path.portal_to = target_portal->id;
                path.start_cell = portal->cells[portal->cells.size() / 2]; // Middle cell as representative

                // Find end cell in target portal closest to start cell
                // First step is not rotation aware, just straight line distance but movement will set the orientation and later steps will be rotation aware

                const int orientations[] = {0, 90, 180, 270};

                const int dr[] = {-1, 1, 0, 0};
                const int dc[] = {0, 0, -1, 1};

                int search_orientation = -1;

                int min_distance = 0;
                int end_cell = -1;

                std::unordered_map<int, int> cell_distances;

                for (int cell : portalDistances_[target_portal->id].distances) {
                    cell_distances[cell.first] = cell.second;
                }

                int current_cell = path.start_cell;
                bool found = false;

                while (!found) {
                    int min_d = -1;
                    int min_total_d = -1;
                    int next_cell = -1;

                    for (int i = 0; i < 4; ++i) {
                        int nr = getRow(path.start_cell) + dr[i];
                        int nc = getCol(path.start_cell) + dc[i];

                        if (nr >= 0 && nr < rows_ && nc >= 0 && nc < cols_) {
                            int neighbor_loc = getLoc(nr, nc);
                            if (clusterMap_[neighbor_loc] == current_cluster_id) {
                                int d = cell_distances[neighbor_loc];
                                if (d == 0) {
                                    found = true;
                                    next_cell = neighbor_loc;
                                    end_cell = neighbor_loc;
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
                        // Rotation aware movement cost
                        // Agent can rotate clockwise and counterclockwise too
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
                }

                path.end_cell = end_cell;
                path.timesteps = min_distance;

                intraClusterHeuristics_[portal->id].paths_from_portal[portal->id].push_back(path);
            }
        }
        
    }

    void PreprocessPipeline::computeInterClusterHeuristics() {
        if (clusters_.empty()) return;

        int num_clusters = clusters_.size();
        int total_portals = portals_.size();

        // 1. Initialize the main table [From_Cluster][To_Cluster]
        interClusterHeuristics_.resize(num_clusters);
        for (int i = 0; i < num_clusters; ++i) {
            interClusterHeuristics_[i].resize(num_clusters);
        }

        // 2. We need to map Global Portal IDs back to Local Cluster Indices 
        std::vector<int> portal_local_index(total_portals, -1);
        for (const auto& cluster : clusters_) {
            for (int i = 0; i < static_cast<int>(cluster.portals.size()); ++i) {
                portal_local_index[cluster.portals[i]->id] = i;
            }
        }

        // 3. Resize the inner matrices for every pair of clusters
        // This ensures interClusterHeuristics_[c1][c2] is a matrix of size [num_portals_c1][num_portals_c2]
        for (int start_c = 0; start_c < num_clusters; ++start_c) {
            for (int end_c = 0; end_c < num_clusters; ++end_c) {
                int start_p_count = clusters_[start_c].portals.size();
                int end_p_count = clusters_[end_c].portals.size();
                
                // Resize vector of vectors
                interClusterHeuristics_[start_c][end_c].resize(start_p_count);
                for (int k = 0; k < start_p_count; ++k) {
                    interClusterHeuristics_[start_c][end_c][k].resize(end_p_count);
                }
            }
        }

        // 4. Run Dijkstra from EVERY portal in the map
        for (int start_p_id = 0; start_p_id < total_portals; ++start_p_id) {
            
            Portal* start_portal = portals_[start_p_id];
            int start_cluster_id = start_portal->from_cluster_id;
            int start_local_idx = portal_local_index[start_p_id];

            // Dijkstra Data Structures
            std::vector<int> dist(total_portals, std::numeric_limits<int>::max());
            // Parent stores: {Previous_Portal_ID, Timesteps_Taken_In_Edge}
            std::vector<std::pair<int, int>> parent(total_portals, {-1, 0});
            
            // Min-Priority Queue: <Current_Dist, Portal_ID>
            std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

            // Init Start
            dist[start_p_id] = 0;
            pq.push({0, start_p_id});

            while (!pq.empty()) {
                int d = pq.top().first;
                int u_id = pq.top().second;
                pq.pop();

                if (d > dist[u_id]) continue;

                // --- TRANSITION 1: Cross the Portal (Inter-Cluster) ---
                // Move from this portal to its opposite (physically stepping through the door)
                Portal* u_portal = portals_[u_id];
                if (u_portal->opposite_portal_id != -1) {
                    int v_id = u_portal->opposite_portal_id;
                    int weight = 1; // Assuming 1 timestep to cross "through" the portal boundary
                    
                    if (dist[u_id] + weight < dist[v_id]) {
                        dist[v_id] = dist[u_id] + weight;
                        parent[v_id] = {u_id, weight};
                        pq.push({dist[v_id], v_id});
                    }
                }

                // --- TRANSITION 2: Cross the Room (Intra-Cluster) ---
                // Move from this portal to other portals in the SAME cluster
                // We use the precomputed intraClusterHeuristics_
                if (static_cast<size_t>(u_id) < intraClusterHeuristics_.size()) {
                    const auto& heuristic_entry = intraClusterHeuristics_[u_id];
                    
                    // Iterate over all reachable portals within the cluster
                    for (const auto& [target_p_id, path_heuristics] : heuristic_entry.paths_from_portal) {
                        if (path_heuristics.empty()) continue;

                        // path_heuristics is a vector, assuming index 0 is the best path calculated previously
                        int weight = path_heuristics[0].timesteps;
                        int v_id = target_p_id;

                        if (dist[u_id] + weight < dist[v_id]) {
                            dist[v_id] = dist[u_id] + weight;
                            parent[v_id] = {u_id, weight};
                            pq.push({dist[v_id], v_id});
                        }
                    }
                }
            }

            // 5. Reconstruct paths for all reachable destinations
            for (int end_p_id = 0; end_p_id < total_portals; ++end_p_id) {
                if (dist[end_p_id] == std::numeric_limits<int>::max()) continue;
                if (start_p_id == end_p_id) continue;

                Portal* end_portal = portals_[end_p_id];
                int end_cluster_id = end_portal->from_cluster_id;
                int end_local_idx = portal_local_index[end_p_id];

                // Build the PortalPath object
                PortalPath path;
                path.total_timesteps = dist[end_p_id];
                
                // Backtrack from end_p_id to start_p_id
                int curr = end_p_id;
                int current_time_cursor = dist[end_p_id];

                while (curr != start_p_id) {
                    int prev = parent[curr].first;
                    int edge_cost = parent[curr].second;

                    PortalPathStep step;
                    step.from_portal = prev;
                    step.to_portal = curr;
                    step.to_timestep = current_time_cursor;
                    step.from_timestep = current_time_cursor - edge_cost;
                    step.is_critical = portals_[curr]->is_critical;

                    path.steps.push_back(step);

                    current_time_cursor -= edge_cost;
                    curr = prev;
                }

                // Reverse steps because we backtracked
                std::reverse(path.steps.begin(), path.steps.end());

                // Store in the heuristic matrix
                interClusterHeuristics_[start_cluster_id][end_cluster_id][start_local_idx][end_local_idx] = path;
            }
        }
    }

    void PreprocessPipeline::runPreprocessing() {
        // Main function to run the entire preprocessing pipeline

        if (preprocessing_done_) return;

        identifyDistinctAreas();
        computeDistanceTransform();
        findLocalMaximas();
        formInitialClusters();
        clusterDT1Cells(MINIMUM_DT1_CLUSTER_SIZE);
        assignLeftoverCells();
        updateTopology();
        updatePortals();
        computePortalDistances();
        computeIntraClusterShortestPaths();
        computeInterClusterHeuristics();
        preprocessing_done_ = true;
    }
}