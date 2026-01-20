#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <string>
#include <fstream>

// Everything related to preprocessing the static environment for pathfinding
namespace PreprocessingPipeline {
    const int MINIMUM_DT1_CLUSTER_SIZE = 4;

    const bool DEBUG_PREPROCESSING = true;
    const std::string PREPROCESSING_DEBUG_PATH = "preprocessing_debug";

    struct Portal {
        int id; // id of the portal
        int from; // cluster id where the portal is
        int to; // cluster id where the portal leads to
        int size; // size of the portal in cells
        int cells_begin; // beginning index of the portal cells in the global portalCells_ vector
        int cells_end; // ending index of the portal cells in the global portalCells_ vector
        int portal_begin; // beginning index of the portal cells in the global portalCells_ vector
        int portal_end; // ending index of the portal cells in the global portalCells_ vector
        int opposite_portal_id; // id of the opposite direction portal
        bool is_critical_this_side; // true if this portal is critical for pathfinding from this side
        bool is_critical_other_side; // true if the opposite portal is critical for pathfinding
        bool has_shared_area; // true if in the same cluster, another portal shares area with this portal
        int shared_begin; // beginning index of shared area portals in sharedAreaPortals_ vector
        int shared_end; // ending index of shared area portals in sharedAreaPortals_ vector
        int distances_index; // index to identify which slice of portalDistances_ contains distances for this portal
        int intra_h_begin; // beginning index of intra-cluster heuristics for this portal
        int intra_h_end; // ending index of intra-cluster heuristics for this portal
        int inter_h_begin; // beginning index of inter-cluster heuristics for this portal
        int inter_h_end; // ending index of inter-cluster heuristics for this portal
    };

    struct Cluster {
        int id; // cluster id
        int area_id; // area id where the cluster is located
        int size; // number of cells in the cluster
        int maxima; // local maxima value of the cluster
        int cells_begin; // beginning index of the cluster cells in the global clusterCells_ vector
        int cells_end; // ending index of the cluster cells in the global clusterCells_ vector
        int neighbor_begin; // beginning index of the cluster neighbors in the global clusterNeighbors_ vector
        int neighbor_end; // ending index of the cluster neighbors in the global clusterNeighbors_ vector
        int portal_begin; // beginning index of the cluster portals in the global portals_ vector
        int portal_end; // ending index of the cluster portals in the global portals_ vector
    };

    class PreprocessPipeline {
    public:
        // Get singleton instance
        static PreprocessPipeline& getInstance() {
            static PreprocessPipeline instance;
            return instance;
        }
        
        // Delete copy constructor and assignment operator
        PreprocessPipeline(const PreprocessPipeline&) = delete;
        PreprocessPipeline& operator=(const PreprocessPipeline&) = delete;
        
        // Initialize with map data - runs clustering internally
        void initialize(const std::vector<int>& map, int rows, int cols);

        // Check if initialized
        bool isInitialized() const { return initialized_; }

        // Check if preprocessing is done
        bool isPreprocessingDone() const { return preprocessing_done_; }

        // Run the full preprocessing pipeline
        void runPreprocessing();

        // Accessors

        // Get area map
        const std::vector<int>& getAreaMap() const {
            return areaMap_;
        }

        // Get distance transform
        const std::vector<int>& getDistanceTransform() const {
            return distanceTransform_;
        }

        // Get clusters
        const std::vector<Cluster>& getClusters() const {
            return clusters_;
        }

        // Get cluster map
        const std::vector<int>& getClusterMap() const {
            return clusterMap_;
        }

        const std::vector<int>& getClusterCells() const {
            return clusterCells_;
        }

        // Get portals
        const std::vector<Portal>& getPortals() const {
            return portals_;
        }

        const std::vector<int>& getPortalMap() const {
            return portalMap_;
        }

        const std::vector<int>& getPortalCells() const {
            return portalCells_;
        }

        // Get portal distances
        const std::vector<int>& getPortalDistances() const {
            return portalDistances_;
        }

        // Get intra-cluster heuristics
        const std::vector<int>& getIntraClusterHeuristics() const {
            return intraClusterHeuristics_;
        }

        // Get inter-cluster heuristics
        const std::vector<int>& getInterClusterHeuristics() const {
            return interClusterHeuristics_;
        }

        const std::vector<int>& getProjectionBase() const {
            return projectionBase_;
        }

        int getClusterId(int loc) const {
            if (loc < 0 || loc >= static_cast<int>(clusterMap_.size())) {
                return -1;
            }
            return clusterMap_[loc];
        }

        int getAreaId(int loc) const {
            if (loc < 0 || loc >= static_cast<int>(areaMap_.size())) {
                return -1;
            }
            return areaMap_[loc];
        }

    private:
        // Private constructor for singleton
        PreprocessPipeline();
        ~PreprocessPipeline() = default;

        inline int getLoc(int row, int col) const { return row * cols_ + col; }

        inline int getRow(int loc) const { return loc / cols_; }
        inline int getCol(int loc) const { return loc % cols_; }

        std::pair<int, int> findGlobalMinMax();

        std::vector<int> selectCellsAtDistance(int targetDistance);

        // Distict area identification
        void identifyDistinctAreas();

        // Distance transform computation
        void computeDistanceTransform();

        // Local maxima identification
        void findLocalMaximas();

        // Merge diagonally connected local maxima plateaus
        void mergeLocalMaximas();

        // Initial cluster formation from local maxima
        void formInitialClusters();

        // Clustering possible dt=1 cells
        void clusterDT1Cells(int minSize);

        // Greedy assignment of leftover cells
        void assignLeftoverCells();

        // // Update cluster neighbor information
        // void updateTopology();

        // Identify neighbors
        void updateNeighbors();

        // Identify portal connections
        void updatePortals();

        // Precompute portal distances within clusters
        void computePortalDistances();

        // Precompute intra-cluster shortest paths between portals
        void computeIntraClusterShortestPaths();

        // Precompute inter-cluster shortest paths between clusters
        void computeInterClusterHeuristics();

        // Compute projection base map for dynamic environment
        void computeProjectionBase();

        void debugResults();

        bool initialized_;
        bool preprocessing_done_;

        // base map data
        std::vector<int> map_;
        int rows_;
        int cols_;
        
        // Preprocessing data members
        std::vector<int> areaMap_; // cell to area mapping
        std::vector<int> distanceTransform_; // distance to nearest obstacle
        std::vector<std::pair<int, std::vector<int>>> localMaximaPlateaus_; // local maxima plateaus, only used during preprocessing
        std::vector<Cluster> clusters_; // computed clusters
        std::vector<int> clusterMap_; // cell-to-cluster mapping
        std::vector<int> clusterCells_; // cells in each cluster, ordered by cluster id
        std::vector<int> clusterNeighbors_; // neighbors of each cluster, ordered by cluster id
        std::vector<Portal> portals_; // computed portals
        std::vector<int> portalMap_; // cell-to-portal mapping
        std::vector<int> portalCells_; // cells in each portal, ordered by portal id
        std::vector<int> sharedAreaPortals_; // portals that share area with other portals
        // portal distances within clusters, flattened vector of map size × maximum number of portals in a cluster
        // contains distances from a mix of portals, indexed by a slice id to get specific portal distances
        // mixed data to save memory allocations and also to keep data locality
        std::vector<int> portalDistances_; // flattened portal distances
        std::vector<int> intraClusterHeuristics_; // precomputed intra-cluster shortest paths between portals
        std::vector<int> interClusterHeuristics_; // precomputed inter-cluster shortest paths between portals
        std::vector<int> projectionBase_; // base map for projection logic in dynamic environment
    };

} // namespace PreprocessingPipeline

#endif // PREPROCESS_H