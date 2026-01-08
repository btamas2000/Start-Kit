#ifndef PREPROCESS_H
#define PREPROCESS_H

// Everything related to preprocessing the static environment for pathfinding
namespace PreprocessingPipeline {
    const int MINIMUM_DT1_CLUSTER_SIZE = 4;

    struct Portal {
        int id;
        int from;
        int to;
        std::vector<int> cells;  // Flattened 1D indices
        int opposite_portal_id;
        bool is_critical;
        bool has_shared_area;
        std::unordered_set<int> affected_portals;
    };

    struct Cluster {
        int id;
        int area_id;
        std::vector<int> cells;  // Flattened 1D indices
        int size;
        int maxima;
        std::vector<int> neighbors;
        std::vector<Portal*> portals;
    };

    // Precomputed costs to reach the portal's cells within the cluster
    struct PortalDistances {
        int p_id;
        std::vector<std::pair<int, int>> distances;
    };

    struct PortalHeuristic {
        int portal_to;
        int start_cell;
        int end_cell;
        int timesteps;
    };

    struct IntraClusterHeuristic {
        int portal_from;
        std::unordered_map<int, std::vector<PortalHeuristic>> paths_from_portal;
    };

    struct PortalPathStep {
        int from_portal;
        int to_portal;
        bool is_critical;
        int from_timestep;
        int to_timestep;
    };

    struct PortalPath {
        std::vector<PortalPathStep> steps;
        int total_timesteps;
    };

    // Indexed by [start_portal][end_portal]
    typedef std::vector<std::vector<PortalPath>> InterClusterPath;

    // Indexed by [cluster_from][cluster_to]
    typedef std::vector<std::vector<InterClusterPath>> InterClusterHeuristic;

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
        void initialize(const std::vector<int>& map, int rows, int cols) {
            if (initialized_) return;

            map_ = map;
            rows_ = rows;
            cols_ = cols;

            initialized_ = true;
        }

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

        // Get portals
        const std::vector<Portal*>& getPortals() const {
            return portals_;
        }

        // Get portal distances
        const std::vector<PortalDistances*>& getPortalDistances() const {
            return portalDistances_;
        }

        // Get intra-cluster heuristics
        const std::vector<IntraClusterHeuristic>& getIntraClusterHeuristics() const {
            return intraClusterHeuristics_;
        }

        // Get inter-cluster heuristics
        const InterClusterHeuristic& getInterClusterHeuristics() const {
            return interClusterHeuristics_;
        }

        int getClusterId(int loc) const {
            if (loc < 0 || loc >= static_cast<int>(clusterMap_.size())) {
                return -1;
            }
            return clusterMap_[loc];
        }

    private:
        // Private constructor for singleton
        PreprocessPipeline();
        ~PreprocessPipeline() = default;

        inline int getLoc(int row, int col) const { return row * cols_ + col; }

        // Distict area identification
        void identifyDistinctAreas();

        // Distance transform computation
        void computeDistanceTransform();

        // Local maxima identification
        void findLocalMaximas(int threshold);

        // Merge diagonally connected local maxima plateaus
        void mergeLocalMaximas();

        // Initial cluster formation from local maxima
        void formInitialClusters();

        // Clustering possible dt=1 cells
        void clusterDT1Cells(int minSize);

        // Greedy assignment of leftover cells
        void assignLeftoverCells();

        // Update cluster neighbor information
        void updateTopology();

        // Identify portal connections
        void updatePortals();

        // Precompute portal distances within clusters
        void computePortalDistances();

        // Precompute intra-cluster shortest paths between portals
        void computeIntraClusterShortestPaths();

        // Precompute inter-cluster paths via portals
        void computeInterClusterHeuristics();

        bool initialized_;
        bool preprocessing_done_;

        // base map data
        std::vector<int> map_;
        int rows_;
        int cols_;
        
        // Preprocessing data members
        std::vector<int> areaMap_; // distinct area IDs for each cell
        std::vector<int> distanceTransform_; // distance to nearest obstacle
        std::vector<std::pair<int, std::vector<int>>> localMaximaPlateaus_; // local maxima plateaus
        std::vector<Cluster> clusters_; // computed clusters
        std::vector<int> clusterMap_; // cell-to-cluster mapping
        std::vector<Portal*> portals_; // computed portals
        std::vector<PortalDistances*> portalDistances_; // precomputed portal distances within clusters
        std::vector<IntraClusterHeuristic> intraClusterHeuristics_; // precomputed intra-cluster shortest paths between portals
        InterClusterHeuristic interClusterHeuristics_; // precomputed inter-cluster paths via portals
    };

} // namespace PreprocessingPipeline

#endif // PREPROCESS_H