#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <vector>
#include <string>
#include <unordered_set>

namespace clustering {

// Cluster data structure with flattened cell indices
struct Cluster {
    int id;
    int degree;
    int maxima;
    std::vector<int> neighbors;
    int size;
    std::vector<int> cells;  // Flattened 1D indices
};

// Main clustering pipeline class - works with Grid's 1D map representation
class ClusteringPipeline {
public:
    // Constructor takes Grid's map data and dimensions
    ClusteringPipeline(const std::vector<int>& map, int rows, int cols);
    ~ClusteringPipeline();
    
    // Run the full clustering pipeline
    void runClustering(int threshold = 2, float dropFraction = 0.5f, int minCorridorSize = 4);
    
    // Individual pipeline steps (exposed for debugging/testing)
    void computeDistanceTransform();
    std::vector<std::pair<int, std::vector<int>>> findLocalMaximas(int threshold);
    std::vector<Cluster> formInitialClusters(
        const std::vector<std::pair<int, std::vector<int>>>& localMaxima,
        float dropFraction
    );
    std::vector<Cluster> clusterDT1Cells(const std::vector<Cluster>& clusters, int minSize);
    std::vector<Cluster> assignLeftoverCells(const std::vector<Cluster>& clusters);
    std::vector<Cluster> updateClusterNeighbors(const std::vector<Cluster>& clusters);
    
    // Getters
    const std::vector<int>& getMap() const { return map_; }
    const std::vector<int>& getDistances() const { return distances_; }
    const std::vector<Cluster>& getClusters() const { return clusters_; }
    const std::vector<int>& getCellToClusterMap() const { return cellToClusterMap_; }
    int getClusterID(int loc) const { return cellToClusterMap_[loc]; }
    int getRows() const { return rows_; }
    int getCols() const { return cols_; }

private:
    int rows_;
    int cols_;
    const std::vector<int>& map_;  // Reference to Grid's map (1=obstacle, 0=free)
    std::vector<int> distances_;   // Flattened distance transform
    std::vector<Cluster> clusters_; // Computed clusters
    std::vector<int> cellToClusterMap_; // Fast lookup: cell location -> cluster ID
    
    // Helper functions
    inline bool isFree(int loc) const { return map_[loc] == 0; }
    inline int getRow(int loc) const { return loc / cols_; }
    inline int getCol(int loc) const { return loc % cols_; }
    inline int getLoc(int row, int col) const { return row * cols_ + col; }
    
    std::pair<int, int> findGlobalMinMax() const;
    std::vector<int> selectCellsAtDistance(int targetDistance) const;
    void buildCellToClusterMap();
    
    // Direction vectors for 4-connectivity
    static const int DR[4];
    static const int DC[4];
};

} // namespace clustering

#endif // CLUSTERING_H
