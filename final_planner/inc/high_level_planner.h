#ifndef HIGH_LEVEL_PLANNER_H
#define HIGH_LEVEL_PLANNER_H

#include "dynamic.h"
#include "preprocess.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace DynamicData {

    struct HLNode {
        WaypointType type;
        int id; // portal id or location id
        int arrival_time;
        float g_score;
        float h_score;
        HLNode* parent;

        float getF() const { return g_score + h_score; }

        HLNode(WaypointType t, int i, int at, float g, float h, HLNode* p) :
            type(t), id(i), arrival_time(at), g_score(g), h_score(h), parent(p) {}
    };

    
    struct HLNodeCompare {
        bool operator()(const HLNode* a, const HLNode* b) const {
            if (std::abs(a->getF() - b->getF()) < 0.001f) {
                return a->g_score < b->g_score; 
            }
            return a->getF() > b->getF(); 
        }
    };

    struct HLNodeHash {
        std::size_t operator()(const HLNode* n) const {
            std::size_t h1 = std::hash<int>()(n->id);
            std::size_t h2 = std::hash<int>()(static_cast<int>(n->type));
            std::size_t h3 = std::hash<int>()(n->arrival_time / TIME_BUCKET);

            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    struct HLNodeEqual {
        bool operator()(const HLNode* a, const HLNode* b) const {
            return a->id == b->id &&
                a->type == b->type &&
                (a->arrival_time / TIME_BUCKET) == (b->arrival_time / TIME_BUCKET);
        }
    };

    class HighLevelPlanner {
    public:
        HighLevelPlanner();
        ~HighLevelPlanner() { clearNodePool(); }
        
        std::vector<HighLevelStep> planHighLevelPath(int agent_id);

    private:

        const float HL_LARGE_COST = 1e6f;
        const float CLUSTER_PENALTY = 5.0f;
        const float PORTAL_PENALTY = 20.0f;

        float manhattanDistance(int r1, int c1, int r2, int c2);

        std::vector<std::pair<int, float>> getPortalDistancesFromLocation(int location_id);

        void clearNodePool();

        std::vector<std::pair<int, WaypointType>> getNeighbors(HLNode* current_node, int goal_loc);

        std::pair<float, int> getGScore(HLNode* from_node, int id, WaypointType type);

        float getHScore(int from_id, WaypointType from_type, int to_id, WaypointType to_type);

        std::vector<HighLevelStep> reconstructPath(HLNode* goal_node);

        std::pair<bool, std::vector<HighLevelStep>> findPath(int start_id, WaypointType start_type, int goal_id, WaypointType goal_type, int start_time); 

        std::vector<HLNode*> node_pool_; // to manage memory of created nodes
    };
}

#endif // HIGH_LEVEL_PLANNER_H