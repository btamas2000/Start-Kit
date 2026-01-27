#ifndef LOW_LEVEL_PLANNER_H
#define LOW_LEVEL_PLANNER_H

#include "dynamic.h"
#include "preprocess.h"
#include <deque>
#include <unordered_map>
#include <tuple>

namespace DynamicData {

    struct LLNode {
        int location;
        int orientation;
        int timestep;
        int hl_step_index;
        float g_score;
        float h_score;
        LLNode* parent;
        Action action_taken; // action taken to reach this state

        float getF() const { return g_score + h_score; }

        LLNode(int l, int o, int t, int hl_idx, float g, float h, LLNode* p, Action a) :
            location(l), orientation(o), timestep(t), hl_step_index(hl_idx), g_score(g), h_score(h), parent(p), action_taken(a) {}
    };

    struct LLNodeCompare {
        bool operator()(const LLNode* a, const LLNode* b) const {           
            if (a->hl_step_index != b->hl_step_index) {
                return a->hl_step_index < b->hl_step_index;
            }
            if (std::abs(a->getF() - b->getF()) > 0.0001f) {
                return a->getF() > b->getF();
            }
            return a->g_score > b->g_score; 
        }
    };

    struct LLNodeHash {
        std::size_t operator()(const LLNode* node) const {
            std::size_t h1 = std::hash<int>()(node->location);
            std::size_t h2 = std::hash<int>()(node->orientation);
            std::size_t h3 = std::hash<int>()(node->timestep);
            std::size_t h4 = std::hash<int>()(node->hl_step_index);

            return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
        }
    };

    struct LLNodeEqual {
        bool operator()(const LLNode* a, const LLNode* b) const {
            return a->location == b->location &&
                   a->orientation == b->orientation &&
                   a->timestep == b->timestep &&
                   a->hl_step_index == b->hl_step_index;
        }
    };

    class LowLevelPlanner {
    public:
        LowLevelPlanner();
        ~LowLevelPlanner() { clearNodePool(); }

        std::pair<bool, std::vector<LowLevelStep>> planLowLevelPath(int agent_id);

        std::pair<bool, std::vector<LowLevelStep>> extendLowLevelPath(int agent_id, const LowLevelStep& last_step, int extend_until_timestep);

    private:
        std::vector<LLNode*> node_pool_;

        const float SOFT_RESERVATION_PENALTY = 5.0f;
        const float LL_LARGE_COST = 1e6f;

        void clearNodePool();

        float manhattanDistance(int r1, int c1, int r2, int c2);

        float getGScore(LLNode* from_node, int to_location, int to_orientation, int agent_id);

        float getHScore(int location, bool cluster_crossing, const HighLevelStep& hl_step);

        std::vector<LowLevelStep> reconstructPath(LLNode* goal_node);
    };
}

#endif // LOW_LEVEL_PLANNER_H