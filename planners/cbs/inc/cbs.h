#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <memory>

// Conflict types
enum class ConflictType {
    VERTEX,  // Two agents at same location at same time
    EDGE     // Two agents swap locations (edge collision)
};

// Constraint on an agent
struct Constraint {
    int agent_id;
    int location;
    int timestep;
    ConflictType type;
    int from_location; // For edge constraints
    
    Constraint(int agent, int loc, int time, ConflictType t = ConflictType::VERTEX, int from = -1)
        : agent_id(agent), location(loc), timestep(time), type(t), from_location(from) {}
    
    bool operator==(const Constraint& other) const {
        return agent_id == other.agent_id && 
               location == other.location && 
               timestep == other.timestep &&
               type == other.type &&
               from_location == other.from_location;
    }
};

// Hash function for constraints
struct ConstraintHash {
    size_t operator()(const Constraint& c) const {
        return std::hash<int>()(c.agent_id) ^ 
               (std::hash<int>()(c.location) << 1) ^
               (std::hash<int>()(c.timestep) << 2) ^
               (std::hash<int>()(c.from_location) << 3);
    }
};

// Conflict between two agents
struct Conflict {
    int agent1;
    int agent2;
    int location;
    int timestep;
    ConflictType type;
    int location2; // For edge conflicts (second location)
    
    Conflict(int a1, int a2, int loc, int time, ConflictType t = ConflictType::VERTEX, int loc2 = -1)
        : agent1(a1), agent2(a2), location(loc), timestep(time), type(t), location2(loc2) {}
};

// CBS Node (high-level search node)
struct CBSNode {
    std::unordered_set<Constraint, ConstraintHash> constraints;
    std::vector<std::vector<State>> solution; // Path for each agent
    std::vector<Conflict> conflicts;
    int cost; // Sum of individual path costs
    
    CBSNode() : cost(0) {}
    
    // Compare for priority queue (min heap)
    bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }
};

namespace CBSPlanner {
    
    // Low-level A* search with constraints
    std::vector<State> lowLevelSearch(
        int agent_id,
        const State& start,
        int goal_location,
        const std::unordered_set<Constraint, ConstraintHash>& constraints,
        SharedEnvironment* env,
        int max_timestep = 1000
    );
    
    // Detect conflicts in solution
    std::vector<Conflict> detectConflicts(
        const std::vector<std::vector<State>>& solution,
        SharedEnvironment* env
    );
    
    // Check if constraint applies to a state
    bool violatesConstraint(
        const State& state,
        const State& prev_state,
        const std::unordered_set<Constraint, ConstraintHash>& constraints,
        int agent_id
    );
    
    // Get valid neighbors for a state
    std::vector<State> getNeighbors(
        const State& state,
        SharedEnvironment* env
    );
    
    // Heuristic function
    int getHeuristic(int location, int goal_location, SharedEnvironment* env);
    
    // Initialize planner
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    
    // Main CBS planning function
    void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env);
    
} // namespace CBSPlanner