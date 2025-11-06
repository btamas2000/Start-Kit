#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <memory>

namespace CooperativeAStarPlanner {
    
    // Reservation table to track occupied space-time locations
    class ReservationTable {
    public:
        ReservationTable() {}
        
        // Reserve a location at a specific timestep for an agent
        void addReservation(int location, int timestep, int agent_id);
        
        // Check if a location is reserved at a timestep (by another agent)
        bool isReserved(int location, int timestep, int agent_id) const;
        
        // Check for edge conflict (swap)
        bool hasEdgeConflict(int from_loc, int to_loc, int timestep, int agent_id) const;
        
        // Clear all reservations
        void clear();
        
        // Get agent at location and timestep
        int getAgent(int location, int timestep) const;
        
    private:
        // Map: location -> timestep -> agent_id
        std::unordered_map<int, std::unordered_map<int, int>> reservations;
    };
    
    // A* node for space-time A* search
    struct AStarNode {
        State state;
        int g_cost;
        int h_cost;
        int f_cost;
        std::shared_ptr<AStarNode> parent;
        
        AStarNode(const State& s, int g, int h, std::shared_ptr<AStarNode> p = nullptr)
            : state(s), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}
        
        bool operator>(const AStarNode& other) const {
            if (f_cost != other.f_cost) return f_cost > other.f_cost;
            return g_cost < other.g_cost;
        }
    };
    
    // Compute heuristic distance
    int getHeuristic(int location, int goal_location, SharedEnvironment* env);
    
    // Get valid neighboring states
    std::vector<State> getNeighbors(const State& state, SharedEnvironment* env);
    
    // Space-time A* search for a single agent considering reservations
    std::vector<State> spaceTimeAStar(
        int agent_id,
        const State& start,
        int goal_location,
        const ReservationTable& reservations,
        SharedEnvironment* env,
        int max_timestep = 1000
    );
    
    // Plan paths for all agents cooperatively
    std::vector<std::vector<State>> cooperativePlan(
        const std::vector<int>& agent_order,
        const std::vector<int>& goals,
        SharedEnvironment* env,
        int time_limit
    );
    
    // Determine agent ordering heuristic
    std::vector<int> determineAgentOrder(
        const std::vector<int>& goals,
        SharedEnvironment* env
    );
    
    // Initialize planner
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    
    // Main planning function
    void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env);
    
} // namespace CooperativeAStarPlanner