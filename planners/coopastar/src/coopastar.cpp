#include "coopastar.h"
#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>
#include <iostream>

// Forward declare heuristic from default planner
namespace DefaultPlanner {
    int get_h(SharedEnvironment* env, int source, int target);
    void init_heuristics(SharedEnvironment* env);
}

namespace CooperativeAStarPlanner {
    
    // ==================== ReservationTable Implementation ====================
    
    void ReservationTable::addReservation(int location, int timestep, int agent_id) {
        reservations[location][timestep] = agent_id;
    }
    
    bool ReservationTable::isReserved(int location, int timestep, int agent_id) const {
        auto loc_it = reservations.find(location);
        if (loc_it == reservations.end()) return false;
        
        auto time_it = loc_it->second.find(timestep);
        if (time_it == loc_it->second.end()) return false;
        
        // Reserved by another agent
        return time_it->second != agent_id;
    }
    
    bool ReservationTable::hasEdgeConflict(int from_loc, int to_loc, int timestep, int agent_id) const {
        // Check if another agent is moving from to_loc to from_loc at the same time
        // (i.e., agents swapping positions)
        auto from_it = reservations.find(from_loc);
        auto to_it = reservations.find(to_loc);
        
        if (from_it == reservations.end() || to_it == reservations.end()) return false;
        
        auto from_time_prev = from_it->second.find(timestep - 1);
        auto to_time_curr = to_it->second.find(timestep);
        
        if (from_time_prev == from_it->second.end() || to_time_curr == to_it->second.end()) {
            return false;
        }
        
        // Check if same agent (not this one) is doing the reverse move
        int other_agent = from_time_prev->second;
        if (other_agent != agent_id && other_agent == to_time_curr->second) {
            // Check if that agent was at to_loc and moved to from_loc
            auto to_time_prev = to_it->second.find(timestep - 1);
            if (to_time_prev != to_it->second.end() && to_time_prev->second == other_agent) {
                return true; // Edge conflict detected
            }
        }
        
        return false;
    }
    
    void ReservationTable::clear() {
        reservations.clear();
    }
    
    int ReservationTable::getAgent(int location, int timestep) const {
        auto loc_it = reservations.find(location);
        if (loc_it == reservations.end()) return -1;
        
        auto time_it = loc_it->second.find(timestep);
        if (time_it == loc_it->second.end()) return -1;
        
        return time_it->second;
    }
    
    // ==================== Helper Functions ====================
    
    // Compute heuristic distance between two locations
    int getHeuristic(int location, int goal_location, SharedEnvironment* env) {
        try {
            return DefaultPlanner::get_h(env, location, goal_location);
        } catch (...) {
            // Fall back to Manhattan distance
            int row1 = location / env->cols;
            int col1 = location % env->cols;
            int row2 = goal_location / env->cols;
            int col2 = goal_location % env->cols;
            return std::abs(row1 - row2) + std::abs(col1 - col2);
        }
    }
    
    // Get valid neighboring states for an agent
    std::vector<State> getNeighbors(const State& state, SharedEnvironment* env) {
        std::vector<State> neighbors;
        int location = state.location;
        int orientation = state.orientation;
        int timestep = state.timestep + 1;
        
        // Wait action - stay at current location with same orientation
        neighbors.push_back(State(location, timestep, orientation));
        
        // Rotation actions
        int new_orient_cw = (orientation + 1) % 4;       // Clockwise
        int new_orient_ccw = (orientation + 3) % 4;      // Counter-clockwise
        neighbors.push_back(State(location, timestep, new_orient_cw));
        neighbors.push_back(State(location, timestep, new_orient_ccw));
        
        // Forward movement based on current orientation
        // Orientations: 0=East, 1=South, 2=West, 3=North
        int moves[4] = {1, env->cols, -1, -env->cols};
        int new_location = location + moves[orientation];
        
        // Validate forward move
        if (new_location >= 0 && new_location < env->map.size() && 
            env->map[new_location] == 0) { // 0 = free space
            
            // Check grid boundary wrapping
            int row = location / env->cols;
            int new_row = new_location / env->cols;
            int col = location % env->cols;
            int new_col = new_location % env->cols;
            
            bool valid = true;
            if (orientation == 0 && new_col != col + 1) valid = false; // East
            if (orientation == 2 && new_col != col - 1) valid = false; // West
            if (orientation == 1 && new_row != row + 1) valid = false; // South
            if (orientation == 3 && new_row != row - 1) valid = false; // North
            
            if (valid) {
                neighbors.push_back(State(new_location, timestep, orientation));
            }
        }
        
        return neighbors;
    }
    
    // ==================== Space-Time A* Search ====================
    
    // A* search in space-time considering reservations from other agents
    std::vector<State> spaceTimeAStar(
        int agent_id,
        const State& start,
        int goal_location,
        const ReservationTable& reservations,
        SharedEnvironment* env,
        int max_timestep
    ) {
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        
        // Track visited states: [location][timestep][orientation] -> g_cost
        std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>> closed_list;
        
        // Initialize search
        int h = getHeuristic(start.location, goal_location, env);
        auto start_node = std::make_shared<AStarNode>(start, 0, h);
        open_list.push(*start_node);
        
        while (!open_list.empty()) {
            AStarNode current = open_list.top();
            open_list.pop();
            
            // Goal check - reached goal location
            if (current.state.location == goal_location && current.state.timestep < max_timestep) {
                // Reconstruct path
                std::vector<State> path;
                auto node = std::make_shared<AStarNode>(current);
                while (node != nullptr) {
                    path.push_back(node->state);
                    node = node->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            // Check if already visited with better or equal cost
            int loc = current.state.location;
            int time = current.state.timestep;
            int orient = current.state.orientation;
            
            if (closed_list[loc][time].count(orient) && 
                closed_list[loc][time][orient] <= current.g_cost) {
                continue;
            }
            closed_list[loc][time][orient] = current.g_cost;
            
            // Expand neighbors
            std::vector<State> neighbors = getNeighbors(current.state, env);
            
            for (const State& next_state : neighbors) {
                // Check if next state is reserved by another agent
                if (reservations.isReserved(next_state.location, next_state.timestep, agent_id)) {
                    continue; // Vertex conflict - skip this neighbor
                }
                
                // Check for edge conflicts (agents swapping positions)
                if (current.state.location != next_state.location) {
                    if (reservations.hasEdgeConflict(current.state.location, 
                                                     next_state.location, 
                                                     next_state.timestep, 
                                                     agent_id)) {
                        continue; // Edge conflict - skip this neighbor
                    }
                }
                
                // Check if already visited with better cost
                if (closed_list[next_state.location][next_state.timestep].count(next_state.orientation) &&
                    closed_list[next_state.location][next_state.timestep][next_state.orientation] <= current.g_cost + 1) {
                    continue;
                }
                
                // Add to open list
                int g = current.g_cost + 1;
                int h = getHeuristic(next_state.location, goal_location, env);
                auto next_node = std::make_shared<AStarNode>(
                    next_state, g, h, 
                    std::make_shared<AStarNode>(current)
                );
                open_list.push(*next_node);
            }
        }
        
        // No path found
        return std::vector<State>();
    }
    
    // ==================== Agent Ordering Heuristic ====================
    
    // Determine the order in which agents should plan
    // Heuristic: agents with longer paths or more constrained should go first
    std::vector<int> determineAgentOrder(
        const std::vector<int>& goals,
        SharedEnvironment* env
    ) {
        std::vector<std::pair<int, int>> agent_distances; // (distance_to_goal, agent_id)
        
        for (int i = 0; i < env->num_of_agents; ++i) {
            int dist = getHeuristic(env->curr_states[i].location, goals[i], env);
            agent_distances.push_back({dist, i});
        }
        
        // Sort by distance (descending) - agents with longer paths plan first
        std::sort(agent_distances.begin(), agent_distances.end(), 
                 [](const auto& a, const auto& b) { return a.first > b.first; });
        
        std::vector<int> order;
        for (const auto& pair : agent_distances) {
            order.push_back(pair.second);
        }
        
        return order;
    }
    
    // ==================== Cooperative Planning ====================
    
    // Plan paths for all agents cooperatively in the given order
    std::vector<std::vector<State>> cooperativePlan(
        const std::vector<int>& agent_order,
        const std::vector<int>& goals,
        SharedEnvironment* env,
        int time_limit
    ) {
        auto start_time = std::chrono::steady_clock::now();
        
        std::vector<std::vector<State>> solution(env->num_of_agents);
        ReservationTable reservations;
        
        // Plan for each agent in order
        for (int agent_id : agent_order) {
            // Check time limit
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count();
            if (elapsed > time_limit * 0.8) {
                // Running out of time, create simple wait path for remaining agents
                State wait_state = env->curr_states[agent_id];
                wait_state.timestep = 0;
                solution[agent_id].push_back(wait_state);
                wait_state.timestep = 1;
                solution[agent_id].push_back(wait_state);
                continue;
            }
            
            // Plan path for this agent avoiding previously planned agents
            std::vector<State> path = spaceTimeAStar(
                agent_id,
                env->curr_states[agent_id],
                goals[agent_id],
                reservations,
                env
            );
            
            // If no path found, create a simple wait path
            if (path.empty()) {
                State wait_state = env->curr_states[agent_id];
                wait_state.timestep = 0;
                path.push_back(wait_state);
                wait_state.timestep = 1;
                path.push_back(wait_state);
            }
            
            solution[agent_id] = path;
            
            // Add this agent's path to reservations
            for (const State& state : path) {
                reservations.addReservation(state.location, state.timestep, agent_id);
            }
            
            // Also reserve the final location for future timesteps
            // (agent stays at goal after reaching it)
            if (!path.empty()) {
                const State& final_state = path.back();
                for (int t = final_state.timestep + 1; t <= final_state.timestep + 100; ++t) {
                    reservations.addReservation(final_state.location, t, agent_id);
                }
            }
        }
        
        return solution;
    }
    
    // ==================== Main Interface Functions ====================
    
    void initialize(int preprocess_time_limit, SharedEnvironment* env) {
        // Initialize heuristics for distance calculations
        DefaultPlanner::init_heuristics(env);
    }
    
    void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env) {
        auto start_time = std::chrono::steady_clock::now();
        
        // Initialize actions to WAIT
        actions.resize(env->num_of_agents, Action::W);
        
        // Prepare goal locations for each agent
        std::vector<int> goals(env->num_of_agents);
        for (int i = 0; i < env->num_of_agents; ++i) {
            if (env->goal_locations[i].empty()) {
                goals[i] = env->curr_states[i].location; // Stay at current location
            } else {
                goals[i] = env->goal_locations[i].front().first;
            }
        }
        
        // Determine agent planning order
        std::vector<int> agent_order = determineAgentOrder(goals, env);
        
        // Plan cooperatively
        std::vector<std::vector<State>> solution = cooperativePlan(
            agent_order, goals, env, time_limit
        );
        
        // Extract actions from solution
        for (int i = 0; i < env->num_of_agents; ++i) {
            if (solution[i].size() < 2) {
                actions[i] = Action::W;
                continue;
            }
            
            const State& current = solution[i][0];
            const State& next = solution[i][1];
            
            // Determine action based on state transition
            if (current.location != next.location) {
                // Agent moved to a new location
                actions[i] = Action::FW;
            } else if (current.orientation != next.orientation) {
                // Agent rotated
                int diff = (next.orientation - current.orientation + 4) % 4;
                if (diff == 1) {
                    actions[i] = Action::CR; // Clockwise rotation
                } else if (diff == 3) {
                    actions[i] = Action::CCR; // Counter-clockwise rotation
                } else {
                    actions[i] = Action::W; // Wait (shouldn't happen)
                }
            } else {
                // Agent stayed in place
                actions[i] = Action::W;
            }
        }
    }
    
} // namespace CooperativeAStarPlanner