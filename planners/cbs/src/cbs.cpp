#include "cbs.h"
#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>

// Forward declare heuristic from default planner
namespace DefaultPlanner {
    int get_h(SharedEnvironment* env, int source, int target);
    void init_heuristics(SharedEnvironment* env);
}

namespace CBSPlanner {
    
    // A* node for low-level search
    struct AStarNode {
        State state;
        int g_cost; // Cost from start
        int h_cost; // Heuristic to goal
        int f_cost; // g + h
        std::shared_ptr<AStarNode> parent;
        
        AStarNode(const State& s, int g, int h, std::shared_ptr<AStarNode> p = nullptr)
            : state(s), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}
        
        bool operator>(const AStarNode& other) const {
            if (f_cost != other.f_cost) return f_cost > other.f_cost;
            return g_cost < other.g_cost; // Tie-break by preferring higher g
        }
    };
    
    // Heuristic function using Manhattan distance or precomputed heuristics
    int getHeuristic(int location, int goal_location, SharedEnvironment* env) {
        // Try to use precomputed heuristics if available
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
    
    // Get valid neighboring states considering orientation
    std::vector<State> getNeighbors(const State& state, SharedEnvironment* env) {
        std::vector<State> neighbors;
        int location = state.location;
        int orientation = state.orientation;
        int timestep = state.timestep + 1;
        
        // Wait action
        neighbors.push_back(State(location, timestep, orientation));
        
        // Rotation actions
        int new_orient_cw = (orientation + 1) % 4;
        int new_orient_ccw = (orientation + 3) % 4; // Same as (orientation - 1 + 4) % 4
        neighbors.push_back(State(location, timestep, new_orient_cw));
        neighbors.push_back(State(location, timestep, new_orient_ccw));
        
        // Forward movement based on current orientation
        int moves[4] = {1, env->cols, -1, -env->cols}; // East, South, West, North
        int new_location = location + moves[orientation];
        
        // Check if forward move is valid
        if (new_location >= 0 && new_location < env->map.size() && 
            env->map[new_location] == 0) {
            // Check if move doesn't wrap around grid edges
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
    
    // Check if a state violates any constraints
    bool violatesConstraint(
        const State& state,
        const State& prev_state,
        const std::unordered_set<Constraint, ConstraintHash>& constraints,
        int agent_id
    ) {
        for (const auto& constraint : constraints) {
            if (constraint.agent_id != agent_id) continue;
            if (constraint.timestep != state.timestep) continue;
            
            if (constraint.type == ConflictType::VERTEX) {
                if (constraint.location == state.location) {
                    return true;
                }
            } else if (constraint.type == ConflictType::EDGE) {
                // Edge constraint: can't move from constraint.from_location to constraint.location
                if (constraint.from_location == prev_state.location && 
                    constraint.location == state.location) {
                    return true;
                }
            }
        }
        return false;
    }
    
    // Low-level A* search for a single agent with constraints
    std::vector<State> lowLevelSearch(
        int agent_id,
        const State& start,
        int goal_location,
        const std::unordered_set<Constraint, ConstraintHash>& constraints,
        SharedEnvironment* env,
        int max_timestep
    ) {
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        std::unordered_map<int, std::unordered_map<int, int>> closed_list; // [location][timestep] -> g_cost
        
        int h = getHeuristic(start.location, goal_location, env);
        auto start_node = std::make_shared<AStarNode>(start, 0, h);
        open_list.push(*start_node);
        
        while (!open_list.empty()) {
            AStarNode current = open_list.top();
            open_list.pop();
            
            // Goal check
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
            
            // Check if already visited with lower cost
            int loc = current.state.location;
            int time = current.state.timestep;
            if (closed_list[loc].count(time) && closed_list[loc][time] <= current.g_cost) {
                continue;
            }
            closed_list[loc][time] = current.g_cost;
            
            // Expand neighbors
            std::vector<State> neighbors = getNeighbors(current.state, env);
            for (const State& next_state : neighbors) {
                // Check constraints
                if (violatesConstraint(next_state, current.state, constraints, agent_id)) {
                    continue;
                }
                
                // Check if already visited
                if (closed_list[next_state.location].count(next_state.timestep) &&
                    closed_list[next_state.location][next_state.timestep] <= current.g_cost + 1) {
                    continue;
                }
                
                int g = current.g_cost + 1;
                int h = getHeuristic(next_state.location, goal_location, env);
                auto next_node = std::make_shared<AStarNode>(next_state, g, h, 
                                                             std::make_shared<AStarNode>(current));
                open_list.push(*next_node);
            }
        }
        
        // No path found
        return std::vector<State>();
    }
    
    // Detect all conflicts in a solution
    std::vector<Conflict> detectConflicts(
        const std::vector<std::vector<State>>& solution,
        SharedEnvironment* env
    ) {
        std::vector<Conflict> conflicts;
        
        int max_time = 0;
        for (const auto& path : solution) {
            if (!path.empty()) {
                max_time = std::max(max_time, path.back().timestep);
            }
        }
        
        // Check each timestep for conflicts
        for (int t = 0; t <= max_time; ++t) {
            // Vertex conflicts
            std::map<int, std::vector<int>> location_to_agents;
            
            for (int i = 0; i < solution.size(); ++i) {
                if (solution[i].empty()) continue;
                
                // Get state at time t (or last state if path is shorter)
                const State* state = nullptr;
                for (const auto& s : solution[i]) {
                    if (s.timestep == t) {
                        state = &s;
                        break;
                    }
                }
                
                if (state == nullptr && !solution[i].empty()) {
                    state = &solution[i].back();
                }
                
                if (state != nullptr) {
                    location_to_agents[state->location].push_back(i);
                }
            }
            
            // Report vertex conflicts
            for (const auto& entry : location_to_agents) {
                if (entry.second.size() > 1) {
                    // Conflict found
                    conflicts.push_back(Conflict(
                        entry.second[0], 
                        entry.second[1], 
                        entry.first, 
                        t, 
                        ConflictType::VERTEX
                    ));
                }
            }
            
            // Edge conflicts (agents swapping positions)
            if (t > 0) {
                for (int i = 0; i < solution.size(); ++i) {
                    for (int j = i + 1; j < solution.size(); ++j) {
                        if (solution[i].empty() || solution[j].empty()) continue;
                        
                        // Get states at time t and t-1
                        const State *state_i_t = nullptr, *state_i_t1 = nullptr;
                        const State *state_j_t = nullptr, *state_j_t1 = nullptr;
                        
                        for (const auto& s : solution[i]) {
                            if (s.timestep == t) state_i_t = &s;
                            if (s.timestep == t - 1) state_i_t1 = &s;
                        }
                        
                        for (const auto& s : solution[j]) {
                            if (s.timestep == t) state_j_t = &s;
                            if (s.timestep == t - 1) state_j_t1 = &s;
                        }
                        
                        if (!state_i_t) state_i_t = &solution[i].back();
                        if (!state_i_t1) state_i_t1 = (solution[i].size() > 1) ? &solution[i][solution[i].size()-2] : &solution[i][0];
                        if (!state_j_t) state_j_t = &solution[j].back();
                        if (!state_j_t1) state_j_t1 = (solution[j].size() > 1) ? &solution[j][solution[j].size()-2] : &solution[j][0];
                        
                        // Check for edge conflict (swap)
                        if (state_i_t1->location == state_j_t->location &&
                            state_j_t1->location == state_i_t->location &&
                            state_i_t1->location != state_i_t->location) {
                            conflicts.push_back(Conflict(
                                i, j,
                                state_i_t->location,
                                t,
                                ConflictType::EDGE,
                                state_i_t1->location
                            ));
                        }
                    }
                }
            }
        }
        
        return conflicts;
    }
    
    // Initialize CBS planner
    void initialize(int preprocess_time_limit, SharedEnvironment* env) {
        // Initialize heuristics for distance calculations
        DefaultPlanner::init_heuristics(env);
    }
    
    // Main CBS planning function
    void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env) {
        auto start_time = std::chrono::steady_clock::now();
        
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
        
        // Initialize root CBS node
        CBSNode root;
        root.solution.resize(env->num_of_agents);
        root.cost = 0;
        
        // Find initial paths for all agents
        bool initial_success = true;
        for (int i = 0; i < env->num_of_agents; ++i) {
            root.solution[i] = lowLevelSearch(
                i,
                env->curr_states[i],
                goals[i],
                root.constraints,
                env
            );
            
            if (root.solution[i].empty()) {
                // Can't find path for this agent, use wait action
                root.solution[i].push_back(env->curr_states[i]);
                State wait_state = env->curr_states[i];
                wait_state.timestep++;
                root.solution[i].push_back(wait_state);
            }
            
            root.cost += root.solution[i].size();
        }
        
        // Detect conflicts in root solution
        root.conflicts = detectConflicts(root.solution, env);
        
        // Priority queue for CBS nodes
        std::priority_queue<CBSNode, std::vector<CBSNode>, std::greater<CBSNode>> open_list;
        open_list.push(root);
        
        CBSNode best_node = root;
        int iterations = 0;
        const int MAX_ITERATIONS = 1000;
        
        // CBS high-level search
        while (!open_list.empty() && iterations < MAX_ITERATIONS) {
            // Check time limit
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count();
            if (elapsed > time_limit * 0.8) break; // Leave time for action extraction
            
            CBSNode current = open_list.top();
            open_list.pop();
            iterations++;
            
            // If no conflicts, we found optimal solution
            if (current.conflicts.empty()) {
                best_node = current;
                break;
            }
            
            // Update best node if better
            if (current.conflicts.size() < best_node.conflicts.size()) {
                best_node = current;
            }
            
            // Pick first conflict to resolve
            Conflict conflict = current.conflicts.front();
            
            // Create two child nodes with constraints
            for (int agent : {conflict.agent1, conflict.agent2}) {
                CBSNode child = current;
                
                // Add constraint for this agent
                if (conflict.type == ConflictType::VERTEX) {
                    child.constraints.insert(Constraint(
                        agent,
                        conflict.location,
                        conflict.timestep,
                        ConflictType::VERTEX
                    ));
                } else { // EDGE
                    int from_loc = (agent == conflict.agent1) ? conflict.location2 : conflict.location;
                    int to_loc = (agent == conflict.agent1) ? conflict.location : conflict.location2;
                    child.constraints.insert(Constraint(
                        agent,
                        to_loc,
                        conflict.timestep,
                        ConflictType::EDGE,
                        from_loc
                    ));
                }
                
                // Replan path for constrained agent
                child.solution[agent] = lowLevelSearch(
                    agent,
                    env->curr_states[agent],
                    goals[agent],
                    child.constraints,
                    env
                );
                
                // If path found, add to open list
                if (!child.solution[agent].empty()) {
                    // Recalculate cost
                    child.cost = 0;
                    for (const auto& path : child.solution) {
                        child.cost += path.size();
                    }
                    
                    // Detect new conflicts
                    child.conflicts = detectConflicts(child.solution, env);
                    
                    open_list.push(child);
                }
            }
        }
        
        // Extract actions from best solution
        for (int i = 0; i < env->num_of_agents; ++i) {
            if (best_node.solution[i].size() < 2) {
                actions[i] = Action::W;
                continue;
            }
            
            const State& current = best_node.solution[i][0];
            const State& next = best_node.solution[i][1];
            
            // Determine action
            if (current.location != next.location) {
                actions[i] = Action::FW; // Forward move
            } else if (current.orientation != next.orientation) {
                int diff = (next.orientation - current.orientation + 4) % 4;
                if (diff == 1) {
                    actions[i] = Action::CR; // Clockwise rotate
                } else if (diff == 3) {
                    actions[i] = Action::CCR; // Counter-clockwise rotate
                } else {
                    actions[i] = Action::W; // Wait
                }
            } else {
                actions[i] = Action::W; // Wait
            }
        }
    }
    
} // namespace CBSPlanner