#include "cluster_pibt_planner.h"
#include <algorithm>
#include <random>
#include <cassert>

namespace ClusterPIBTPlanner {

// Static data structures
std::vector<double> priorities;
std::vector<int> agent_ids;
std::vector<int> prev_decision;
std::vector<int> decision;
std::vector<bool> occupied;
std::vector<State> prev_states;
std::vector<State> next_states;
std::vector<int> agent_goals;
std::vector<int> dummy_goals;
std::vector<DecisionRecord> decided;
std::vector<bool> checked;

{
    // Initialize cluster heuristics
    clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
    clusterHeuristics.initialize(env->map, env->rows, env->cols, env);
    
    // Initialize data structures
    int num_agents = env->num_of_agents;
    int map_size = env->map.size();
    
    priorities.resize(num_agents);
    agent_ids.resize(num_agents);
    prev_decision.resize(map_size, -1);
    decision.resize(map_size, -1);
    occupied.resize(map_size, false);
    prev_states.resize(num_agents);
    next_states.resize(num_agents);
    agent_goals.resize(num_agents);
    dummy_goals.resize(num_agents);
    decided.resize(num_agents);
    checked.resize(num_agents, false);
    
    // Initialize agent IDs and priorities
    for (int i = 0; i < num_agents; i++) {
        agent_ids[i] = i;
        dummy_goals[i] = env->curr_states[i].location;
    }
    
    // Shuffle and assign initial priorities
    std::mt19937 rng(0);
    std::shuffle(agent_ids.begin(), agent_ids.end(), rng);
    for (int i = 0; i < num_agents; i++) {
        priorities[agent_ids[i]] = (double)(num_agents - i) / (double)(num_agents + 1);
    }
}

void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env)
{
    actions.resize(env->num_of_agents);
    
    // Reset decision tracking
    std::fill(prev_decision.begin(), prev_decision.end(), -1);
    std::fill(decision.begin(), decision.end(), -1);
    std::fill(occupied.begin(), occupied.end(), false);
    
    // Update agent states and goals
    for (int i = 0; i < env->num_of_agents; i++) {
        prev_states[i] = env->curr_states[i];
        next_states[i] = State(-1, -1, -1);
        prev_decision[env->curr_states[i].location] = i;
        
        // Set goal location (use dummy goal if no goal assigned)
        if (env->goal_locations[i].empty()) {
            agent_goals[i] = dummy_goals[i];
        } else {
            agent_goals[i] = env->goal_locations[i].front().first;
        }
        
        // Handle agents that are still turning from previous timestep
        if (decided[i].location == -1) {
            decided[i].location = env->curr_states[i].location;
            decided[i].done = true;
        }
        if (prev_states[i].location == decided[i].location) {
            decided[i].done = true;
        }
        if (!decided[i].done) {
            decision[decided[i].location] = i;
            next_states[i] = State(decided[i].location, -1, -1);
        }
    }
    
    // Sort agents by priority (higher priority first)
    std::vector<int> sorted_agents = agent_ids;
    std::sort(sorted_agents.begin(), sorted_agents.end(), 
        [this](int a, int b) { return priorities[a] > priorities[b]; });
    
    // Execute PIBT for each agent in priority order
    for (int agent_id : sorted_agents) {
        if (decided[agent_id].done && next_states[agent_id].location == -1) {
            executePIBT(agent_id, -1, env);
        }
    }
    
    // Post-process: convert targeted locations to actions
    for (int agent_id : sorted_agents) {
        // Clear decision table
        if (next_states[agent_id].location != -1) {
            decision[next_states[agent_id].location] = -1;
        }
        
        // Record new decisions as not done (will require turning)
        if (next_states[agent_id].location >= 0) {
            decided[agent_id].location = next_states[agent_id].location;
            decided[agent_id].done = false;
        }
        
        // Compute action (turning or forward)
        actions[agent_id] = computeAction(prev_states[agent_id], decided[agent_id].location, env);
        checked[agent_id] = false;
    }
    
    // Check if forward moves are valid (agents ahead must also move forward)
    for (int i = 0; i < env->num_of_agents; i++) {
        if (!checked[i] && actions[i] == Action::FW) {
            moveCheck(i, actions, env);
        }
    }
}

bool executePIBT(int agent_id, int higher_priority_agent, SharedEnvironment* env)
{
    assert(next_states[agent_id].location == -1);
    
    int current_loc = prev_states[agent_id].location;
    int goal_loc = agent_goals[agent_id];
    
    // Generate successor locations (neighbors + wait)
    struct Successor {
        int location;
        int heuristic;
        int tie_breaker;
        
        Successor(int loc, int h, int tb) : location(loc), heuristic(h), tie_breaker(tb) {}
    };
    
    std::vector<Successor> successors;
    auto neighbors = getNeighbors(current_loc, env);
    
    // Add neighbor locations
    for (int neighbor : neighbors) {
        if (validateMove(current_loc, neighbor, env)) {
            int h = getHeuristic(neighbor, goal_loc);
            successors.emplace_back(neighbor, h, rand());
        }
    }
    
    // Add wait action
    int wait_h = getHeuristic(current_loc, goal_loc);
    successors.emplace_back(current_loc, wait_h, rand());
    
    // Sort successors by heuristic (lower is better)
    std::sort(successors.begin(), successors.end(),
        [](const Successor& a, const Successor& b) {
            if (a.heuristic == b.heuristic) {
                return a.tie_breaker < b.tie_breaker;
            }
            return a.heuristic < b.heuristic;
        });
    
    // Try each successor in order
    for (const auto& successor : successors) {
        int next_loc = successor.location;
        
        // Skip if occupied or already decided
        if (occupied[next_loc]) continue;
        if (next_loc == -1) continue;
        if (decision[next_loc] != -1) continue;
        
        // Skip if higher priority agent is moving to this location
        if (higher_priority_agent != -1 && prev_decision[next_loc] == higher_priority_agent) {
            continue;
        }
        
        // Tentatively assign this location
        next_states[agent_id] = State(next_loc, -1, -1);
        decision[next_loc] = agent_id;
        
        // Check if another agent needs to move out of this location
        if (prev_decision[next_loc] != -1 && next_states[prev_decision[next_loc]].location == -1) {
            int lower_priority_agent = prev_decision[next_loc];
            
            // Recursively plan for the lower priority agent
            if (!executePIBT(lower_priority_agent, agent_id, env)) {
                // Failed to move lower priority agent, try next successor
                decision[next_loc] = -1;
                next_states[agent_id] = State(-1, -1, -1);
                continue;
            }
        }
        
        // Success!
        return true;
    }
    
    // No valid move found, wait in place
    next_states[agent_id] = State(current_loc, -1, -1);
    decision[current_loc] = agent_id;
    return false;
}

int getHeuristic(int from_loc, int to_loc)
{
    // Use cluster heuristics for distance estimation
    clustering::ClusterHeuristics& clusterHeuristics = clustering::ClusterHeuristics::getInstance();
    return clusterHeuristics.getHeuristicDistance(from_loc, to_loc);
}

std::vector<int> getNeighbors(int location, SharedEnvironment* env)
{
    std::vector<int> neighbors;
    int row = location / env->cols;
    int col = location % env->cols;
    
    // Check all 4 cardinal directions
    std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    
    for (const auto& [dr, dc] : directions) {
        int new_row = row + dr;
        int new_col = col + dc;
        
        if (new_row >= 0 && new_row < env->rows && 
            new_col >= 0 && new_col < env->cols) {
            int neighbor_loc = new_row * env->cols + new_col;
            if (env->map[neighbor_loc] == 0) {  // Not an obstacle
                neighbors.push_back(neighbor_loc);
            }
        }
    }
    
    return neighbors;
}

bool validateMove(int from_loc, int to_loc, SharedEnvironment* env)
{
    // Check if move is to an adjacent cell or wait
    if (from_loc == to_loc) return true;
    
    int diff = abs(to_loc - from_loc);
    bool is_adjacent = (diff == 1) || (diff == env->cols);
    
    // Check if destination is not an obstacle
    bool is_valid = is_adjacent && (to_loc >= 0) && 
                    (to_loc < env->map.size()) && 
                    (env->map[to_loc] == 0);
    
    return is_valid;
}

Action computeAction(const State& prev, int next_loc, SharedEnvironment* env)
{
    // If staying in place, wait
    if (prev.location == next_loc) {
        return Action::W;
    }
    
    // Determine required orientation for the move
    int diff = next_loc - prev.location;
    int required_orientation;
    
    if (diff == 1) {
        required_orientation = 0;  // East
    } else if (diff == -1) {
        required_orientation = 2;  // West
    } else if (diff == env->cols) {
        required_orientation = 1;  // South
    } else if (diff == -env->cols) {
        required_orientation = 3;  // North
    } else {
        // Invalid move
        return Action::W;
    }
    
    // Check if agent is already facing the right direction
    if (required_orientation == prev.orientation) {
        return Action::FW;
    }
    
    // Need to rotate - choose shortest rotation
    int rotation_diff = (required_orientation - prev.orientation + 4) % 4;
    
    if (rotation_diff == 1) {
        return Action::CR;  // Clockwise
    } else if (rotation_diff == 3) {
        return Action::CCR;  // Counter-clockwise
    } else if (rotation_diff == 2) {
        return Action::CR;  // 180 degrees - choose clockwise
    }
    
    return Action::W;
}

bool moveCheck(int agent_id, std::vector<Action>& actions, SharedEnvironment* env)
{
    // If already checked and moving forward, return true
    if (checked[agent_id] && actions[agent_id] == Action::FW) {
        return true;
    }
    
    checked[agent_id] = true;
    
    // If not moving forward, no need to check further
    if (actions[agent_id] != Action::FW) {
        return false;
    }
    
    // Check if the target location has another agent
    int target_loc = decided[agent_id].location;
    assert(target_loc != -1);
    
    int blocking_agent = prev_decision[target_loc];
    if (blocking_agent == -1) {
        return true;  // No agent blocking
    }
    
    // Recursively check if blocking agent can move forward
    if (moveCheck(blocking_agent, actions, env)) {
        return true;
    }
    
    // Blocking agent cannot move, so this agent must wait
    actions[agent_id] = Action::W;
    return false;
}

} // namespace ClusterPIBTPlanner
