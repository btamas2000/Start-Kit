#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "ActionModel.h"
#include "Tasks.h"

namespace DynamicData {

    // --- Bitmask Constants for the Reservation Grid ---
    // Using bits allows us to store "Blockage" and "Shadow Flow" in 1 byte
    constexpr uint8_t CELL_EMPTY      = 0x00;
    constexpr uint8_t SHADOW_EAST    = 0x01; // Bit 0
    constexpr uint8_t SHADOW_SOUTH     = 0x02; // Bit 1
    constexpr uint8_t SHADOW_WEST    = 0x04; // Bit 2
    constexpr uint8_t SHADOW_NORTH     = 0x08; // Bit 3
    constexpr uint8_t HARD_OBSTACLE   = 0x80; // Bit 7 (128) - The agent's actual body

    const int PLANNING_HORIZON = 20; // Default rolling horizon window size
    const int TIME_BUCKET = 10;

    enum AgentBehavior {
        WAITING,
        MOVING_TO_GOAL,
        MOVING_TO_TEMPORARY_GOAL, // E.g., parking or stepping aside
        FINISHED
    };

    struct AgentPlanStep {
        int t;              // Absolute Simulation Time
        int location;       // 1D Index
        int orientation;    // 0:east, 1:south, 2:west, 3:north
        Action nextAction;  // Next action to take
    };

    class Agent {
    public:
        Agent(int id, int start_loc) 
            : agentID(id), currentLocation(start_loc), 
              currentBehavior(WAITING), priority(0), 
              current_plan_index(0), current_orientation(0) {}
        
        ~Agent() = default;

        // Core Data
        int agentID;
        int priority;
        AgentBehavior currentBehavior;
        
        // Location State
        int currentLocation;
        int current_orientation; // Critical for Shadow Logic
        
        // Planning State
        bool replanningNeeded = true;
        std::vector<int> goalLocations; // Main goals
        int temporaryGoalLocation = -1; // For parking or stepping aside
        
        // --- Hierarchical State ---
        // We need to know where we are in the High-Level Graph
        // Store Cluster IDs: [Current, Next, Next...]
        std::deque<int> high_level_path; 
        
        // --- Low-Level Execution ---
        // The concrete steps for the immediate future (Window)
        std::vector<AgentPlanStep> plan;
        int current_plan_index; // Points to the next step to execute in 'plan'

        bool isFree() const {
            return goalLocations.empty();
        }

        // Helper to get next immediate move without popping vector
        bool hasNextStep() const { return current_plan_index < plan.size(); }
        const AgentPlanStep& getNextStep() const { return plan[current_plan_index]; }
        
        void advance() { current_plan_index++; }
        
        void resetPlan() { 
            plan.clear(); 
            current_plan_index = 0; 
        }

        void assignTask(const Task& task) {
            goalLocations.clear();
            for (int loc : task.locations) {
                goalLocations.push_back(loc);
            }
            replanningNeeded = true;
        }
    };

    class DynamicEnvironment {
    public:
        static DynamicEnvironment& getInstance() {
            static DynamicEnvironment instance;
            return instance;
        }

        DynamicEnvironment(const DynamicEnvironment&) = delete;
        DynamicEnvironment& operator=(const DynamicEnvironment&) = delete;

        // Initialize the reservation table size
        void initialize(int map_size, int horizon_window = PLANNING_HORIZON) {
            map_size_ = map_size;
            window_size_ = horizon_window;
            current_simulation_time_ = 0;
            preprocessing_ = PreprocessPipeline::getInstance();
            reservation_table_.clear();
            agents_.clear();

            // [Time_Window][Map_Size]
            // We only allocate the WINDOW, not infinite time.
            reservation_table_.resize(window_size_, std::vector<uint8_t>(map_size_, 0));

            cluster_congestion_.clear();

            initialized_ = true;
        }

        bool isInitialized() const { return initialized_; }

        void setAgents(const std::vector<Agent>& agents) {
            agents_ = agents;
        }

        const std::vector<Agent>& getAgents() const {
            return agents_;
        }

        const Agent& getAgent(int agent_id) const {
            // agent_id is index in agents_ vector
            return agents_[agent_id];
        }

        // --- Core Time Management ---
        // Call this at the end of every simulation tick
        void advanceTime() {
            // 1. Clear the oldest time slice
            int slice_to_wipe = current_simulation_time_ % window_size_;
            std::fill(reservation_table_[slice_to_wipe].begin(), 
                      reservation_table_[slice_to_wipe].end(), 0);
            
            // 2. Increment global clock
            current_simulation_time_++;
        }

        int getCurrentTime() const { return current_simulation_time_; }
        int getWindowSize() const { return window_size_; }

        // --- Fast Lookups (O(1)) ---
        
        // Get the mask at a specific absolute time and location
        // Returns 0 if time is outside current window
        uint8_t getReservations(int time, int location) const {
            if (time < current_simulation_time_) return 0; // Past
            if (time >= current_simulation_time_ + window_size_) return 0; // Too far future

            int t_idx = time % window_size_;
            return reservation_table_[t_idx][location];
        }

        // --- Updating the Table ---

        // Add a Hard Block (Agent Body)
        void addReservation(int time, int location) {
            if (time >= current_simulation_time_ && time < current_simulation_time_ + window_size_) {
                reservation_table_[time % window_size_][location] |= HARD_OBSTACLE;
            }
        }

        // Add a Soft Shadow (Flow Direction)
        // dir: 0=east, 1=south, 2=west, 3=north
        void addShadow(int time, int location, int dir) {
             if (time >= current_simulation_time_ && time < current_simulation_time_ + window_size_) {
                uint8_t mask = (1 << dir); 
                
                reservation_table_[time % window_size_][location] |= mask;
            }
        }

        // dir: 0=east, 1=south, 2=west, 3=north
        // Output: Corresponding Bitmask
        inline uint8_t dirToMask(int dir) {
            return static_cast<uint8_t>(1 << (dir & 3)); 
        }

        inline uint8_t getHeadOnThreatMask(int my_dir) {
            int opposite_dir = (my_dir + 2) & 3;
            return static_cast<uint8_t>(1 << opposite_dir);
        }

        inline uint8_t getCrossingThreatMask(int my_dir) {
            uint8_t my_mask = (1 << (my_dir & 3));
            uint8_t opposite_mask = getHeadOnThreatMask(my_dir);
            
            return (0x0F ^ (my_mask | opposite_mask));
        }

        // --- Congestion Management ---

        // Update congestion for a cluster over a time range.
        // delta: +1 to add path, -1 to remove path (replanning)
        void updateClusterCongestion(int cluster_id, int start_t, int end_t, int delta) {
            // Ensure cluster table is large enough
            if (cluster_id >= static_cast<int>(cluster_congestion_.size())) {
                cluster_congestion_.resize(preprocessing_->getClusterCount() + 1); // Use accessor from preprocess if available, or resize dynamically
                if (cluster_id >= static_cast<int>(cluster_congestion_.size())) {
                     cluster_congestion_.resize(cluster_id + 1);
                }
            }

            int bucket_start = start_t / TIME_BUCKET;
            int bucket_end = end_t / TIME_BUCKET;

            // Resize time buckets if needed
            if (bucket_end >= static_cast<int>(cluster_congestion_[cluster_id].size())) {
                cluster_congestion_[cluster_id].resize(bucket_end + 10, 0); // Pre-allocate a bit more
            }

            for (int b = bucket_start; b <= bucket_end; ++b) {
                // Safety check to prevent underflow
                int new_val = cluster_congestion_[cluster_id][b] + delta;
                if (new_val < 0) new_val = 0;
                cluster_congestion_[cluster_id][b] = new_val;
            }
        }

        // Get congestion cost (Heuristic penalty)
        int getClusterCongestion(int cluster_id, int t) const {
            if (cluster_id >= static_cast<int>(cluster_congestion_.size())) return 0;
            
            int bucket = t / TIME_BUCKET;
            if (bucket >= static_cast<int>(cluster_congestion_[cluster_id].size())) return 0;
            
            return cluster_congestion_[cluster_id][bucket];
        }

        // --- Removing Reservations (For Replanning) ---

        // Remove a Hard Block (Agent Body)
        // usage: call this when an agent abandons its old plan
        void removeReservation(int time, int location) {
            if (time >= current_simulation_time_ && time < current_simulation_time_ + window_size_) {
                // Bitwise AND with NOT mask to clear specific bit
                // 1111 1111 & 0111 1111 = Clears bit 7
                reservation_table_[time % window_size_][location] &= ~HARD_OBSTACLE;
            }
        }

        // Remove a Soft Shadow
        // dir: 0=east, 1=south, 2=west, 3=north
        void removeShadow(int time, int location, int dir) {
             if (time >= current_simulation_time_ && time < current_simulation_time_ + window_size_) {
                uint8_t mask = (1 << dir); 
                // Clear specific direction bit
                reservation_table_[time % window_size_][location] &= ~mask;
            }
        }

        // Helper to clear an entire plan from the reservation table
        // Useful when an agent needs to fully replan
        void clearPlanReservations(const std::vector<AgentPlanStep>& plan) {
            for (const auto& step : plan) {
                // We only clear future steps, not past ones (history is immutable)
                if (step.t >= current_simulation_time_) {
                    removeReservation(step.t, step.location);
                    
                    // TODO: remove shadows as well
                }
            }
        }

        // Accessors
        PreprocessingPipeline::PreprocessPipeline& getPreprocessing() {
            return preprocessing_;
        }


    private:
        DynamicEnvironment() = default;
        ~DynamicEnvironment() = default;

        bool initialized_;

        int window_size_;
        int map_size_;
        int current_simulation_time_;
        PreprocessingPipeline::PreprocessPipeline preprocessing_;

        // The Circular Buffer
        // usage: reservation_table_[t % window][loc]
        std::vector<std::vector<uint8_t>> reservation_table_;

        // Congestion tracking for clusters
        // Indexed by [Cluster_ID][Time_Bucket]
        std::vector<std::vector<int>> cluster_congestion_;

        // All agents in the environment
        std::vector<Agent> agents_;
    };
}  // namespace DynamicData

#endif // DYNAMIC_H