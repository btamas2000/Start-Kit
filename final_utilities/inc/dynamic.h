#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "ActionModel.h"
#include "Tasks.h"
#include "SharedEnv.h"
#include "preprocess.h"
#include <deque>

namespace DynamicData {

    const uint8_t DIR_EAST = 0x01;      // (0000 0001)
    const uint8_t DIR_SOUTH  = 0x02;    // (0000 0010)
    const uint8_t DIR_WEST = 0x04;      // (0000 0100)
    const uint8_t DIR_NORTH  = 0x08;    // (0000 1000)

    // occupancy/reservation flags
    const uint8_t RES_HARD  = 0x10;     // (0001 0000) - actual agent
    const uint8_t RES_SOFT  = 0x20;     // (0010 0000) - projection
    const uint8_t OBS_STATIC= 0x40;     // (0100 0000) - wall

    // edge conflict flags
    const uint8_t MOVE_EAST = 0x01;    // (0000 0001)
    const uint8_t MOVE_SOUTH = 0x02;   // (0000 0010)
    const uint8_t MOVE_WEST = 0x04;    // (0000 0100)
    const uint8_t MOVE_NORTH = 0x08;   // (0000 1000)

    const int PLANNING_HORIZON = 20;    // default rolling horizon window size
    const int TIME_BUCKET = 10;         // time bucket size for congestion tracking
    const int NUM_TIME_BUCKETS = 50;    // number of time buckets for congestion tracking

    const int FILL_PLAN_AFTER = 7;      // fill low-level plan after this many steps with new steps

    struct Projections {
        bool projected_east;
        bool projected_south;
        bool projected_west;
        bool projected_north;
    };

    enum class WaypointType {
        LOCATION,
        PORTAL,
        NONE
    };

    struct HighLevelStep {
        WaypointType type;
        int id;                 // location index or portal id
        int arrival_time_est;   // estimated arrival time at this waypoint

        HighLevelStep(WaypointType w, int i, int at )
            : type(w), id(i), arrival_time_est(at) {}
    };

    struct LowLevelStep {
        int t;                  // time step
        int location;           // location index
        int orientation;        // 0:east, 1:south, 2:west, 3:north
        Action nextAction;      // next action to take (from ActionModel: FW, CR, CCR, W, NA)
        bool placeholder_step;  // true if no plan is made but we need it to synchronize with the reservation table

        int hl_step_index; // index of corresponding high-level step

        LowLevelStep(int time, int loc, int ori, Action action, bool placeholder, int hl_index) 
            : t(time), location(loc), orientation(ori), nextAction(action), placeholder_step(placeholder), hl_step_index(hl_index) {}
    };

    // wrapper class for agent data
    class Agent {
    public:
        Agent(int id);
        ~Agent() = default;

        void setPriority(int p) { priority = p; }
        int getPriority() const { return priority; }
        int getAgentID() const { return agentID; }
        void setCurrentLocation(int loc) { currentLocation = loc; }
        int getCurrentLocation() const { return currentLocation; }
        void setCurrentOrientation(int ori) { currentOrientation = ori; }
        int getCurrentOrientation() const { return currentOrientation; }
        void setHighLevelReplanNeeded(bool flag) { highLevelReplanNeeded = flag; }
        bool isHighLevelReplanNeeded() const { return highLevelReplanNeeded; }
        void setLowLevelReplanNeeded(bool flag) { lowLevelReplanNeeded = flag; }
        bool isLowLevelReplanNeeded() const { return lowLevelReplanNeeded; }
        void setTemporaryGoalLocation(int loc) { temporaryGoalLocation = loc; }
        int getTemporaryGoalLocation() const { return temporaryGoalLocation; }

        int getAssignedTaskID() const { return assigned_task_id; }
        void assignTask(int task_id);

        const std::vector<HighLevelStep>& getHighLevelPlan() const { return highLevelPlan; }
        const std::vector<LowLevelStep>& getLowLevelPlan() const { return lowLevelPlan; }

        const std::vector<LowLevelStep>& getLowLevelPlanFromIndex() const;

        int getRemainingPlannedSteps() const { return ll_planned_until - ll_step_index + 1; }

        int getHLStepIndex() const { return hl_step_index; }
        int getLLStepIndex() const { return ll_step_index; }
        int getLLPlannedUntil() const { return ll_planned_until; }

        void setHighLevelPlan(const std::vector<HighLevelStep>& plan);
        void setLowLevelPlan(const std::vector<LowLevelStep>& plan);
        void extendLowLevelPlan(const std::vector<LowLevelStep>& additional_steps, bool placeholder_extension);

        void executeStep();
        const std::pair<bool, LowLevelStep> getCurrentLowLevelStep() const;

    private:

        // core Data
        int agentID;
        int priority;
        
        // location State
        int currentLocation;
        int currentOrientation;
        
        // planning State
        bool highLevelReplanNeeded;
        bool lowLevelReplanNeeded;
        int temporaryGoalLocation = -1;

        std::vector<HighLevelStep> highLevelPlan; 
        std::vector<LowLevelStep> lowLevelPlan;
        int hl_step_index;
        int ll_step_index;
        int ll_planned_until;

        int assigned_task_id = -1; // -1 if no task assigned
    };

    // class for tracking congestion in the environment
    // used by high-level planner to guide path selection
    // uses decaying weights over time buckets
    class CongestionTracker {
    public:
        CongestionTracker();
        ~CongestionTracker() = default;

        void initialize(int num_portals, int num_clusters);

        void decay();

        void addPortalUsage(int portal_id, int timestep);
        void addClusterUsage(int cluster_id, int arrival_timestep, int departure_timestep);

        void pruneAndReuse(int current_time);

        float getPortalCongestion(int portal_id, int arrival_timestep);
        float getClusterCongestion(int cluster_id, int arrival_timestep, int departure_timestep);

        const float DECAY_FACTOR = 0.95f; // decay factor for congestion values
        const float CRITICAL_PORTAL_PENALTY = 100.0f; // penalty for using critical portals during congestion

    private:
        std::vector<float> portal_congestion_; // congestion values for each portal
        std::vector<float> cluster_congestion_; // congestion values for each cluster

        int oldest_time_bucket_;
    };

    class ReservationTable {
    public:
        ReservationTable();
        ~ReservationTable() = default;

        void initialize(int window_size, std::vector<int> map_base, int rows, int cols);
        
        void reservePath(std::vector<LowLevelStep>& path);
        void releasePath(std::vector<LowLevelStep>& path);
        
        void extendPathReservation(const LowLevelStep& last_step, std::vector<LowLevelStep>& path);

        void advanceTable(int current_time);

        bool isCellFree(int location, int timestep);
        bool isCellSoftReserved(int location, int timestep);
        Projections getProjections(int location, int timestep);
        uint8_t getEdgeReservations(int location, int timestep);

    private:
        std::vector<uint8_t> table_;        // for vertex conflicts, static obstacles and projections
        std::vector<uint8_t> edge_table_;   // for edge conflicts

        // projection vectors for each direction, contain counters for projections
        std::vector<int> proj_east_;
        std::vector<int> proj_south_;
        std::vector<int> proj_west_;
        std::vector<int> proj_north_;

        std::vector<int> soft_reservations_; // counters for soft reservations

        std::vector<int> map_base_; // contains information about map geometry related to projecting

        int window_size_;

        int rows_;
        int cols_;
    };

    // singleton class for dynamic environment data, used for scheduling and planning
    class DynamicEnvironment {
    public:
        static DynamicEnvironment& getInstance() {
            static DynamicEnvironment instance;
            return instance;
        }

        DynamicEnvironment(const DynamicEnvironment&) = delete;
        DynamicEnvironment& operator=(const DynamicEnvironment&) = delete;

        // Initialize the reservation table size
        void initialize(SharedEnvironment* shared_env);

        bool isInitialized() const { return initialized_; }

        void initializeLiveData();

        bool isLiveDataInitialized() const { return initialized_live_data_; }

        SharedEnvironment* getSharedEnvironment() const { return shared_env_; }

        std::vector<Agent>& getAgents() {
            return agents_;
        }

        Agent& getAgent(int agent_id) {
            // agent_id is index in agents_ vector
            return agents_[agent_id];
        }

        PreprocessingPipeline::PreprocessPipeline& getPreprocessing() {
            return PreprocessingPipeline::PreprocessPipeline::getInstance();
        }

        std::unordered_set<int>& getFreeAgents() {
            return free_agents_;
        }

        std::unordered_set<int>& getTaskPool() {
            return task_pool_;
        }

        ReservationTable& getReservationTable() {
            return reservation_table_;
        }

        CongestionTracker& getCongestionTracker() {
            return congestion_tracker_;
        }

        // advance the simulation by one time step
        bool advanceTimeStep();

        int getCurrentTime() const {
            return current_simulation_time_;
        }

    private:
        DynamicEnvironment();
        ~DynamicEnvironment() = default;

        bool initialized_;
        bool initialized_live_data_;

        int global_next_priority_;

        int current_simulation_time_;

        SharedEnvironment* shared_env_;

        std::vector<Agent> agents_;

        std::unordered_set<int> free_agents_; // agents that are free to be assigned new tasks
        std::unordered_set<int> task_pool_; // tasks that are unassigned

        ReservationTable reservation_table_;

        CongestionTracker congestion_tracker_;
    };
}  // namespace DynamicData

#endif // DYNAMIC_H