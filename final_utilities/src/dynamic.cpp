#include "dynamic.h"

namespace DynamicData {
    Agent::Agent(int id) {
        agentID = id;
        priority = -1;

        currentLocation = -1;
        currentOrientation = -1;

        highLevelReplanNeeded = false;
        lowLevelReplanNeeded = false;
        temporaryGoalLocation = -1;

        highLevelPlan.clear();
        lowLevelPlan.clear();

        assigned_task_id = -1;

        hl_step_index = -1;
        ll_step_index = -1;
        ll_planned_until = -1;
    }

    void Agent::setHighLevelPlan(const std::vector<HighLevelStep>& plan) {
        highLevelPlan = plan;
        hl_step_index = 0;
    }

    void Agent::setLowLevelPlan(const std::vector<LowLevelStep>& plan) {
        lowLevelPlan = plan;
        ll_step_index = 0;
        ll_planned_until = static_cast<int>(plan.size()) - 1;
    }

    void Agent::extendLowLevelPlan(const std::vector<LowLevelStep>& additional_steps, bool placeholder_extension) {
        for (const auto& step : additional_steps) {
            lowLevelPlan.emplace_back(step);
        }
        if (!placeholder_extension) {
            ll_planned_until = static_cast<int>(lowLevelPlan.size()) - 1;
        }
    }

    void Agent::executeStep() {
        if (ll_step_index < 0 || ll_step_index >= static_cast<int>(lowLevelPlan.size() - 1)) {
            return; // no valid step to execute
        }

        ll_step_index += 1;

        LowLevelStep& step = lowLevelPlan[ll_step_index];
        currentLocation = step.location;
        currentOrientation = step.orientation;
        hl_step_index = step.hl_step_index;
    }

    void Agent::assignTask(int task_id) {
        assigned_task_id = task_id;
        lowLevelReplanNeeded = true;
        highLevelReplanNeeded = true;
    }

    const std::pair<bool, LowLevelStep> Agent::getCurrentLowLevelStep() const {
        if (ll_step_index < 0 || ll_step_index >= static_cast<int>(lowLevelPlan.size())) {
            return {false, LowLevelStep(-1, -1, -1, Action::W, false, -1)};
        } else {
            return {true, lowLevelPlan[ll_step_index]};
        }
    }

    CongestionTracker::CongestionTracker() {
        portal_congestion_.clear();
        cluster_congestion_.clear();
        oldest_time_bucket_ = 0;
    }

    void CongestionTracker::initialize(int num_portals, int num_clusters) {
        portal_congestion_.resize(num_portals * NUM_TIME_BUCKETS, 0.0f);
        cluster_congestion_.resize(num_clusters * NUM_TIME_BUCKETS, 0.0f);
        oldest_time_bucket_ = 0;
    }

    void CongestionTracker::decay() {
        for (auto& val : portal_congestion_) {
            val *= DECAY_FACTOR;
        }
        for (auto& val : cluster_congestion_) {
            val *= DECAY_FACTOR;
        }
    }

    void CongestionTracker::addPortalUsage(int portal_id, int timestep) {
        int time_bucket = timestep / TIME_BUCKET;
        if (time_bucket >= oldest_time_bucket_ + NUM_TIME_BUCKETS) return;
        int bucket_index = portal_id * NUM_TIME_BUCKETS + (time_bucket % NUM_TIME_BUCKETS);
        portal_congestion_[bucket_index] += 1.0f;
    }

    void CongestionTracker::addClusterUsage(int cluster_id, int arrival_timestep, int departure_timestep) {
        int time_bucket_start = arrival_timestep / TIME_BUCKET;
        int time_bucket_end = departure_timestep / TIME_BUCKET;
        if (time_bucket_end >= oldest_time_bucket_ + NUM_TIME_BUCKETS || time_bucket_start < oldest_time_bucket_) return;

        for (int tb = time_bucket_start; tb <= time_bucket_end; ++tb) {
            int bucket_index = cluster_id * NUM_TIME_BUCKETS + (tb % NUM_TIME_BUCKETS);
            cluster_congestion_[bucket_index] += 1.0f;
        }
    }

    void CongestionTracker::pruneAndReuse(int current_time) {
        int current_bucket = current_time / TIME_BUCKET;

        if (current_bucket <= oldest_time_bucket_) return;

        int portal_size = static_cast<int>(portal_congestion_.size()) / NUM_TIME_BUCKETS;

        for (int p = 0; p < portal_size; p++) {
            portal_congestion_[p * NUM_TIME_BUCKETS + (oldest_time_bucket_ % NUM_TIME_BUCKETS)] = 0.0f;
        }

        int cluster_size = static_cast<int>(cluster_congestion_.size()) / NUM_TIME_BUCKETS;

        for (int c = 0; c < cluster_size; c++) {
            cluster_congestion_[c * NUM_TIME_BUCKETS + (oldest_time_bucket_ % NUM_TIME_BUCKETS)] = 0.0f;
        }

        oldest_time_bucket_ = current_bucket;
    }

    float CongestionTracker::getPortalCongestion(int portal_id, int arrival_timestep) {
        // get congestion value at the time bucket of arrival

        int time_bucket = arrival_timestep / TIME_BUCKET;

        if (time_bucket >= oldest_time_bucket_ + NUM_TIME_BUCKETS) {
            return 0.0f; // no congestion data
        }

        int bucket_index = portal_id * NUM_TIME_BUCKETS + (time_bucket % NUM_TIME_BUCKETS);
        return portal_congestion_[bucket_index];
    }

    float CongestionTracker::getClusterCongestion(int cluster_id, int arrival_timestep, int departure_timestep) {
        // get average congestion value over the time buckets from arrival to departure

        int time_bucket_start = arrival_timestep / TIME_BUCKET;
        int time_bucket_end = departure_timestep / TIME_BUCKET;

        if (time_bucket_end < oldest_time_bucket_ || time_bucket_start >= oldest_time_bucket_ + NUM_TIME_BUCKETS) {
            return 0.0f; // no congestion data
        }

        time_bucket_start = std::max(time_bucket_start, oldest_time_bucket_);
        time_bucket_end = std::min(time_bucket_end, oldest_time_bucket_ + NUM_TIME_BUCKETS - 1);

        float total_congestion = 0.0f;
        int count = 0;

        for (int tb = time_bucket_start; tb <= time_bucket_end; ++tb) {
            int bucket_index = cluster_id * NUM_TIME_BUCKETS + (tb % NUM_TIME_BUCKETS);
            total_congestion += cluster_congestion_[bucket_index];
            count++;
        }

        if (count == 0) return 0.0f;

        return total_congestion / static_cast<float>(count);
    }

    ReservationTable::ReservationTable() {
        table_.clear();
        edge_table_.clear();

        window_size_ = PLANNING_HORIZON;
        map_base_.clear();

        soft_reservations_.clear();

        proj_east_.clear();
        proj_south_.clear();
        proj_west_.clear();
        proj_north_.clear();
    }
    
    void ReservationTable::initialize(int window_size, std::vector<int> map_base, int rows, int cols) {
        window_size_ = window_size;
        map_base_ = map_base;
        rows_ = rows;
        cols_ = cols;

        table_.resize(window_size_ * rows_ * cols_, 0x00);

        for (int w = 0; w < window_size_; w++) {
            for (size_t i = 0; i < rows_ * cols_; i++) {
                if (map_base_[i] == -1) { // obstacle
                    table_[w * rows_ * cols_ + i] = OBS_STATIC; // hard reserved
                }
            }
        }

        edge_table_.resize(window_size_ * rows_ * cols_, 0x00);

        soft_reservations_.resize(window_size_ * rows_ * cols_, -1);

        for (int w = 0; w < window_size_; w++) {
            for (size_t i = 0; i < rows_ * cols_; i++) {
                if (map_base_[i] != -1) { // not obstacle
                    soft_reservations_[w * rows_ * cols_ + i] = 0;
                }
            }
        }
        
        proj_east_.resize(window_size_ * rows_ * cols_, -1);

        for (int w = 0; w < window_size_; w++) {
            for (size_t i = 0; i < rows_ * cols_; i++) {
                if (map_base_[i] != -1) { // not obstacle
                    proj_east_[w * rows_ * cols_ + i] = 0;
                }
            }
        }

        proj_south_.resize(window_size_ * rows_ * cols_, -1);

        for (int w = 0; w < window_size_; w++) {
            for (size_t i = 0; i < rows_ * cols_; i++) {
                if (map_base_[i] != -1) { // not obstacle
                    proj_south_[w * rows_ * cols_ + i] = 0;
                }
            }
        }

        proj_west_.resize(window_size_ * rows_ * cols_, -1);

        for (int w = 0; w < window_size_; w++) {
            for (size_t i = 0; i < rows_ * cols_; i++) {
                if (map_base_[i] != -1) { // not obstacle
                    proj_west_[w * rows_ * cols_ + i] = 0;
                }
            }
        }

        proj_north_.resize(window_size_ * rows_ * cols_, -1);

        for (int w = 0; w < window_size_; w++) {
            for (size_t i = 0; i < rows_ * cols_; i++) {
                if (map_base_[i] != -1) { // not obstacle
                    proj_north_[w * rows_ * cols_ + i] = 0;
                }
            }
        }
    }

    bool ReservationTable::isCellFree(int location, int timestep) {
        int index = (timestep % window_size_) * rows_ * cols_ + location;
        // free if no static or hard reservation
        return (table_[index] & (OBS_STATIC | RES_HARD)) == 0;
    }

    bool ReservationTable::isCellSoftReserved(int location, int timestep) {
        int index = (timestep % window_size_) * rows_ * cols_ + location;
        return (table_[index] & RES_SOFT) != 0;
    }

    Projections ReservationTable::getProjections(int location, int timestep) {
        int index = (timestep % window_size_) * rows_ * cols_ + location;
        Projections projections;
        projections.projected_east = (table_[index] & DIR_EAST) != 0;
        projections.projected_south = (table_[index] & DIR_SOUTH) != 0;
        projections.projected_west = (table_[index] & DIR_WEST) != 0;
        projections.projected_north = (table_[index] & DIR_NORTH) != 0;
        return projections;
    }

    void ReservationTable::reservePath(std::vector<LowLevelStep>& path) {
        // not validating path correctness nor if it surpasses window size, just reserve
        // assuming valid path is given

        for (int i = 0; i < static_cast<int>(path.size()); i++) {
            const LowLevelStep& step = path[i];
            int index = (step.t % window_size_) * rows_ * cols_ + step.location;

            if (step.placeholder_step) {
                table_[index] |= RES_SOFT;
                soft_reservations_[index]++;
                continue;
            }

            table_[index] |= RES_HARD;

            // update edge reservations if needed
            if (path[i].nextAction == Action::FW && i + 1 < static_cast<int>(path.size())) {
                const LowLevelStep& next_step = path[i + 1];
                int next_index = (next_step.t % window_size_) * rows_ * cols_ + next_step.location;

                int direction = next_step.location - step.location;
                if (direction == 1) {
                    // east
                    edge_table_[index] |= MOVE_EAST;
                } else if (direction == -1) {
                    // west
                    edge_table_[index] |= MOVE_WEST;
                } else if (direction == cols_) {
                    // south
                    edge_table_[index] |= MOVE_SOUTH;
                } else if (direction == -cols_) {
                    // north
                    edge_table_[index] |= MOVE_NORTH;
                }
            }

            // update projections for timestep t
            // base case: project the next move
            // advanced case: based on static geometry, if projectable further (map_base_[idx] == 1), project further

            int j = i + 1;
            bool can_project = true;

            while (can_project && j < static_cast<int>(path.size())) {
                const LowLevelStep& next_step = path[j];
                // int next_index = (step.t % window_size_) * table_.size() + next_step.location;     // intentional seg fault
                int next_index = (step.t % window_size_) * rows_ * cols_ + next_step.location;

                if (next_step.placeholder_step) {
                    can_project = false;
                    continue;
                }

                if (next_step.location == step.location) {
                    // same location, no projection
                    can_project = false;
                    continue;
                }

                // determine move direction
                int direction = next_step.location - step.location;
                if (direction == 1) {
                    // east
                    proj_east_[next_index]++;
                    table_[next_index] |= DIR_EAST;
                } else if (direction == -1) {
                    // west
                    proj_west_[next_index]++;
                    table_[next_index] |= DIR_WEST;
                } else if (direction == cols_) {
                    // south
                    proj_south_[next_index]++;
                    table_[next_index] |= DIR_SOUTH;
                } else if (direction == -cols_) {
                    // north
                    proj_north_[next_index]++;
                    table_[next_index] |= DIR_NORTH;
                } else {
                    // non-adjacent move, stop projecting
                    can_project = false;
                }

                if (map_base_[next_step.location] != 1) {
                    // cannot project further based on static geometry
                    can_project = false;
                }

                j++;
            }
        }
    }

    void ReservationTable::releasePath(const std::vector<LowLevelStep>& path) {
        // not validating path correctness nor if it surpasses window size, just release
        // assuming valid path is given
        for (int i = 0; i < static_cast<int>(path.size()); i++) {
            const LowLevelStep& step = path[i];
            int index = (step.t % window_size_) * rows_ * cols_ + step.location;

            if (step.placeholder_step) {
                if (soft_reservations_[index] > 0) {
                    soft_reservations_[index]--;
                }
                if (soft_reservations_[index] == 0) {
                    table_[index] &= ~RES_SOFT;
                }
            } else {
                table_[index] &= ~RES_HARD;
            }

            // remove edge reservations
            if (step.nextAction == Action::FW && i + 1 < static_cast<int>(path.size())) {
                const LowLevelStep& next_step = path[i + 1];
                int next_index = (next_step.t % window_size_) * rows_ * cols_ + next_step.location;

                int direction = next_step.location - step.location;
                if (direction == 1) {
                    // east
                    edge_table_[index] &= ~MOVE_EAST;
                } else if (direction == -1) {
                    // west
                    edge_table_[index] &= ~MOVE_WEST;
                } else if (direction == cols_) {
                    // south
                    edge_table_[index] &= ~MOVE_SOUTH;
                } else if (direction == -cols_) {
                    // north
                    edge_table_[index] &= ~MOVE_NORTH;
                }
            }

            // using the projection logic in reservePath to unproject
            int j = i + 1;
            bool can_unproject = true;

            while (can_unproject && j < static_cast<int>(path.size())) {
                const LowLevelStep& next_step = path[j];
                int next_index = (step.t % window_size_) * rows_ * cols_ + next_step.location;

                if (next_step.placeholder_step) {
                    can_unproject = false;
                    continue;
                }

                if (next_step.location == step.location) {
                    // same location, no projection
                    can_unproject = false;
                    continue;
                }

                // determine move direction
                int direction = next_step.location - step.location;
                if (direction == 1) {
                    // east
                    if (proj_east_[next_index] > 0) {
                        proj_east_[next_index]--;
                    }
                    if (proj_east_[next_index] == 0) {
                        table_[next_index] &= ~DIR_EAST;
                    }
                } else if (direction == -1) {
                    // west
                    if (proj_west_[next_index] > 0) {
                        proj_west_[next_index]--;
                    }
                    if (proj_west_[next_index] == 0) {
                        table_[next_index] &= ~DIR_WEST;
                    }
                } else if (direction == cols_) {
                    // south
                    if (proj_south_[next_index] > 0) {
                        proj_south_[next_index]--;
                    }
                    if (proj_south_[next_index] == 0) {
                        table_[next_index] &= ~DIR_SOUTH;
                    }
                } else if (direction == -cols_) {
                    // north
                    if (proj_north_[next_index] > 0) {
                        proj_north_[next_index]--;
                    }
                    if (proj_north_[next_index] == 0) {
                        table_[next_index] &= ~DIR_NORTH;
                    }
                } else {
                    // non-adjacent move, stop unprojecting
                    can_unproject = false;
                }

                if (map_base_[next_step.location] != 1) {
                    // cannot unproject further based on static geometry
                    can_unproject = false;
                }

                j++;
            }
        }
    }

    void ReservationTable::extendPathReservation(const LowLevelStep& extension_step) {
        // extending reservation with placeholder step

        int index = (extension_step.t % window_size_) * rows_ * cols_ + extension_step.location;

        table_[index] |= RES_SOFT;
        soft_reservations_[index]++;
    }

    void ReservationTable::advanceTable(int current_time) {
        // advance the reservation table by one time step
        if (current_time <= 0) return;

        int entry_to_switch = (current_time - 1) % window_size_;

        for (int i = 0; i < static_cast<int>(rows_ * cols_); i++) {
            int index = entry_to_switch * rows_ * cols_ + i;
            table_[index] = 0x00;
            edge_table_[index] = 0x00;

            if (map_base_[i] == -1) { // obstacle
                table_[index] = OBS_STATIC; // hard reserved
            }

            soft_reservations_[index] = 0;

            proj_east_[index] = 0;
            proj_south_[index] = 0;
            proj_west_[index] = 0;
            proj_north_[index] = 0;
        }
    }

    uint8_t ReservationTable::getEdgeReservations(int location, int timestep) {
        int index = (timestep % window_size_) * rows_ * cols_ + location;
        return edge_table_[index];
    }

    void ReservationTable::printTablesAtTimestep(int timestep) {
        // print reservation table first
        int index_offset = (timestep % window_size_) * rows_ * cols_;
        for (int i = 0; i < rows_ * cols_; i++) {
            if (table_[index_offset + i] & OBS_STATIC) {
                std::cout << "X";
            } else if (table_[index_offset + i] & RES_HARD) {
                std::cout << "A";
            } else if (table_[index_offset + i] & RES_SOFT) {
                std::cout << "S";
            } else {
                std::cout << ".";
            }
            if ((i + 1) % cols_ == 0 && i != 0) {
                std::cout << "\n";
            }
        }

        std::cout << "----\n";

        // print edge reservation table
        for (int i = 0; i < rows_ * cols_; i++) {
            uint8_t edge_res = edge_table_[index_offset + i];
            if (table_[index_offset + i] & OBS_STATIC) {
                std::cout << "X";
            } else if (edge_res & MOVE_NORTH) {
                std::cout << "A";
            } else if (edge_res & MOVE_SOUTH) {
                std::cout << "v";
            } else if (edge_res & MOVE_EAST) {
                std::cout << ">";
            } else if (edge_res & MOVE_WEST) {
                std::cout << "<";
            } else {
                std::cout << ".";
            }
            if ((i + 1) % cols_ == 0 && i != 0) {
                std::cout << "\n";
            }
        }

        std::cout << "====\n";

        // print projections
        // since multiple projections can exist at the same cell, we use codes:
        // . = none
        // E = east only
        // S = south only
        // W = west only
        // N = north only
        // M = multiple directions
        // X = obstacle
        for (int i = 0; i < rows_ * cols_; i++) {
            bool east = proj_east_[index_offset + i] > 0;
            bool south = proj_south_[index_offset + i] > 0;
            bool west = proj_west_[index_offset + i] > 0;
            bool north = proj_north_[index_offset + i] > 0;

            int count = static_cast<int>(east) + static_cast<int>(south) + static_cast<int>(west) + static_cast<int>(north);

            if (proj_east_[index_offset + i] == -1) {
                std::cout << "X";
            } else if (count == 0) {
                std::cout << ".";
            } else if (count > 1) {
                std::cout << "M";
            } else {
                if (east) {
                    std::cout << "E";
                } else if (south) {
                    std::cout << "S";
                } else if (west) {
                    std::cout << "W";
                } else if (north) {
                    std::cout << "N";
                } 
            }

            if ((i + 1) % cols_ == 0 && i != 0) {
                std::cout << "\n";
            }
        }

        std::cout << "====\n";
    }

    void DynamicEnvironment::initialize(SharedEnvironment* shared_env) {
        if (initialized_) {
            std::cout << "Dynamic Environment already initialized. Skipping re-initialization." << std::endl;
            return;
        }

        std::cout << "Initializing Dynamic Environment..." << std::endl;

        shared_env_ = shared_env;

        global_next_priority_ = 0;

        current_simulation_time_ = shared_env_->curr_timestep;

        int a = 0;

        for (int i = 0; i < shared_env_->num_of_agents; i++) {
            agents_.emplace_back(Agent(i));
        }

        free_agents_.clear();
        task_pool_.clear();

        PreprocessingPipeline::PreprocessPipeline::getInstance().initialize(shared_env_->map, shared_env_->rows, shared_env_->cols);
        PreprocessingPipeline::PreprocessPipeline::getInstance().runPreprocessing();

        reservation_table_.initialize(PLANNING_HORIZON, PreprocessingPipeline::PreprocessPipeline::getInstance().getProjectionBase(), shared_env_->rows, shared_env_->cols);

        int num_portals = PreprocessingPipeline::PreprocessPipeline::getInstance().getPortals().size();
        int num_clusters = PreprocessingPipeline::PreprocessPipeline::getInstance().getClusters().size();

        congestion_tracker_.initialize(num_portals, num_clusters);

        initialized_ = true;
        std::cout << "Dynamic Environment Initialization Complete." << std::endl;
    }

    DynamicEnvironment::DynamicEnvironment() {
        initialized_ = false;
        initialized_live_data_ = false;
        
        global_next_priority_ = 0;

        current_simulation_time_ = -1;

        shared_env_ = nullptr;

        agents_.clear();
        free_agents_.clear();
        task_pool_.clear();
    }

    void DynamicEnvironment::initializeLiveData() {
        if (initialized_live_data_) return;

        std::cout << "Initializing Dynamic Environment Live Data at Timestep 0..." << std::endl;

        if (shared_env_->curr_timestep != 0) return; // live data can only be initialized at timestep 0

        for (int i = 0; i < shared_env_->num_of_agents; i++) {
            // first initialize agent current states
            // make placeholder low-level plans for each agent to synchronize with reservation table
            agents_[i].setCurrentLocation(shared_env_->curr_states[i].location);
            agents_[i].setCurrentOrientation(shared_env_->curr_states[i].orientation);
            LowLevelStep first_step(0, shared_env_->curr_states[i].location, shared_env_->curr_states[i].orientation, Action::NA, false, -1);
            std::vector<LowLevelStep> initial_plan;
            initial_plan.emplace_back(first_step);
            for (int t = 1; t < PLANNING_HORIZON; t++) {
                LowLevelStep placeholder_step(t, shared_env_->curr_states[i].location, shared_env_->curr_states[i].orientation, Action::NA, true, -1);
                initial_plan.emplace_back(placeholder_step);
            }
            agents_[i].setLowLevelPlan(initial_plan);
            agents_[i].setHighLevelReplanNeeded(true);
            agents_[i].setLowLevelReplanNeeded(true);

            reservation_table_.reservePath(initial_plan);
        }

        current_simulation_time_ = 0;

        initialized_live_data_ = true;
    }

    bool DynamicEnvironment::advanceTimeStep() {
        // advance internal simulation time
        // confirm states with shared environment
        // if all is as expected, good
        // if agents did not move as expected, trigger replanning for them
        
        if (shared_env_->curr_timestep == current_simulation_time_) return true; // no error, just no advancement

        if (shared_env_->curr_timestep != current_simulation_time_ + 1) {
            std::cout << "Error: DynamicEnvironment internal time " << current_simulation_time_ 
                      << " out of sync with SharedEnvironment time " << shared_env_->curr_timestep << std::endl;
            return false; // error: internal time very off
        }

        current_simulation_time_ = shared_env_->curr_timestep;

        reservation_table_.advanceTable(current_simulation_time_);
        
        for (int i = 0; i < shared_env_->num_of_agents; i++) {
            // move agents and check if agent moved as expected
            agents_[i].executeStep();

            std::pair<bool, LowLevelStep> current_step = agents_[i].getCurrentLowLevelStep();
            if (current_step.first == false) {
                // no last step, should not happen even with placeholder plans
                std::cout << "Error: Agent " << i << " has no valid last step at time " << current_simulation_time_ << std::endl;
                return false;
            }

            if (current_step.second.placeholder_step ||
                current_step.second.location != shared_env_->curr_states[i].location ||
                current_step.second.orientation != shared_env_->curr_states[i].orientation) {
                // agent did not move as expected, trigger replanning
                agents_[i].setHighLevelReplanNeeded(true);
                agents_[i].setLowLevelReplanNeeded(true);

                std::cout << "Warning: Agent " << i << " deviated from expected state at time " << current_simulation_time_ << std::endl;

                // also release current reservations as they are now invalid
                std::vector<LowLevelStep> invalid_path;
                const std::vector<LowLevelStep>& full_path = agents_[i].getLowLevelPlan();
                int ll_index = agents_[i].getLLStepIndex();
                for (int t = ll_index; t < static_cast<int>(full_path.size()); t++) {
                    invalid_path.emplace_back(full_path[t]);
                }
                reservation_table_.releasePath(invalid_path);

                // create new placeholder plan to resynchronize
                LowLevelStep first_step(current_simulation_time_, shared_env_->curr_states[i].location, shared_env_->curr_states[i].orientation, Action::NA, false, -1);
                std::vector<LowLevelStep> placeholder_plan;
                placeholder_plan.emplace_back(first_step);
                
                for (int t = 1; t < PLANNING_HORIZON; t++) {
                    LowLevelStep placeholder_step(current_simulation_time_ + t, shared_env_->curr_states[i].location, shared_env_->curr_states[i].orientation, Action::NA, true, -1);
                    placeholder_plan.emplace_back(placeholder_step);
                }

                agents_[i].setLowLevelPlan(placeholder_plan);
                reservation_table_.reservePath(placeholder_plan);
            }
            else {
                // agent moved as expected, just extend reservation at the end to synchronize with reservation table
                LowLevelStep last_step = agents_[i].getLowLevelPlan().back();
                std::vector<LowLevelStep> extension_plan;
                LowLevelStep placeholder_step(current_simulation_time_ + PLANNING_HORIZON - 1, last_step.location, last_step.orientation, Action::NA, true, -1);
                extension_plan.emplace_back(placeholder_step);
                agents_[i].extendLowLevelPlan(extension_plan, true);
                reservation_table_.extendPathReservation(placeholder_step);
            }
        }

        congestion_tracker_.pruneAndReuse(current_simulation_time_);
        congestion_tracker_.decay();

        return true;
    }
}