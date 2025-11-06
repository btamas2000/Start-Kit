#pragma once
#include <vector>
#include <unordered_map>

namespace AuctionScheduler
{
    struct TaskBid {
        int agent_id;
        int task_id;
        double cost;
        
        bool operator>(const TaskBid& other) const {
            return cost > other.cost;
        }
    };
    
    // Helper function to calculate cost for an agent to execute a task
    double calculateTaskCost(int agent_id, int task_id, 
                            const std::vector<int>& current_schedule,
                            SharedEnvironment* env);
    
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

    void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
} // namespace AuctionScheduler