#pragma once

namespace AuctionScheduler
{
    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

    void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
} // namespace AuctionScheduler