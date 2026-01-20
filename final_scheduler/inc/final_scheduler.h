#pragma once
#include "SharedEnv.h"
#include "dynamic.h"

namespace HungarianScheduler3 {
    void hun_schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);
    void hun_schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

} // namespace HungarianScheduler3