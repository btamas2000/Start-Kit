#pragma once
#include "SharedEnv.h"

namespace HungarianScheduler2
{
    void hun_schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);
    void hun_schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

} // namespace HungarianScheduler2
