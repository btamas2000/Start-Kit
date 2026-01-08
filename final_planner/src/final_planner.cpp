#include "final_planner.h"

void MyPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    // Initialize preprocessing
    auto& pre = PreprocessingPipeline::PreprocessPipeline::getInstance();
    if (!pre.isInitialized()) {
        pre.initialize(env->map, env->rows, env->cols);
        pre.runPreprocessing(preprocess_time_limit);
    }
}

void MyPlanner::plan(int time_limit, std::vector<Action> & actions,  SharedEnvironment* env) {
    // Placeholder for planning logic
    // This function should populate 'actions' based on the current state in 'env'
    
    actions.clear();

    actions.resize(env->num_of_agents, Action::WAIT);

    return;
}