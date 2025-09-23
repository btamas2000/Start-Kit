#pragma once
#include "common.h"

// i have to modify this so deadlines are a part of the task
// proposal: add an int deadline to the task struct
struct Task
{
    int task_id;
    int t_completed = -1;
    int t_revealed = -1;
    int agent_assigned = -1;
    int deadline = -1;

    vector<int> locations;
    int idx_next_loc = 0;




    int get_next_loc()
    {
        if (idx_next_loc < locations.size())
        {
            return locations.at(idx_next_loc);
        } 
        else 
        {
            assert(false);
            return -1;
        }
    }

    bool is_finished()
    {
        return idx_next_loc == locations.size();
    }

    // kept if t_revealed + deadline <= t_completed
    bool deadline_kept()
    {
        if (deadline < 0) // no deadline
            return true;
        return t_revealed + deadline >= t_completed;
    }

    //Task(int task_id, int location): task_id(task_id), locations({location}) {};
    Task(int task_id, list<int> location, int t_revealed, int deadline): task_id(task_id), t_revealed(t_revealed)
    {
        if (deadline > 0)
            this->deadline = deadline;
        else
            this->deadline = -1; // no deadline
        for (auto loc: location)
            locations.push_back(loc);
    };

    Task(){};

    Task(Task* other)
    {
        task_id = other->task_id;
        t_completed = other->t_completed;
        locations = other->locations;
        t_revealed = other->t_revealed;
        idx_next_loc = other->idx_next_loc;
        agent_assigned = other->agent_assigned;
    };

    Task(const Task& other)
    {
        task_id = other.task_id;
        t_completed = other.t_completed;
        locations = other.locations;
        t_revealed = other.t_revealed;
        idx_next_loc = other.idx_next_loc;
        agent_assigned = other.agent_assigned;
    };
};
