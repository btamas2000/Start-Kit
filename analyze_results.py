#!/usr/bin/env python3
"""
Analyze and compare scheduling algorithm results
"""

import json
import sys

def load_results(filename):
    """Load JSON results file"""
    with open(filename, 'r') as f:
        return json.load(f)

def analyze_scheduling_quality(data, name):
    """Analyze the scheduling quality from result data"""
    print(f"\n{'='*60}")
    print(f"Analysis for: {name}")
    print(f"{'='*60}")
    
    # Basic metrics
    print(f"\n📊 OVERALL METRICS:")
    print(f"  Tasks Finished:     {data['numTaskFinished']}")
    print(f"  Deadlines Met:      {data['deadlineMet']}")
    print(f"  Deadlines Failed:   {data['deadlineFailed']}")
    print(f"  Makespan:           {data['makespan']}")
    
    # Calculate rates
    total_tasks = data['deadlineMet'] + data['deadlineFailed']
    deadline_rate = (data['deadlineMet'] / total_tasks * 100) if total_tasks > 0 else 0
    print(f"  Deadline Success:   {deadline_rate:.1f}%")
    
    # Analyze task completion timing
    tasks = data.get('tasks', [])
    print(f"\n📋 TASK ANALYSIS:")
    print(f"  Total Tasks:        {len(tasks)}")
    
    # Analyze slack time (deadline - completion time)
    deadline_slacks = []
    missed_by = []
    
    for task in tasks:
        if 't_completed' in task and task['t_completed'] > 0:
            deadline = task.get('t_deadline', -1)
            revealed = task.get('t_revealed', 0)
            completed = task['t_completed']
            
            if deadline > 0:
                absolute_deadline = revealed + deadline
                slack = absolute_deadline - completed
                
                if slack >= 0:
                    deadline_slacks.append(slack)
                else:
                    missed_by.append(-slack)
    
    if deadline_slacks:
        print(f"\n  Tasks meeting deadline: {len(deadline_slacks)}")
        print(f"    Avg slack time:   {sum(deadline_slacks)/len(deadline_slacks):.1f} timesteps")
        print(f"    Min slack:        {min(deadline_slacks)} timesteps")
        print(f"    Max slack:        {max(deadline_slacks)} timesteps")
    
    if missed_by:
        print(f"\n  Tasks missing deadline: {len(missed_by)}")
        print(f"    Avg missed by:    {sum(missed_by)/len(missed_by):.1f} timesteps")
        print(f"    Min missed by:    {min(missed_by)} timesteps")
        print(f"    Max missed by:    {max(missed_by)} timesteps")
    
    # Analyze planner times
    planner_times = data.get('plannerTimes', [])
    if planner_times:
        avg_planner_time = sum(planner_times) / len(planner_times)
        max_planner_time = max(planner_times)
        print(f"\n⏱️  PLANNER PERFORMANCE:")
        print(f"  Avg planning time:  {avg_planner_time:.4f}s")
        print(f"  Max planning time:  {max_planner_time:.4f}s")
        print(f"  Total plans:        {len(planner_times)}")
    
    # Analyze agent utilization
    schedules = data.get('actualSchedule', [])
    if schedules:
        active_periods = []
        for schedule in schedules:
            if schedule and schedule != "-1":
                # Parse schedule like "1:73,40:105,84:161"
                periods = schedule.split(',')
                total_active = 0
                for period in periods:
                    if ':' in period:
                        start, end = map(int, period.split(':'))
                        total_active += (end - start)
                active_periods.append(total_active)
        
        if active_periods:
            avg_utilization = sum(active_periods) / len(active_periods)
            print(f"\n🤖 AGENT UTILIZATION:")
            print(f"  Avg active time:    {avg_utilization:.1f} timesteps per agent")
            print(f"  Utilization rate:   {avg_utilization/data['makespan']*100:.1f}%")
    
    return {
        'tasks_finished': data['numTaskFinished'],
        'deadlines_met': data['deadlineMet'],
        'deadlines_failed': data['deadlineFailed'],
        'deadline_rate': deadline_rate,
        'avg_slack': sum(deadline_slacks)/len(deadline_slacks) if deadline_slacks else 0,
        'avg_missed': sum(missed_by)/len(missed_by) if missed_by else 0,
    }

def compare_results(results1, results2, name1, name2):
    """Compare two result sets"""
    print(f"\n{'='*60}")
    print(f"COMPARISON: {name1} vs {name2}")
    print(f"{'='*60}")
    
    print(f"\n📈 DELTA (positive = {name1} is better):")
    print(f"  Tasks Finished:     {results1['tasks_finished'] - results2['tasks_finished']:+d}")
    print(f"  Deadlines Met:      {results1['deadlines_met'] - results2['deadlines_met']:+d}")
    print(f"  Deadline Rate:      {results1['deadline_rate'] - results2['deadline_rate']:+.1f}%")
    
    if results1['avg_slack'] > 0 and results2['avg_slack'] > 0:
        print(f"  Avg Slack (met):    {results1['avg_slack'] - results2['avg_slack']:+.1f} timesteps")
    
    if results1['avg_missed'] > 0 and results2['avg_missed'] > 0:
        print(f"  Avg Miss (failed):  {results2['avg_missed'] - results1['avg_missed']:+.1f} timesteps")
    
    print(f"\n💡 INSIGHTS:")
    
    task_diff = results1['tasks_finished'] - results2['tasks_finished']
    deadline_diff = results1['deadlines_met'] - results2['deadlines_met']
    
    if task_diff < 0:
        print(f"  ⚠️  {name2} completes {abs(task_diff)} MORE tasks")
    elif task_diff > 0:
        print(f"  ✅ {name1} completes {task_diff} MORE tasks")
    
    if deadline_diff < 0:
        print(f"  ⚠️  {name2} meets {abs(deadline_diff)} MORE deadlines")
        print(f"      This suggests {name2} has BETTER scheduling for deadline-critical tasks")
    elif deadline_diff > 0:
        print(f"  ✅ {name1} meets {deadline_diff} MORE deadlines")

if __name__ == "__main__":
    # Load both result files
    default_data = load_results('default60_01.json')
    hungarian_data = load_results('hungarian60_02.json')
    
    # Analyze each
    default_results = analyze_scheduling_quality(default_data, "Default Scheduler")
    hungarian_results = analyze_scheduling_quality(hungarian_data, "Hungarian Scheduler (with deadline awareness)")
    
    # Compare
    compare_results(default_results, hungarian_results, "Default", "Hungarian")
    
    print(f"\n{'='*60}")
    print("CONCLUSION:")
    print(f"{'='*60}")
    
    if hungarian_results['tasks_finished'] < default_results['tasks_finished']:
        print("\n⚠️  The Hungarian scheduler is completing FEWER tasks overall.")
        print("    Possible reasons:")
        print("    1. Spending too much time on optimal assignment computation")
        print("    2. Not adapting quickly to dynamic task arrivals")
        print("    3. Global optimization may be sub-optimal in dynamic settings")
    
    if hungarian_results['deadlines_met'] < default_results['deadlines_met']:
        print("\n⚠️  The Hungarian scheduler is meeting FEWER deadlines.")
        print("    This suggests the deadline-aware cost function needs tuning:")
        print("    - Increase DEADLINE_WEIGHT to prioritize urgent tasks more")
        print("    - Increase DEADLINE_CRITICAL_THRESHOLD to start worrying earlier")
        print("    - Consider if Hungarian algorithm overhead is causing delays")
    else:
        print("\n✅ The Hungarian scheduler IS meeting more deadlines!")
        print("    The deadline-aware improvements are working!")
