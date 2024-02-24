#include "TaskPlanner.cpp"
#include "CBS.h"
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;

struct Solution
{
    int status;    // 0 - fail, 1 - success, 2 - timeout
    PathPlannerConfig plans;
    vector<vector<vector<int>>> paths;
    int makespan;
    int total_cost;
    float total_time;
};

class IntegratedPlanner
{
private:
    Configuration *config;
    int screen;
    const string &jsonOutLoc;

    unsigned int get_time()
    {
        auto clock_time = chrono::high_resolution_clock::now();
        unsigned int time = chrono::duration_cast<chrono::seconds>(clock_time.time_since_epoch()).count();
        return time;
    }

    float round(float input)
    {
        return (floor(input * 100.0) / 100.0);
    }

    bool PathPlanner(
        vector<vector<int>> &world_descriptor, 
        PathPlannerConfig &plan, 
        ResultPaths &res, 
        int optimizer, 
        int screen, 
        int timeout
    )
    {
        heuristics_type heu_type = heuristics_type::ZERO;
        conflict_selection conflict = conflict_selection::RANDOM;
        node_selection node_sel = node_selection::NODE_RANDOM;

        Instance instance(
            world_descriptor,
            plan.start_locations,
            plan.goal_locations,
            plan.deadlines,
            plan.temp_constraints,
            screen);
        CBS cbs(instance, false, heu_type, screen, optimizer);

        // Set Parameteres Options for CBS. Using default for all
        cbs.setConflictSelectionRule(conflict);
        cbs.setDisjointSplitting(false);
        cbs.setBypass(false);
        cbs.setTargetReasoning(false);
        cbs.setNodeSelectionRule(node_sel);
        // Functions below are not supported right now by CBS
        cbs.setPrioritizeConflicts(false);
        cbs.setRectangleReasoning(false);
        cbs.setCorridorReasoning(false);
        cbs.setMutexReasoning(mutex_strategy::N_MUTEX);

        cbs.clear();
        cbs.solve(timeout, res, 0);
        if (cbs.solution_found) {
            if(screen) 
                cout << "Collison Free Solution Found\n";
            return true;
        }
        if(screen) 
            cout << "No solution found\n";
        return false;
    }

public:
    ResultPaths finalSolution;

    IntegratedPlanner(Configuration *config, const string &jsonOutLoc = string(), int screen = 0) :
        config(config), screen(screen), jsonOutLoc(jsonOutLoc) {}

    int execute_singleZ(Solution &finalSolution, int mode, int Z, int time_limit = 1800)
    {
        int start_time = this->get_time();
        int end_time = start_time + time_limit;
        int remaining_time;
        int status;

        finalSolution.paths.clear();
        finalSolution.status = 0;
        finalSolution.makespan = INT_MAX;
        finalSolution.total_cost = INT_MAX;
        finalSolution.total_time = 0;

        TaskPlanner task_planner(this->config, mode, Z, this->screen);
        task_planner.push();
        if (screen > 0)
        {
            cout << "############################################\n";
            cout << "START Integrated Task And Path Planner\n";
            cout << "Find a satisfiable solution\n";
        }

        PathPlannerConfig curr_plan;
        ResultPaths curr_paths;
        // Finding a satisfiable solution
        while (true)
        {
            remaining_time = ceil(end_time - this->get_time());
            status = task_planner.check_solver(remaining_time);
            if (!status) {
                finalSolution.total_time = this->round(this->get_time() - start_time);
                return 0;
            }

            task_planner.get_assignment(*(task_planner.curr_model), &curr_plan);
            task_planner.add_exclusion(*(task_planner.curr_model));
            if (screen > 0) {
                cout << "New Assignment Found with cost : " << curr_plan.opt_metric << endl;
            }

            remaining_time = ceil(end_time - this->get_time());
            status = this->PathPlanner(
                config->world_descriptor, curr_plan, curr_paths,
                mode, screen, remaining_time
            );
            if(status) {
                finalSolution.plans = curr_plan;
                finalSolution.paths = curr_paths.paths;
                finalSolution.makespan = curr_paths.makespan;
                finalSolution.total_cost = curr_paths.total_cost;
                finalSolution.status = 1;
                finalSolution.total_time = ceil(this->get_time() - start_time);

                // Added it so that an intermediate plan is saved in case of memory limit or time limit
                if(!jsonOutLoc.empty()) {
                    json jsonSolution;
                    jsonSolution["status"] 		= 3;
                    jsonSolution["goal_plans"]	= finalSolution.plans.goal_locations;
                    jsonSolution["temp_cons"]	= finalSolution.plans.temp_constraints;
                    jsonSolution["paths"] 		= finalSolution.paths;
                    jsonSolution["makespan"] 	= finalSolution.makespan;
                    jsonSolution["total_cost"] 	= finalSolution.total_cost;
                    jsonSolution["total_time"] 	= finalSolution.total_time;

                    // Dump the json to a file
                    ofstream outputFile(jsonOutLoc);
                    if (outputFile.is_open()) {
                        outputFile << jsonSolution.dump(4);  // Use dump(4) for pretty formatting with indentation of 4 spaces
                        outputFile.close();
                    }
                    outputFile.close();
                }
                break;
            }
        }
        if(screen > 0) {
            cout << "Satisfiable Solution Found\n"
                 << "Total Cost : " << finalSolution.total_cost << " :: "
                 << "Makespan : " << finalSolution.makespan << " :: "
                 << "Time Elpased : " << ceil(this->get_time() - start_time) << "\n";
            cout << "############################################\n\n";
        }

        int smt_estimate = this->config->get_lower_bound(mode);
        int min_path_len;
        if(mode == 1)
            min_path_len = finalSolution.makespan;
        else
            min_path_len = finalSolution.total_cost;

        // Find Optimal Solution
        while (smt_estimate < min_path_len) {
            remaining_time = ceil(end_time - this->get_time());
            if(remaining_time <= 0) {
                finalSolution.status = 2;
                finalSolution.total_time = ceil(this->get_time() - start_time);
                break;
            }

            if(screen > 0) {
                cout << "############################################\n";
                cout << "SMT Estimate/Lower Bound : " << smt_estimate 
                    << " Min Path Len/Upper Bound : " << min_path_len 
                    << " Time Remaining : " << remaining_time << endl;
            }

            status = task_planner.check_opt_solver(smt_estimate, min_path_len, remaining_time);
            if (!status) {
                if(screen > 0) { cout << "No more solution exist\n";}
                break;
            }

            task_planner.get_assignment(*(task_planner.curr_model), &curr_plan);
            task_planner.add_exclusion(*(task_planner.curr_model));
            int new_smt_estimate = curr_plan.opt_metric;
            if (screen > 0) {
                cout << "New Assignment Found with cost : " 
                    << smt_estimate << " " << new_smt_estimate << endl;
            }
            
            // If new cost is more than old cost, remove all previous exclusions
            if(smt_estimate < new_smt_estimate) {
                smt_estimate = new_smt_estimate;
                task_planner.pop();
                task_planner.push();
            }

            remaining_time = ceil(end_time - this->get_time());
            status = this->PathPlanner(
                config->world_descriptor, curr_plan, curr_paths,
                mode, screen, remaining_time
            );

            if(!status) {
                if(screen > 0) cout << "No path found for the current plan\n";
                continue;
            }
            if(mode == 1) {
                if(screen > 0){
                    cout << "Collision Free Path Found\n"
                        << " Old Cost : " << finalSolution.makespan 
                        << " New Cost : " << curr_paths.makespan 
                        << " TimeRemaining : " <<  ceil(end_time - this->get_time()) << endl;
                }
                if(min_path_len > curr_paths.makespan) {
                    finalSolution.plans = curr_plan;
                    finalSolution.paths = curr_paths.paths;
                    finalSolution.makespan = curr_paths.makespan;
                    finalSolution.total_cost = curr_paths.total_cost;
                    finalSolution.status = 1;
                    finalSolution.total_time = ceil(this->get_time() - start_time);
                    min_path_len = curr_paths.makespan;
                    // Added it so that an intermediate plan is saved in case of memory limit or time limit
                    if(!jsonOutLoc.empty()) {
                        json jsonSolution;
                        jsonSolution["status"] 		= 2;
                        jsonSolution["goal_plans"]	= finalSolution.plans.goal_locations;
                        jsonSolution["temp_cons"]	= finalSolution.plans.temp_constraints;
                        jsonSolution["paths"] 		= finalSolution.paths;
                        jsonSolution["makespan"] 	= finalSolution.makespan;
                        jsonSolution["total_cost"] 	= finalSolution.total_cost;
                        jsonSolution["total_time"] 	= finalSolution.total_time;

                        // Dump the json to a file
                        ofstream outputFile(jsonOutLoc);
                        if (outputFile.is_open()) {
                            outputFile << jsonSolution.dump(4);  // Use dump(4) for pretty formatting with indentation of 4 spaces
                            outputFile.close();
                        }
                        outputFile.close();
                    }
                }
            } else {
                if(screen > 0){
                    cout << "Collision Free Path Found\n"
                        << " Old Cost : " << finalSolution.total_cost 
                        << " New Cost : " << curr_paths.total_cost 
                        << " TimeRemaining : " <<  ceil(end_time - this->get_time()) << endl;
                }
                if(min_path_len > curr_paths.total_cost) {
                    finalSolution.plans = curr_plan;
                    finalSolution.paths = curr_paths.paths;
                    finalSolution.makespan = curr_paths.makespan;
                    finalSolution.total_cost = curr_paths.total_cost;
                    finalSolution.status = 1;
                    finalSolution.total_time = ceil(this->get_time() - start_time);
                    min_path_len = curr_paths.total_cost;
                    // Added it so that an intermediate plan is saved in case of memory limit or time limit
                    if(!jsonOutLoc.empty()) {
                        json jsonSolution;
                        jsonSolution["status"] 		= 2;
                        jsonSolution["goal_plans"]	= finalSolution.plans.goal_locations;
                        jsonSolution["temp_cons"]	= finalSolution.plans.temp_constraints;
                        jsonSolution["paths"] 		= finalSolution.paths;
                        jsonSolution["makespan"] 	= finalSolution.makespan;
                        jsonSolution["total_cost"] 	= finalSolution.total_cost;
                        jsonSolution["total_time"] 	= finalSolution.total_time;

                        // Dump the json to a file
                        ofstream outputFile(jsonOutLoc);
                        if (outputFile.is_open()) {
                            outputFile << jsonSolution.dump(4);  // Use dump(4) for pretty formatting with indentation of 4 spaces
                            outputFile.close();
                        }
                        outputFile.close();
                    }
                }
            }

            if(screen>0) {
                cout << "############################################\n\n";
            }
        }
        finalSolution.total_time = ceil(this->get_time() - start_time);
        return finalSolution.status;
    }

    int execute_minZ(Solution &finalSolution, int mode, int time_limit=1800) {
        if(mode != 1 && mode != 2) {
            cerr << "Wrong Optimization Mode Provided\n";
            exit(-1);
        }
        
        int minZ;
        if(config->num_tasks % config->num_init_pos) {
            minZ = max(1, 4 + 2 * (config->num_tasks / config->num_init_pos));
        } else {
            minZ = max(1, 2 + 2 * (config->num_tasks / config->num_init_pos));
        }
        int maxZ = (1 + 2 * config->num_tasks);

        int end_time = this->get_time() + time_limit;
        for(int Z=minZ; Z < maxZ + 1; Z++) {
            int remaining_time = end_time - this->get_time();
            if(screen > 0)
                cout << "Start for Z = " << Z << " Time Left : " << remaining_time << endl;
            this->execute_singleZ(finalSolution, mode, Z, remaining_time);
            if(finalSolution.status != 0) {
                return 1;
            }
        }
        return 0;
    }
};
