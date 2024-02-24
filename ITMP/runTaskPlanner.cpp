#include "src/TaskPlanner.cpp"
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;

using namespace std;
template <typename T, typename Y>
auto operator<<(ostream &stream, pair<T, Y> const &p) -> ostream &
{
	stream << "(" << p.first << ", " << p.second << ") ";
	return stream;
}

int main(int argc, char **argv)
{
	// -------------------------------------------------------------------------------
	namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
	("help", "produce help message")
	("config,c", po::value<string>()->required(), "Input file")
	("opt,o", po::value<int>()->required(), "Optimzation Criteria")
	("numZ,z", po::value<int>()->required(), "Z number")
	("result,r", po::value<string>()->default_value(""), "Output file")
	("timeout,t", po::value<int>()->default_value(1800), "Time out")
	("screen,s", po::value<int>()->default_value(0), "screen for printing information");
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	// -------------------------------------------------------------------------------
	// -------------------------------------------------------------------------------
	ifstream file(vm["config"].as<string>());
	json data;
	file >> data;
	// Access the values in the JSON object
	vector<vector<int>> world_descriptor = data["world_descriptor"];
	vector<vector<int>> robots = data["robots"];
	auto temp_tasks = data["tasks"];
	vector<Task> tasks;
	for (const auto &task : temp_tasks)
	{
		tasks.push_back(Task(
			Cell(task[0][0], task[0][1]),
			Cell(task[1][0], task[1][1]), 
			task[2], 
			task[3]
		));
	}
	vector<Cell> init_pos;
	for (const auto &robot : robots)
	{
		init_pos.push_back(Cell(robot[0], robot[1]));
	}
	vector<vector<int>> intermediatePositions = data["intermediate_positions"];
	vector<Cell> intermediate_pos;
	for (const auto &ipos : intermediatePositions)
	{
		intermediate_pos.push_back(Cell(ipos[0], ipos[1]));
	}
	vector<int> capacities = data["capacities"];
	// -------------------------------------------------------------------------------
	// -------------------------------------------------------------------------------

	// Dump an early empty inputFile incase the program hits memory limit.
	if (vm["result"].as<string>() != "")
	{
		// Convert the Solution structure to json
		json jsonSolution;
		jsonSolution["status"] 	= 0; // No Solution Found
		jsonSolution["time"] 	= 0;
		// Dump the json to a inputFile
		ofstream outputFile(vm["result"].as<string>());
		if (outputFile.is_open()) {
			outputFile << jsonSolution.dump(4); // Use dump(4) for pretty formatting with indentation of 4 spaces
		}
		outputFile.close();
	}
	// -------------------------------------------------------------------------------

	Configuration config(
		world_descriptor,
		init_pos,
		tasks,
		intermediate_pos,
		capacities,
		vm["screen"].as<int>());
	TaskPlanner tp(&config, vm["opt"].as<int>(), vm["numZ"].as<int>(), vm["screen"].as<int>());

	int timeout = vm["timeout"].as<int>();
	clock_t start_time = tp.get_time();

	PathPlannerConfig res;
	int lower_bound 	= config.get_lower_bound(vm["opt"].as<int>());
	int remaining_time 	= timeout - tp.get_elapsed_time(start_time);
	int existStatus	 	= tp.check_solver(remaining_time);
	int optimalStatus 	= 0;
	int upper_bound;
	if(vm["screen"].as<int>() > 0) {
		cout << "Satisfiable Solution Status :: " << existStatus << endl;
	}
	if (existStatus)
	{
		// Save Found Result
		if (vm["screen"].as<int>() > 0) {
			tp.print_model_info(*(tp.curr_model), "all");
		}
		tp.get_assignment(*(tp.curr_model), &res);
		upper_bound = tp.get_cost(*(tp.curr_model));

		if (vm["result"].as<string>() != "")
		{
			// Convert the Solution structure to json
			json jsonSolution;
			jsonSolution["status"] 					= 3; // Solution Exists But Not Optimal
			jsonSolution["plan"] 					= res.goal_locations;
			jsonSolution["cost"] 					= upper_bound;
			jsonSolution["optimizer"] 				= vm["opt"].as<int>();
			jsonSolution["temporal_constraints"] 	= res.temp_constraints;
			jsonSolution["time"] 					= tp.get_elapsed_time(start_time);
			// Dump the json to a inputFile
			ofstream outputFile(vm["result"].as<string>());
			if (outputFile.is_open()) {
				outputFile << jsonSolution.dump(4); // Use dump(4) for pretty formatting with indentation of 4 spaces
			}
			outputFile.close();
		}

		// Execute
		remaining_time 	= timeout - tp.get_elapsed_time(start_time);
		optimalStatus	= tp.check_opt_solver(lower_bound, upper_bound + 1, remaining_time);
		if(vm["screen"].as<int>() > 0) {
			cout << "Lower Bound & Upper Bound :: " << lower_bound << " " << upper_bound << endl;
			cout << "Optimal Solution Status :: " << optimalStatus << endl;
		}
		if (optimalStatus)
		{
			if (vm["screen"].as<int>() > 0)
				tp.print_model_info(*(tp.curr_model), "all");
			tp.get_assignment(*(tp.curr_model), &res);
			upper_bound = tp.get_cost(*(tp.curr_model));

			if (vm["result"].as<string>() != "")
			{
				// Convert the Solution structure to json
				json jsonSolution;
				jsonSolution["status"] 					= optimalStatus; // Solution Exists
				jsonSolution["plan"] 					= res.goal_locations;
				jsonSolution["cost"] 					= upper_bound;
				jsonSolution["optimizer"] 				= vm["opt"].as<int>();
				jsonSolution["temporal_constraints"] 	= res.temp_constraints;
				jsonSolution["time"] 					= tp.get_elapsed_time(start_time);
				// Dump the json to a inputFile
				ofstream outputFile(vm["result"].as<string>());
				if (outputFile.is_open()) {
					outputFile << jsonSolution.dump(4); // Use dump(4) for pretty formatting with indentation of 4 spaces
				}
				outputFile.close();
			}
		}
	}
	else
	{
		if(vm["screen"].as<int>() > 0) {
			cout << "No Solution Exists\n";
		}
		// No Solution Exists
		if (vm["result"].as<string>() != "")
		{
			// Convert the Solution structure to json
			json jsonSolution;
			jsonSolution["status"] 	= 0; // Solution Does Not Exists
			jsonSolution["time"] 	= tp.get_elapsed_time(start_time);
			// Dump the json to a inputFile
			ofstream outputFile(vm["result"].as<string>());
			if (outputFile.is_open())
			{
				outputFile << jsonSolution.dump(4); // Use dump(4) for pretty formatting with indentation of 4 spaces
				outputFile.close();
			}
			outputFile.close();
		}
	}

	// fstream logFile;
	// logFile.open("./cpp_op.smt", ios::trunc | ios::out);
	// logFile << tp.smt->to_smt2() << endl;
	// logFile.close();

	return 0;
}