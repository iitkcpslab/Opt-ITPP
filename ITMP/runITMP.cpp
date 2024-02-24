#include "src/IntegratedPlanner.cpp"
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;

using namespace std;
template<typename T, typename Y>
auto operator<<(ostream& stream, pair<T,Y> const& p) -> ostream& {
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
    ("config,c", po::value<string>()->required(), "Input inputFile")
	("opt,o", po::value<int>()->required(), "Optimzation Criteria")
	("numZ,z", po::value<int>()->default_value(0), "Z number")
	("result,r", po::value<string>()->default_value(""), "Output File")
	("timeout,t", po::value<int>()->default_value(1800), "Time out")
	("screen,s", po::value<int>()->default_value(0), "screen for printing information");
    po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

	// -------------------------------------------------------------------------------
	// -------------------------------------------------------------------------------
	cout << "Start :: " << vm["config"].as<string>().c_str() << endl;
	ifstream inputFile(vm["config"].as<string>());
	if (!inputFile.good()) {
		cerr << "Input File < " << vm["config"].as<string>().c_str() << " > not Found\n";
	}
    json data;
    inputFile >> data;
    // Access the values in the JSON object
    vector<vector<int>> world_descriptor = data["world_descriptor"];
    vector<vector<int>> robots = data["robots"];
	auto temp_tasks	= data["tasks"];
	vector<Task> tasks;
    for (const auto& task : temp_tasks) {
        tasks.push_back( Task( Cell(task[0][0], task[0][1]), 
			Cell(task[1][0], task[1][1]), task[2], task[3] ) );
    }
	vector<Cell> init_pos;
	for(const auto &robot: robots) {
		init_pos.push_back(Cell(robot[0], robot[1]));
	}
    vector<vector<int>> intermediatePositions = data["intermediate_positions"];
	vector<Cell> intermediate_pos;
	for(const auto &ipos: intermediatePositions) {
		intermediate_pos.push_back(Cell(ipos[0], ipos[1]));
	}
    vector<int> capacities	= data["capacities"];
    // -------------------------------------------------------------------------------
	// -------------------------------------------------------------------------------

	// Dump an empty inputFile incase the program hits memory limit.
	if(vm["result"].as<string>() != "") {
		// Convert the Solution structure to json
		json jsonSolution;
		jsonSolution["status"] 		= 0;
		// Dump the json to a inputFile
		ofstream outputFile(vm["result"].as<string>());
		if (outputFile.is_open()) {
			outputFile << jsonSolution.dump(4);  // Use dump(4) for pretty formatting with indentation of 4 spaces
			outputFile.close();
		}
		outputFile.close();
	}

	Configuration config(
		world_descriptor, 
		init_pos, 
		tasks, 
		intermediate_pos, 
		capacities, 
		vm["screen"].as<int>()
	);	
	IntegratedPlanner planner(&config, vm["result"].as<string>(), vm["screen"].as<int>());
	Solution finalSolution;
	if(vm["numZ"].as<int>() != 0) {
		planner.execute_singleZ(
			finalSolution,
			vm["opt"].as<int>(), 
			vm["numZ"].as<int>(), 
			vm["timeout"].as<int>()
		);
	} else {
		planner.execute_minZ(
			finalSolution,
			vm["opt"].as<int>(),
			vm["timeout"].as<int>()
		);
	}

	if(vm["screen"].as<int>() > 0) {
		cout << "\n=========================================================\n";
		cout << "FinalSolution Status :: " << finalSolution.status << "\n";
		cout << "FinalSolution total_time :: " << finalSolution.total_time << "\n";
		cout << "FinalSolution makespan :: " << finalSolution.makespan << "\n";
		cout << "FinalSolution total_cost :: " << finalSolution.total_cost << "\n";
		
		cout << "FinalPath : ";
		for(auto row : finalSolution.paths) {
			cout << "\nSTART :: \n";
			int counter = 0;
			for(auto ele : row) {
				cout << "[" << ele[0] << ", " << ele[1] << "]@" << counter << " --> ";
				if(++counter %4 == 0)
					cout << endl;
			}
			cout << endl;
		}
		cout << endl;
	} 
	// Dump the solution with success status.
	if(vm["result"].as<string>() != "") {
		// Convert the Solution structure to json
		json jsonSolution;
		jsonSolution["status"] 		= 1;
		jsonSolution["goal_plans"]	= finalSolution.plans.goal_locations;
		jsonSolution["temp_cons"]	= finalSolution.plans.temp_constraints;
		jsonSolution["paths"] 		= finalSolution.paths;
		jsonSolution["makespan"] 	= finalSolution.makespan;
		jsonSolution["total_cost"] 	= finalSolution.total_cost;
		jsonSolution["total_time"] 	= finalSolution.total_time;

		// Dump the json to a inputFile
		ofstream outputFile(vm["result"].as<string>());
		if (outputFile.is_open()) {
			outputFile << jsonSolution.dump(4);  // Use dump(4) for pretty formatting with indentation of 4 spaces
			outputFile.close();
		}
		outputFile.close();
	}
	return 0;
}