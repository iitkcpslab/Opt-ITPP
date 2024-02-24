#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;

typedef struct cell{
    int x;
    int y;

    cell() : x(0), y(0) {}
    cell(int x, int y) : x(x), y(y) {}

    bool operator<(const cell &o) const 
    {
        return (this->x < o.x) || (this->x == o.x && this->y < o.y);
    }
} Cell;

typedef struct task{
    Cell start;
    Cell end;
    int deadline;
    int weight;
    task(Cell s, Cell e, int d, int w) :
        start(s), end(e), deadline(d), weight(w) {}
} Task;

int main(int argc, char **argv) {
    // Read the JSON string from a file
    namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()("help", "produce help message")
    ("config,c", po::value<string>()->required(), "input file for configuration");
    po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    ifstream file(vm["config"].as<string>());
    json data;
    file >> data;

    // Access the values in the JSON object
    vector<vector<int>> worldDescriptor = data["world_descriptor"];
    vector<vector<int>> robots = data["robots"];
    vector<int> capacities = data["capacities"];
    vector<vector<int>> intermediatePositions = data["intermediate_positions"];
    vector<Task> tasks;
    auto temp_tasks = data["tasks"];
    for (const auto& task : temp_tasks) {
        tasks.push_back(
            Task(
                Cell(task[0][0], task[0][1]),
                Cell(task[1][0], task[1][1]),
                task[2],
                task[3]
            )
        );
    }

    // Access and print individual values
    cout << "===============================================================\n";
    cout << " World Descriptor\n";
    for (const auto& row : worldDescriptor) {
        for (const auto& value : row) {
            cout << value << " ";
        }
        cout << endl;
    }
    
    cout << "===============================================================\n";
    cout << " Robots Inital Location \n";
    for (const auto& robot : robots) {
        cout << "[" <<  robot[0] << ", " << robot[1] << "]" << endl;
    }
    
    cout << "===============================================================\n";
    cout << " Tasks (Start/ End/ Deadline / Weight)\n";
    for (const auto& task : tasks) {
        cout << "[" << task.start.x << ", " << task.start.y << "] -> "
                  << "[" << task.end.x << ", " << task.end.y << "], "
                  << task.deadline << ", " << task.deadline << endl;
    }
    
    cout << "===============================================================\n";
    cout << " Robots Capacities \n";
    for (const auto& capacity : capacities) {
        cout << capacity << ", ";
    }
    cout << endl;
    
    cout << "===============================================================\n";
    cout << " Available Intermediate Positions \n";
    for (const auto& position : intermediatePositions) {
        cout << "[" << position[0] << ", " << position[1] << "]" << endl;
    }
    cout << endl;
    
    cout << "===============================================================\n";
    return 0;
}
