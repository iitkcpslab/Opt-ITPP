#include "Instance.h"
#include "SpaceTimeAStar.h"

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;

using namespace std;
template<typename T, typename Y>
auto operator<<(std::ostream& stream, std::pair<T,Y> const& p) -> std::ostream& {
    stream << "(" << p.first << ", " << p.second << ") ";
    return stream;
}

int main(int argc, char **argv) {

    
	// Declare the supported options.
    namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()("help", "produce help message")
    ("config,c", po::value<string>()->required(), "input file for configuration");
    po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::ifstream file(vm["config"].as<string>());
    json data = json::parse(file);
    file.close();

	// Access the values in the JSON object
    std::vector<std::vector<int>> worldDescriptor = data["world_descriptor"];
    std::vector<std::vector<int>> start_locations = data["start_locations"];
    std::vector<std::vector<std::vector<int>>> goal_locations = data["goal_locations"];
    std::vector<std::vector<int>> deadlines = data["deadlines"];
    std::vector<std::vector<int>> temp_constraint = data["temp_constraints"];
    // std::vector<std::vector<int>> temp_constraint;

    Instance instance(worldDescriptor, start_locations, goal_locations, deadlines, temp_constraint, 2);

    CBSNode *dummy = new CBSNode();
    dummy->g_val = 0;
    vector<Path *> paths;
    paths.resize(1, nullptr);


    ConstraintTable initCons(instance.num_of_cols, instance.map_size);
    initCons.build(*dummy, 0, instance.goal_locations[0].size());
    // initCons.insertLandmark(instance.goal_locations[0][0], 8);
    // initCons.insert2CT(98, 4, 5);
    // initCons.insert2CT(99, 5, 6);
    // initCons.insert2CT(89, 6, 7);
    // initCons.insert2CT(79, 7, 8);
    // initCons.insert2CT(69, 8, 9);
    // initCons.insert2CT(59, 9, 10);
    // initCons.insert2CT(49, 10, 11);
    // initCons.insert2CT(39, 11, 12);
    // initCons.insert2CT(29, 12, 13);
    // initCons.insert2CT(19, 13, 14);
    // initCons.insert2CT(93, 1, 3);
    initCons.g_goal_time[0] = 11;
    // initCons.leq_goal_time[0] = 8;

    MultiLabelSpaceTimeAStar engine(instance, 0);
    Path resPath = engine.findPath(*dummy, initCons, paths, 0, 0);
    
    cout << "Result Path :: \n";
    // for(auto p:resPath.path) {
    for(int i=0; i < (int)resPath.path.size(); i++) {
        cout << i << "   " << resPath.path[i].location << " " << instance.getCoordinate(resPath.path[i].location) << "\n";
    }
    cout << "\n\n\n";

    for(auto p:resPath.timestamps) {
        cout << p << "  -> ";
    }
    cout << endl;

    // I will write test code here

}