#include <boost/tokenizer.hpp>
#include <algorithm> // std::shuffle
#include <random>	 // std::default_random_engine
#include <chrono>	 // std::chrono::system_clock
#include "Instance.h"

int RANDOM_WALK_STEPS = 100000;

template <typename S>
std::ostream& operator<<(std::ostream& os, const std::vector<S>& vector)
{
    for (auto element : vector) os << element << " ";
    return os;
}

template<typename T, typename Y>
auto operator<<(std::ostream& stream, std::pair<T,Y> const& p) -> std::ostream& {
    stream << "(" << p.first << ", " << p.second << ") ";
    return stream;
}

Instance::Instance(
	std::vector<std::vector<int>> m_my_map,
	std::vector<std::vector<int>> m_start_locations,
	std::vector<std::vector<std::vector<int>>> m_goal_locations,
	std::vector<std::vector<int>> deadlines,
	std::vector<std::vector<int>> m_temp_constraints,
	int screen
) {
	this->num_of_agents = m_start_locations.size();
	this->num_of_rows = m_my_map.size();
	if(this->num_of_rows == 0) {
		cerr << "Error in Map\n";
		exit(-1);
	}
	this->num_of_cols = m_my_map[0].size();
	this->map_size = this->num_of_rows * this->num_of_cols;
	this->num_of_constraints = m_temp_constraints.size();

	// --------Load Map----------------------------------------
	this->my_map.resize(this->map_size, false);
	for(int i=0; i<this->num_of_rows; i++) {
		for(int j=0; j<this->num_of_cols; j++) {
			if(m_my_map[i][j])
				this->my_map[this->linearizeCoordinate(i, j)] = true;
		}
	}

	// --------Load Agents-------------------------------------
	this->goals_count.resize(this->num_of_agents, 0);
	this->goal_locations.resize(this->num_of_agents, {});
	this->start_locations.resize(this->num_of_agents, 0);
	for(int i=0; i<this->num_of_agents; i++) {
		auto tmp = m_start_locations[i];

		this->start_locations[i] = this->linearizeCoordinate(tmp[0], tmp[1]);
		this->goals_count[i] = m_goal_locations[i].size();
		this->goal_locations[i].resize(this->goals_count[i], 0);
		if(m_goal_locations[i].size() != deadlines[i].size()) {
			cout << m_goal_locations[i].size() << " " << deadlines[i].size() << endl;
			cerr << "Deadlines Size does not match goals size\n";
			exit(-1);
		}
		for(unsigned int j=0; j < m_goal_locations[i].size(); j++) {
			auto tmp2 = m_goal_locations[i][j];
			this->goal_locations[i][j] = this->linearizeCoordinate(tmp2[0], tmp2[1]);
		}
	}

	this->deadlines = deadlines;
	// ---------Load Temporal Constraints-----------------------
	this->temporal_cons.resize(this->num_of_agents * this->num_of_agents);
	for(auto tcons : m_temp_constraints) {
		int from_id 		= tcons[0];
		int from_landmark 	= tcons[1];
		int to_id 			= tcons[2];
		int to_landmark 	= tcons[3];
		if(
			from_landmark > goals_count[from_id] || 
			to_landmark > goals_count[to_id]
		) {
			cerr << "Wrong Temporal Constraint : " << from_id << " " << from_landmark << " " << to_id << " " << to_landmark << endl;
			continue;
		}
		this->temporal_cons[from_id * this->num_of_agents + to_id].push_back({from_landmark, to_landmark});
	}


	// ----------Print All Information--------------------------
	if(screen > 0) {
		// cout << "--------Print Map------------------\n";
		// for(unsigned int i=0; i<this->my_map.size(); i++) {
		// 	if(this->my_map[i])	cout << "#";
		// 	else				cout << ".";
		// 	if( (i+1) % this->num_of_cols == 0 )
		// 		cout << "\n";
		// }
		cout << "------------------------------------\n";
		cout << "--------Print Agents, Goals And Deadlines-----\n";
		for(int i=0; i<this->num_of_agents; i++) {
			cout << "Agent " << i << " : " << this->getCoordinate(this->start_locations[i]) << endl;  
			for(unsigned int j=0; j<this->goal_locations[i].size(); j++) {
				cout << " => " << this->getCoordinate(this->goal_locations[i][j]);
				if(this->deadlines[i][j] != -1)
					cout << ":" << this->deadlines[i][j];
				if((j+1) % 5 == 0)
					cout << endl;
			}
			cout << endl;
		}

		cout << "--------Print Temporal Constraint---\n";
		for(int i=0; i<this->num_of_agents; i++) {
			for(int j=0; j<this->num_of_agents; j++) {
				int loc_id = i * this->num_of_agents + j;
				if(this->temporal_cons[loc_id].size() > 0) {
					cout << this->getCoordinate(i) << " - " << this->getCoordinate(j) << "  ::  ";
					for(auto lm : this->temporal_cons[loc_id]) {
						cout << lm << " :: ";
					}
					cout << endl;
				}
			}
		}

		cout << "------------------------------------\n";
	}
}

bool Instance::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_size)
		return false;
	if (my_map[next])
		return false;
	return getManhattanDistance(curr, next) < 2;
}

bool Instance::isConnected(int start, int goal)
{
	std::queue<int> open;
	vector<bool> closed(map_size, false);
	open.push(start);
	closed[start] = true;
	while (!open.empty())
	{
		int curr = open.front();
		open.pop();
		if (curr == goal)
			return true;
		for (int next : getNeighbors(curr))
		{
			if (closed[next])
				continue;
			open.push(next);
			closed[next] = true;
		}
	}
	return false;
}

list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}


int Instance::randomWalk(int curr, int steps) const
{
	for (int walk = 0; walk < steps; walk++)
	{
		list<int> l = getNeighbors(curr);
		vector<int> next_locations(l.cbegin(), l.cend());
		auto rng = std::default_random_engine{};
		std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
		for (int next : next_locations)
		{
			if (validMove(curr, next))
			{
				curr = next;
				break;
			}
		}
	}
	return curr;
}

void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
{
	cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << endl;
	int i, j;
	num_of_rows = rows + 2;
	num_of_cols = cols + 2;
	map_size = num_of_rows * num_of_cols;
	my_map.resize(map_size, false);
	// Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/
	// add padding
	i = 0;
	for (j = 0; j < num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	i = num_of_rows - 1;
	for (j = 0; j < num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = 0;
	for (i = 0; i < num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = num_of_cols - 1;
	for (i = 0; i < num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;

	// add obstacles uniformly at random
	i = 0;
	while (i < obstacles)
	{
		int loc = rand() % map_size;
		if (addObstacle(loc))
		{
			printMap();
			i++;
		}
	}
}




//-=-==-=-==-==-==-=-==-=--==-=-==-=-=-==-=-==-==-==-=-=-=--===-=-=-=-==--=--===-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-
//-=-==-=-==-==-==-=-==-=--==-=-==-=-=-==-=-==-==-==-=-=-=--===-=-=-=-==--=--===-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-

Instance::Instance(const string &map_fname, const string &agent_fname,
				   int num_of_agents, int num_of_rows, int num_of_cols, int num_of_obstacles, int warehouse_width) : map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents)
{
	bool succ = loadMap();
	if (!succ)
	{
		if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 &&
			num_of_obstacles < num_of_rows * num_of_cols) // generate random grid
		{
			generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
			saveMap();
		}
		else
		{
			cerr << "Map file " << map_fname << " not found." << endl;
			exit(-1);
		}
	}

	succ = loadAgents();
	if (!succ)
	{
		if (num_of_agents > 0)
		{
			generateRandomAgents(warehouse_width);
			saveAgents();
		}
		else
		{
			cerr << "Agent file " << agent_fname << " not found." << endl;
			exit(-1);
		}
	}
}

bool Instance::loadMap()
{
	using namespace boost;
	using namespace std;
	ifstream myfile(map_fname.c_str());
	if (!myfile.is_open())
		return false;
	string line;
	tokenizer<char_separator<char>>::iterator beg;
	getline(myfile, line);
	if (line[0] == 't') // Nathan's benchmark
	{
		char_separator<char> sep(" ");
		getline(myfile, line);
		tokenizer<char_separator<char>> tok(line, sep);
		beg = tok.begin();
		beg++;
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		getline(myfile, line);
		tokenizer<char_separator<char>> tok2(line, sep);
		beg = tok2.begin();
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
		getline(myfile, line);				// skip "map"
	}
	else // my benchmark
	{
		char_separator<char> sep(",");
		tokenizer<char_separator<char>> tok(line, sep);
		beg = tok.begin();
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
	}
	map_size = num_of_cols * num_of_rows;
	my_map.resize(map_size, false);
	// read map (and start/goal locations)
	for (int i = 0; i < num_of_rows; i++)
	{
		getline(myfile, line);
		for (int j = 0; j < num_of_cols; j++)
		{
			my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
		}
	}
	myfile.close();

	// initialize moves_offset array
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/
	return true;
}

void Instance::printMap() const
{
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (this->my_map[linearizeCoordinate(i, j)])
				cout << '@';
			else
				cout << '.';
		}
		cout << endl;
	}
}

void Instance::saveMap() const
{
	ofstream myfile;
	myfile.open(map_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the map to " << map_fname << endl;
		return;
	}
	myfile << num_of_rows << "," << num_of_cols << endl;
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (my_map[linearizeCoordinate(i, j)])
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}

bool Instance::loadAgents()
{
	using namespace std;
	using namespace boost;

	string line;
	ifstream myfile(agent_fname.c_str());
	if (!myfile.is_open())
		return false;

	getline(myfile, line);
	// My benchmark
	if (num_of_agents == 0)
	{
		cerr << "The number of agents should be larger than 0" << endl;
		exit(-1);
	}
	start_locations.resize(num_of_agents);
	goal_locations.resize(num_of_agents);
	temporal_cons.resize(num_of_agents * num_of_agents);

	char_separator<char> sep("\t");
	for (int i = 0; i < num_of_agents; i++)
	{
		getline(myfile, line);
		while (line[0] == '#')
		{
			getline(myfile, line);
		}
		tokenizer<char_separator<char>> tok(line, sep);
		tokenizer<char_separator<char>>::iterator beg = tok.begin();
		// read start [row,col] for agent i
		int num_landmarks = atoi((*beg).c_str());
		beg++;
		auto col = atoi((*beg).c_str());
		beg++;
		auto row = atoi((*beg).c_str());

		start_locations[i] = linearizeCoordinate(row, col);
		goal_locations[i].resize(num_landmarks);
		//  getline(myfile, line);
		//  tokenizer<char_separator<char>> tok_landmakrs(line, sep);
		//  tokenizer<char_separator<char>>::iterator beg_landmarks = tok_landmakrs.begin();
		for (int j = 0; j < num_landmarks; j++)
		{
			beg++;
			col = atoi((*beg).c_str());
			beg++;
			row = atoi((*beg).c_str());
			goal_locations[i][j] = linearizeCoordinate(row, col);
		}
	}

	getline(myfile, line);
	while (!myfile.eof() && line[0] != 't')
	{
		getline(myfile, line);
	}
	while (!myfile.eof())
	{
		getline(myfile, line);
		tokenizer<char_separator<char>> tok(line, sep);
		tokenizer<char_separator<char>>::iterator beg = tok.begin();
		if (std::distance(tok.begin(), tok.end()) >= 4)
		{
			int from_agent = atoi((*beg).c_str());
			beg++;
			int from_landmark = atoi((*beg).c_str());
			beg++;
			int to_agent = atoi((*beg).c_str());
			beg++;
			int to_landmark = atoi((*beg).c_str());
			if (from_agent < num_of_agents && to_agent < num_of_agents)
			{
				cout << from_agent << ": " << from_landmark << " -> " << to_agent << ": " << to_landmark << endl;
				temporal_cons[from_agent * num_of_agents + to_agent].push_back({from_landmark, to_landmark});
			}
			else
			{
				// cout << "temporal edge not considered" << endl ;
			}
		}
	}

	myfile.close();
	return true;
}

void Instance::printAgents() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i])
			 << ") ; 0: (" << getRowCoordinate(goal_locations[i][0]) << "," << getColCoordinate(goal_locations[i][0]) << ")";
		for (int j = 1; j < (int)goal_locations[i].size(); j++)
		{
			cout << " =>" << j << ": (" << getRowCoordinate(goal_locations[i][j]) << "," << getColCoordinate(goal_locations[i][j]) << ")";
		}
		cout << endl;
	}
}

void Instance::generateRandomAgents(int warehouse_width)
{
}

bool Instance::addObstacle(int obstacle)
{
	if (my_map[obstacle])
		return false;
	my_map[obstacle] = true;
	int obstacle_x = getRowCoordinate(obstacle);
	int obstacle_y = getColCoordinate(obstacle);
	int x[4] = {obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1};
	int y[4] = {obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y};
	int start = 0;
	int goal = 1;
	while (start < 3 && goal < 4)
	{
		if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols || my_map[linearizeCoordinate(x[start], y[start])])
			start++;
		else if (goal <= start)
			goal = start + 1;
		else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols || my_map[linearizeCoordinate(x[goal], y[goal])])
			goal++;
		else if (isConnected(linearizeCoordinate(x[start], y[start]),
							 linearizeCoordinate(x[goal], y[goal]))) // cannot find a path from start to goal
		{
			start = goal;
			goal++;
		}
		else
		{
			my_map[obstacle] = false;
			return false;
		}
	}
	return true;
}

void Instance::saveAgents() const
{
	ofstream myfile;
	myfile.open(agent_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the agents to " << agent_fname << endl;
		return;
	}
	myfile << num_of_agents << endl;
	for (int i = 0; i < num_of_agents; i++)
	{
		myfile << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) << "," << goal_locations[i].size() << endl;

		for (auto g : goal_locations[i])
		{
			cout << getRowCoordinate(g) << "," << getColCoordinate(g) << ",";
		}
	}
	cout << endl;
	myfile.close();
}