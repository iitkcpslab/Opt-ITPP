#include <vector>
#include <iostream>
#include <algorithm>
#include <tuple>
#include <queue>
#include <vector>
#include <map>
#include <math.h>
#include <limits.h>
#include <string.h>
#include <exception>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using namespace std;

template <typename S>
ostream& operator<<(ostream& os, const vector<S>& vector)
{
    os << "[";
    for (auto element : vector) os << element << ", ";
    os << "]";
    return os;
}

#ifndef COMMON_INCLUDES_H
struct Cell{
    int x;
    int y;

    Cell() : x(0), y(0) {}
    Cell(int x, int y) : x(x), y(y) {}

    bool operator<(const Cell &o) const 
    {
        return (this->x < o.x) || (this->x == o.x && this->y < o.y);
    }
};
#endif

struct Node{
    Cell cell;
    int f;
    Node(Cell n, int tf) : cell(n), f(tf) {}
} ;

struct compareNodes {
    bool operator()(Node * const p1, Node * const p2)
    {
        return p1->f > p2->f;
    }
};

typedef struct task{
    Cell start;
    Cell end;
    int deadline;
    int weight;
    task(Cell s, Cell e, int d, int w) :
        start(s), end(e), deadline(d), weight(w) {}
} Task;

class Configuration
{
    public:
        int R;
        int C;
        int num_init_pos;
        int num_tasks;
        int num_ipos;
        int total_positions;

        vector<vector<int>> world_descriptor;
        vector<Cell> init_pos;
        vector<Task> tasks;
        vector<Cell> intermediate_pos;
        vector<int> capacities;
        vector<int> deadlines;
        vector<int> weights;

        vector<vector<int>> distances;
        vector<Cell> all_pos;
        map<Cell, int> all_pos_inv;
        vector<int> init_pos_id;
        vector<int> tasks_start_id;
        vector<int> tasks_end_id;
        vector<int> intermediate_pos_id;
        boost::unordered_set<int> intermediate_pos_set;
        
        bool isValid(Cell n) {
            return (n.x >= 0) && (n.y >= 0) 
                && (n.x < this->R) && (n.y < this->C);
        }

        bool isAvailable(Cell n) {
            return (this->world_descriptor[n.x][n.y] == 0);
        }

        int getHeuristicScore(Cell curr, Cell dest) {
            return ( (double)abs((curr.x - dest.x) * (curr.x - dest.x)
                + (curr.y - dest.y) * (curr.y - dest.y)));
        }

        std::vector<int> getLocationFromCell(Cell curr) {
            return {curr.x, curr.y};
        }

        std::vector<int> getLocationFromId(int id) {
            return {this->all_pos[id].x, this->all_pos[id].y};
        }

        int getDeadline(int task_id) {
            auto it = std::find(tasks_start_id.begin(), tasks_start_id.end(), task_id);
            if(it == tasks_start_id.end()) {
                std::cout << "Error processing deadline\n";
            }
            int index = it - tasks_start_id.begin();
            return this->deadlines[index];
        }
        
        int astar_path_length(Cell source, Cell dest)
        {
            // std::cout << "Validity Source :: " << isValid(source) << std::endl;
            // std::cout << "Validity Dest :: " << isValid(dest) << std::endl;
            // std::cout << "Available Source :: " << isAvailable(source) << std::endl;
            // std::cout << "Available Dest :: " << isAvailable(dest) << std::endl;
            if(!isValid(source) || !isValid(dest) || !isAvailable(source) || !isAvailable(dest))
            {
                return INT_MAX;
            }

            bool closedList[this->R][this->C];
            int openListG[this->R][this->C];
            int openListH[this->R][this->C];

            priority_queue<Node*, vector<Node*>, compareNodes> openList;
            memset(closedList, false, sizeof(closedList));
            for(int i=0; i<this->R; i++)
            {
                for(int j=0; j<this->C; j++)
                {
                    openListG[i][j] = INT_MAX;
                    openListH[i][j] = INT_MAX;
                }
            }
            
            openListG[source.x][source.y] = 1;
            openListH[source.x][source.y] = getHeuristicScore(source, dest);
            Node *sourceNode = new Node(source, openListG[source.x][source.y] + openListH[source.x][source.y]);
            openList.push(sourceNode);
            // std::cout << "\nSource node :: " << sourceNode->cell.x << " " << sourceNode->cell.y << " " << sourceNode->f << "\n";
            while(!openList.empty())
            {
                Node *current = openList.top();
                Cell currentCell = current->cell;
                openList.pop();
                // std::cout << "open node :: " << current->cell.x << " " << current->cell.y << " " << current->f << "\n";
                if( currentCell.x == dest.x && currentCell.y == dest.y )
                {
                    return current->f;
                }
                if( true == closedList[currentCell.x][currentCell.y] )  continue;
                closedList[currentCell.x][currentCell.y] = true;

                vector<Cell> nbors;
                Cell l(currentCell.x-1, currentCell.y);
                Cell r(currentCell.x+1, currentCell.y);
                Cell u(currentCell.x, currentCell.y-1);
                Cell b(currentCell.x, currentCell.y+1);
                if( isValid(l) && isAvailable(l) && (closedList[l.x][l.y] == false) ) nbors.push_back(l);
                if( isValid(r) && isAvailable(r) && (closedList[r.x][r.y] == false) ) nbors.push_back(r);
                if( isValid(u) && isAvailable(u) && (closedList[u.x][u.y] == false) ) nbors.push_back(u);
                if( isValid(b) && isAvailable(b) && (closedList[b.x][b.y] == false) ) nbors.push_back(b);

                for(Cell nbor : nbors)
                {
                    int gCost = openListG[currentCell.x][currentCell.y] + 1;
                    int hCost = getHeuristicScore(nbor, dest);
                    if(gCost < openListG[nbor.x][nbor.y])
                    {
                        Node *tempN = new Node(nbor, gCost + hCost);
                        openListG[nbor.x][nbor.y] = gCost;
                        openListH[nbor.x][nbor.y] = hCost;
                        openList.push(tempN);
                    }
                }
            }
            return INT_MAX;
        }

    // public:
        Configuration(
            vector<vector<int>> world_descriptor, 
            vector<Cell> init_pos,
            vector<Task> tasks, 
            vector<Cell> intermediate_pos,
            vector<int> capacities,
            int screen=0
        ) : world_descriptor(world_descriptor),
            init_pos(init_pos),
            tasks(tasks),
            intermediate_pos(intermediate_pos),
            capacities(capacities)
        {
            this->R = world_descriptor.size();
            this->C = world_descriptor[0].size();
            this->deadlines.resize(tasks.size());
            this->weights.resize(tasks.size());

            for (unsigned int i = 0; i < tasks.size(); i++)
            {
                this->deadlines[i] = tasks[i].deadline;
                this->weights[i] = tasks[i].weight;
            }

            this->num_init_pos = init_pos.size();
            this->num_tasks = tasks.size();
            this->num_ipos = intermediate_pos.size();
            if((int)this->capacities.size() != this->num_init_pos) {
                cerr << "Wrong Configuration Size\n";
                exit(-1);
            }
            this->total_positions = this->num_init_pos + 2 * this->num_tasks + this->num_ipos;
            this->distances.resize( this->total_positions + 1, vector<int>( this->total_positions+1, INT_MAX ) );\
            this->all_pos.resize(this->total_positions + 1);

            if(screen > 1) cout << "==================================================================\n";
            if(screen > 1) cout << "Init/Task/Inter/Total : " << num_init_pos << " / " << num_tasks << " / " <<  num_ipos  << " / " << total_positions << "\n\n";
            for(int i=0; i<this->total_positions; i++)
            {
                if (i < this->num_init_pos) {
                    if(screen > 1) printf("Processing Initial Positions :: %d %d : (%d %d)\n", i, i, init_pos[i].x, init_pos[i].y);
                    
                    init_pos_id.push_back(i+1);
                    all_pos[i+1] = init_pos[i];
                    Cell start = init_pos[i];
                    all_pos_inv[init_pos[i]] = i+1;

                    if(!isValid(start) || !isAvailable(start)) {
                        cerr << "Location Error : (" << start.x << ", " << start.y << ")\n";
                        exit(-1);
                    }
                    
                    for (int j = 0; j < num_tasks; j++) {
                        Cell end = tasks[j].start;
                        if(!isValid(end) || !isAvailable(end)) {
                            cerr << "Location Error : (" << end.x << ", " << end.y << ")\n";
                            exit(-1);
                        }
                        int path_len = astar_path_length(start, end);
                        distances[i][num_init_pos + j * 2] = (path_len > 0) ? path_len-1 : 0;

                        if(screen > 1)  printf("0.1  %d %d (%d %d), (%d %d), dist: %d\n", i, j, start.x, start.y, end.x, end.y, distances[i][num_init_pos + j * 2]);
                    }
                    
                    for (int j = 0; j < num_ipos; j++) {
                        Cell end = intermediate_pos[j];
                        if(!isValid(end) || !isAvailable(end)) {
                            cerr << "Location Error : (" << end.x << ", " << end.y << ")\n";
                            exit(-1);
                        }
                        int path_len = astar_path_length(start, end);
                        distances[i][num_init_pos + num_tasks * 2 + j] = (path_len > 0) ? path_len-1 : 0;

                        if(screen > 1)  printf("0.2  %d %d (%d %d), (%d %d), dist: %d\n", i, j, start.x, start.y, end.x, end.y, distances[i][num_init_pos + num_tasks * 2 + j]);
                    }
                }
                else if (i < num_init_pos + 2 * num_tasks) 
                {
                    if ((i - num_init_pos) % 2 == 0)  // From task start location, we can go to task end or other tasks start or an intermediate loc but cant return to start
                    {
                        if(screen > 1)  printf("Processing Tasks Start Positions :: %d %d : (%d %d)\n", i, (i - num_init_pos)/2, tasks[(i - num_init_pos)/2].start.x, tasks[(i - num_init_pos)/2].start.y);

                        this->tasks_start_id.push_back(i + 1);
                        all_pos[i + 1] = tasks[(i - num_init_pos)/2].start;
                        all_pos_inv[tasks[(i - num_init_pos)/2].start] = i + 1;
                        Cell start = tasks[(i - num_init_pos)/2].start;
                        if(!isValid(start) || !isAvailable(start)) {
                            cerr << "Location Error : (" << start.x << ", " << start.y << ")\n";
                            exit(-1);
                        }
                        for (int j = 0; j < total_positions; ++j) 
                        {
                            Cell end;
                            if (j < num_init_pos || j == i)                 // Ignore Task Start to Initial agent position and self
                                continue;
                            else if (j < num_init_pos + 2 * num_tasks)      // Update Distance Task Starting Location -> Other task starts or ends
                                end = (j - num_init_pos) % 2 ? tasks[(j - num_init_pos)/2].end : tasks[(j - num_init_pos)/2].start;
                            else                                            // Update Distance Task Starting Location -> Intermediate locations 
                                end = intermediate_pos[j - (num_init_pos + 2 * num_tasks)];
                            if(!isValid(end) || !isAvailable(end)) {
                                cerr << "Location Error : (" << end.x << ", " << end.y << ")\n";
                                exit(-1);
                            }
                            int path_len = astar_path_length(start, end);
                            distances[i][j] = (path_len > 0) != 0 ? (path_len - 1) : 0;

                            if(screen > 1)  printf("1.1  %d %d (%d %d), (%d %d), dist: %d\n", i, j, start.x, start.y, end.x, end.y, distances[i][j]);
                        }
                    }
                    else // From task end location, we can go to start, other task start, other task end and intermediate 
                    {
                        if(screen > 1)  printf("Processing Tasks Ending Positions :: %d %d : (%d %d)\n", i, (i - num_init_pos)/2, tasks[(i - num_init_pos)/2].end.x, tasks[(i - num_init_pos)/2].end.y);

                        this->tasks_end_id.push_back(i + 1);
                        all_pos[i + 1] = tasks[(i - num_init_pos) / 2].end;
                        all_pos_inv[tasks[(i - num_init_pos) / 2].end] = i + 1;
                        auto start = tasks[(i - num_init_pos) / 2].end;
                        if(!isValid(start) || !isAvailable(start)) {
                            cerr << "Location Error : (" << start.x << ", " << start.y << ")\n";
                            exit(-1);
                        }
                        for (int j = 0; j < total_positions; j++) 
                        {
                            Cell end;
                            if (j == i || j == i - 1)                       // Ignore task ending to self and its start
                                continue;
                            else if (j < num_init_pos)                      // Update Distance Task Ending Location -> Initial Agent Positions
                                end = init_pos[j];
                            else if (j < num_init_pos + num_tasks * 2)      // Update Distance Task Starting Location -> Other Task Starts and Ends
                                end = (j - num_init_pos) % 2 ? tasks[(j - num_init_pos)/2].end : tasks[(j - num_init_pos)/2].start;
                            else                                            // Update Distance Task Starting Location -> Intermediate Locations
                                end = intermediate_pos[j - (num_init_pos + num_tasks * 2)];
                            if(!isValid(end) || !isAvailable(end)) {
                                cerr << "Location Error : (" << end.x << ", " << end.y << ")\n";
                                exit(-1);
                            }
                            auto path_len = astar_path_length(start, end);
                            distances[i][j] = (path_len > 0) != 0 ? (path_len - 1) : 0;

                            if(screen > 1)  printf("1.2  %d %d (%d %d), (%d %d), dist: %d\n", i, j, start.x, start.y, end.x, end.y, distances[i][j]);
                        }
                    }
                }
                else
                {
                    if(screen > 1)  printf("Processing Intermediate Positions %d %d\n", i, i - (num_init_pos + num_tasks * 2));
                    
                    this->intermediate_pos_id.push_back(i + 1);
                    this->intermediate_pos_set.insert(i + 1);
                    all_pos[i+1] = intermediate_pos[i - (num_init_pos + num_tasks * 2)];
                    all_pos_inv[intermediate_pos[i - (num_init_pos + num_tasks * 2)]] = i + 1;
                    Cell start = intermediate_pos[i - (num_init_pos + num_tasks * 2)];
                    if(!isValid(start) || !isAvailable(start)) {
                        cerr << "Location Error : (" << start.x << ", " << start.y << ")\n";
                        exit(-1);
                    }
                    for (int j = 0; j < total_positions; j++) {
                        Cell end;
                        if (i == j)  // Update intermediate to self
                            continue; 
                        else if (j < num_init_pos)  // Update Distance Intermediate Locations -> Initial Agent Positions
                            end = init_pos[j];
                        else if (j < num_init_pos + num_tasks * 2)  // Update Distance Intermediate Locations -> Other Start and End Positions
                            end = (j - num_init_pos) % 2 ? tasks[(j - num_init_pos)/2].end : tasks[(j - num_init_pos)/2].start;
                        else  
                            end = intermediate_pos[j - (num_init_pos + num_tasks * 2)];
                        if(!isValid(end) || !isAvailable(end)) {
                            cerr << "Location Error : (" << end.x << ", " << end.y << ")\n";
                            exit(-1);
                        }
                        auto path_len = astar_path_length(start, end);
                        distances[i][j] = (path_len > 0) ? (path_len - 1) : 0;

                        if(screen > 1)  printf("2.1  %d %d (%d %d), (%d %d), dist: %d\n", i, j, start.x, start.y, end.x, end.y, distances[i][j]);
                    }
                }
            }
        
            if(screen > 0) {
                cout << "-----------------------------------------------------------------\n";
                cout << "World Descriptor : {\n";
                for(auto row : this->world_descriptor) {
                    cout << "\t" << row << endl;
                } 
                cout << "}\n";
                cout << "Initial Position : {" << init_pos_id << "}\n";
                cout << "Task Start Id : {" << this->tasks_start_id << "}\n";
                cout << "Task End Id : {" << this->tasks_end_id << "}\n";
                cout << "Intermediate Start Id : {" << this->intermediate_pos_id << "}\n";
                cout << "Capacities : {" <<  this->capacities << "}\n";
                cout << "Distances : {\n";
                for(int i=0; i<this->total_positions; i++)
                {
                    cout << "\t";
                    for(int j=0; j<this->total_positions; j++)
                    {
                        if(distances[i][j] == INT_MAX)
                            cout << "\t";
                        else
                            cout << distances[i][j] << "\t";
                    }
                    cout << endl;
                }
                cout << "}\n";

                cout << "All Pos : {\n";
                for(unsigned int i=0; i<all_pos.size(); i++) {
                    cout << "\t" <<  i << "  -> (" << all_pos[i].x << ", " << all_pos[i].y << ")\n"; 
                }
                cout << "}\n";
                cout << "-----------------------------------------------------------------\n";
            }
        }

        ~Configuration()
        {
            // for (unsigned int i = 0; i < this->total_positions; i++) {
            //     delete[] this->distances[i];
            // }
            // delete[] this->distances;
        }

        int get_lower_bound(int mode) {
            int min_start = INT_MAX;
            for (unsigned int i : this->init_pos_id) {
                for (int t : this->tasks_start_id) {
                    min_start = min(min_start, this->distances[i-1][t-1]);
                }
                for (int t : this->intermediate_pos_id) {
                    min_start = min(min_start, this->distances[i-1][t-1]);
                }
            }

            int max_task = 0;
            int sum_task = 0;
            for (int t : this->tasks_start_id) {
                max_task = max(max_task, this->distances[t-1][t]);
                sum_task += this->distances[t-1][t];
            }

            int min_end = INT_MAX;
            for (int t : this->tasks_end_id) {
                for (unsigned int i : this->init_pos_id) {
                    min_end = min(min_end, this->distances[t-1][i-1]);
                }
            }
            for (int t : this->intermediate_pos_id) {
                for (unsigned int i : this->init_pos_id) {
                    min_end = min(min_end, this->distances[t-1][i-1]);
                }
            }

            int lower_bound = 0;
            if (mode == 1) {
                lower_bound = min_start + max_task + min_end;
            } else {
                lower_bound = min_start + sum_task + min_end;
            }

            return lower_bound;
        }

        int get_upper_bound(int mode) 
        {
            int min_start = INT_MAX;
            for (unsigned int i : this->init_pos_id) {
                for (int t : this->tasks_start_id) {
                    min_start = min(min_start, this->distances[i-1][t-1]);
                }
                for (int t : this->intermediate_pos_id) {
                    min_start = min(min_start, this->distances[i-1][t-1]);
                }
            }

            int max_task = 0;
            int sum_task = 0;
            for (int t : this->tasks_start_id) {
                max_task = max(max_task, this->distances[t-1][t]);
                sum_task += this->distances[t-1][t];
            }

            int min_end = INT_MAX;
            for (int t : this->tasks_end_id) {
                for (unsigned int i : this->init_pos_id) {
                    min_end = min(min_end, this->distances[t-1][i-1]);
                }
            }
            for (int t : this->intermediate_pos_id) {
                for (unsigned int i : this->init_pos_id) {
                    min_end = min(min_end, this->distances[t-1][i-1]);
                }
            }

            int upper_bound = 0;
            for (int i=0; i<this->total_positions; i++) {
                 for (int j=0; j<this->total_positions; j++) {
                    if (this->distances[i][j] != INT_MAX) {
                        upper_bound += this->distances[i][j] + 1;
                    }
                }
            }

            return upper_bound;
        }
};

// int main()
// {
//     // Example usage
//     vector<vector<int>> world_descriptor = {
//         {0, 1, 0, 0}, 
//         {0, 1, 0, 1}, 
//         {0, 1, 0, 0},
//         {0, 1, 1, 0},
//         {0, 0, 0, 0}
//     };
//     vector<Cell> init_pos = { Cell(0, 0), Cell(4, 3)};
//     vector<Task> tasks = { Task( Cell(2, 0), Cell(0, 3), 1, 1), Task( Cell(0, 2), Cell(1, 0), 1, 1) };
//     // vector<Cell> init_pos;
//     // vector<Task> tasks;
//     vector<Cell> intermediate_pos = { Cell(2, 2), Cell(1, 2) };
//     vector<int> capacities = {10, 20};
//     Configuration config(world_descriptor, init_pos, tasks, intermediate_pos, capacities);
//     config.printObjectString();
//     // printf("Testing :: Astar length : %d\n", config.astar_path_length(Cell(0, 1), Cell(0, 1)) );
//     return 0;
// }
