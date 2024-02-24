#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::bidirectionalS> searchGraphTraits_t;
typedef searchGraphTraits_t::edge_descriptor edge_t;
typedef searchGraphTraits_t::vertex_descriptor vertex_t;

struct TemporalEdge
{
	int edge_weight;	// We only model upper bound for simplicity;
};

struct TemporalNode
{
	std::string name;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, TemporalNode, TemporalEdge> TemporalGraph_t;
typedef boost::unordered_map<vertex_t, boost::unordered_map<vertex_t, int>> distance_matrix_t;
typedef std::unordered_map<std::string, int> schedule_t;

class TemporalGraph
{
public:
	vertex_t x0;
	TemporalGraph_t G;
	boost::unordered_map<std::string, vertex_t> vertex_map;

	TemporalGraph()
	{
		x0 = boost::add_vertex(G);
		G[x0].name = "x0";
	};
	TemporalGraph(TemporalGraph &graph_to_copy);
	~TemporalGraph(){};

	std::pair<edge_t, bool> add_lb(std::string name, int lb);
	std::pair<edge_t, bool> add_ub(std::string name, int ub);
	std::pair<edge_t, bool> add_edge(std::string from, std::string to, int ub);

	void add_ub(int ub);				// add ub to every node
	bool add_node(std::string name);
	void print_graph();
};

schedule_t to_schedule(const TemporalGraph &tg, distance_matrix_t &distance_matrix);
bool compute_distance(const TemporalGraph &tg, distance_matrix_t &distance_matrix);

int makespan(const TemporalGraph &tg, distance_matrix_t &distance_matrix);
int get_dist(const TemporalGraph &tg, const distance_matrix_t matrix, const std::string name_0, const std::string name_1);
int get_lb(const TemporalGraph &tg, const distance_matrix_t &matrix, const std::string name);
int get_ub(const TemporalGraph &tg, const distance_matrix_t &matrix, const std::string name);
