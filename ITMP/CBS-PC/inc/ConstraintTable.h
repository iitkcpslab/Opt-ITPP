#pragma once

#include "common.h"
#include "CBSNode.h"

class ConstraintTable
{
private:
	size_t map_size_threshold = 10000;
	vector<list<size_t>> cat_large; // conflict avoidance table for large maps
	vector<vector<bool>> cat_small; // conflict avoidance table for small maps

protected:
	// Constraint Table (CT)
	unordered_map<size_t, size_t> landmarks; // <timestep, location>: the agent must be at the given location at the given timestep
	unordered_map<size_t, list<pair<int, int>>> ct; // location -> time range, or edge -> time range

	void insertLandmark(size_t loc, int t); // insert a landmark, i.e., the agent has to be at the given location at the given timestep
	inline size_t getEdgeIndex(size_t from, size_t to) const { return (1 + from) * map_size + to; }

public:
	int length_min 		= 0;
	int length_max 		= MAX_TIMESTEP;
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.
	size_t num_col;
	size_t map_size;
	int cat_size 		= 0;

	vector<int> leq_goal_time;
	vector<int> g_goal_time;

	ConstraintTable() = default;
	ConstraintTable(size_t num_col, size_t map_size, int goal_location = -1) : goal_location(goal_location), num_col(num_col), map_size(map_size) {}
	ConstraintTable(const ConstraintTable &other) { copy(other); }

	int getHoldingTime(); // the earliest timestep that the agent can hold its goal location

	bool constrained(size_t loc, int t) const;
	bool constrained(size_t curr_loc, size_t next_loc, int next_t) const;
	int getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const;	

	void copy(const ConstraintTable &other);
	void build(const CBSNode &node, int agent, int num_of_stops);			// build the constraint table for the given agent at the given node
	void buildCAT(int agent, const vector<Path *> &paths, size_t cat_size); // build the conflict avoidance table

	void addPath(const Path &path, bool wait_at_goal);

	void insert2CT(size_t loc, int t_min, int t_max);			  			// insert a vertex constraint to the constraint table
	void insert2CT(size_t from, size_t to, int t_min, int t_max); 			// insert an edge constraint to the constraint table

	size_t getNumOfLandmarks() const { return landmarks.size(); }
	unordered_map<size_t, size_t> getLandmarks() const { return landmarks; }
	list<pair<int, int>> decodeBarrier(int B1, int B2, int t);
};
