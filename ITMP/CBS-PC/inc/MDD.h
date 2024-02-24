#pragma once

#include "SingleAgentSolver.h"

class MDDNode
{
public:
	int stage;
	int location;
	int level;
	int cost = 0; // minimum cost of path traversing this MDD node
	list<MDDNode *> children;
	list<MDDNode *> parents;

	MDDNode(int currloc, MDDNode *parent, int stage = 0) : stage(stage)
	{
		location = currloc;
		if (parent == nullptr)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
	}

	bool operator==(const MDDNode &node) const
	{
		return (this->location == node.location) && (this->level == node.level) && (this->stage == node.stage);
	}
};

class MDD
{
private:
	const SingleAgentSolver *solver;

public:
	vector<list<MDDNode *>> levels;
	vector<list<MDDNode *>> flatten_levels;

	bool buildMDD(const ConstraintTable &ct, int num_of_levels, const SingleAgentSolver *solver);

	void flattenMDD();

	MDDNode *find(int location, int level) const;
	void deleteNode(MDDNode *node);
	void clear();

	void increaseBy(const ConstraintTable &ct, int dLevel, SingleAgentSolver *solver);
	MDDNode *goalAt(int level);

	MDD() = default;
	MDD(const MDD &cpy);
	~MDD();
};

std::ostream &operator<<(std::ostream &os, const MDD &mdd);

class SyncMDDNode
{
public:
	int location;
	list<SyncMDDNode *> children;
	list<SyncMDDNode *> parents;
	list<const MDDNode *> coexistingNodesFromOtherMdds;

	SyncMDDNode(int currloc, SyncMDDNode *parent)
	{
		location = currloc;
		if (parent != nullptr)
		{
			parents.push_back(parent);
		}
	}

	bool operator==(const SyncMDDNode &node) const
	{
		return (this->location == node.location);
	}
};

class SyncMDD
{
public:
	vector<list<SyncMDDNode *>> levels;

	SyncMDDNode *find(int location, int level) const;
	void deleteNode(SyncMDDNode *node, int level);
	void clear();

	explicit SyncMDD(const MDD &cpy);
	~SyncMDD();
};

class MDDTable
{
private:
	int max_num_of_mdds = 10000;

	vector<unordered_map<ConstraintsHasher, MDD *, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>> lookupTable;

	const vector<ConstraintTable> &initial_constraints;
	const vector<SingleAgentSolver *> &search_engines;
	void releaseMDDMemory(int id);

public:
	double accumulated_runtime = 0; // runtime of building MDDs
	uint64_t num_released_mdds = 0; // number of released MDDs ( to save memory)

	MDDTable(
		const vector<ConstraintTable> &initial_constraints,
		const vector<SingleAgentSolver *> &search_engines
	) : 
		initial_constraints(initial_constraints), 
		search_engines(search_engines) {}

	~MDDTable() { clear(); }

	void init(int number_of_agents)
	{
		lookupTable.resize(number_of_agents);
	}

	MDD *getMDD(CBSNode &node, int agent, size_t mdd_levels);
	void findSingletons(CBSNode &node, int agent, Path &path);
	double getAverageWidth(CBSNode &node, int agent, size_t mdd_levels);
	void clear();
};

unordered_map<int, MDDNode *> collectMDDlevel(MDD *mdd, int i);
