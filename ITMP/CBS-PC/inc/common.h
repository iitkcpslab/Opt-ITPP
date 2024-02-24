#pragma once

#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream> // std::cout, std::fixed
#include <iomanip>	// std::setprecision
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::pairing_heap;
using std::list;
using std::set;
using std::vector;
// using std::get;
using std::cerr;
using std::clock;
using std::cout;
using std::endl;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::max;
using std::min;
using std::ofstream;
using std::pair;
using std::shared_ptr;
using std::string;
using std::tie;
using std::tuple;

// #define NDEBUG

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry
{
	int location = -1;
	int mdd_width; 				// TODO:: Myabe this can be deleted as we always build/look for MDDs when we classify conflicts
	bool is_goal;
	bool is_single() const {
		return mdd_width == 1;
	}
};

struct Path
{
	int begin_time = 0;
	vector<PathEntry> path;
	vector<int> timestamps;

	Path(){};
	Path(int size) : path(vector<PathEntry>(size)) {}

	int end_time() {
		return begin_time + size() - 1;
	}

	bool empty() const { return path.empty(); }
	size_t size() const { return path.size(); }
	PathEntry &back() { return path.back(); }
	PathEntry &front() { return path.front(); }
	const PathEntry &back() const { return path.back(); }
	const PathEntry &front() const { return path.front(); }
	PathEntry &at(int idx) { return path[idx]; }

	PathEntry &operator[](int idx) { return path[idx]; }
	const PathEntry &operator[](int idx) const { return path[idx]; }
};

std::ostream &operator<<(std::ostream &os, const Path &path);
bool isSamePath(const Path &p1, const Path &p2);