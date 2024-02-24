#include "Graph.cpp"
#include <string>
#include <z3++.h>
#include <chrono>
#include <fstream>
#include <ctime>
#include <unordered_set>
using namespace std;

struct PathNode {
	Cell cell;
	int intermediate_time;
	int deadline;
	string nodeType;
};

enum RobotState {
	START=0,
	MOVE,
	PICKUP,
	DROP,
	INTER_MOVE,
	INTER_DROP,
	INTER_PICK,
	FINISH
};

struct PathPlannerConfig {
	vector<vector<int>> start_locations;
	vector<vector<vector<int>>> goal_locations;
	vector<vector<int>> deadlines;
	vector<vector<int>> temp_constraints;
	vector<vector<RobotState>> states;
	int opt_metric;
};

struct IntermediateEvent {
	int time;
	int robot_idx;
	int goal_index;
	RobotState state;
	IntermediateEvent(int time, int robot_idx, int goal_index, RobotState state) :
		time(time), robot_idx(robot_idx), goal_index(goal_index), state(state) {}
};

struct CompareTime {
    bool operator()(IntermediateEvent const& p1, IntermediateEvent const& p2)
    {
        // return "true" if "p1" is ordered
        // before "p2", for example:
        return p1.time > p2.time;
    }
};

class TaskPlanner
{
public:
	int mode;
	int Z;
	z3::context *ctx;
	z3::solver *smt;
	Configuration *config;
	vector<vector<int>> curr_plans;
	string v_timeout = "timeout";
	std::shared_ptr<z3::model> curr_model;
	int screen;

	TaskPlanner(Configuration *m_config, int mode, int Z, int screen = 0)
	{
		this->config = m_config;
		this->mode = mode;
		this->Z = Z;
		this->screen=screen;

		this->ctx = new z3::context();
		this->smt = new z3::solver(*ctx);
		this->initialize_smt_instance();
		this->add_optimizer();
		//curr_model = make_shared<z3::model>();
	}

	// SMT expression generator
	void initialize_smt_instance()
	{
		// -----------------------------------------------------------------------------------------------------------------------
		// Creating 'pos' variables and 'pos_time' variable
		for (int i : this->config->init_pos_id)
		{
			for (int j = 0; j < this->Z; ++j)
			{
				string curr_index = to_string(i) + "_" + to_string(j);
				z3::expr var_pos_curr = ctx->int_const(("pos_" + curr_index).c_str());
				z3::expr var_pos_time_curr = ctx->int_const(("pos_time_" + curr_index).c_str());
				z3::expr var_capacity_curr = ctx->int_const(("capacity_" + curr_index).c_str());
				z3::expr var_action_curr = ctx->int_const(("action_" + curr_index).c_str());

				if (j == 0)
				{
					smt->add(var_pos_curr == i);
					smt->add(var_pos_time_curr == 0);
					smt->add(var_capacity_curr == this->config->capacities[i - 1]);
					smt->add(var_action_curr == -1);
				}
				else if (j == this->Z - 1)
				{
					smt->add(var_pos_curr == i);
					smt->add(var_capacity_curr == this->config->capacities[i - 1]);
					smt->add(var_action_curr == -1);
				}
			}
		}
		// -----------------------------------------------------------------------------------------------------------------------
		// Createing 'loc', 'loc_time' and 'being_carried' variables.
		for (unsigned int k = 0; k < this->config->tasks_start_id.size(); ++k)
		{
			int i = this->config->tasks_start_id[k];
			int e = this->config->tasks_end_id[k];

			for (int j = 0; j < this->Z; ++j)
			{
				string curr_index = to_string(i) + "_" + to_string(j);
				z3::expr var_loc_curr = ctx->int_const(("loc_" + curr_index).c_str());
				z3::expr var_loc_time_curr = ctx->int_const(("loc_time_" + curr_index).c_str());
				z3::expr var_bc_curr = ctx->int_const(("being_carried_" + curr_index).c_str());

				if (j == 0)
				{
					smt->add(var_loc_curr == i);
					smt->add(var_loc_time_curr == 0);
					smt->add(var_bc_curr == -1);
				}
				else if (j == this->Z - 1)
				{
					smt->add(var_loc_curr == e);
					smt->add(var_bc_curr == -1);
				}
			}
		}

		// -----------------------------------------------------------------------------------------------------------------------
		// Create a equation of all movements.
		if (this->screen>1)
		{
			cout << "Possible Movements\n";
		}
		z3::expr_vector Equation(*ctx);
		for (int i : this->config->init_pos_id)
		{
			if (this->screen>1)
			{
				cout << "Robot : " << i << endl;
			}
			z3::expr_vector Equation_move_per_Z(*ctx);
			for (int j = 1; j < this->Z; ++j)
			{
				z3::expr_vector Equation_movements(*ctx);

				if (this->screen>1)
				{
					cout << "\t Step : " << j << endl;
				}
				string prev_index = to_string(i) + "_" + to_string(j - 1);
				string curr_index = to_string(i) + "_" + to_string(j);

				z3::expr var_pos_before = ctx->int_const(("pos_" + prev_index).c_str());
				z3::expr var_pos_after = ctx->int_const(("pos_" + curr_index).c_str());
				z3::expr var_pos_time_before = ctx->int_const(("pos_time_" + prev_index).c_str());
				z3::expr var_pos_time_after = ctx->int_const(("pos_time_" + curr_index).c_str());
				z3::expr var_capacity_before = ctx->int_const(("capacity_" + prev_index).c_str());
				z3::expr var_capacity_after = ctx->int_const(("capacity_" + curr_index).c_str());
				z3::expr var_action_before = ctx->int_const(("action_" + prev_index).c_str());
				z3::expr var_action_after = ctx->int_const(("action_" + curr_index).c_str());

				// CONSTRAINT stay(ri, j) ::  Robot Does nothing
				if (this->screen>1)
				{
					cout << "\t\t stay : " << i << " " << j << endl;
				}
				{
					z3::expr_vector stay_vectors(*ctx);
					stay_vectors.push_back(var_pos_before == var_pos_after);
					stay_vectors.push_back(var_pos_time_before == var_pos_time_after);
					stay_vectors.push_back(var_capacity_before == var_capacity_after);
					stay_vectors.push_back(var_action_after == -1);
					z3::expr var_stay = z3::mk_and(stay_vectors);
					Equation_movements.push_back(var_stay);
				}

				// Generate a vector of accessible locations
				vector<int> movement_locations = {i};
				movement_locations.insert(movement_locations.end(), config->tasks_start_id.begin(), config->tasks_start_id.end());
				movement_locations.insert(movement_locations.end(), config->tasks_end_id.begin(), config->tasks_end_id.end());
				movement_locations.insert(movement_locations.end(), config->intermediate_pos_id.begin(), config->intermediate_pos_id.end());

				// Iterate through accessible locations
				for (int k : movement_locations)
				{
					z3::expr_vector Equation_per_pos(*ctx);

					// Equation to return to base station
					if (j == this->Z - 1)
					{
						if (
								(find(config->tasks_end_id.begin(), config->tasks_end_id.end(), k) != config->tasks_end_id.end()) || find(config->intermediate_pos_id.begin(), config->intermediate_pos_id.end(), k) != config->intermediate_pos_id.end()
						)
						{
							if (this->screen>1)
							{
								cout << "\t\t return from : " << k << " ---> " << i << endl;
							}

							// CONSTRAINT Equation to return to base station
							z3::expr_vector return_vectors(*ctx);
							return_vectors.push_back(var_pos_after == i);
							return_vectors.push_back(var_pos_time_after == var_pos_time_before + config->distances[k - 1][i - 1]);
							return_vectors.push_back(var_action_after == -1);
							return_vectors.push_back(var_capacity_after == var_capacity_before);
							z3::expr var_return = z3::mk_and(return_vectors);
							Equation_per_pos.push_back(var_return);
						}
					}
					else
					{
						for (int l : config->tasks_start_id)
						{
							if (k == l || config->distances[k - 1][l - 1] == INT_MAX)
								continue;

							string prev_loc_index = to_string(l) + "_" + to_string(j - 1);
							string curr_loc_index = to_string(l) + "_" + to_string(j);
							z3::expr var_loc_before = ctx->int_const(("loc_" + prev_loc_index).c_str());
							z3::expr var_loc_after = ctx->int_const(("loc_" + curr_loc_index).c_str());
							z3::expr var_loc_time_before = ctx->int_const(("loc_time_" + prev_loc_index).c_str());
							z3::expr var_loc_time_after = ctx->int_const(("loc_time_" + curr_loc_index).c_str());
							z3::expr var_being_carried_before = ctx->int_const(("being_carried_" + prev_loc_index).c_str());
							z3::expr var_being_carried_after = ctx->int_const(("being_carried_" + curr_loc_index).c_str());

							if (this->screen>1)
							{
								cout << "\t\t pick task : " << k << " --> " << l << endl;
							}

							vector<int>::iterator itr = find(config->tasks_start_id.begin(), config->tasks_start_id.end(), l);
							int task_weight = config->weights[distance(config->tasks_start_id.begin(), itr)];

							// CONSTRAINT pick(ri, tm, j) :: Pickup task from initial location
							z3::expr_vector pick_vectors(*ctx);
							pick_vectors.push_back(var_loc_before == l);
							pick_vectors.push_back(var_loc_after == -1);
							pick_vectors.push_back(var_pos_after == l);
							pick_vectors.push_back(var_being_carried_after == i);
							pick_vectors.push_back(var_pos_time_after == var_pos_time_before + config->distances[k - 1][l - 1] + 1);
							pick_vectors.push_back(var_action_after == l);
							pick_vectors.push_back(var_loc_time_after == var_loc_time_before);
							pick_vectors.push_back(var_capacity_before >= task_weight);
							pick_vectors.push_back(var_capacity_after == var_capacity_before - task_weight);
							z3::expr var_pick = z3::mk_and(pick_vectors);
							Equation_per_pos.push_back(var_pick);
						}

						for (int m : config->tasks_end_id)
						{
							if (k == m || config->distances[k - 1][m - 1] == INT_MAX)
								continue;
							vector<int>::iterator itr = find(config->tasks_end_id.begin(), config->tasks_end_id.end(), m);
							int l = config->tasks_start_id[distance(config->tasks_end_id.begin(), itr)];

							string prev_loc_index = to_string(l) + "_" + to_string(j - 1);
							string curr_loc_index = to_string(l) + "_" + to_string(j);
							z3::expr var_loc_before = ctx->int_const(("loc_" + prev_loc_index).c_str());
							z3::expr var_loc_after = ctx->int_const(("loc_" + curr_loc_index).c_str());
							z3::expr var_loc_time_before = ctx->int_const(("loc_time_" + prev_loc_index).c_str());
							z3::expr var_loc_time_after = ctx->int_const(("loc_time_" + curr_loc_index).c_str());
							z3::expr var_being_carried_before = ctx->int_const(("being_carried_" + prev_loc_index).c_str());
							z3::expr var_being_carried_after = ctx->int_const(("being_carried_" + curr_loc_index).c_str());

							if (this->screen>1)
							{
								cout << "\t\t drop task : " << k << " --> " << m << " :startat " << l << " : " << endl;
								cout << "\t\t Drop :: " << k-1 << ", " << m-1 << " : " << config->distances[k - 1][m - 1] << endl;
							}

							itr = find(config->tasks_start_id.begin(), config->tasks_start_id.end(), l);
							int task_weight = config->weights[distance(config->tasks_start_id.begin(), itr)];

							// CONSTRAINT drop(ri, tm, j) :: Drop task at goal location
							z3::expr_vector drop_vectors(*ctx);
							drop_vectors.push_back(var_being_carried_before == i);
							drop_vectors.push_back(var_pos_after == m);
							drop_vectors.push_back(var_being_carried_after == -1);
							drop_vectors.push_back(var_pos_time_after == var_pos_time_before + config->distances[k - 1][m - 1] + 1);
							drop_vectors.push_back(var_loc_after == m);
							drop_vectors.push_back(var_loc_time_after == var_pos_time_after);
							drop_vectors.push_back(var_capacity_after == var_capacity_before + task_weight);
							drop_vectors.push_back(var_action_after == l);
							z3::expr var_drop = z3::mk_and(drop_vectors);
							Equation_per_pos.push_back(var_drop);
						}

						for (int m : config->intermediate_pos_id)
						{
							if (k == m || config->distances[k - 1][m - 1] == INT_MAX)
								continue;

							for (int l : config->tasks_start_id)
							{
								string prev_loc_index = to_string(l) + "_" + to_string(j - 1);
								string curr_loc_index = to_string(l) + "_" + to_string(j);
								z3::expr var_loc_before = ctx->int_const(("loc_" + prev_loc_index).c_str());
								z3::expr var_loc_after = ctx->int_const(("loc_" + curr_loc_index).c_str());
								z3::expr var_loc_time_before = ctx->int_const(("loc_time_" + prev_loc_index).c_str());
								z3::expr var_loc_time_after = ctx->int_const(("loc_time_" + curr_loc_index).c_str());
								z3::expr var_being_carried_before = ctx->int_const(("being_carried_" + prev_loc_index).c_str());
								z3::expr var_being_carried_after = ctx->int_const(("being_carried_" + curr_loc_index).c_str());

								if (this->screen > 1)
								{
									cout << "\t\t interPick : " << k << " ---> " << m << " picktask " << l << endl;
								}

								// CONSTRAINT : pick_intermediate(ri,tm,in,j):: Pick Up from Intermediate location eq1
								vector<int>::iterator itr = find(config->tasks_start_id.begin(), config->tasks_start_id.end(), l);
								int task_weight = config->weights[distance(config->tasks_start_id.begin(), itr)];

								z3::expr_vector pick_inter_vectors(*ctx);
								pick_inter_vectors.push_back(var_loc_before == m);
								pick_inter_vectors.push_back(var_loc_time_before <= var_pos_time_before + config->distances[k - 1][m - 1] + 1);
								pick_inter_vectors.push_back(var_pos_after == m);
								pick_inter_vectors.push_back(var_being_carried_after == i);
								pick_inter_vectors.push_back(var_pos_time_after == var_pos_time_before + config->distances[k - 1][m - 1] + 1);
								pick_inter_vectors.push_back(var_loc_after == -1);
								pick_inter_vectors.push_back(var_loc_time_after == var_loc_time_before);
								pick_inter_vectors.push_back(var_action_after == l);
								pick_inter_vectors.push_back(var_capacity_before >= task_weight);
								pick_inter_vectors.push_back(var_capacity_after == var_capacity_before - task_weight);
								z3::expr var_pick_inter = z3::mk_and(pick_inter_vectors);

								Equation_per_pos.push_back(var_pick_inter);

								if (this->screen>1)
								{
									cout << "\t\t waitInter : " << k << " ---> " << m << " waitTask " << l << endl;
								}

								// CONSTRAINT wait_intermediate(ri,tm,in, j):: Pick Up  from Intermediate location eq2
								z3::expr_vector wait_inter_vectors(*ctx);
								wait_inter_vectors.push_back(var_loc_before == m);
								wait_inter_vectors.push_back(var_loc_time_before > var_pos_time_before + config->distances[k - 1][m - 1] + 1);
								wait_inter_vectors.push_back(var_pos_after == m);
								wait_inter_vectors.push_back(var_being_carried_after == i);
								wait_inter_vectors.push_back(var_pos_time_after == var_loc_time_before + 2);
								wait_inter_vectors.push_back(var_loc_after == -1);
								wait_inter_vectors.push_back(var_loc_time_after == var_loc_time_before + 2);
								wait_inter_vectors.push_back(var_action_after == l);
								wait_inter_vectors.push_back(var_capacity_before >= task_weight);
								wait_inter_vectors.push_back(var_capacity_after == var_capacity_before - task_weight);
								z3::expr var_wait_inter = z3::mk_and(wait_inter_vectors);

								Equation_per_pos.push_back(var_wait_inter);

								if (this->screen>1)
								{
									cout << "\t\t dropInter : " << k << " ---> " << m << " waitTask " << l << endl;
								}

								// CONSTRAINT drop_intermediate(ri,tm,in, j) :: DROP OFF at intermediate location
								z3::expr_vector drop_inter_vectors(*ctx);
								drop_inter_vectors.push_back(var_being_carried_before == i);
								drop_inter_vectors.push_back(var_pos_after == m);
								drop_inter_vectors.push_back(var_being_carried_after == -1);
								drop_inter_vectors.push_back(var_pos_time_after == var_pos_time_before + config->distances[k - 1][m - 1] + 1);
								drop_inter_vectors.push_back(var_loc_after == m);
								drop_inter_vectors.push_back(var_loc_time_after == var_pos_time_before + config->distances[k - 1][m - 1] + 1);
								drop_inter_vectors.push_back(var_capacity_after == var_capacity_before + task_weight);
								drop_inter_vectors.push_back(var_action_after == l);
								z3::expr var_drop_inter = z3::mk_and(drop_inter_vectors);

								Equation_per_pos.push_back(var_drop_inter);
							}
						}
					}

					z3::expr_vector movements_vector(*ctx);
					movements_vector.push_back(var_pos_before == k);
					movements_vector.push_back(z3::mk_or(Equation_per_pos));
					Equation_movements.push_back(z3::mk_and(movements_vector));
				}

				Equation_move_per_Z.push_back(z3::mk_or(Equation_movements));
			}
			Equation.push_back(z3::mk_and(Equation_move_per_Z));
		}
		this->smt->add(z3::mk_and(Equation));

		// -----------------------------------------------------------------------------------------------------------------------
		// CONSTRAINT :: Equation to ensure that task objects doesn't move on their own
		z3::expr_vector Equation_steady(*ctx);
		for (int i : config->tasks_start_id)
		{
			for (int j = 1; j < this->Z; j++)
			{
				z3::expr_vector Equation_per_agent(*ctx);

				string prev_loc_index = to_string(i) + "_" + to_string(j - 1);
				string curr_loc_index = to_string(i) + "_" + to_string(j);

				z3::expr var_loc_before = ctx->int_const(("loc_" + prev_loc_index).c_str());
				z3::expr var_loc_after = ctx->int_const(("loc_" + curr_loc_index).c_str());
				z3::expr var_loc_time_before = ctx->int_const(("loc_time_" + prev_loc_index).c_str());
				z3::expr var_loc_time_after = ctx->int_const(("loc_time_" + curr_loc_index).c_str());
				z3::expr var_being_carried_before = ctx->int_const(("being_carried_" + prev_loc_index).c_str());
				z3::expr var_being_carried_after = ctx->int_const(("being_carried_" + curr_loc_index).c_str());

				for (int k : config->init_pos_id)
				{
					z3::expr var_action_before = ctx->int_const(("action_" + to_string(k) + "_" + to_string(j - 1)).c_str());
					z3::expr var_action_after = ctx->int_const(("action_" + to_string(k) + "_" + to_string(j)).c_str());
					Equation_per_agent.push_back(var_action_after != i);
				}

				z3::expr_vector Equation_true(*ctx);
				Equation_true.push_back(var_loc_after == var_loc_before);
				Equation_true.push_back(var_being_carried_after == var_being_carried_before);
				Equation_true.push_back(var_loc_time_after == var_loc_time_before);

				Equation_steady.push_back(
						// z3::implies(
						// 		z3::mk_and(Equation_per_agent),
						// 		z3::mk_and(Equation_true))
						z3::ite(
						    z3::mk_and(Equation_per_agent),
						    z3::mk_and(Equation_true),
						    ctx->bool_val(true)
						)
				);
			}
		}
		this->smt->add(z3::mk_and(Equation_steady));

		// -----------------------------------------------------------------------------------------------------------------------
		// CONSTRAINT :: Equation to ensure that tasks are at their goal locations and deadlines
		z3::expr_vector Equation_ensure(*ctx);
		for (int i : config->tasks_start_id)
		{
			vector<int>::iterator itr = find(config->tasks_start_id.begin(), config->tasks_start_id.end(), i);
			int end_id = config->tasks_end_id[distance(config->tasks_start_id.begin(), itr)];
			int deadline = config->deadlines[distance(config->tasks_start_id.begin(), itr)];

			z3::expr var_loc = ctx->int_const(("loc_" + to_string(i) + "_" + to_string(this->Z - 1)).c_str());
			z3::expr var_loc_time = ctx->int_const(("loc_time_" + to_string(i) + "_" + to_string(this->Z - 1)).c_str());
			Equation_ensure.push_back(var_loc == end_id);
			Equation_ensure.push_back(var_loc_time <= deadline);
		}
		this->smt->add(z3::mk_and(Equation_ensure));
		// -----------------------------------------------------------------------------------------------------------------------
	}

	void add_optimizer()
	{
		if (this->mode == 1)
		{
			z3::expr minValue = ctx->int_const("minValue");
			this->smt->add(minValue == INT_MIN);

			vector<z3::expr> vectorCostValues;
			vectorCostValues.push_back(minValue);
			for (int m : config->init_pos_id)
			{
				z3::expr v_pos_time = ctx->int_const(("pos_time_" + to_string(m) + "_" + to_string(this->Z - 1)).c_str());
				vectorCostValues.push_back(z3::max(vectorCostValues.back(), v_pos_time));
			}
			z3::expr makespan = ctx->int_const("costx");
			this->smt->add(makespan == vectorCostValues.back());
		}
		else if (this->mode == 2)
		{
			z3::expr zeroValue = ctx->int_const("minValue");
			this->smt->add(zeroValue == 0);

			vector<z3::expr> vectorCostValues;
			vectorCostValues.push_back(zeroValue);
			for (int m : config->init_pos_id)
			{
				z3::expr v_pos_time = ctx->int_const(("pos_time_" + to_string(m) + "_" + to_string(this->Z - 1)).c_str());
				vectorCostValues.push_back(vectorCostValues.back() + v_pos_time);
			}
			z3::expr total_cost = ctx->int_const("costx");
			this->smt->add(total_cost == vectorCostValues.back());

			// z3::expr_vector allCostsVector(*ctx);
			// for (int m : config->init_pos_id)
			// {
			// 	z3::expr v_pos_time = ctx->int_const(("pos_time_" + to_string(m) + "_" + to_string(this->Z - 1)).c_str());
			// 	allCostsVector.push_back(v_pos_time);
			// }
			// z3::expr total_cost = ctx->int_const("costx");
			// this->smt->add(total_cost == z3::sum(allCostsVector));
		}
	}

	// SMT Optimization Functions
	int check_solver(int timeout = 1800)
	{
		clock_t start_time = this->get_time();
		unsigned int remaining_time = timeout * 1000;
		this->smt->set(v_timeout.c_str(), remaining_time);
		if (this->smt->check() == z3::sat)
		{
			if(this->screen > 0)
				cout << "Satisfiable :: Time Elapsed " << this->get_elapsed_time(start_time) << endl;
			const z3::model &c_model = this->smt->get_model();
			curr_model = make_shared<z3::model>(c_model);
			return 1;
		}
		if(this->screen)
			cout << "UnSatisfiable :: Time Elapsed" << this->get_elapsed_time(start_time) << endl;
		return 0;
	}

	int check_opt_solver(int lower_bound, int upper_bound, int timeout = 1800)
	{
		int status = 0;
		int curr_cost = upper_bound;
		clock_t start_time = this->get_time();
		z3::expr total_cost = ctx->int_const("costx");

		if (this->screen > 0) 
			cout << "-->\n";

		while (lower_bound <= upper_bound)
		{
			int elapsed_time = this->get_elapsed_time(start_time);
			if (elapsed_time > timeout) {
				if(this->screen>0)
					cout << "Timeout for Task Planner :: " << elapsed_time << "\n";
				return 2;
			}

			int mid = int(ceil(lower_bound + upper_bound) / 2);
			this->smt->push();

			this->smt->add(total_cost >= lower_bound);
			this->smt->add(total_cost <= mid);
			unsigned int remaining_time = (timeout - elapsed_time) * 1000;
			if (this->screen > 0)
				cout << "\tL:M:U ::: "
					<< lower_bound << ":" << mid << ":" << upper_bound
					<< " Time Left : " << remaining_time / 1000 << "s\n";
			this->smt->set(v_timeout.c_str(), remaining_time);

			if (this->smt->check() == z3::sat)
			{
				const z3::model &c_model = this->smt->get_model();
				curr_model = make_shared<z3::model>(c_model);
				curr_cost = c_model.eval(total_cost).get_numeral_int();
				upper_bound = curr_cost - 1;
				status = 1;
				if(this->screen>0) cout << " :: SAT " << curr_cost << endl;
			}
			else
			{
				lower_bound = mid + 1;
				if(this->screen>0) cout << " :: UNSAT\n";
			}
			this->smt->pop();
		}
		return status;
	}

	// Functions to avoid redundant plans. Adding exclusions.
	void compute_choices(vector<vector<int>> plans, vector<int> &arr, int i, int j, int rem_len)
	{
		if (j == (int)plans[i].size() - 1)
		{
			vector<int> n_arr(arr);
			n_arr.insert(n_arr.end(), rem_len + 1, plans[i][j]);
			curr_plans.push_back(n_arr);
		}
		else
		{
			for (int t = 0; t < rem_len + 1; ++t)
			{
				vector<int> n_arr(arr);
				n_arr.insert(n_arr.end(), t + 1, plans[i][j]);
				this->compute_choices(plans, n_arr, i, j + 1, rem_len - t);
			}
		}
	}

	vector<vector<vector<int>>> compute_all_choices(vector<vector<int>> plans)
	{
		vector<vector<vector<int>>> total_sol;
		for(unsigned int i=0; i < plans.size(); i++) {
			this->curr_plans.clear();
			vector<int> arr;
			this->compute_choices(plans, arr, i, 0, this->Z - plans[i].size());
			total_sol.push_back(this->curr_plans);
		}
		return total_sol;
	}

	void generate_exclusions(vector<vector<vector<int>>> &total_sol, vector<int> &indexes, int i_next)
	{
		if(i_next == (int)total_sol.size()) {
			vector<vector<int>> sol;
			for(unsigned int i=0; i<indexes.size(); i++) {
				int j = indexes[i];
				sol.push_back(total_sol[i][j]);
			}

			z3::expr_vector exclusions(*ctx);
			for(int i : config->init_pos_id) {
				for(int j=0; j<this->Z; j++) {
					z3::expr v_pos = ctx->int_const( ("pos_" + to_string(i) + "_" + to_string(j) ).c_str() );
					exclusions.push_back( v_pos != sol[i-1][j] );
				}
			}
			this->smt->add( z3::mk_or(exclusions) );
			// cout << "###############################\n";
			// cout << i_next << " : " << indexes << endl;
			// cout << z3::mk_or(exclusions) << endl;
			// cout << "###############################\n";
		} else {
			for(unsigned int i=0; i<total_sol[i_next].size(); i++) {
				indexes.push_back(i);
				this->generate_exclusions(total_sol, indexes, i_next + 1);
				indexes.pop_back();
			}
		}
	}

	void add_exclusion(z3::model &model) {
		// z3::model model = *(this->curr_model.get());
		vector<vector<int>> goals;
		for(int i : config->init_pos_id) {
			vector<int> goal;
			for(int j=0; j<this->Z; j++) {
				z3::expr v_expr = ctx->int_const(("pos_" + to_string(i) + "_" + to_string(j)).c_str());
				int ipos = model.eval(v_expr).get_numeral_int();
				if(j==0 || ipos != goal.back()) {
					goal.push_back(ipos);
				}
			}
			goals.push_back(goal);
		}
		vector<vector<vector<int>>> total_sol = this->compute_all_choices(goals);
		// cout << "-------------------------------------------\nGoals \n";
		// for(auto goals : total_sol) {
		// 	for(auto goal : goals) {
		// 		cout << goal << "  ";
		// 	}
		// 	cout << endl;
		// }
		// cout << "-------------------------------------------\n";
		vector<int> indexes;
		this->generate_exclusions(total_sol, indexes, 0);
	}

	// Z3 functions to extract model informations
	void print_model_info(z3::model &model, string print_option="all"){
		cout << "--------------------------------INFORMATION------------------------------\n";
		cout << "Plan Cost :: " << model.eval( ctx->int_const("costx") ).get_numeral_int() << endl;
		if(print_option == "all" || print_option == "pos") {
			for(int i : config->init_pos_id) {
				for(int j=0; j<this->Z; j++) {
					string curr_index = to_string(i) + "_" + to_string(j);
					z3::expr ipos 		= ctx->int_const(("pos_" + curr_index).c_str());
					z3::expr iaction 	= ctx->int_const(("action_" + curr_index).c_str());
					z3::expr ipostime 	= ctx->int_const(("pos_time_" + curr_index).c_str());
					cout << "-->> " << i << "-" << j
						 <<	"\t: pos = " << model.eval(ipos).get_numeral_int()
						 << "\t: pos_time = " << model.eval(ipostime).get_numeral_int()
						 << "\t: action = " << model.eval(iaction).get_numeral_int() << endl;
				}
			}
		}
		cout << "---------------\n";
		if(print_option == "all" || print_option == "loc") {
			for(int i : config->tasks_start_id) {
				for(int j=0; j<this->Z; j++) {
					string curr_index = to_string(i) + "_" + to_string(j);
					z3::expr iloc 		= ctx->int_const(("loc_" + curr_index).c_str());
					z3::expr iloctime 	= ctx->int_const(("loc_time_" + curr_index).c_str());
					z3::expr ibc 		= ctx->int_const(("being_carried_" + curr_index).c_str());
					cout << "-->> " << i << "-" << j
						 <<	"\t: loc = " << model.eval(iloc).get_numeral_int()
						 << "\t: loc_time = " << model.eval(iloctime).get_numeral_int()
						 << "\t: being_carried = " << model.eval(ibc).get_numeral_int() << endl;
				}
			}
		}
		cout << "-------------------------------------------------------------------------\n";
	}
	
	bool get_assignment(z3::model &model, PathPlannerConfig *plan) {
		// Update as required by new task planner
		// clearOld values
		plan->start_locations.clear();
		plan->goal_locations.clear();
		plan->deadlines.clear();
		plan->temp_constraints.clear();
		plan->states.clear();

		// Create and Initialize a map for <IntermediatePos, Tasks, Events>
		boost::unordered_map<int, boost::unordered_map<int, 
			std::priority_queue<IntermediateEvent, vector<IntermediateEvent>, CompareTime>>> intermediateEvents;
		for(int i: this->config->intermediate_pos_id) {
			intermediateEvents[i] = boost::unordered_map<int, 
				std::priority_queue<IntermediateEvent, vector<IntermediateEvent>, CompareTime>>();
			for(int j: this->config->tasks_start_id) {
				intermediateEvents[i][j] = 
					priority_queue<IntermediateEvent, vector<IntermediateEvent>, CompareTime>();
			}		 
		}

		// Start filling fresh
		int counter = -1;
		for(int i: config->init_pos_id) {
			counter += 1;
			vector<int> start_location;
			vector<vector<int>> goal_location;
			vector<int> deadline;
			vector<RobotState> state;

			boost::unordered_set<int> check_set;

			// Add entries for Start Index into Goal Index as well;
			string curr_index = to_string(i) + "_" + to_string(0);
			z3::expr ipos 		= ctx->int_const(("pos_" + curr_index).c_str());
			z3::expr iaction 	= ctx->int_const(("action_" + curr_index).c_str());
			z3::expr ipostime 	= ctx->int_const(("pos_time_" + curr_index).c_str());
			int curr_pos 		= model.eval(ipos).get_numeral_int();
			int curr_postime 	= model.eval(ipostime).get_numeral_int();
			int curr_action 	= model.eval(iaction).get_numeral_int()	;

			start_location 		= config->getLocationFromId(curr_pos);
			goal_location.push_back( config->getLocationFromId(curr_pos) );
			deadline.push_back(-1);
			state.push_back(RobotState::START);
			int prev_pos = curr_pos;
			if(this->screen > 2) {
				std::cout << i << " " << 0 <<  " -> " << curr_pos << ", " << curr_postime << ", " << curr_action << endl;
			}

			for(int j=1; j<this->Z; j++) {
				string curr_index = to_string(i) + "_" + to_string(j);
				ipos 		= ctx->int_const(("pos_" + curr_index).c_str());
				iaction 	= ctx->int_const(("action_" + curr_index).c_str());
				ipostime 	= ctx->int_const(("pos_time_" + curr_index).c_str());
				curr_pos 		= model.eval(ipos).get_numeral_int();
				curr_postime 	= model.eval(ipostime).get_numeral_int();
				curr_action 	= model.eval(iaction).get_numeral_int()	;



				if(curr_action != -1) {
					if(curr_pos != prev_pos) {
						goal_location.push_back( config->getLocationFromId(curr_pos) );   	// Reach the location
						deadline.push_back(-1);
						if(config->intermediate_pos_set.find(curr_pos) != config->intermediate_pos_set.end()) {
							state.push_back(RobotState::INTER_MOVE);
						} else {
							state.push_back(RobotState::MOVE);
						}
					}
					if(check_set.find(curr_action) != check_set.end()) {
						check_set.erase(curr_action);
						goal_location.push_back( config->getLocationFromId(curr_pos) );	// Drop the task
						if(config->intermediate_pos_set.find(curr_pos) != config->intermediate_pos_set.end()) {
							deadline.push_back(-1);
							state.push_back(RobotState::INTER_DROP);
							intermediateEvents[ curr_pos ][ curr_action ].push( 
								IntermediateEvent(curr_postime, counter, goal_location.size() - 1, RobotState::INTER_DROP)
							);
						} else {
							deadline.push_back(config->getDeadline(curr_action));
							state.push_back(RobotState::DROP);
						}
					} else {
						check_set.insert(curr_action);
						goal_location.push_back( config->getLocationFromId(curr_pos) );	// Pick up the task
						deadline.push_back(-1);
						if(config->intermediate_pos_set.find(curr_pos) != config->intermediate_pos_set.end()) {
							state.push_back(RobotState::INTER_PICK);
							intermediateEvents[ curr_pos ][ curr_action ].push( 
								IntermediateEvent(curr_postime, counter, goal_location.size() - 1, RobotState::INTER_PICK)
							);
						} else {
							state.push_back(RobotState::PICKUP);
						}
					}

				} 
				if(curr_action == -1  && curr_pos != prev_pos) {
					check_set.insert(curr_action);											
					goal_location.push_back( config->getLocationFromId(curr_pos) );		// Finish the task
					deadline.push_back(-1);
					state.push_back(RobotState::FINISH);
				}	

				prev_pos = curr_pos;
				if(this->screen > 2) {
					std::cout << i << " " << j <<  " -> " << curr_pos << ", " << curr_postime << ", " << curr_action << endl;
				}
			}
			if(this->screen > 2) {
				cout << "\n";
			}

			plan->start_locations.push_back(start_location);
			plan->goal_locations.push_back(goal_location);
			plan->deadlines.push_back(deadline);
			plan->states.push_back(state);
		}
		plan->opt_metric = model.eval(ctx->int_const("costx")).get_numeral_int();
		plan->temp_constraints.clear();
		for(auto fmap : intermediateEvents) {
			for(auto smap : fmap.second) {
				auto q = smap.second;
				if(q.size() % 2 != 0) {
					return 1;
				}
				while(!q.empty()) {
					auto dropEvent = q.top();
					q.pop();
					auto pickEvent = q.top();
					q.pop();
					plan->temp_constraints.push_back( 
						{dropEvent.robot_idx, dropEvent.goal_index, pickEvent.robot_idx, pickEvent.goal_index - 1});
				}
			}
		}

		if(this->screen > 1) {
			cout << " \"start_locations\" : [";
			for(auto sloc : plan->start_locations) {
				cout << "[" << sloc[0] << ", " << sloc[1] << "], ";
			}
			cout << "]\n \"goal_locations\" : [\n";
			for(auto gloc : plan->goal_locations) {
				cout << "\t[";
				for(auto sloc:gloc) {
					cout << "[" << sloc[0] << ", " << sloc[1] << "], ";
				}
				cout <<"]\n";
			}
			cout << "]\n \"deadlines\" : [\n";
			for(auto dline : plan->deadlines) {
				cout << "\t" << dline << "\n";
			}
			cout << "]\n \"states\" : [\n";
			for(auto state : plan->states) {
				cout << "\t" << state << "\n";
			}
			cout << "]\n \"temp_constraints\" : [\n";
			for(auto tcons : plan->temp_constraints) {
				cout << "\t" << tcons << endl;
			}
			cout << "]\n~~~~~~~~~~~~~~~~~~~~~~\n";
		}

		return 0;
	}

	z3::model get_model()
	{
		return *(curr_model.get());
	}

	int get_cost(z3::model &model) {
		return model.eval( ctx->int_const("costx")).get_numeral_int();
	}

	clock_t get_time() {
		return clock();
	}

	int get_elapsed_time(clock_t start) {
		return int(double(clock() - start) / CLOCKS_PER_SEC);
	}

	void push() {
		this->smt->push();
	}
    
    void pop() {
		this->smt->pop();
	}

	// ~TaskPlanner()
	// {
	//     delete ctx;
	//     delete smt;
	//     // delete config;
	// }
};