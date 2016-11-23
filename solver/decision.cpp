/*
 * Copyright (C) 2016 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "decision.h"
#include "solver.h"
#include "choice.h"

using namespace oratio;

decision::decision(solver * const s, choice * const cause) : ac::propagator(&s->_net), _solver(s), _cause(cause), _in_plan(s->_net.new_bool()) {
	_vars.push_back(_in_plan);
}

decision::~decision() { }

bool decision::expand() {
	assert(!_expanded);

	bool solved = compute_choices(_choices);
	_expanded = true;

	if (_choices.empty()) {
		assert(!solved);
		if (!_solver->_net.add({ _solver->_net.negate(_in_plan) })) {
			return false;
		}
	}
	else if (_choices.size() == 1) {
		if (!_solver->_net.add({ _solver->_net.imply(_in_plan, (*_choices.begin())->_in_plan) })) {
			return false;
		}
	}
	else {
		std::vector<ac::bool_var*> vars;
		for (const auto& ch : _choices) {
			vars.push_back(ch->_in_plan);
		}
		if (!_solver->_net.add({ _solver->_net.imply(_in_plan, _solver->_net.exactly_one(vars)) })) {
			return false;
		}
	}
	return true;
}

bool decision::propagate(const ac::var* v) {
	if (!_in_plan->allows(true)) {
		_solver->_layers.top()->_updated_decisions.insert({ this, _estimated_cost });
		_estimated_cost = std::numeric_limits<double>::infinity();
		if (_cause) {
			std::unordered_set<decision*> visited;
			_cause->update_costs(visited);
		}
	}
	return true;
}

void decision::update_costs(std::unordered_set<decision*>& visited) {
	if (visited.find(this) == visited.end()) {
		visited.insert(this);
		double c_cost = std::numeric_limits<double>::infinity();
		for (const auto& ch : _choices) {
			double ch_cost = ch->_cost->eval();
			if (ch->_estimated_cost + ch_cost < c_cost) {
				c_cost = ch->_estimated_cost + ch_cost;
			}
		}
		if (c_cost != _estimated_cost) {
			_estimated_cost = c_cost;
			if (_cause) {
				_cause->update_costs(visited);
			}
		}
	}
}
