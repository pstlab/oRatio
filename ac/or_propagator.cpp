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

#include "or_propagator.h"

using namespace ac;

or_propagator::or_propagator(network * const net, const std::vector<bool_var*> vars) : propagator(net), _vars(vars), _or(evaluate(net, vars)) {
	assert(!evaluate(_vars).empty());
	for (const auto& i : _vars) {
		propagator::_vars.push_back(i);
	}
	propagator::_vars.push_back(_or);
	if (!_net->root_level()) {
		bool i = intersect(_or, evaluate(_vars));
		assert(i);
	}
}

or_propagator::~or_propagator() {
	delete _or;
}

bool or_propagator::propagate(const var * const v) {
	if (_or->singleton()) {
		if (_or->allows(true)) {
			// The constraint must be satisfied..
			std::unordered_set<bool_var*> true_or_free_vars;
			for (const auto& i : _vars) {
				if (i->allows(true)) {
					true_or_free_vars.insert(i);
				}
			}
			if (true_or_free_vars.empty()) {
				return false;
			}
			else if (true_or_free_vars.size() == 1 && !assign_val(*true_or_free_vars.begin(), true)) {
				return false;
			}
		}
		else {
			// The constraint must be not satisfied..
			for (const auto& i : _vars) {
				if (!assign_val(i, false)) {
					return false;
				}
			}
		}
	}
	else if (satisfied(_vars)) {
		// The constraint is already satisfied..
		if (!assign_val(_or, true)) {
			return false;
		}
	}
	else if (unsatisfiable(_vars)) {
		// The constraint cannot be satisfied..
		if (!assign_val(_or, false)) {
			return false;
		}
	}
	return true;
}

bool or_propagator::satisfied(const std::vector<bool_var*>& vars) {
	for (const auto& i : vars) {
		if (i->singleton() && i->allows(true)) {
			return true;
		}
	}
	return false;
}

bool or_propagator::unsatisfiable(const std::vector<bool_var*>& vars) {
	for (const auto& i : vars) {
		if (!i->singleton() || i->allows(true)) {
			return false;
		}
	}
	return true;
}

std::unordered_set<bool> or_propagator::evaluate(const std::vector<bool_var*>& vars) {
	if (satisfied(vars)) {
		return{ true };
	}
	else if (unsatisfiable(vars)) {
		return{ false };
	}
	else {
		return{ true, false };
	}
}

bool_var* or_propagator::evaluate(network * const net, const std::vector<bool_var*>& vars) {
	if (net->root_level()) {
		return new bool_var(net, to_string(vars), evaluate(vars));
	}
	else {
		return new bool_var(net, to_string(vars), { true, false });
	}
}

std::string or_propagator::to_string(const std::vector<bool_var*>& vars) {
	std::string s("(or ");
	for (const auto& var : vars) {
		s.append(var->_name);
		s.append(", ");
	}
	s.pop_back();
	s.pop_back();
	s.append(")");
	return s;
}
