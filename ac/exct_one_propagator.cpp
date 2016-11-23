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

#include "exct_one_propagator.h"

using namespace ac;

exct_one_propagator::exct_one_propagator(network * const net, const std::vector<bool_var*> vars) : propagator(net), _vars(vars), _exct_one(evaluate(net, vars)) {
	assert(!evaluate(_vars).empty());
	for (const auto& i : _vars) {
		propagator::_vars.push_back(i);
	}
	propagator::_vars.push_back(_exct_one);
	if (!_net->root_level()) {
		bool i = intersect(_exct_one, evaluate(_vars));
		assert(i);
	}
}

exct_one_propagator::~exct_one_propagator() {
	delete _exct_one;
}

bool exct_one_propagator::propagate(const var * const v) {
	if (_exct_one->singleton()) {
		int n_trues = 0;
		std::vector<bool_var*> free_vars;
		for (const auto& i : _vars) {
			if (i->singleton()) {
				if (i->allows(true)) {
					n_trues++;
				}
			}
			else {
				free_vars.push_back(i);
			}
		}
		if (_exct_one->allows(true)) {
			// The constraint must be satisfied..
			switch (n_trues) {
			case 0:
				if (free_vars.size() == 1 && !assign_val(*free_vars.begin(), true)) {
					return false;
				}
				break;
			case 1:
				for (const auto& i : free_vars) {
					if (!assign_val(i, false)) {
						return false;
					}
				}
				break;
			default:
				return false;
			}
		}
		else if (free_vars.size() == 1) {
			// The constraint must be not satisfied.. 
			switch (n_trues) {
			case 0:
				if (!assign_val(*free_vars.begin(), false)) {
					return false;
				}
				break;
			case 1:
				if (!assign_val(*free_vars.begin(), true)) {
					return false;
				}
				break;
			default:
				return true;
			}
		}
	}
	else if (satisfied(_vars)) {
		// The constraint is already satisfied..
		if (!assign_val(_exct_one, true)) {
			return false;
		}
	}
	else if (unsatisfiable(_vars)) {
		// The constraint cannot be satisfied..
		if (!assign_val(_exct_one, false)) {
			return false;
		}
	}
	return true;
}

bool exct_one_propagator::satisfied(const std::vector<bool_var*>& vars) {
	int n_trues = 0;
	for (const auto& i : vars) {
		if (i->singleton()) {
			if (i->allows(true)) {
				if (n_trues == 0) {
					n_trues++;
				}
				else {
					return false;
				}
			}
		}
		else {
			return false;
		}
	}
	return n_trues == 1;
}

bool exct_one_propagator::unsatisfiable(const std::vector<bool_var*>& vars) {
	int n_trues = 0;
	int n_frees = 0;
	for (const auto& i : vars) {
		if (i->singleton()) {
			if (i->allows(true)) {
				n_trues++;
				if (n_trues > 1) {
					return true;
				}
			}
		}
		else {
			n_frees++;
		}
	}
	return n_trues == 0 && n_frees == 0;
}

std::unordered_set<bool> exct_one_propagator::evaluate(const std::vector<bool_var*>& vars) {
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

bool_var* exct_one_propagator::evaluate(network * const net, const std::vector<bool_var*>& vars) {
	if (net->root_level()) {
		return new bool_var(net, to_string(vars), evaluate(vars));
	}
	else {
		return new bool_var(net, to_string(vars), { true, false });
	}
}

std::string exct_one_propagator::to_string(const std::vector<bool_var*>& vars) {
	std::string s("(exct-one ");
	for (const auto& var : vars) {
		s.append(var->_name);
		s.append(", ");
	}
	s.pop_back();
	s.pop_back();
	s.append(")");
	return s;
}
