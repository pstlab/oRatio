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

#include "not_propagator.h"

using namespace ac;

not_propagator::not_propagator(network * const net, bool_var * const v) : propagator(net), _var(v), _not(evaluate(net, v)) {
	assert(!evaluate(_var).empty());
	_vars.push_back(_var);
	_vars.push_back(_not);
	if (!_net->root_level()) {
		bool i = intersect(_not, evaluate(_var));
		assert(i);
	}
}

not_propagator::~not_propagator() {
	delete _not;
}

bool not_propagator::propagate(const var * const v)
{
	if (_var->singleton()) {
		std::unordered_set<bool> vals = _var->to_vals();
		return complement(_not, vals);
	}
	else if (_not->singleton()) {
		std::unordered_set<bool> vals = _not->to_vals();
		return complement(_var, vals);
	}
	return true;
}

bool_var * not_propagator::evaluate(network * const net, const bool_var * const v) {
	if (net->root_level()) {
		return new bool_var(net, "!" + v->_name, evaluate(v));
	}
	else {
		return new bool_var(net, "!" + v->_name, { true, false });
	}
}

std::unordered_set<bool> not_propagator::evaluate(const bool_var * const v) {
	if (v->singleton()) {
		return{ !*v->to_vals().begin() };
	}
	else {
		return{ true, false };
	}
}