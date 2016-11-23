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

#include "enum_decision.h"
#include "../ac/enum_eq_propagator.h"
#include "../core/item.h"
#include "solver.h"

using namespace oratio;

enum_decision::enum_decision(solver * const s, choice * const cause, enum_item * const e) : decision(s, cause), _enum(e) { }

enum_decision::~enum_decision() { }

bool enum_decision::compute_choices(std::vector<choice*>& cs) {
	std::unordered_set<item*> vals = _enum->get_enum_var().to_vals();
	for (const auto& val : vals) {
		cs.push_back(new choose_value(_solver, _solver->_net.new_real(1.0 / vals.size()), this, val));
	}
	return true;
}

enum_decision::choose_value::choose_value(solver * const s, ac::arith_var* cost, enum_decision * const ep, item * const val) : choice(s, cost, ep), _value(val) { }

enum_decision::choose_value::~choose_value() { }

bool enum_decision::choose_value::apply() {
	_estimated_cost = 0;
	return _solver->_net.add({ _solver->_net.eq(_in_plan, static_cast<enum_decision*> (_effect)->_enum->allows(_value)) });
}
