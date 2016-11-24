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

#include "fact_decision.h"
#include "solver.h"
#include "../ac/enum_eq_propagator.h"
#include "../core/atom.h"

using namespace oratio;

fact_decision::fact_decision(solver * const s, choice * const cause, atom * const a) : decision(s, cause), _atom(a) { }

fact_decision::~fact_decision() { }

bool fact_decision::compute_choices(std::vector<choice*>& cs) {
	cs.push_back(new add_fact(_solver, _solver->_net.new_real(0), this));
	return true;
}

fact_decision::add_fact::add_fact(solver * const s, ac::arith_var* cost, fact_decision * const f) : choice(s, cost, f) { }

fact_decision::add_fact::~add_fact() { }

bool fact_decision::add_fact::apply() {
	_estimated_cost = 0;
	return _solver->_net.add({ _solver->_net.eq(_in_plan, _solver->_net.eq<atom_state*>(&static_cast<fact_decision*> (_effect)->_atom->get_state(), &active)) });
}
