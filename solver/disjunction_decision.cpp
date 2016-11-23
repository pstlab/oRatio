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

#include "disjunction_decision.h"
#include "../core/disjunction.h"
#include "solver.h"

using namespace oratio;

disjunction_decision::disjunction_decision(solver * const s, choice * const cause, env * const e, disjunction * const d) : decision(s, cause), _env(e), _disjunction(d) { }

disjunction_decision::~disjunction_decision() { }

bool disjunction_decision::compute_choices(std::vector<choice*>& cs) {
    for (const auto& c : _disjunction->get_conjunctions()) {
        cs.push_back(new choose_conjunction(_solver, _solver->_net.new_real(1), this, c));
    }
    return true;
}

disjunction_decision::choose_conjunction::choose_conjunction(solver * const s, ac::arith_var* cost, disjunction_decision * const dp, conjunction * const c) : choice(s, cost, dp), _conjunction(c) { }

disjunction_decision::choose_conjunction::~choose_conjunction() { }

bool disjunction_decision::choose_conjunction::apply() {
    return _conjunction->apply(static_cast<disjunction_decision*> (_effect)->_env);
}
