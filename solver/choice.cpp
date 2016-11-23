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

#include "choice.h"
#include "solver.h"
#include "decision.h"

using namespace oratio;

choice::choice(solver * const s, ac::arith_var * const cost, decision * const effect) : ac::propagator(&s->_net), _solver(s), _in_plan(s->_net.new_bool()), _cost(cost), _effect(effect) {
    _vars.push_back(_in_plan);
}

choice::~choice() { }

bool choice::add_precondition(decision * const d) {
    _preconditions.push_back(d);
    // if this choice is in plan, its preconditions must be in plan as well..
    bool add = _solver->_net.add({ _solver->_net.imply(_in_plan, d->_in_plan) });
    return add;
}

bool choice::propagate(const ac::var* v) {
    if (!_in_plan->allows(true)) {
        _solver->_no_good.push_back(_in_plan);
        _solver->_layers.top()->_updated_choices.insert({ this, _estimated_cost });
        _estimated_cost = std::numeric_limits<double>::infinity();
        if (_effect) {
            std::unordered_set<decision*> visited;
            _effect->update_costs(visited);
        }
    }
    return true;
}

void choice::update_costs(std::unordered_set<decision*>& visited) {
    double c_cost = -std::numeric_limits<double>::infinity();
    for (const auto& d : _preconditions) {
        if (d->_estimated_cost > c_cost) {
            c_cost = d->_estimated_cost;
        }
    }
    if (c_cost != _estimated_cost) {
        _estimated_cost = c_cost;
        if (_effect) {
            _effect->update_costs(visited);
        }
    }
}
