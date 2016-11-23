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

#include "and_propagator.h"

using namespace ac;

and_propagator::and_propagator(network * const net, const std::vector<bool_var*> vars) : propagator(net), _vars(vars), _and(evaluate(net, vars)) {
    assert(!evaluate(_vars).empty());
    for (const auto& i : _vars) {
        propagator::_vars.push_back(i);
    }
    propagator::_vars.push_back(_and);
    if (!_net->root_level()) {
        bool i = intersect(_and, evaluate(_vars));
        assert(i);
    }
}

and_propagator::~and_propagator() {
    delete _and;
}

bool and_propagator::propagate(const var * const v) {
    if (_and->singleton()) {
        if (_and->allows(true)) {
            // The constraint must be satisfied..
            for (const auto& i : _vars) {
                if (!assign_val(i, true)) {
                    return false;
                }
            }
        }
        else {
            // The constraint must be not satisfied..
            std::unordered_set<bool_var*> false_or_free_vars;
            for (const auto& i : _vars) {
                if (i->allows(false)) {
                    false_or_free_vars.insert(i);
                }
            }
            if (false_or_free_vars.empty()) {
                return false;
            }
            else if (false_or_free_vars.size() == 1 && !assign_val(*false_or_free_vars.begin(), false)) {
                return false;
            }
        }
    }
    else if (satisfied(_vars)) {
        // The constraint is already satisfied..
        if (!assign_val(_and, true)) {
            return false;
        }
    }
    else if (unsatisfiable(_vars)) {
        // The constraint cannot be satisfied..
        if (!assign_val(_and, false)) {
            return false;
        }
    }
    return true;
}

bool and_propagator::satisfied(const std::vector<bool_var*>& vars) {
    for (const auto& i : vars) {
        if (!i->singleton() || i->allows(false)) {
            return false;
        }
    }
    return true;
}

bool and_propagator::unsatisfiable(const std::vector<bool_var*>& vars) {
    for (const auto& i : vars) {
        if (i->singleton() && i->allows(false)) {
            return true;
        }
    }
    return false;
}

std::unordered_set<bool> and_propagator::evaluate(const std::vector<bool_var*>& vars) {
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

bool_var* and_propagator::evaluate(network * const net, const std::vector<bool_var*>& vars) {
    if (net->root_level()) {
        return new bool_var(net, to_string(vars), evaluate(vars));
    }
    else {
        return new bool_var(net, to_string(vars), { true, false });
    }
}

std::string and_propagator::to_string(const std::vector<bool_var*>& vars) {
    std::string s("(and ");
    for (const auto& var : vars) {
        s.append(var->_name);
        s.append(", ");
    }
    s.pop_back();
    s.pop_back();
    s.append(")");
    return s;
}
