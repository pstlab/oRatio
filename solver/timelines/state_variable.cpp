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

#include "state_variable.h"
#include "../../core/core.h"
#include "../../core/predicate.h"

using namespace oratio;
using namespace oratio::timelines;

state_variable::state_variable(core * const c) : crafty_type(c, c, STATE_VARIABLE) { }

state_variable::~state_variable() { }

void state_variable::predicate_defined(predicate * const p) {
    p->get_supertypes().push_back(_core->get_predicate("IntervalPredicate"));
}

std::vector<decision*> state_variable::get_inconsistencies() {
    std::vector<decision*> incs;
    return incs;
}

state_variable::state_variable_constructor::state_variable_constructor(core * const c, state_variable * const sv) : constructor(c, sv, std::vector<field*>(0)) { }

bool state_variable::state_variable_constructor::invoke(item * const i, const std::vector<item*>& exprs) {
    return true;
}

state_variable::state_variable_constructor::~state_variable_constructor() { }
