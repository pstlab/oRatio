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

#include "goal_decision.h"
#include "solver.h"
#include "../ac/enum_eq_propagator.h"
#include "../core/atom.h"
#include "../core/type.h"
#include "../core/predicate.h"

using namespace oratio;

goal_decision::goal_decision(solver * const s, choice * const cause, atom * const a) : decision(s, cause), _atom(a) {
    // if in_plan, atom state is not equal to inactive..
    bool assert_facts = _solver->_net.add({
        _solver->_net.eq(_in_plan, _solver->_net.negate(_solver->_net.eq<int>(&_atom->get_state(), atom_state::inactive)))
    });
    assert(assert_facts);
}

goal_decision::~goal_decision() { }

bool goal_decision::compute_choices(std::vector<choice*>& cs) {
    bool solved = false;
    for (auto& inst : _atom->get_type()->get_instances()) {
        atom * const a = static_cast<atom * const> (inst);
        if (_atom != a && a->get_state().allows(atom_state::active) && _solver->_atom_decision.at(a)->expanded() && _atom->equates(a)) {
            // this atom is a good candidate for unification
            unify_choice* u = new unify_choice(_solver, _solver->_net.new_real(0), this, a);
            _solver->_net.push();
            if (_solver->_net.add({ u->_in_plan })) {
                // unification is actually possible!
                solved = true;
                cs.push_back(u);
                bool add_pre = u->add_precondition(_solver->_atom_decision.at(a));
                assert(add_pre);
            }
            _solver->_net.pop();
        }
    }

    cs.push_back(new expand_choice(_solver, _solver->_net.new_real(1), this));

    if (!solved) {
        // we remove unification from atom state..
        bool assert_facts = _solver->_net.add({ _solver->_net.negate(_solver->_net.eq<int>(&_atom->get_state(), atom_state::unified)) });
        assert(assert_facts);
    }

    return solved;
}

goal_decision::expand_choice::expand_choice(solver * const s, ac::arith_var* cost, goal_decision * const g) : choice(s, cost, g) { }

goal_decision::expand_choice::~expand_choice() { }

bool goal_decision::expand_choice::apply() {
    goal_decision* g = static_cast<goal_decision*> (_effect);
    return _solver->_net.add({ _solver->_net.eq(_in_plan, _solver->_net.eq<int>(&g->_atom->get_state(), atom_state::active)) }) && static_cast<const predicate*> (g->_atom->get_type())->apply_rule(g->_atom);
}

goal_decision::unify_choice::unify_choice(solver * const s, ac::arith_var* cost, goal_decision * const g, atom * const a) : choice(s, cost, g), _atom(a), _eq(evaluate()) { }

goal_decision::unify_choice::~unify_choice() { }

bool goal_decision::unify_choice::apply() {
    _estimated_cost = 0;
    return _solver->_net.add({
        _solver->_net.imply(_in_plan, _solver->_net.eq<int>(&static_cast<goal_decision*> (_effect)->_atom->get_state(), atom_state::unified)),
        _solver->_net.eq(_in_plan, _eq)
    });
}

ac::bool_var* goal_decision::unify_choice::evaluate() {
    std::vector<ac::bool_var*> vars;
    decision* d = _solver->_atom_decision.at(_atom);
    while (d) {
        assert(d->in_plan()->allows(true));
        assert(d->cause()->in_plan()->allows(true));
        if (!d->in_plan()->singleton()) {
            vars.push_back(d->in_plan());
        }
        if (!d->cause()->in_plan()->singleton()) {
            vars.push_back(d->cause()->in_plan());
        }
        d = d->cause()->effect();
    }
    d = _effect;
    while (d) {
        assert(d->in_plan()->allows(true));
        assert(d->cause()->in_plan()->allows(true));
        if (!d->in_plan()->singleton()) {
            vars.push_back(d->in_plan());
        }
        if (!d->cause()->in_plan()->singleton()) {
            vars.push_back(d->cause()->in_plan());
        }
        d = d->cause()->effect();
    }
    vars.push_back(_solver->_net.eq<int>(&static_cast<goal_decision*> (_effect)->_atom->get_state(), atom_state::unified));
    vars.push_back(_solver->_net.eq<int>(&_atom->get_state(), atom_state::active));
    vars.push_back(static_cast<goal_decision*> (_effect)->_atom->eq(_atom));
    return _solver->_net.conjunction(vars);
}
