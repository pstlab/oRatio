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

#include "solver.h"
#include "enum_decision.h"
#include "fact_decision.h"
#include "../core/atom.h"
#include "../core/type.h"
#include "goal_decision.h"
#include "disjunction_decision.h"
#include "crafty_type.h"

using namespace oratio;

solver::solver() {
    _choice = new find_solution_choice(this);
    _ctr_var = _choice->_in_plan;
    bool assert_facts = _net.add({ _choice->_in_plan });
    assert(assert_facts);
}

solver::~solver() { }

enum_item* solver::new_enum(const type * const t, const std::unordered_set<item*>& allowed_vals) {
    enum_item* ei = core::new_enum(t, allowed_vals);
    enum_decision* ed = new enum_decision(this, _choice, ei);
    if (!_choice->add_precondition(ed)) {
        std::cerr << "cannot create enum: inconsistent problem.." << std::endl;
    }
    _decision_q.push(ed);
    return ei;
}

bool solver::new_fact(atom * const a) {
    fact_decision* fd = new fact_decision(this, _choice, a);
    if (a->get_type()->_scope->fact_created(a) && _choice->add_precondition(fd)) {
        _decision_q.push(fd);
        _atom_decision.insert({ a, fd });
        return true;
    }
    else {
        std::cerr << "cannot create fact: inconsistent problem.." << std::endl;
        return false;
    }
}

bool solver::new_goal(atom * const a) {
    goal_decision* gd = new goal_decision(this, _choice, a);
    if (a->get_type()->_scope->goal_created(a) && _choice->add_precondition(gd)) {
        _decision_q.push(gd);
        _atom_decision.insert({ a, gd });
        return true;
    }
    else {
        std::cerr << "cannot create goal: inconsistent problem.." << std::endl;
        return false;
    }
}

bool solver::new_disjunction(env * const e, oratio::disjunction * const d) {
    disjunction_decision* dd = new disjunction_decision(this, _choice, e, d);
    if (_choice->add_precondition(dd)) {
        _decision_q.push(dd);
        return true;
    }
    else {
        std::cerr << "cannot create disjunction: inconsistent problem.." << std::endl;
        return false;
    }
}

bool sort_by_estimated_cost(const choice* lhs, const choice* rhs) {
    return lhs->estimated_cost() < rhs->estimated_cost();
}

bool solver::solve() {
    if (!build_planning_graph()) {
        // the problem is unsolvable..
        return false;
    }

    // we update the planning graph with the inconsistencies..
    std::vector<decision*> incs = get_inconsistencies();
    while (!incs.empty()) {
        for (const auto& d : incs) {
            if (_choice->add_precondition(d)) {
                _decision_q.push(d);
            }
            else {
                // the problem is unsolvable..
                return false;
            }
        }
        if (!build_planning_graph()) {
            // the problem is unsolvable..
            return false;
        }
        incs = get_inconsistencies();
    }

    assert(_layers.empty());
    layer* l = new layer(_choice);
    _layers.push(l);
    for (const auto& d : _choice->_preconditions) {
        assert(d->expanded());
        assert(!d->_in_plan->allows(false));
        l->_pending_decisions.push_back(d);
    }

    // the main solving loop..
main_loop:
    assert(_choice->_estimated_cost < std::numeric_limits<double>::infinity());
    while (!_layers.empty()) {
        // we clean up the pending decisions..
        std::list<decision*>::iterator it = _layers.top()->_pending_decisions.begin();
        while (it != _layers.top()->_pending_decisions.end()) {
            std::vector<choice*> allowed_choices;
            for (const auto& ch : (*it)->_choices) {
                if (ch->_in_plan->allows(true)) {
                    allowed_choices.push_back(ch);
                }
            }
            if (allowed_choices.size() == 1) {
                choice* ch = *allowed_choices.begin();
                assert(!ch->_in_plan->allows(false));
                for (const auto& d : ch->_preconditions) {
                    _layers.top()->_pending_decisions.push_back(d);
                }
                it = _layers.top()->_pending_decisions.erase(it);
            }
            else {
                it++;
            }
        }

        if (_layers.top()->_pending_decisions.empty()) {
            // Hurray!!! We have found a solution!
            return true;
        }

        // this is the best decision to take..
        decision* most_expensive_decision = nullptr;
        double d_cost = -std::numeric_limits<double>::infinity();
        for (const auto& d : _layers.top()->_pending_decisions) {
            assert(d->_in_plan->allows(true));
            if (d->_estimated_cost > d_cost) {
                most_expensive_decision = d;
                d_cost = d->_estimated_cost;
            }
        }

        // this is the best choice to take..
        choice* least_expensive_choice = nullptr;
        double ch_cost = std::numeric_limits<double>::infinity();
        for (const auto& ch : most_expensive_decision->choices()) {
            if (ch->_estimated_cost < ch_cost) {
                assert(ch->_in_plan->allows(true));
                least_expensive_choice = ch;
                ch_cost = ch->_estimated_cost;
            }
        }

        _no_good.clear();
        if (!_net.assign_true(least_expensive_choice->_in_plan)) {
            // if we cannot take the best choice we need to back-jump..
            pop(_layers.size() - _net.level());
            goto main_loop;
        }

        if (_choice->_estimated_cost == std::numeric_limits<double>::infinity()) {
            // the heuristic is gone..
            pop(_layers.size());
            while (!_net.root_level()) {
                _net.pop();
            }
            std::vector<ac::bool_var*> no_good;
            for (const auto& v : _no_good) {
                no_good.push_back(_net.negate(v));
            }

            if (!_net.add({ _net.disjunction(no_good) }) || !build_planning_graph()) {
                // the problem is unsolvable..
                return false;
            }

            assert(_layers.empty());
            layer* l = new layer(_choice);
            _layers.push(l);
            for (const auto& d : _choice->_preconditions) {
                assert(d->expanded());
                assert(!d->_in_plan->allows(false));
                l->_pending_decisions.push_back(d);
            }
            goto main_loop;
        }

        // we update the planning graph with the inconsistencies..
        incs = get_inconsistencies();
        while (!incs.empty()) {
            for (const auto& d : incs) {
                if (least_expensive_choice->add_precondition(d)) {
                    _decision_q.push(d);
                    _layers.top()->_pending_decisions.push_back(d);
                }
                else {
                    // if we cannot add a precondition we need to back-jump..
                    pop(_layers.size() - _net.level());
                    goto main_loop;
                }
            }
            if (!build_planning_graph()) {
                // the problem is unsolvable..
                return false;
            }
            incs = get_inconsistencies();
        }
        push(least_expensive_choice);
    }

    // the problem is unsolvable..
    return false;
}

bool solver::build_planning_graph() {
    choice* c = _choice;
    while (c->_estimated_cost == std::numeric_limits<double>::infinity() && !_decision_q.empty()) {
        decision* d = _decision_q.front();
        _decision_q.pop();
        if (!d->expanded()) {
            bool should_expand = true;
            decision* c_d = d;
            while (c_d->cause()->effect()) {
                if (c_d->cause()->effect()->solved()) {
                    should_expand = false;
                    break;
                }
                else {
                    c_d = c_d->cause()->effect();
                }
            }
            if (should_expand) {
                if (!d->expand()) {
                    return false;
                }
                for (const auto& ch : d->choices()) {
                    _choice = ch;
                    _ctr_var = _choice->_in_plan;
                    if (!_choice->apply()) {
                        return false;
                    }
                    if (_choice->_estimated_cost < std::numeric_limits<double>::infinity()) {
                        std::unordered_set<decision*> visited;
                        _choice->_effect->update_costs(visited);
                    }
                }
            }
            else {
                // we postpone the expansion..
                _decision_q.push(d);
            }
        }
    }

    assert(c->_estimated_cost < std::numeric_limits<double>::infinity());

    _choice = c;
    _ctr_var = _choice->_in_plan;
    return true;
}

std::vector<decision*> solver::get_inconsistencies() {
    std::set<type*> ts;
    std::queue<type*> q;
    for (const auto& t : _types) {
        q.push(t.second);
    }
    while (!q.empty()) {
        ts.insert(q.front());
        for (const auto& st : q.front()->get_supertypes()) {
            q.push(st);
        }
        q.pop();
    }

    std::vector<decision*> inconsistencies;
    for (const auto& t : ts) {
        if (crafty_type * ct = dynamic_cast<crafty_type*> (t)) {
            std::vector<decision*> incs = ct->get_inconsistencies();
            inconsistencies.insert(inconsistencies.end(), incs.begin(), incs.end());
        }
    }
    return inconsistencies;
}

void solver::push(choice * const ch) {
    layer* l = new layer(ch);
    for (const auto& d : _layers.top()->_pending_decisions) {
        l->_pending_decisions.push_back(d);
    }
    for (const auto& d : ch->_preconditions) {
        assert(d->expanded());
        assert(!d->_in_plan->allows(false));
        l->_pending_decisions.push_back(d);
    }
    _layers.push(l);
}

void solver::pop(long unsigned int n) {
    for (long unsigned int i = 0; i < n; i++) {
        for (const auto& ch : _layers.top()->_updated_choices) {
            ch.first->_estimated_cost = ch.second;
        }
        for (const auto& d : _layers.top()->_updated_decisions) {
            d.first->_estimated_cost = d.second;
        }
        _layers.pop();
    }
}

find_solution_choice::find_solution_choice(solver * const s) : choice(s, s->_net.new_real(0), nullptr) { }

find_solution_choice::~find_solution_choice() { }

bool find_solution_choice::apply() {
    return true;
}