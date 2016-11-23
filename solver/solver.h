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

#pragma once

#include "../core/core.h"
#include "choice.h"

namespace oratio {

    class decision;
    class choice;

    class solver : public core {
        friend class choice;
        friend class decision;
        friend class goal_decision;
    public:
        solver();
        solver(solver&&) = delete;
        virtual ~solver();

        bool solve();

        enum_item* new_enum(const type * const t, const std::unordered_set<item*>& allowed_vals) override;
        bool new_fact(atom * const a) override;
        bool new_goal(atom * const a) override;
        bool new_disjunction(env * const e, oratio::disjunction * const d) override;
    private:
        choice * _choice;
        std::queue<decision*> _decision_q;
        std::unordered_map<atom*, decision*> _atom_decision;

        std::vector<ac::bool_var*> _no_good;

        bool build_planning_graph();

        std::vector<decision*> get_inconsistencies();

        bool push(choice * const ch);
        void pop(long unsigned int n = 1);

        struct layer {

            layer(choice * const ch) : _choice(ch) { }

            choice * const _choice;
            std::list<decision*> _pending_decisions;
            std::unordered_map<choice*, double> _updated_choices;
            std::unordered_map<decision*, double> _updated_decisions;
        };
        std::stack<layer*> _layers;
    };

    class find_solution_choice : public choice {
    public:
        find_solution_choice(solver * const s);
        find_solution_choice(find_solution_choice&&) = delete;
        virtual ~find_solution_choice();
    private:
        bool apply() override;
    };
}

