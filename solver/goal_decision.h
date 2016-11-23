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

#include "decision.h"
#include "choice.h"

namespace oratio {

    class atom;

    class goal_decision : public decision {
    public:
        goal_decision(solver * const s, choice * const cause, atom * const a);
        goal_decision(goal_decision&&) = delete;
        virtual ~goal_decision();
    private:
        atom * const _atom;

        bool compute_choices(std::vector<choice*>& cs) override;

        class expand_choice : public choice {
        public:

            expand_choice(solver * const s, ac::arith_var * const cost, goal_decision * const g);
            expand_choice(expand_choice&&) = delete;
            virtual ~expand_choice();
        private:
            bool apply() override;
        };

        class unify_choice : public choice {
            friend class goal_decision;
        public:

            unify_choice(solver * const s, ac::arith_var * const cost, goal_decision * const g, atom * const a);
            unify_choice(unify_choice&&) = delete;
            virtual ~unify_choice();
        private:
            atom * const _atom;
            ac::bool_var * const _eq;

            bool apply() override;
            ac::bool_var * evaluate();
        };
    };
}

