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

#include "../ac/var.h"
#include "../ac/propagator.h"

namespace oratio {

    class solver;
    class decision;

    class choice : public ac::propagator {
        friend class solver;
        friend class decision;
    public:
        choice(solver * const s, ac::arith_var * const cost, decision * const effect);
        choice(choice&&) = delete;
        virtual ~choice();

        ac::bool_var* in_plan() const {
            return _in_plan;
        }

        ac::arith_var* cost() const {
            return _cost;
        }

        decision * const effect() const {
            return _effect;
        }

        bool add_precondition(decision * const d);

        std::vector<decision*> preconditions() const {
            return _preconditions;
        }

        double estimated_cost() const {
            return _estimated_cost;
        }

    private:
        virtual bool apply() = 0;
        bool propagate(const ac::var * const v) override;
    protected:
        solver * const _solver;
        ac::bool_var * const _in_plan;
        ac::arith_var * const _cost;
        decision * const _effect;
        double _estimated_cost = std::numeric_limits<double>::infinity();

        void update_costs(std::unordered_set<decision*>& visited);
    private:
        std::vector<decision*> _preconditions;
    };
}

