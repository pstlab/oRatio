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

#include "../ac/propagator.h"

namespace oratio {

    class solver;
    class choice;

    class decision : public ac::propagator {
        friend class solver;
        friend class choice;
    public:
        decision(solver * const s, choice * const cause);
        decision(decision&&) = delete;
        virtual ~decision();

        ac::bool_var* in_plan() const {
            return _in_plan;
        }

        choice * const cause() const {
            return _cause;
        }

        bool solved() const {
            return _estimated_cost < std::numeric_limits<double>::infinity();
        }

        double estimated_cost() const {
            return _estimated_cost;
        }

        std::vector<choice*> choices() const {
            return _choices;
        }

        bool expanded() const {
            return _expanded;
        }

        bool expand();
    private:
        bool propagate(const ac::var * const v) override;
    protected:
        solver * const _solver;
        choice * const _cause;
        ac::bool_var * const _in_plan;
        double _estimated_cost = std::numeric_limits<double>::infinity();

        virtual bool compute_choices(std::vector<choice*>& cs) = 0;
        void update_costs(std::unordered_set<decision*>& visited);
    private:
        bool _expanded = false;
        std::vector<choice*> _choices;
    };
}

