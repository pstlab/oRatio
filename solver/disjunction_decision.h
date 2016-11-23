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

    class env;
    class disjunction;
    class conjunction;

    class disjunction_decision : public decision {
    public:
        disjunction_decision(solver * const s, choice * const cause, env * const e, disjunction * const d);
        disjunction_decision(disjunction_decision&&) = delete;
        virtual ~disjunction_decision();
    private:
        env * const _env;
        disjunction * const _disjunction;

        bool compute_choices(std::vector<choice*>& cs) override;

        class choose_conjunction : public choice {
        public:
            choose_conjunction(solver * const s, ac::arith_var * const cost, disjunction_decision * const dp, conjunction * const c);
            choose_conjunction(choose_conjunction&&) = delete;
            virtual ~choose_conjunction();
        private:
            conjunction * const _conjunction;

            bool apply() override;
        };
    };
}

