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

    class fact_decision : public decision {
    public:
        fact_decision(solver * const s, choice * const cause, atom * const a);
        fact_decision(fact_decision&&) = delete;
        virtual ~fact_decision();
    private:
        atom * const _atom;

        bool compute_choices(std::vector<choice*>& cs) override;

        class add_fact : public choice {
        public:
            add_fact(solver * const s, ac::arith_var * const cost, fact_decision * const f);
            add_fact(add_fact&&) = delete;
            virtual ~add_fact();
        private:
            bool apply() override;
        };
    };
}

