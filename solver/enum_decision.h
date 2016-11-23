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

    class enum_item;
    class item;

    class enum_decision : public decision {
    public:
        enum_decision(solver * const s, choice * const cause, enum_item * const e);
        enum_decision(enum_decision&&) = delete;
        virtual ~enum_decision();
    private:
        enum_item * const _enum;

        bool compute_choices(std::vector<choice*>& cs) override;

        class choose_value : public choice {
        public:
            choose_value(solver * const s, ac::arith_var * const cost, enum_decision * const ep, item * const val);
            choose_value(choose_value&&) = delete;
            virtual ~choose_value();
        private:
            item * const _value;

            bool apply() override;
        };
    };
}

