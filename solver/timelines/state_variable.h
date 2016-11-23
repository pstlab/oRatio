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

#include "../crafty_type.h"
#include "../../core/constructor.h"

#define STATE_VARIABLE "StateVariable"

namespace oratio {

    namespace timelines {

        class state_variable : public crafty_type {
        public:
            state_variable(core * const c);
            state_variable(state_variable&&) = delete;
            virtual ~state_variable();

            void predicate_defined(predicate * const p) override;
            std::vector<decision*> get_inconsistencies() override;
        private:

            class state_variable_constructor : public constructor {
            public:
                state_variable_constructor(core * const c, state_variable * const sv);
                state_variable_constructor(state_variable_constructor&&) = delete;
                virtual ~state_variable_constructor();
            private:
                bool invoke(item * const i, const std::vector<item*>& exprs) override;
            };
        };
    }
}

