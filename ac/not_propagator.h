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

#include "propagator.h"

namespace ac {

    class network;
    template<typename T>
    class enum_var;
    typedef enum_var<bool> bool_var;

    class not_propagator : public propagator {
        friend class network;
    public:
        not_propagator(not_propagator&&) = delete;
        virtual ~not_propagator();

        static std::unordered_set<bool> evaluate(const bool_var * const v);
    private:
        bool_var * const _var;
        bool_var * const _not;

        not_propagator(network * const net, bool_var * const v);
        bool propagate(const var * const v) override;
        static bool_var * evaluate(network * const net, const bool_var * const v);
    };
}

