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

    class arith_eq_propagator : public propagator {
        friend class network;
    public:
        arith_eq_propagator(arith_eq_propagator&&) = delete;
        virtual ~arith_eq_propagator();

        static std::unordered_set<bool> evaluate(arith_var * const left, arith_var * const right);
    private:
        arith_var * const _left;
        arith_var * const _right;
        bool_var * const _eq;
        z3::expr _eq_expr;

        arith_eq_propagator(network * const net, arith_var * const left, arith_var * const right);
        bool propagate(const var * const v) override;
        static bool_var * evaluate(network * const net, arith_var * const left, arith_var * const right);
        static std::string to_string(arith_var * const left, arith_var * const right);
    };
}

