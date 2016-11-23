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

    class product_propagator : public propagator {
        friend class network;
    public:
        product_propagator(product_propagator&&) = delete;
        virtual ~product_propagator();

        static interval evaluate(const std::vector <arith_var*>& vars);
    private:
        const std::vector<arith_var*> _vars;
        arith_var * const _prod;

        product_propagator(network * const net, const std::vector<arith_var*> vars);
        bool propagate(const var * const v) override;
        static arith_var * evaluate(network * const net, const std::vector <arith_var*>& vars);
        static std::string to_string(const std::vector<arith_var*>& vars);
    };
}

