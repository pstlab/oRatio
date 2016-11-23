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

    class exct_one_propagator : public propagator {
        friend class network;
    public:
        exct_one_propagator(network * const net, const std::vector<bool_var*> vars);
        exct_one_propagator(exct_one_propagator&&) = delete;
        virtual ~exct_one_propagator();

        static bool satisfied(const std::vector <bool_var*>& vars);
        static bool unsatisfiable(const std::vector <bool_var*>& vars);
        static std::unordered_set<bool> evaluate(const std::vector <bool_var*>& vars);
    private:
        const std::vector<bool_var*> _vars;
        bool_var * const _exct_one;

        bool propagate(const var * const v) override;
        static bool_var * evaluate(network * const net, const std::vector <bool_var*>& vars);
        static std::string to_string(const std::vector<bool_var*>& vars);
    };
}

