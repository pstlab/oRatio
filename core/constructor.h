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

#include "scope.h"

namespace oratio {

    class env;
    class item;

    class constructor : public scope {
        friend class type;
    public:
        constructor(core * const c, scope * const s, const std::vector<field*>& args);
        constructor(constructor&&) = delete;
        virtual ~constructor();

        virtual item * new_instance(env * const e, const std::vector<item*>& exprs);
        virtual bool invoke(item * const i, const std::vector<item*>& exprs) = 0;
    protected:
        const std::vector<field*> _args;
    };
}

