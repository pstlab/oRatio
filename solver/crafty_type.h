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

#include "../core/type.h"

namespace oratio {

    class decision;

    class crafty_type : public type {
    public:
        crafty_type(core * const c, scope * const s, const std::string& name);
        crafty_type(const crafty_type& orig);
        virtual ~crafty_type();

        virtual std::vector<decision*> get_inconsistencies() = 0;
    };
}

