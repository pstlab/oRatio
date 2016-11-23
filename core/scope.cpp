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

#include "scope.h"

using namespace oratio;

scope::scope(core * const c, scope * const s) : _core(c), _scope(s) { }

scope::~scope() { }

field * scope::get_field(const std::string& name) const {
    const auto& f_it = _fields.find(name);
    if (f_it != _fields.end()) {
        return (*f_it).second;
    }

    // not found
    return nullptr;
}

std::unordered_map<std::string, field*> scope::get_fields() const {
    return _fields;
}