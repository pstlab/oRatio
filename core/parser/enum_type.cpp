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

#include "enum_type.h"
#include "../core.h"

using namespace oratio;

enum_type::enum_type(core * const c, scope * const s, const std::string& name) : type(c, s, name) { }

enum_type::~enum_type() { }

std::vector<string_item*> enum_type::get_enums() {
    std::vector<string_item*> enums;
    for (const auto& i : _instances) {
        enums.push_back(static_cast<string_item*> (i));
    }
    for (const auto& e : _enums) {
        std::vector<string_item*> c_enums = e->get_enums();
        for (const auto& i : c_enums) {
            enums.push_back(i);
        }
    }
    return enums;
}

item* enum_type::new_instance(env * const e) {
    std::unordered_set<item*> enums(get_enums().begin(), get_enums().end());
    return _core->new_enum(this, enums);
}

void enum_type::add_enum(string_item * const i) {
    _instances.insert(i);
}
