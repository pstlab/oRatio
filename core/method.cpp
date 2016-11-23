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

#include "method.h"
#include "type.h"
#include "field.h"

using namespace oratio;

method::method(core * const c, scope * const s, const std::string& name, const std::vector<field*>& args, const type * const return_type) : scope(c, s), _name(name), _args(args), _return_type(return_type) {
    _fields.insert({ THIS_KEYWORD, new field(static_cast<type*> (_scope), THIS_KEYWORD, true) });
    if (return_type) {
        _fields.insert({ RETURN_KEYWORD, new field(return_type, RETURN_KEYWORD, true) });
    }
    for (const auto& arg : args) {
        _fields.insert({ arg->_name, new field(arg->_type, arg->_name) });
    }
}

method::~method() { }

