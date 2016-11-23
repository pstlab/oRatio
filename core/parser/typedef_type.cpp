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

#include "typedef_type.h"
#include "../core.h"
#include "expression_visitor.h"

using namespace oratio;

typedef_type::typedef_type(core * const c, scope * const s, const std::string& name, type * const base_type, oRatioParser::ExprContext * const expr) : type(c, s, name), _base_type(base_type), _expr(expr) { }

typedef_type::~typedef_type() { }

item* typedef_type::new_instance(env * const e) {
    item* i = type::new_instance(e);
    bool assert_facts = _core->assert_facts(std::vector<bool_item*>({ _core->eq(i, expression_visitor(_core, e).visit(_expr)) }));
    assert(assert_facts && "new typedef instance creation failed..");
    return i;
}
