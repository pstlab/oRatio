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

#include "default_constructor.h"
#include "../type.h"
#include "instantiated_field.h"
#include "../item.h"
#include "expression_visitor.h"
#include "../core.h"

using namespace oratio;

default_constructor::default_constructor(core * const c, scope * const s) : constructor(c, s, std::vector<field*>(0)) { }

default_constructor::~default_constructor() { }

bool default_constructor::invoke(item * const i, const std::vector<item*>& exprs) {
	for (const auto& st : static_cast<type*> (_scope)->get_supertypes()) {
		st->get_constructor(std::vector<const type*>(0))->invoke(i, exprs);
	}
	for (const auto& f : _scope->get_fields()) {
		if (!f.second->_synthetic) {
			if (instantiated_field * inst_f = dynamic_cast<instantiated_field*> (f.second)) {
				i->_items.insert({ f.second->_name, expression_visitor(_core, i).visit(inst_f->_expr) });
			}
			else if (f.second->_type->_primitive) {
				i->_items.insert({ f.second->_name, const_cast<type*> (f.second->_type)->new_instance(i) });
			}
			else {
				i->_items.insert({ f.second->_name, const_cast<type*> (f.second->_type)->new_existential() });
			}
		}
	}
	return true;
}
