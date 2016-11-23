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

#include "defined_method.h"
#include "statement_visitor.h"
#include "../core.h"
#include "../env.h"
#include "../item.h"
#include "../field.h"

using namespace oratio;

defined_method::defined_method(core * const c, scope * const s, const std::string& name, const std::vector<field*>& args, oRatioParser::BlockContext * const b, const type * const return_type) : method(c, s, name, args, return_type), _block(b) { }

defined_method::~defined_method() { }

bool defined_method::invoke(env * const e, const std::vector<item*>& exprs) {
	env* c_env = new env(_core, e);
	if (item * i = dynamic_cast<item*> (e)) {
		c_env->_items.insert({ THIS_KEYWORD, i });
	}
	for (unsigned int j = 0; j < _args.size(); j++) {
		c_env->_items.insert({ _args[j]->_name, exprs[j] });
	}
	return statement_visitor(_core, e).visit(_block);
}
