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

#include "defined_predicate.h"
#include "statement_visitor.h"
#include "../core.h"
#include "../env.h"
#include "../atom.h"

using namespace oratio;

defined_predicate::defined_predicate(core * const c, scope * const s, const std::string& name, const std::vector<field*>& args, oRatioParser::BlockContext * const b) : predicate(c, s, name, args), _block(b) { }

defined_predicate::~defined_predicate() { }

bool defined_predicate::apply_rule(atom * const a) const {
	for (const auto& sp : get_supertypes()) {
		static_cast<predicate*> (sp)->apply_rule(a);
	}

	env* c_env = new env(_core, a);
	c_env->_items.insert({ THIS_KEYWORD, a });
	return statement_visitor(_core, c_env).visit(_block);
}
