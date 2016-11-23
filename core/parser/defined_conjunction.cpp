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

#include "defined_conjunction.h"
#include "statement_visitor.h"
#include "../core.h"
#include "../env.h"

using namespace oratio;

defined_conjunction::defined_conjunction(core * const c, scope * const s, ac::arith_var& cst, oRatioParser::BlockContext * const b) : conjunction(c, s, cst), _block(b) { }

defined_conjunction::~defined_conjunction() { }

bool defined_conjunction::apply(env * const e) const {
	return statement_visitor(_core, new env(_core, e)).visit(_block);
}
