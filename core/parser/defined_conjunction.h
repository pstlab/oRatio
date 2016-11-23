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

#include "../disjunction.h"
#include "oRatioParser.h"

namespace oratio {

	class defined_conjunction : public conjunction {
	public:
		defined_conjunction(core * const c, scope * const s, ac::arith_var& cst, oRatioParser::BlockContext * const b);
		defined_conjunction(defined_conjunction&&) = delete;
		virtual ~defined_conjunction();
	private:
		oRatioParser::BlockContext * const _block;

		bool apply(env * const e) const override;
	};
}

