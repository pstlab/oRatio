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

#include "../constructor.h"
#include "oRatioParser.h"

namespace oratio {

	class defined_constructor : public constructor {
	public:
		defined_constructor(core * const c, scope * const s, const std::vector<field*>& args, std::vector<oRatioParser::Initializer_elementContext*> init_els, oRatioParser::BlockContext * const b);
		defined_constructor(defined_constructor&&) = delete;
		virtual ~defined_constructor();
	private:
		std::vector<oRatioParser::Initializer_elementContext*> _init_els;
		oRatioParser::BlockContext * const _block;

		bool invoke(item * const i, const std::vector<item*>& exprs) override;
	};
}

