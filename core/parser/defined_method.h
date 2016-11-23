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

#include "../method.h"
#include "oRatioParser.h"

namespace oratio {

    class defined_method : public method {
    public:
        defined_method(core * const c, scope * const s, const std::string& name, const std::vector<field*>& args, oRatioParser::BlockContext * const b, const type * const return_type = nullptr);
        defined_method(defined_method&&) = delete;
        virtual ~defined_method();
    private:
        oRatioParser::BlockContext * const _block;

        bool invoke(env * const e, const std::vector<item*>& exprs) override;
    };
}

