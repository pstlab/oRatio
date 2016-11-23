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

/* 
 * File:   defined_predicate.h
 * Author: Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * Created on November 9, 2016, 6:29 PM
 */

#ifndef DEFINED_PREDICATE_H
#define DEFINED_PREDICATE_H

#include "../predicate.h"
#include "oRatioParser.h"

namespace oratio {

    class defined_predicate : public predicate {
    public:
        defined_predicate(core * const c, scope * const s, const std::string& name, const std::vector<field*>& args, oRatioParser::BlockContext * const b);
        defined_predicate(defined_predicate&&) = delete;
        virtual ~defined_predicate();
    private:
        oRatioParser::BlockContext * const _block;

        bool apply_rule(atom * const a) const override;
    };
}

#endif /* DEFINED_PREDICATE_H */

