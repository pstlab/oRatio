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
 * File:   default_constructor.h
 * Author: Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * Created on November 9, 2016, 6:05 PM
 */

#ifndef DEFAULT_CONSTRUCTOR_H
#define DEFAULT_CONSTRUCTOR_H

#include "../constructor.h"

namespace oratio {

    class default_constructor : public constructor {
    public:
        default_constructor(core * const c, scope * const s);
        default_constructor(default_constructor&&) = delete;
        virtual ~default_constructor();
    private:
        bool invoke(item * const i, const std::vector<item*>& exprs) override;
    };
}

#endif /* DEFAULT_CONSTRUCTOR_H */

