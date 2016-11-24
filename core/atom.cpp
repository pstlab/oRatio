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

#include "atom.h"
#include "predicate.h"
#include "core.h"

using namespace oratio;

atom::atom(core * const c, env * const e, const predicate * const p) : env(c, e), item(c, e, p), _state(*c->_net.new_enum(std::unordered_set<int>({ atom_state::active, atom_state::inactive, atom_state::unified }))) { }

atom::~atom() { }
