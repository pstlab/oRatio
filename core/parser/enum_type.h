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

#include "../type.h"
#include "../item.h"

namespace oratio {

	class enum_type : public type {
		friend class type_declaration_listener;
		friend class type_refinement_listener;
	public:
		enum_type(core * const c, scope * const s, const std::string& name);
		enum_type(enum_type&&) = delete;
		virtual ~enum_type();

		std::vector<string_item*> get_enums();
		item * new_instance(env * const e) override;
	private:
		std::vector<enum_type*> _enums;

		void add_enum(string_item * const i);
	};
}

