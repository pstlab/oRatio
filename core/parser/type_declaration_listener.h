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

#include "oRatioBaseListener.h"

namespace oratio {

	class parser;
	class scope;

	class type_declaration_listener : public oRatioBaseListener {
	public:
		type_declaration_listener(parser * const p);
		type_declaration_listener(type_declaration_listener&&) = delete;
		virtual ~type_declaration_listener();
	private:
		parser * const _parser;
		scope * _scope;

		void enterCompilation_unit(oRatioParser::Compilation_unitContext* ctx) override;
		void enterTypedef_declaration(oRatioParser::Typedef_declarationContext* ctx) override;
		void enterEnum_declaration(oRatioParser::Enum_declarationContext* ctx) override;
		void enterClass_declaration(oRatioParser::Class_declarationContext* ctx) override;
		void exitClass_declaration(oRatioParser::Class_declarationContext* ctx) override;
		void enterClass_type(oRatioParser::Class_typeContext* ctx) override;
	};
}

