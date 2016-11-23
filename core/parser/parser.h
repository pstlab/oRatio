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

#include "oRatioLexer.h"
#include "oRatioParser.h"

namespace oratio {

	class core;
	class scope;

	class parser {
		friend class type_declaration_listener;
		friend class type_refinement_listener;
		friend class type_visitor;
		friend class expression_visitor;
		friend class statement_visitor;
	public:
		parser(core * const c);
		parser(parser&&) = delete;
		virtual ~parser();

		virtual bool read(const std::string& script);
		virtual bool read(const std::vector<std::string>& files);
	public:
		core * const _core;

	private:
		std::map<antlr4::tree::ParseTree*, scope*> _scopes;
		oRatioParser * _parser;

		class snippet {
			friend class parser;
		private:
			const std::string& _file;
			oRatioParser * const _parser;
			oRatioParser::Compilation_unitContext * const _cu;

			snippet(const std::string& file, oRatioParser * const p, oRatioParser::Compilation_unitContext * const cu);
			snippet(snippet&&) = delete;
			virtual ~snippet();
		};
	};
}

