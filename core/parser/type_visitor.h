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

#include "oRatioBaseVisitor.h"

namespace oratio {

	class parser;

	class type_visitor : public oRatioBaseVisitor {
	public:
		type_visitor(parser * const p);
		type_visitor(type_visitor&&) = delete;
		virtual ~type_visitor();
	private:
		parser * const _parser;

		antlrcpp::Any visitLiteral_expression(oRatioParser::Literal_expressionContext* ctx) override;
		antlrcpp::Any visitCast_expression(oRatioParser::Cast_expressionContext* ctx) override;
		antlrcpp::Any visitPrimitive_type(oRatioParser::Primitive_typeContext* ctx) override;
		antlrcpp::Any visitClass_type(oRatioParser::Class_typeContext* ctx) override;
		antlrcpp::Any visitQualified_id(oRatioParser::Qualified_idContext* ctx) override;
		antlrcpp::Any visitQualified_id_expression(oRatioParser::Qualified_id_expressionContext* ctx) override;
		antlrcpp::Any visitConstructor_expression(oRatioParser::Constructor_expressionContext* ctx) override;
	};
}

