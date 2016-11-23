
// Generated from oRatio.g4 by ANTLR 4.5.3

#pragma once


#include "antlr4-runtime.h"
#include "oRatioVisitor.h"


namespace oratio {

	/**
	 * This class provides an empty implementation of oRatioVisitor, which can be
	 * extended to create a visitor which only needs to handle a subset of the available methods.
	 */
	class oRatioBaseVisitor : public oRatioVisitor {
	public:

		virtual antlrcpp::Any visitCompilation_unit(oRatioParser::Compilation_unitContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitType_declaration(oRatioParser::Type_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitTypedef_declaration(oRatioParser::Typedef_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitEnum_declaration(oRatioParser::Enum_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitEnum_constants(oRatioParser::Enum_constantsContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitClass_declaration(oRatioParser::Class_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitMember(oRatioParser::MemberContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitField_declaration(oRatioParser::Field_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitVariable_dec(oRatioParser::Variable_decContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitVoid_method_declaration(oRatioParser::Void_method_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitType_method_declaration(oRatioParser::Type_method_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitConstructor_declaration(oRatioParser::Constructor_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitInitializer_element(oRatioParser::Initializer_elementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitPredicate_declaration(oRatioParser::Predicate_declarationContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitStatement(oRatioParser::StatementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitBlock(oRatioParser::BlockContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitAssignment_statement(oRatioParser::Assignment_statementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitLocal_variable_statement(oRatioParser::Local_variable_statementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitExpression_statement(oRatioParser::Expression_statementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitDisjunction_statement(oRatioParser::Disjunction_statementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitConjunction(oRatioParser::ConjunctionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitFormula_statement(oRatioParser::Formula_statementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitReturn_statement(oRatioParser::Return_statementContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitAssignment_list(oRatioParser::Assignment_listContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitAssignment(oRatioParser::AssignmentContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitCast_expression(oRatioParser::Cast_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitQualified_id_expression(oRatioParser::Qualified_id_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitDivision_expression(oRatioParser::Division_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitSubtraction_expression(oRatioParser::Subtraction_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitExtc_one_expression(oRatioParser::Extc_one_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitPlus_expression(oRatioParser::Plus_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitFunction_expression(oRatioParser::Function_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitAddition_expression(oRatioParser::Addition_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitParentheses_expression(oRatioParser::Parentheses_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitMinus_expression(oRatioParser::Minus_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitImplication_expression(oRatioParser::Implication_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitLt_expression(oRatioParser::Lt_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitNot_expression(oRatioParser::Not_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitConjunction_expression(oRatioParser::Conjunction_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitGeq_expression(oRatioParser::Geq_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitRange_expression(oRatioParser::Range_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitMultiplication_expression(oRatioParser::Multiplication_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitLeq_expression(oRatioParser::Leq_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitGt_expression(oRatioParser::Gt_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitConstructor_expression(oRatioParser::Constructor_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitDisjunction_expression(oRatioParser::Disjunction_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitLiteral_expression(oRatioParser::Literal_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitEq_expression(oRatioParser::Eq_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitNeq_expression(oRatioParser::Neq_expressionContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitExpr_list(oRatioParser::Expr_listContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitLiteral(oRatioParser::LiteralContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitQualified_id(oRatioParser::Qualified_idContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitType(oRatioParser::TypeContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitClass_type(oRatioParser::Class_typeContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitPrimitive_type(oRatioParser::Primitive_typeContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitType_list(oRatioParser::Type_listContext *ctx) override {
			return visitChildren(ctx);
		}

		virtual antlrcpp::Any visitTyped_list(oRatioParser::Typed_listContext *ctx) override {
			return visitChildren(ctx);
		}


	};

} // namespace oratio
