#pragma once

#include "lexer.h"
#include <vector>

namespace ratio
{

class core;

namespace ast
{
class compilation_unit;
class typedef_declaration;
class enum_declaration;
class class_declaration;
class field_declaration;
class method_declaration;
class constructor_declaration;
class predicate_declaration;
class statement;
class expression;
}

class parser
{
public:
  parser(std::istream &is);
  parser(const parser &orig) = delete;
  virtual ~parser();

  ast::compilation_unit *parse();

private:
  token *next();
  bool match(const symbol &sym);
  void backtrack(const size_t &p);

  ast::typedef_declaration *_typedef_declaration();
  ast::enum_declaration *_enum_declaration();
  ast::class_declaration *_class_declaration();
  ast::field_declaration *_field_declaration();
  ast::method_declaration *_method_declaration();
  ast::constructor_declaration *_constructor_declaration();
  ast::predicate_declaration *_predicate_declaration();
  ast::statement *_statement();
  ast::expression *_expression(const size_t &pr = 0);

  void error(const std::string &err);

private:
  lexer lex;                // the current lexer..
  token *tk = nullptr;      // the current lookahead token..
  std::vector<token *> tks; // all the tokens parsed so far..
  size_t pos = 0;           // the current position within 0tks'..
};
}