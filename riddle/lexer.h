#pragma once

#include "rational.h"
#include <cstddef>
#include <string>
#include <istream>
#include <cmath>

namespace riddle
{

enum symbol
{
  BOOL_ID,          // 'bool'
  INT_ID,           // 'int'
  REAL_ID,          // 'real'
  STRING_ID,        // 'string'
  TYPEDEF_ID,       // 'typedef'
  ENUM_ID,          // 'enum'
  CLASS_ID,         // 'class'
  GOAL_ID,          // 'goal'
  FACT_ID,          // 'fact'
  PREDICATE_ID,     // 'predicate'
  NEW_ID,           // 'new'
  OR_ID,            // 'or'
  THIS_ID,          // 'this'
  VOID_ID,          // 'void'
  TRUE_ID,          // 'true'
  FALSE_ID,         // 'false'
  RETURN_ID,        // 'return'
  DOT_ID,           // '.'
  COMMA_ID,         // ','
  COLON_ID,         // ':'
  SEMICOLON_ID,     // ';'
  LPAREN_ID,        // '('
  RPAREN_ID,        // ')'
  LBRACKET_ID,      // '['
  RBRACKET_ID,      // ']'
  LBRACE_ID,        // '{'
  RBRACE_ID,        // '}'
  PLUS_ID,          // '+'
  MINUS_ID,         // '-'
  STAR_ID,          // '*'
  SLASH_ID,         // '/'
  AMP_ID,           // '&'
  BAR_ID,           // '|'
  EQ_ID,            // '='
  GT_ID,            // '>'
  LT_ID,            // '<'
  BANG_ID,          // '!'
  EQEQ_ID,          // '=='
  LTEQ_ID,          // '<='
  GTEQ_ID,          // '>='
  BANGEQ_ID,        // '!='
  IMPLICATION_ID,   // '->'
  CARET_ID,         // '^'
  ID_ID,            // ('a'..'z'|'A'..'Z'|'_') ('a'..'z'|'A'..'Z'|'0'..'9'|'_')*
  IntLiteral_ID,    // [0-9]+
  RealLiteral_ID,   // [0-9]+ '.' [0-9]+)? | '.' [0-9]+
  StringLiteral_ID, // '" . . ."'
  EOF_ID
};

class token
{
public:
  token(const symbol &sym, const size_t &start_line, const size_t &start_pos, const size_t &end_line, const size_t &end_pos) : sym(sym), start_line(start_line), start_pos(start_pos), end_line(end_line), end_pos(end_pos) {}
  virtual ~token() {}

public:
  const symbol sym;
  const size_t start_line;
  const size_t start_pos;
  const size_t end_line;
  const size_t end_pos;
};

class id_token : public token
{
public:
  id_token(const size_t &start_line, const size_t &start_pos, const size_t &end_line, const size_t &end_pos, const std::string &id) : token(ID_ID, start_line, start_pos, end_line, end_pos), id(id) {}
  virtual ~id_token() {}

public:
  const std::string id;
};

class int_token : public token
{
public:
  int_token(const size_t &start_line, const size_t &start_pos, const size_t &end_line, const size_t &end_pos, const smt::I &val) : token(IntLiteral_ID, start_line, start_pos, end_line, end_pos), val(val) {}
  virtual ~int_token() {}

public:
  const smt::I val;
};

class real_token : public token
{
public:
  real_token(const size_t &start_line, const size_t &start_pos, const size_t &end_line, const size_t &end_pos, const smt::rational &val) : token(RealLiteral_ID, start_line, start_pos, end_line, end_pos), val(val) {}
  virtual ~real_token() {}

public:
  const smt::rational val;
};

class string_token : public token
{
public:
  string_token(const size_t &start_line, const size_t &start_pos, const size_t &end_line, const size_t &end_pos, const std::string &str) : token(StringLiteral_ID, start_line, start_pos, end_line, end_pos), str(str) {}
  virtual ~string_token() {}

public:
  const std::string str;
};

class lexer
{
public:
  lexer(std::istream &is);
  lexer(const lexer &orig) = delete;
  virtual ~lexer();

  token *next();

private:
  static bool is_id_part(const char &ch) { return ch == '_' || (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9'); }

  token *mk_token(const symbol &sym)
  {
    token *tk = new token(sym, start_line, start_pos, end_line, end_pos);
    start_line = end_line;
    start_pos = end_pos;
    return tk;
  }

  token *mk_id_token(const std::string &id)
  {
    id_token *tk = new id_token(start_line, start_pos, end_line, end_pos, id);
    start_line = end_line;
    start_pos = end_pos;
    return tk;
  }

  token *mk_integer_token(const std::string &str)
  {
    token *tk = new int_token(start_line, start_pos, end_line, end_pos, static_cast<smt::I>(std::stol(str)));
    start_line = end_line;
    start_pos = end_pos;
    return tk;
  }

  token *mk_rational_token(const std::string &intgr, const std::string &dec)
  {
    token *tk = new real_token(start_line, start_pos, end_line, end_pos, smt::rational(static_cast<smt::I>(std::stol(intgr + dec)), static_cast<smt::I>(std::pow(10, dec.size()))));
    start_line = end_line;
    start_pos = end_pos;
    return tk;
  }

  token *mk_string_token(const std::string &str)
  {
    string_token *tk = new string_token(start_line, start_pos, end_line, end_pos, str);
    start_line = end_line;
    start_pos = end_pos;
    return tk;
  }

  token *finish_id(std::string &str);

  void error(const std::string &err);
  int next_char();

private:
  std::string sb;
  size_t pos = 0;
  int ch;
  size_t start_line = 0;
  size_t start_pos = 0;
  size_t end_line = 0;
  size_t end_pos = 0;
};
} // namespace riddle