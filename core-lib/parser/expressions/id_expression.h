#pragma once

#include "expression.h"
#include <vector>
#include <string>

namespace ratio
{

namespace ast
{

class id_expression : public expression
{
public:
  id_expression(const std::vector<std::string> &is);
  id_expression(const id_expression &orig) = delete;
  virtual ~id_expression();

  expr evaluate(const scope &scp, context &ctx) const override;

private:
  const std::vector<std::string> ids;
};
}
}