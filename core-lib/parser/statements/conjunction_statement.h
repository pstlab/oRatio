#pragma once

#include "statement.h"
#include <vector>

namespace ratio
{

namespace ast
{

class conjunction_statement : public statement
{
public:
  conjunction_statement(const std::vector<const statement *> &stmnts);
  conjunction_statement(const conjunction_statement &orig) = delete;
  virtual ~conjunction_statement();

  void execute(const scope &scp, context &ctx) const override;

private:
  const std::vector<const statement *> statements;
};
}
}