#pragma once

#include <map>
#include <vector>
#include <string>
#include <iostream>

namespace smt
{

  class json_core;

  class json
  {
  public:
    json();
    json(json_core *const ptr);
    json(const json &orig);
    ~json();

    json &operator=(const json &t);
    json_core *operator->() const { return ptr; }

    void to_json(std::ostream &os) const;
    static json from_json(std::istream &is);

    friend std::ostream &operator<<(std::ostream &os, const json &j);

  private:
    json_core *const ptr;
  };

  class json_core
  {
    friend class json;
    friend class array_val;

  public:
    json_core();
    json_core(const std::map<std::string, json> &vals);
    ~json_core();

    json &get(const std::string &id) { return vals.at(id); }
    void set(const std::string &id, const json &val) { vals.emplace(id, val); }

    void set_null(const std::string &id);

  private:
    virtual void to_json(std::ostream &os) const;

  private:
    unsigned ref_count = 0;
    std::map<std::string, json> vals;
  };

  class null_val : public json_core
  {
  public:
    null_val();
    ~null_val();

    void to_json(std::ostream &os) const override;
  };

  class bool_val : public json_core
  {
  public:
    bool_val();
    bool_val(const bool &val);
    ~bool_val();

    bool get() const { return val; }
    void set(const bool &val) { this->val = val; }

    bool operator*() const { return val; }

    void to_json(std::ostream &os) const override;

  private:
    bool val;
  };

  class string_val : public json_core
  {
  public:
    string_val();
    string_val(const std::string &val);
    ~string_val();

    std::string get() const { return val; }
    void set(const std::string &val) { this->val = val; }

    std::string operator*() const { return val; }

    void to_json(std::ostream &os) const override;

  private:
    std::string val;
  };

  class long_val : public json_core
  {
  public:
    long_val();
    long_val(const long &val);
    ~long_val();

    long get() const { return val; }
    void set(const long &val) { this->val = val; }

    long operator*() const { return val; }

    void to_json(std::ostream &os) const override;

  private:
    long val;
  };

  class double_val : public json_core
  {
  public:
    double_val();
    double_val(const double &val);
    ~double_val();

    double get() const { return val; }
    void set(const double &val) { this->val = val; }

    double operator*() const { return val; }

    void to_json(std::ostream &os) const override;

  private:
    double val;
  };

  class array_val : public json_core
  {
  public:
    array_val();
    array_val(const std::vector<json> &vals);
    ~array_val();

    json get(const size_t &i) const { return vals.at(i); }
    void set(const size_t &i, const json val) { vals[i] = val; }

    void to_json(std::ostream &os) const override;

  private:
    std::vector<json> vals;
  };
} // namespace smt
