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
    inline json_core *operator->() const { return ptr; }

    void to_json(std::ostream &os) const noexcept;
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

    inline json &get(const std::string &id) noexcept { return vals.at(id); }
    inline void set(const std::string &id, const json &val) noexcept { vals.emplace(id, val); }

    void set_null(const std::string &id);

  private:
    virtual void to_json(std::ostream &os) const noexcept;

  private:
    unsigned ref_count = 0;
    std::map<std::string, json> vals;
  };

  class null_val : public json_core
  {
  public:
    null_val();
    ~null_val();

    void to_json(std::ostream &os) const noexcept override;
  };

  class bool_val : public json_core
  {
  public:
    bool_val();
    bool_val(const bool &val);
    ~bool_val();

    inline bool get() const noexcept { return val; }
    inline void set(const bool &val) noexcept { this->val = val; }

    bool operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    bool val;
  };

  class string_val : public json_core
  {
  public:
    string_val();
    string_val(const std::string &val);
    ~string_val();

    inline std::string get() const noexcept { return val; }
    inline void set(const std::string &val) noexcept { this->val = val; }

    std::string operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    std::string val;
  };

  class long_val : public json_core
  {
  public:
    long_val();
    long_val(const long &val);
    ~long_val();

    inline long get() const noexcept { return val; }
    inline void set(const long &val) noexcept { this->val = val; }

    long operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    long val;
  };

  class double_val : public json_core
  {
  public:
    double_val();
    double_val(const double &val);
    ~double_val();

    inline double get() const noexcept { return val; }
    inline void set(const double &val) noexcept { this->val = val; }

    double operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    double val;
  };

  class array_val : public json_core
  {
  public:
    array_val();
    array_val(const std::vector<json> &vals);
    ~array_val();

    inline json get(const size_t &i) const noexcept { return vals.at(i); }
    inline void set(const size_t &i, const json val) noexcept { vals[i] = val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    std::vector<json> vals;
  };
} // namespace smt
