#pragma once

#include "json_export.h"
#include <map>
#include <vector>
#include <string>
#include <iostream>

namespace smt
{
  class json_core;
  class null_val;
  class bool_val;
  class string_val;
  class long_val;
  class double_val;
  class array_val;

  class json
  {
  public:
    JSON_EXPORT json();
    JSON_EXPORT json(json_core *const ptr);
    JSON_EXPORT json(const json &orig);
    JSON_EXPORT ~json();

    json &operator=(const json &t);
    inline json_core &operator*() const { return *ptr; }
    inline json_core *operator->() const { return ptr; }

    JSON_EXPORT void to_json(std::ostream &os) const noexcept;
    JSON_EXPORT static json from_json(std::istream &is);

    JSON_EXPORT friend std::ostream &operator<<(std::ostream &os, const json &j);

  private:
    json_core * ptr;
  };

  class json_core
  {
    friend class json;
    friend class array_val;

  public:
    json_core() = default;
    json_core(const std::map<std::string, json> &vals);
    virtual ~json_core() = default;

    inline bool has(const std::string &id) const noexcept { return vals.count(id); }
    inline json &get(const std::string &id) noexcept { return vals.at(id); }
    inline void set(const std::string &id, const json &val) noexcept { vals.emplace(id, val); }

    virtual bool is_null() const noexcept { return false; }
    virtual bool is_bool() const noexcept { return false; }
    virtual bool is_long() const noexcept { return false; }
    virtual bool is_double() const noexcept { return false; }
    virtual bool is_object() const noexcept { return false; }
    virtual bool is_array() const noexcept { return false; }
    virtual bool is_string() const noexcept { return false; }

    JSON_EXPORT void set_null(const std::string &id);

  private:
    virtual void to_json(std::ostream &os) const noexcept;

  private:
    unsigned ref_count = 0;
    std::map<std::string, json> vals;
  };

  class null_val : public json_core
  {
  public:
    JSON_EXPORT null_val() = default;
    JSON_EXPORT ~null_val() = default;

    void to_json(std::ostream &os) const noexcept override;

    bool is_null() const noexcept override { return true; }
  };

  class bool_val : public json_core
  {
  public:
    JSON_EXPORT bool_val() = default;
    JSON_EXPORT bool_val(const bool &val);
    JSON_EXPORT ~bool_val() = default;

    inline bool get() const noexcept { return val; }
    inline void set(const bool &v) noexcept { val = v; }

    bool is_bool() const noexcept override { return true; }

    bool operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    bool val;
  };

  class string_val : public json_core
  {
  public:
    JSON_EXPORT string_val() = default;
    JSON_EXPORT string_val(const std::string &val);
    JSON_EXPORT ~string_val() = default;

    inline std::string get() const noexcept { return val; }
    inline void set(const std::string &v) noexcept { val = v; }

    bool is_string() const noexcept override { return true; }

    std::string operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    std::string val;
  };

  class long_val : public json_core
  {
  public:
    JSON_EXPORT long_val() = default;
    JSON_EXPORT long_val(const long &val);
    JSON_EXPORT ~long_val() = default;

    inline long get() const noexcept { return val; }
    inline void set(const long &v) noexcept { val = v; }

    bool is_long() const noexcept override { return true; }

    long operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    long val;
  };

  class double_val : public json_core
  {
  public:
    JSON_EXPORT double_val() = default;
    JSON_EXPORT double_val(const double &val);
    JSON_EXPORT ~double_val() = default;

    inline double get() const noexcept { return val; }
    inline void set(const double &v) noexcept { val = v; }

    bool is_double() const noexcept override { return true; }

    double operator*() const noexcept { return val; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    double val;
  };

  class array_val : public json_core
  {
  public:
    JSON_EXPORT array_val() = default;
    JSON_EXPORT array_val(const std::vector<json> &vals);
    JSON_EXPORT ~array_val() = default;

    inline size_t size() const noexcept { return vals.size(); }

    inline json get(const size_t &i) const noexcept { return vals.at(i); }
    inline void set(const size_t &i, const json val) noexcept { vals[i] = val; }

    bool is_array() const noexcept override { return true; }

    void to_json(std::ostream &os) const noexcept override;

  private:
    std::vector<json> vals;
  };
} // namespace smt
