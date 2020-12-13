#include "json.h"
#include <sstream>
#include <cassert>

using namespace smt;

void test_json_0()
{
    json j_obj;
    j_obj->set("a", new string_val("a"));
    j_obj->set("b", new long_val(1));
    j_obj->set("c", new double_val(1.5));
    j_obj->set("d", new bool_val(true));
    j_obj->set("e", new array_val({new bool_val(true), new bool_val(false)}));
    j_obj->set_null("f");

    json n_obj;
    n_obj->set("a", new string_val("a"));
    n_obj->set("b", new long_val(1));

    j_obj->set("g", n_obj);

    std::cout << j_obj << std::endl;
}

void test_json_1()
{
    std::stringstream s("{\"a\": \"a\", \"b\": 1, \"c\": 1.500000, \"d\": true, \"e\": [true, false], \"f\": null, \"g\": {\"a\": true, \"b\": 1}}");
    json j_obj = json::from_json(s);
}

int main(int, char **)
{
    test_json_0();
    test_json_1();
}