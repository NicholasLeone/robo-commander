#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <map>
#include <bitset>
#include "rapidjson/document.h"

using namespace std;
using namespace rapidjson;

int main(int argc, char *argv[]){

     const char json[] = " { \"plot\" : {\"id\" : 1, \"t\" : true , \"f\" : false, \"n\": null, \"i\":123, \"pi\": 3.1416, \"a\":[1, 2, 3, 4] }} ";
     // const char json[] = "{\"plot\": {\"id\": 1,\"index\":{\"range\": 21.143,\"cross_range\": 59.0},\"location\": {\"x\":1, \"y\":2, \"z\":3},\"geometry\": {\"row_width\": 0.762,\"row_length\": 15,\"row_count\": 4}}";
     // printf("Original JSON:\n %s\n", json);

     Document document;  // Default template parameter uses UTF8 and MemoryPoolAllocator.

#if 0
    // "normal" parsing, decode strings to new buffers. Can use other input stream via ParseStream().
    if (document.Parse(json).HasParseError())
        return 1;
#else
    // In-situ parsing, decode strings directly in the source string. Source must be string.
    char buffer[sizeof(json)];
    memcpy(buffer, json, sizeof(json));
    if (document.ParseInsitu(buffer).HasParseError())
        return 1;
#endif
    printf("\nParsing to document succeeded.\n");

    printf("\nAccess values in document:\n");
        // assert(document.IsObject());    // Document is a JSON value represents the root of DOM. Root can be either an object or array.

        assert(document.HasMember("plot"));
        assert(document["plot"].HasMember("id"));

        const Value& plot = document["plot"];

        printf("Plot id: %d\n", plot["id"].GetInt());
        // // Since version 0.2, you can use single lookup to check the existing of member and its value:
        // Value::MemberIterator id = plot.FindMember("id");
        // assert(id != plot.MemberEnd());
        // assert(id->value.IsString());
        // assert(strcmp("world", id->value.GetString()) == 0);
        // (void)id;
        //
        // assert(document["t"].IsBool());     // JSON true/false are bool. Can also uses more specific function IsTrue().
        // printf("t = %s\n", document["t"].GetBool() ? "true" : "false");
        //
        // assert(document["f"].IsBool());
        // printf("f = %s\n", document["f"].GetBool() ? "true" : "false");
        //
        // printf("n = %s\n", document["n"].IsNull() ? "null" : "?");
        //
        // assert(document["i"].IsNumber());   // Number is a JSON type, but C++ needs more specific type.
        // assert(document["i"].IsInt());      // In this case, IsUint()/IsInt64()/IsUInt64() also return true.
        // printf("i = %d\n", document["i"].GetInt()); // Alternative (int)document["i"]
        //
        // assert(document["pi"].IsNumber());
        // assert(document["pi"].IsDouble());
        // printf("pi = %g\n", document["pi"].GetDouble());
        //
        // {
        //     const Value& a = document["a"]; // Using a reference for consecutive access is handy and faster.
        //     assert(a.IsArray());
        //     for (SizeType i = 0; i < a.Size(); i++) // rapidjson uses SizeType instead of size_t.
        //         printf("a[%d] = %d\n", i, a[i].GetInt());
        //
        //     int y = a[0].GetInt();
        //     (void)y;
        //
        //     // Iterating array with iterators
        //     printf("a = ");
        //     for (Value::ConstValueIterator itr = a.Begin(); itr != a.End(); ++itr)
        //         printf("%d ", itr->GetInt());
        //     printf("\n");
        // }
        //
        // // Iterating object members
        // static const char* kTypeNames[] = { "Null", "False", "True", "Object", "Array", "String", "Number" };
        // for (Value::ConstMemberIterator itr = document.MemberBegin(); itr != document.MemberEnd(); ++itr)
        //     printf("Type of member %s is %s\n", itr->name.GetString(), kTypeNames[itr->value.GetType()]);

     return 0;

}
