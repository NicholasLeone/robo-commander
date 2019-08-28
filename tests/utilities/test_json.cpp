#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <map>
#include <bitset>
#include <fstream>

#include "rapidjson/document.h"

using namespace std;
using namespace rapidjson;

void enter(const Value &obj, size_t indent = 0) { //print JSON tree

     if (obj.IsObject()) { //check if object
          for (Value::ConstMemberIterator itr = obj.MemberBegin(); itr != obj.MemberEnd(); ++itr) {   //iterate through object
               const Value& objName = obj[itr->name.GetString()]; //make object value

               for (size_t i = 0; i != indent; ++i) //indent
                  cout << " ";

               cout << itr->name.GetString() << ": "; //key name

               if (itr->value.IsNumber()) //if integer
                  std::cout << itr->value.GetFloat() ;

               // else if (itr->value.IsString()) //if string
               //    std::cout << itr->value.GetString();
               //
               //
               // else if (itr->value.IsBool()) //if bool
               //    std::cout << itr->value.GetBool();

               // else if (itr->value.IsArray()){ //if array
               //
               //    for (SizeType i = 0; i < itr->value.Size(); i++) {
               //         if (itr->value[i].IsNumber()) //if array value integer
               //             std::cout << itr->value[i].GetInt() ;
               //
               //         else if (itr->value[i].IsString()) //if array value string
               //             std::cout << itr->value[i].GetString() ;
               //
               //         else if (itr->value[i].IsBool()) //if array value bool
               //             std::cout << itr->value[i].GetBool() ;
               //
               //         else if (itr->value[i].IsObject()){ //if array value object
               //             cout << "\n  ";
               //             const Value& m = itr->value[i];
               //             for (auto& v : m.GetObject()) { //iterate through array object
               //                if (m[v.name.GetString()].IsString()) //if array object value is string
               //                    cout << v.name.GetString() << ": " <<   m[v.name.GetString()].GetString();
               //                else //if array object value is integer
               //                    cout << v.name.GetString() << ": "  <<  m[v.name.GetString()].GetInt();
               //
               //               cout <<  "\t"; //indent
               //             }
               //         }
               //         cout <<  "\t"; //indent
               //    }
               // }

               cout << endl;
               enter(objName, indent + 1); //if couldn't find in object, enter object and repeat process recursively
          }
     }
}



int main(int argc, char *argv[]){

     ifstream in("/home/hunter/devel/robo-dev/test/utils/test.json");
     string contents((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());

     const char* json = contents.c_str();
     printf("Original JSON:\n %s\n",json);

     Document document;  // Default template parameter uses UTF8 and MemoryPoolAllocator.

#if 1
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

        static const char* kTypeNames[] = { "Null", "False", "True", "Object", "Array", "String", "Number" };

        for (Value::ConstMemberIterator i = document.MemberBegin(); i != document.MemberEnd(); ++i){
           enter(i->value);
           cout << endl << endl<< endl;
        }

     return 0;

}
