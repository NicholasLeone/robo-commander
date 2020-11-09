#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

/** SECTION:
     General Definitions
*/
#include <string>

#define M_DEG2RAD (2*M_PI)/360
#define M_RAD2DEG 360/(2*M_PI)

#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

typedef void (*void_int_fun)(int);

// Text Coloring defines for terminal printouts
std::string txt_red(){ return std::string("\033[1;31m"); }
std::string txt_bold_red(){ return std::string("\033[1;31m"); }
std::string txt_yellow(){ return std::string("\033[1;33m"); }
std::string txt_bold_yellow(){ return std::string("\033[01;33m"); }
std::string txt_green(){ return std::string("\033[0;32m"); }
std::string txt_bold_green(){ return std::string("\033[1;32m"); }
std::string txt_blue(){ return std::string("\033[0;34m"); }
std::string txt_bold_blue(){ return std::string("\033[1;34m"); }
std::string txt_magenta(){ return std::string("\033[0;35m"); }
std::string txt_bold_magenta(){ return std::string("\033[1;35m"); }
std::string txt_cyan(){ return std::string("\033[0;36m"); }
std::string txt_bold_cyan(){ return std::string("\033[1;36m"); }
std::string txt_reset_color(){ return std::string("\033[0m"); }

#endif
