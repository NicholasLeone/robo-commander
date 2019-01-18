#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

/** SECTION:
     General Definitions
*/

#define M_DEG2RAD (2*M_PI)/360
#define M_RAD2DEG 360/(2*M_PI)

#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

typedef void (*void_int_fun)(int);


#endif
