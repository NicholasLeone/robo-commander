#ifndef PARAMS_H_
#define PARAMS_H_

using namespace std;

/** SECTION:

     ACTUATOR PARAMETERS

*/

/** SECTION:

     SENSORS PARAMETERS

*/
typedef struct SENSOR_PARAMS{
     // Device Address
     int add;
     // Component Readings
     float x;
     float y;
     float z;
     // Component Offsets
     float x_off;
     float y_off;
     float z_off;
     // Full-Scale Range
     int fs;
     // Sensitivity
     float sens;
} SENSOR_PARAMS;

/** SECTION:

     DATA MESSAGES TYPES

*/

typedef struct XYZ_BASE{
     float x;
     float y;
     float z;
} XYZ_BASE;

typedef struct ORIENTATION_QUATERNION_BASE{
     float x;
     float y;
     float z;
     float w;
} ORIENTATION_QUATERNION_BASE;

typedef struct ORIENTATION_EULER_BASE{
     float roll;
     float pitch;
     float yaw;
} ORIENTATION_EULER_BASE;

/** SECTION:
     EXTRAPOLATED INFORMATION
*/
typedef struct XYZ_DATA{
     float x;
     float y;
     float z;
     XYZ_BASE covariance;
     XYZ_BASE bias;
     XYZ_BASE offset;
} XYZ_DATA;

typedef struct ORIENTATION_DATA{
     float x;
     float y;
     float z;
     float w;
     ORIENTATION_EULER_BASE covariance;
     ORIENTATION_QUATERNION_BASE bias;
} ORIENTATION_DATA;

typedef struct XYZ_BASE_INT{
	int32_t x;
	int32_t y;
	int32_t z;
} XYZ_BASE_INT;

typedef struct ORIENTATION_QUATERNION_BASE_INT{
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t w;
} ORIENTATION_QUATERNION_BASE_INT;

typedef struct ORIENTATION_EULER_BASE_INT{
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
} ORIENTATION_EULER_BASE_INT;

typedef struct XYZ_DATA_INT{
	int32_t x;
	int32_t y;
	int32_t z;
	XYZ_BASE covariance;
	XYZ_BASE bias;
	XYZ_BASE offset;
} XYZ_DATA_INT;

#endif // PARAMS_H_
