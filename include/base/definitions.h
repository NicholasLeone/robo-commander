/******************************************************************************
* RoboClaw Definitions
******************************************************************************/

enum{	M1FORWARD = 0,
          M1BACKWARD = 1,
          SETMINMB = 2,
          SETMAXMB = 3,
          M2FORWARD = 4,
          M2BACKWARD = 5,
          M17BIT = 6,
          M27BIT = 7,
          MIXEDFORWARD = 8,
          MIXEDBACKWARD = 9,
          MIXEDRIGHT = 10,
          MIXEDLEFT = 11,
          MIXEDFB = 12,
          MIXEDLR = 13,
          GETM1ENC = 16,
          GETM2ENC = 17,
          GETM1SPEED = 18,
          GETM2SPEED = 19,
          RESETENC = 20,
          GETVERSION = 21,
          SETM1ENCCOUNT = 22,
          SETM2ENCCOUNT = 23,
          GETMBATT = 24,
          GETLBATT = 25,
          SETMINLB = 26,
          SETMAXLB = 27,
          SETM1PID = 28,
          SETM2PID = 29,
          GETM1ISPEED = 30,
          GETM2ISPEED = 31,
          M1DUTY = 32,
          M2DUTY = 33,
          MIXEDDUTY = 34,
          M1SPEED = 35,
          M2SPEED = 36,
          MIXEDSPEED = 37,
          M1SPEEDACCEL = 38,
          M2SPEEDACCEL = 39,
          MIXEDSPEEDACCEL = 40,
          M1SPEEDDIST = 41,
          M2SPEEDDIST = 42,
          MIXEDSPEEDDIST = 43,
          M1SPEEDACCELDIST = 44,
          M2SPEEDACCELDIST = 45,
          MIXEDSPEEDACCELDIST = 46,
          GETBUFFERS = 47,
          GETPWMS = 48,
          GETCURRENTS = 49,
          MIXEDSPEED2ACCEL              = 50,
          MIXEDSPEED2ACCELDIST          = 51,
          M1DUTYACCEL                   = 52,
          M2DUTYACCEL                   = 53,
          MIXEDDUTYACCEL                = 54,
          READM1PID                     = 55,
          READM2PID                     = 56,
          SETMAINVOLTAGES               = 57,
          SETLOGICVOLTAGES              = 58,
          GETMINMAXMAINVOLTAGES         = 59,
          GETMINMAXLOGICVOLTAGES        = 60,
          SETM1POSPID                   = 61,
          SETM2POSPID                   = 62,
          READM1POSPID                  = 63,
          READM2POSPID                  = 64,
          M1SPEEDACCELDECCELPOS         = 65,
          M2SPEEDACCELDECCELPOS         = 66,
          MIXEDSPEEDACCELDECCELPOS      = 67,
          SETM1DEFAULTACCEL             = 68,
          SETM2DEFAULTACCEL             = 69,
          SETPINFUNCTIONS               = 74,
          GETPINFUNCTIONS               = 75,
          SETDEADBAND	               = 76,
          GETDEADBAND	               = 77,
          GETENCODERS                   = 78,
          GETISPEEDS                    = 79,
          RESTOREDEFAULTS               = 80,
          GETTEMP                       = 82,
          GETTEMP2                      = 83,
          GETERROR                      = 90,
          GETENCODERMODE                = 91,
          SETM1ENCODERMODE              = 92,
          SETM2ENCODERMODE              = 93,
          WRITENVM                      = 94,
          READNVM                       = 95,
          SETCONFIG                     = 98,
          GETCONFIG                     = 99,
          SETM1MAXCURRENT               = 133,
          SETM2MAXCURRENT               = 134,
          GETM1MAXCURRENT               = 135,
          GETM2MAXCURRENT               = 136,
          SETPWMMODE                    = 148,
          GETPWMMODE                    = 149,
          FLAGBOOTLOADER                = 255
};

/******************************************************************************
* PCA-9685 Register Definitions
******************************************************************************/

#define MODE1 0x00			// Mode  register  1
#define MODE2 0x01			// Mode  register  2
#define SUBADR1 0x02		// I2C-bus subaddress 1
#define SUBADR2 0x03		// I2C-bus subaddress 2
#define SUBADR3 0x04		// I2C-bus subaddress 3
#define ALLCALLADR 0x05       // LED All Call I2C-bus address
#define LED0 0x6			// LED0 start register
#define LED0_ON_L 0x6		// LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		// LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		// LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		// LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	     // For the other 15 channels
#define ALLLED_ON_L 0xFA      // load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	     // load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	// load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	// load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		// prescaler for output frequency
#define CLOCK_FREQ 25000000.0 // 25MHz default osc clock
