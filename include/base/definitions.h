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
          MIXEDSPEED2ACCEL = 50,
          MIXEDSPEED2ACCELDIST = 51,
          M1DUTYACCEL = 52,
          M2DUTYACCEL = 53,
          MIXEDDUTYACCEL = 54,
          READM1PID = 55,
          READM2PID = 56,
          SETMAINVOLTAGES = 57,
          SETLOGICVOLTAGES = 58,
          GETMINMAXMAINVOLTAGES = 59,
          GETMINMAXLOGICVOLTAGES = 60,
          SETM1POSPID = 61,
          SETM2POSPID = 62,
          READM1POSPID = 63,
          READM2POSPID = 64,
          M1SPEEDACCELDECCELPOS = 65,
          M2SPEEDACCELDECCELPOS = 66,
          MIXEDSPEEDACCELDECCELPOS = 67,
          SETM1DEFAULTACCEL = 68,
          SETM2DEFAULTACCEL = 69,
          SETPINFUNCTIONS = 74,
          GETPINFUNCTIONS = 75,
          SETDEADBAND	= 76,
          GETDEADBAND	= 77,
          GETENCODERS = 78,
          GETISPEEDS = 79,
          RESTOREDEFAULTS = 80,
          GETTEMP = 82,
          GETTEMP2 = 83,	//Only valid on some models
          GETERROR = 90,
          GETENCODERMODE = 91,
          SETM1ENCODERMODE = 92,
          SETM2ENCODERMODE = 93,
          WRITENVM = 94,
          READNVM = 95,	//Reloads values from Flash into Ram
          SETCONFIG = 98,
          GETCONFIG = 99,
          SETM1MAXCURRENT = 133,
          SETM2MAXCURRENT = 134,
          GETM1MAXCURRENT = 135,
          GETM2MAXCURRENT = 136,
          SETPWMMODE = 148,
          GETPWMMODE = 149,
          FLAGBOOTLOADER = 255  //Only available via USB communications
};

/******************************************************************************
* PiGpioD Error Definitions
******************************************************************************/

#define PI_INIT_FAILED       -1 // gpioInitialise failed
#define PI_BAD_USER_GPIO     -2 // GPIO not 0-31
#define PI_BAD_GPIO          -3 // GPIO not 0-53
#define PI_BAD_MODE          -4 // mode not 0-7
#define PI_BAD_LEVEL         -5 // level not 0-1
#define PI_BAD_PUD           -6 // pud not 0-2
#define PI_BAD_PULSEWIDTH    -7 // pulsewidth not 0 or 500-2500
#define PI_BAD_DUTYCYCLE     -8 // dutycycle outside set range
#define PI_BAD_TIMER         -9 // timer not 0-9
#define PI_BAD_MS           -10 // ms not 10-60000
#define PI_BAD_TIMETYPE     -11 // timetype not 0-1
#define PI_BAD_SECONDS      -12 // seconds < 0
#define PI_BAD_MICROS       -13 // micros not 0-999999
#define PI_TIMER_FAILED     -14 // gpioSetTimerFunc failed
#define PI_BAD_WDOG_TIMEOUT -15 // timeout not 0-60000
#define PI_NO_ALERT_FUNC    -16 // DEPRECATED
#define PI_BAD_CLK_PERIPH   -17 // clock peripheral not 0-1
#define PI_BAD_CLK_SOURCE   -18 // DEPRECATED
#define PI_BAD_CLK_MICROS   -19 // clock micros not 1, 2, 4, 5, 8, or 10
#define PI_BAD_BUF_MILLIS   -20 // buf millis not 100-10000
#define PI_BAD_DUTYRANGE    -21 // dutycycle range not 25-40000
#define PI_BAD_DUTY_RANGE   -21 // DEPRECATED (use PI_BAD_DUTYRANGE)
#define PI_BAD_SIGNUM       -22 // signum not 0-63
#define PI_BAD_PATHNAME     -23 // can't open pathname
#define PI_NO_HANDLE        -24 // no handle available
#define PI_BAD_HANDLE       -25 // unknown handle
#define PI_BAD_IF_FLAGS     -26 // ifFlags > 3
#define PI_BAD_CHANNEL      -27 // DMA channel not 0-14
#define PI_BAD_PRIM_CHANNEL -27 // DMA primary channel not 0-14
#define PI_BAD_SOCKET_PORT  -28 // socket port not 1024-32000
#define PI_BAD_FIFO_COMMAND -29 // unrecognized fifo command
#define PI_BAD_SECO_CHANNEL -30 // DMA secondary channel not 0-6
#define PI_NOT_INITIALISED  -31 // function called before gpioInitialise
#define PI_INITIALISED      -32 // function called after gpioInitialise
#define PI_BAD_WAVE_MODE    -33 // waveform mode not 0-3
#define PI_BAD_CFG_INTERNAL -34 // bad parameter in gpioCfgInternals call
#define PI_BAD_WAVE_BAUD    -35 // baud rate not 50-250K(RX)/50-1M(TX)
#define PI_TOO_MANY_PULSES  -36 // waveform has too many pulses
#define PI_TOO_MANY_CHARS   -37 // waveform has too many chars
#define PI_NOT_SERIAL_GPIO  -38 // no bit bang serial read on GPIO
#define PI_BAD_SERIAL_STRUC -39 // bad (null) serial structure parameter
#define PI_BAD_SERIAL_BUF   -40 // bad (null) serial buf parameter
#define PI_NOT_PERMITTED    -41 // GPIO operation not permitted
#define PI_SOME_PERMITTED   -42 // one or more GPIO not permitted
#define PI_BAD_WVSC_COMMND  -43 // bad WVSC subcommand
#define PI_BAD_WVSM_COMMND  -44 // bad WVSM subcommand
#define PI_BAD_WVSP_COMMND  -45 // bad WVSP subcommand
#define PI_BAD_PULSELEN     -46 // trigger pulse length not 1-100
#define PI_BAD_SCRIPT       -47 // invalid script
#define PI_BAD_SCRIPT_ID    -48 // unknown script id
#define PI_BAD_SER_OFFSET   -49 // add serial data offset > 30 minutes
#define PI_GPIO_IN_USE      -50 // GPIO already in use
#define PI_BAD_SERIAL_COUNT -51 // must read at least a byte at a time
#define PI_BAD_PARAM_NUM    -52 // script parameter id not 0-9
#define PI_DUP_TAG          -53 // script has duplicate tag
#define PI_TOO_MANY_TAGS    -54 // script has too many tags
#define PI_BAD_SCRIPT_CMD   -55 // illegal script command
#define PI_BAD_VAR_NUM      -56 // script variable id not 0-149
#define PI_NO_SCRIPT_ROOM   -57 // no more room for scripts
#define PI_NO_MEMORY        -58 // can't allocate temporary memory
#define PI_SOCK_READ_FAILED -59 // socket read failed
#define PI_SOCK_WRIT_FAILED -60 // socket write failed
#define PI_TOO_MANY_PARAM   -61 // too many script parameters (> 10)
#define PI_NOT_HALTED       -62 // DEPRECATED
#define PI_SCRIPT_NOT_READY -62 // script initialising
#define PI_BAD_TAG          -63 // script has unresolved tag
#define PI_BAD_MICS_DELAY   -64 // bad MICS delay (too large)
#define PI_BAD_MILS_DELAY   -65 // bad MILS delay (too large)
#define PI_BAD_WAVE_ID      -66 // non existent wave id
#define PI_TOO_MANY_CBS     -67 // No more CBs for waveform
#define PI_TOO_MANY_OOL     -68 // No more OOL for waveform
#define PI_EMPTY_WAVEFORM   -69 // attempt to create an empty waveform
#define PI_NO_WAVEFORM_ID   -70 // no more waveforms
#define PI_I2C_OPEN_FAILED  -71 // can't open I2C device
#define PI_SER_OPEN_FAILED  -72 // can't open serial device
#define PI_SPI_OPEN_FAILED  -73 // can't open SPI device
#define PI_BAD_I2C_BUS      -74 // bad I2C bus
#define PI_BAD_I2C_ADDR     -75 // bad I2C address
#define PI_BAD_SPI_CHANNEL  -76 // bad SPI channel
#define PI_BAD_FLAGS        -77 // bad i2c/spi/ser open flags
#define PI_BAD_SPI_SPEED    -78 // bad SPI speed
#define PI_BAD_SER_DEVICE   -79 // bad serial device name
#define PI_BAD_SER_SPEED    -80 // bad serial baud rate
#define PI_BAD_PARAM        -81 // bad i2c/spi/ser parameter
#define PI_I2C_WRITE_FAILED -82 // i2c write failed
#define PI_I2C_READ_FAILED  -83 // i2c read failed
#define PI_BAD_SPI_COUNT    -84 // bad SPI count
#define PI_SER_WRITE_FAILED -85 // ser write failed
#define PI_SER_READ_FAILED  -86 // ser read failed
#define PI_SER_READ_NO_DATA -87 // ser read no data available
#define PI_UNKNOWN_COMMAND  -88 // unknown command
#define PI_SPI_XFER_FAILED  -89 // spi xfer/read/write failed
#define PI_BAD_POINTER      -90 // bad (NULL) pointer
#define PI_NO_AUX_SPI       -91 // no auxiliary SPI on Pi A or B
#define PI_NOT_PWM_GPIO     -92 // GPIO is not in use for PWM
#define PI_NOT_SERVO_GPIO   -93 // GPIO is not in use for servo pulses
#define PI_NOT_HCLK_GPIO    -94 // GPIO has no hardware clock
#define PI_NOT_HPWM_GPIO    -95 // GPIO has no hardware PWM
#define PI_BAD_HPWM_FREQ    -96 // hardware PWM frequency not 1-125M
#define PI_BAD_HPWM_DUTY    -97 // hardware PWM dutycycle not 0-1M
#define PI_BAD_HCLK_FREQ    -98 // hardware clock frequency not 4689-250M
#define PI_BAD_HCLK_PASS    -99 // need password to use hardware clock 1
#define PI_HPWM_ILLEGAL    -100 // illegal, PWM in use for main clock
#define PI_BAD_DATABITS    -101 // serial data bits not 1-32
#define PI_BAD_STOPBITS    -102 // serial (half) stop bits not 2-8
#define PI_MSG_TOOBIG      -103 // socket/pipe message too big
#define PI_BAD_MALLOC_MODE -104 // bad memory allocation mode
#define PI_TOO_MANY_SEGS   -105 // too many I2C transaction segments
#define PI_BAD_I2C_SEG     -106 // an I2C transaction segment failed
#define PI_BAD_SMBUS_CMD   -107 // SMBus command not supported by driver
#define PI_NOT_I2C_GPIO    -108 // no bit bang I2C in progress on GPIO
#define PI_BAD_I2C_WLEN    -109 // bad I2C write length
#define PI_BAD_I2C_RLEN    -110 // bad I2C read length
#define PI_BAD_I2C_CMD     -111 // bad I2C command
#define PI_BAD_I2C_BAUD    -112 // bad I2C baud rate, not 50-500k
#define PI_CHAIN_LOOP_CNT  -113 // bad chain loop count
#define PI_BAD_CHAIN_LOOP  -114 // empty chain loop
#define PI_CHAIN_COUNTER   -115 // too many chain counters
#define PI_BAD_CHAIN_CMD   -116 // bad chain command
#define PI_BAD_CHAIN_DELAY -117 // bad chain delay micros
#define PI_CHAIN_NESTING   -118 // chain counters nested too deeply
#define PI_CHAIN_TOO_BIG   -119 // chain is too long
#define PI_DEPRECATED      -120 // deprecated function removed
#define PI_BAD_SER_INVERT  -121 // bit bang serial invert not 0 or 1
#define PI_BAD_EDGE        -122 // bad ISR edge value, not 0-2
#define PI_BAD_ISR_INIT    -123 // bad ISR initialisation
#define PI_BAD_FOREVER     -124 // loop forever must be last command
#define PI_BAD_FILTER      -125 // bad filter parameter
#define PI_BAD_PAD         -126 // bad pad number
#define PI_BAD_STRENGTH    -127 // bad pad drive strength
#define PI_FIL_OPEN_FAILED -128 // file open failed
#define PI_BAD_FILE_MODE   -129 // bad file mode
#define PI_BAD_FILE_FLAG   -130 // bad file flag
#define PI_BAD_FILE_READ   -131 // bad file read
#define PI_BAD_FILE_WRITE  -132 // bad file write
#define PI_FILE_NOT_ROPEN  -133 // file not open for read
#define PI_FILE_NOT_WOPEN  -134 // file not open for write
#define PI_BAD_FILE_SEEK   -135 // bad file seek
#define PI_NO_FILE_MATCH   -136 // no files match pattern
#define PI_NO_FILE_ACCESS  -137 // no permission to access file
#define PI_FILE_IS_A_DIR   -138 // file is a directory
#define PI_BAD_SHELL_STATUS -139 // bad shell return status
#define PI_BAD_SCRIPT_NAME -140 // bad script name
#define PI_BAD_SPI_BAUD    -141 // bad SPI baud rate, not 50-500k
#define PI_NOT_SPI_GPIO    -142 // no bit bang SPI in progress on GPIO
#define PI_BAD_EVENT_ID    -143 // bad event id

#define PI_PIGIF_ERR_0    -2000
#define PI_PIGIF_ERR_99   -2099

#define PI_CUSTOM_ERR_0   -3000
#define PI_CUSTOM_ERR_999 -3999


// pigif_bad_send           = -2000,
// pigif_bad_recv           = -2001,
// pigif_bad_getaddrinfo    = -2002,
// pigif_bad_connect        = -2003,
// pigif_bad_socket         = -2004,
// pigif_bad_noib           = -2005,
// pigif_duplicate_callback = -2006,
// pigif_bad_malloc         = -2007,
// pigif_bad_callback       = -2008,
// pigif_notify_failed      = -2009,
// pigif_callback_not_found = -2010,



/******************************************************************************
* RoboClaw Definitions
******************************************************************************/
