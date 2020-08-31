/**
 * @file Arduino.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019-2020 Daniel Starke
 * @date 2019-03-08
 * @version 2020-08-30
 * 
 * @todo use busy-wait only with more than 1 core (https://stackoverflow.com/a/150971/2525536)
 * @todo check if we can call ser_write() less often to increase throughput; use separate write thread alternatively
 * @todo check if the delay() call crash (executing address 0) is caused by a GCC 9.2.0 compiler bug or not (-Og instead of -O2 works)
 */
#include <signal.h>
#include <stdarg.h>
#include <time.h>
#include "arduino/Framing.hpp"
#include "arduino/Protocol.hpp"
#include "utility/cvutf8.h"
#include "utility/getopt.h"
#include "utility/mingw-unicode.h"
#include "utility/serial.h"
#include "utility/target.h"
#include "utility/tchar.h"
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "EEPROM.h"
#include "LiquidCrystal.h"
#include "SPI.h"
#include "Wire.h"
#include "License.i"


#undef HIGH
#undef LOW
#undef INPUT
#undef INPUT_PULLUP
#undef OUTPUT
#undef SERIAL
#undef DISPLAY
#undef LSBFIRST
#undef MSBFIRST
#undef CHANGE
#undef FALLING
#undef RISING
#undef DEFAULT
#undef EXTERNAL
#undef INTERNAL1V1
#undef INTERNAL
#undef INTERNAL2V56
#undef INTERNAL2V56_EXTCAP
#undef F_CPU
#undef digitalRead
#undef analogRead
#undef pulseIn
#undef pulseInLong
#undef shiftIn


#if defined(PCF_IS_WIN)
#define boolean BOOLEAN
#include <windows.h>
#include <conio.h>
#include <fcntl.h>
#include <io.h>
#undef ERROR
typedef HANDLE tThread;
extern "C" {
#ifdef __TINYC__
int _CRT_glob = 0;
#else
extern int _CRT_glob;
#endif
#ifdef UNICODE
extern void __wgetmainargs(int *, wchar_t ***, wchar_t ***, int, int *);
#define __tgetmainargs __wgetmainargs
#else /* UNICODE */
extern void __getmainargs(int *, char ***, char ***, int, int *);
#define __tgetmainargs __getmainargs
#endif /* UNICODE */
} /* extern "C" */
#elif defined(PCF_IS_LINUX)
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/stat.h>
typedef pthread_t tThread;
#else /* not PCF_IS_WIN and not PCF_IS_LINUX */
#error Unsupported target OS.
#endif


/** Defines the default timeout for remote function calls in milliseconds. */
#define DEFAULT_TIMEOUT 1000


/** Timeout resolution in milliseconds. Needed to handle SIGINT/SIGTERM quickly. */
#define TIMEOUT_RESOLUTION 100


/** Defines the maximum frame size payload including sequence number and CRC16. */
#define MAX_FRAME_SIZE 1024


/** Defines the maximum number of unprocessed frames. */
#define MAX_FRAME_QUEUE 16


/** Returns the given UTF-8 error message string. */
#define MSGU(x) ((const char *)fmsg[(x)])


/** Returns the given native (TCHAR) error message string. */
#define MSGT(x) ((const TCHAR *)fmsg[(x)])


/* local defaults */
uint8_t _sizeof_bool = uint8_t(sizeof(bool));
uint8_t _sizeof_signed_char = uint8_t(sizeof(signed char));
uint8_t _sizeof_unsigned_char = uint8_t(sizeof(unsigned char));
uint8_t _sizeof_signed_short = uint8_t(sizeof(signed short));
uint8_t _sizeof_unsigned_short = uint8_t(sizeof(unsigned short));
uint8_t _sizeof_signed_int = uint8_t(sizeof(signed int));
uint8_t _sizeof_unsigned_int = uint8_t(sizeof(unsigned int));
uint8_t _sizeof_signed_long = uint8_t(sizeof(signed long));
uint8_t _sizeof_unsigned_long = uint8_t(sizeof(unsigned long));
uint8_t _sizeof_signed_long_long = uint8_t(sizeof(signed long long));
uint8_t _sizeof_unsigned_long_long = uint8_t(sizeof(unsigned long long));
uint8_t _sizeof_float = uint8_t(sizeof(float));
uint8_t _sizeof_double = uint8_t(sizeof(double));
uint8_t _sizeof_long_double = uint8_t(sizeof(long double));
uint8_t _sizeof_void_ptr = uint8_t(sizeof(void *));
uint8_t _sizeof_ptrdiff_t = uint8_t(sizeof(ptrdiff_t));
uint8_t _sizeof_wchar_t = uint8_t(sizeof(wchar_t));
uint8_t _sizeof_size_t = uint8_t(sizeof(size_t));


/* defaults for Arduino.h */
volatile bool _INTERRUPTS = true;
uint8_t _HIGH = 1;
uint8_t _LOW = 0;
uint8_t _INPUT = 0;
uint8_t _INPUT_PULLUP = 2;
uint8_t _OUTPUT = 1;
uint8_t _SERIAL = 0;
uint8_t _DISPLAY = 1;
uint8_t _LSBFIRST = 0;
uint8_t _MSBFIRST = 1;
uint8_t _CHANGE = 1;
uint8_t _FALLING = 2;
uint8_t _RISING = 3;
uint8_t _DEFAULT = 0;
uint8_t _EXTERNAL = 4;
uint8_t _INTERNAL1V1 = 8;
uint8_t _INTERNAL = 8;
uint8_t _INTERNAL2V56 = 9;
uint8_t _INTERNAL2V56_EXTCAP = 13;
uint32_t _F_CPU = UINT32_C(10000000);


/* defaults for HardwareSerial.h */
uint8_t _SERIAL_5N1 = 0x00;
uint8_t _SERIAL_6N1 = 0x02;
uint8_t _SERIAL_7N1 = 0x04;
uint8_t _SERIAL_8N1 = 0x06;
uint8_t _SERIAL_5N2 = 0x08;
uint8_t _SERIAL_6N2 = 0x0A;
uint8_t _SERIAL_7N2 = 0x0C;
uint8_t _SERIAL_8N2 = 0x0E;
uint8_t _SERIAL_5E1 = 0x20;
uint8_t _SERIAL_6E1 = 0x22;
uint8_t _SERIAL_7E1 = 0x24;
uint8_t _SERIAL_8E1 = 0x26;
uint8_t _SERIAL_5E2 = 0x28;
uint8_t _SERIAL_6E2 = 0x2A;
uint8_t _SERIAL_7E2 = 0x2C;
uint8_t _SERIAL_8E2 = 0x2E;
uint8_t _SERIAL_5O1 = 0x30;
uint8_t _SERIAL_6O1 = 0x32;
uint8_t _SERIAL_7O1 = 0x34;
uint8_t _SERIAL_8O1 = 0x36;
uint8_t _SERIAL_5O2 = 0x38;
uint8_t _SERIAL_6O2 = 0x3A;
uint8_t _SERIAL_7O2 = 0x3C;
uint8_t _SERIAL_8O2 = 0x3E;


/* defaults for SPI.h */
uint8_t _SPI_CLOCK_DIV2 = 0x04;
uint8_t _SPI_CLOCK_DIV4 = 0x00;
uint8_t _SPI_CLOCK_DIV8 = 0x05;
uint8_t _SPI_CLOCK_DIV16 = 0x01;
uint8_t _SPI_CLOCK_DIV32 = 0x06;
uint8_t _SPI_CLOCK_DIV64 = 0x02;
uint8_t _SPI_CLOCK_DIV128 = 0x03;
uint8_t _SPI_CLOCK_DIV256 = INVALID_SPI_CLOCK;
uint8_t _SPI_CLOCK_DIV512 = INVALID_SPI_CLOCK;
uint8_t _SPI_CLOCK_DIV1024 = INVALID_SPI_CLOCK;
uint8_t _SPI_MODE0 = 0x00;
uint8_t _SPI_MODE1 = 0x04;
uint8_t _SPI_MODE2 = 0x08;
uint8_t _SPI_MODE3 = 0x0C;


/* defaults for pins_arduino.h */
uint8_t _NUM_DIGITAL_PINS = 0;
uint8_t * _DIGITAL_PIN_TO_PORT = NULL;
uint8_t * _DIGITAL_PIN_TO_INTERRUPT = NULL;
bool * _DIGITAL_PIN_PWM_SUPPORT = NULL;
uint8_t _LED_BUILTIN = INVALID_PIN;
uint8_t _LED_BUILTIN_RX = INVALID_PIN;
uint8_t _LED_BUILTIN_TX = INVALID_PIN;
uint8_t _PIN_SPI_SS0 = INVALID_PIN;
uint8_t _PIN_SPI_SS1 = INVALID_PIN;
uint8_t _PIN_SPI_SS2 = INVALID_PIN;
uint8_t _PIN_SPI_SS3 = INVALID_PIN;
uint8_t _PIN_SPI_MOSI = INVALID_PIN;
uint8_t _PIN_SPI_MISO = INVALID_PIN;
uint8_t _PIN_SPI_SCK = INVALID_PIN;
uint8_t _PIN_WIRE_SDA = INVALID_PIN;
uint8_t _PIN_WIRE_SCL = INVALID_PIN;
uint8_t _PIN_WIRE1_SDA = INVALID_PIN;
uint8_t _PIN_WIRE1_SCL = INVALID_PIN;
uint8_t _NUM_ANALOG_INPUTS = 0;
uint8_t _A0 = INVALID_PIN;
uint8_t _A1 = INVALID_PIN;
uint8_t _A2 = INVALID_PIN;
uint8_t _A3 = INVALID_PIN;
uint8_t _A4 = INVALID_PIN;
uint8_t _A5 = INVALID_PIN;
uint8_t _A6 = INVALID_PIN;
uint8_t _A7 = INVALID_PIN;
uint8_t _A8 = INVALID_PIN;
uint8_t _A9 = INVALID_PIN;
uint8_t _A10 = INVALID_PIN;
uint8_t _A11 = INVALID_PIN;
uint8_t _A12 = INVALID_PIN;
uint8_t _A13 = INVALID_PIN;
uint8_t _A14 = INVALID_PIN;
uint8_t _A15 = INVALID_PIN;
uint8_t _A16 = INVALID_PIN;
uint8_t _A17 = INVALID_PIN;
uint8_t _A18 = INVALID_PIN;
uint8_t _A19 = INVALID_PIN;
uint8_t _A20 = INVALID_PIN;
uint8_t _A21 = INVALID_PIN;
uint8_t _A22 = INVALID_PIN;
uint8_t _A23 = INVALID_PIN;
uint8_t _A24 = INVALID_PIN;
uint8_t _A25 = INVALID_PIN;
uint8_t _A26 = INVALID_PIN;
uint8_t _A27 = INVALID_PIN;
uint8_t _A28 = INVALID_PIN;
uint8_t _A29 = INVALID_PIN;
uint8_t _A30 = INVALID_PIN;
uint8_t _A31 = INVALID_PIN;
uint8_t _A32 = INVALID_PIN;
uint8_t _A33 = INVALID_PIN;
uint8_t _A34 = INVALID_PIN;
uint8_t _A35 = INVALID_PIN;
uint8_t _A36 = INVALID_PIN;
uint8_t _A37 = INVALID_PIN;
uint8_t _A38 = INVALID_PIN;
uint8_t _A39 = INVALID_PIN;
uint8_t _A40 = INVALID_PIN;
uint8_t _A41 = INVALID_PIN;
uint8_t _A42 = INVALID_PIN;
uint8_t _A43 = INVALID_PIN;
uint8_t _A44 = INVALID_PIN;
uint8_t _A45 = INVALID_PIN;
uint8_t _A46 = INVALID_PIN;
uint8_t _A47 = INVALID_PIN;
uint8_t _A48 = INVALID_PIN;
uint8_t _A49 = INVALID_PIN;
uint8_t _A50 = INVALID_PIN;
uint8_t _A51 = INVALID_PIN;
uint8_t _A52 = INVALID_PIN;
uint8_t _A53 = INVALID_PIN;
uint8_t _A54 = INVALID_PIN;
uint8_t _A55 = INVALID_PIN;
uint8_t _A56 = INVALID_PIN;
uint8_t _A57 = INVALID_PIN;
uint8_t _A58 = INVALID_PIN;
uint8_t _A59 = INVALID_PIN;
uint8_t _A60 = INVALID_PIN;
uint8_t _A61 = INVALID_PIN;
uint8_t _A62 = INVALID_PIN;
uint8_t _A63 = INVALID_PIN;


namespace adde {


/* special handling in _FV::set() for these OP codes */
struct _GET_DEV_NAME : _void {};
struct _GET_CONSTANTS : _void {};
struct _GET_PIN_MAP_D : _void {};
struct _GET_PIN_MAP_A : _void {};
struct _GET_PIN_MAP_S : _void {};
struct _GET_PIN_MAP_P : _void {};
struct _GET_PIN_MAP_I : _void {};
struct _SPI_GET_CONSTANTS : _void {};
struct _HARDWARE_SERIAL_GET_CONSTANTS : _void {};


} /* namespace adde */


/**
 * Defines a date time structure.
 */
typedef struct {
	uint16_t year; /**< The year in the range 2000..2136. */
	uint8_t month; /**< The month of the year in the range 1..12. */
	uint8_t day;   /**< The day of the month in the range 1..31. */
	uint8_t hour;  /**< The hour of the day in the range 0..23. */
	uint8_t min;   /**< The minute of the hour in the range 0..59. */
	uint8_t sec;   /**< The second of the minute in the range 0..59. */
} tDateTime;


/* forward declarations */
/**
 *   0 = no signal received (startup/normal running phase)
 *  >0 = signal received; termination requested (termination phase)
 * >99 = abort; critical termination request (destruction phase; no more data to the device)
 */
static volatile int signalReceived = 0;
static bool writeToRemoteHandler(const uint8_t val, const bool eof);
static int printToLog(const TCHAR * fmt, ...) PRINTF_ATTR(1, 2);
static int printToLogWithMore(const TCHAR * fmt, ...) PRINTF_ATTR(1, 2);
static int printToLogV(const TCHAR * fmt, va_list ap, const bool fau) PRINTF_ATTR(1, 0);
static int printToErr(const TCHAR * fmt, ...) PRINTF_ATTR(1, 2);
static int printToErrV(const TCHAR * fmt, va_list ap) PRINTF_ATTR(1, 0);
static int beginTimerRes(const unsigned int resolution);
static int endTimerRes(const unsigned int resolution);
static int createThread(tThread * handle, void (* fn)(void * data), void * data);
static int joinThread(tThread * handle);
static void killThread(tThread * handle);


namespace {


#if defined(PCF_IS_WIN)
class ScopedGuard {
private:
	CRITICAL_SECTION cs[1];
	bool locked;
public:
	explicit ScopedGuard(const size_t spinCount) {
		InitializeCriticalSectionAndSpinCount(this->cs, static_cast<DWORD>(spinCount));
		this->locked = false;
	}
	~ScopedGuard() {
		DeleteCriticalSection(this->cs);
	}
	void lock() {
		EnterCriticalSection(this->cs);
		this->locked = true;
	}
	void unlock() {
		LeaveCriticalSection(this->cs);
		this->locked = false;
	}
	bool isLocked() const {
		return this->locked;
	}
};
#elif defined(PCF_IS_LINUX)
class ScopedGuard {
private:
	pthread_mutex_t mutex[1];
	bool locked;
public:
	explicit ScopedGuard(const size_t /* spinCount */) {
		pthread_mutex_init(this->mutex, NULL);
		this->locked = false;
	}
	~ScopedGuard() {
		pthread_mutex_destroy(this->mutex);
	}
	void lock() {
		pthread_mutex_lock(this->mutex);
		this->locked = true;
	}
	void unlock() {
		pthread_mutex_unlock(this->mutex);
		this->locked = false;
	}
	bool isLocked() const {
		return this->locked;
	}
};
#endif /* PCF_IS_LINUX */


/** Class for handling frame sequence numbers. */
class Seq {
private:
	uint8_t seq;
public:
	explicit Seq(): seq(0) {}
	uint8_t next() {
		do {
			this->seq++;
		} while (this->seq == 0);
		return this->seq;
	}
};


/** Class to handle instances on the remote device. */
class RemoteInstance {
public:
	static const uint8_t npos = 255;
private:
	uint8_t count;
	uint8_t * used;
public:
	explicit RemoteInstance():
		count(0),
		used(NULL)
	{}
	
	explicit RemoteInstance(const uint8_t remoteCount):
		count(remoteCount),
		used(static_cast<uint8_t *>(calloc(1, static_cast<size_t>((remoteCount + 7) >> 3))))
	{}
	
	RemoteInstance(const RemoteInstance & o):
		count(o.count),
		used(static_cast<uint8_t *>(calloc(1, static_cast<size_t>((o.count + 7) >> 3))))
	{}
	
	RemoteInstance(RemoteInstance && o):
		count(o.count),
		used(o.used)
	{
		o.count = 0;
		o.used = NULL;
	}
	
	~RemoteInstance() {
		if (this->used != NULL) free(this->used);
	}
	
	RemoteInstance & operator= (const RemoteInstance & o) {
		if (this != &o) {
			this->count = o.count;
			this->used = static_cast<uint8_t *>(calloc(1, static_cast<size_t>((o.count + 7) >> 3)));
		}
		return *this;
	}
	
	RemoteInstance & operator= (RemoteInstance && o) {
		if (this != &o) {
			if (this->used != NULL) free(this->used);
			this->count = o.count;
			this->used = static_cast<uint8_t *>(calloc(1, static_cast<size_t>((o.count + 7) >> 3)));
			o.count = 0;
			o.used = NULL;
		}
		return *this;
	}
	
	size_t size() const {
		return this->count;
	}
	
	bool get(const uint8_t pos) const {
		if ((pos >> 3) >= this->count) return false;
		const uint8_t & byte = this->used[pos >> 3];
		return bitRead(byte, (pos & 0x07)) != 0;
	}
	
	void set(const uint8_t pos, const bool val) {
		if ((pos >> 3) >= this->count) return;
		uint8_t & byte = this->used[pos >> 3];
		bitWrite(byte, pos & 0x07, val);
	}
	
	uint8_t add() {
		for (uint8_t i = 0; i < this->count; i++) {
			if ( ! this->get(i) ) {
				this->set(i, true);
				return i;
			}
		}
		return RemoteInstance::npos;
	}
	
	bool erase(const uint8_t pos) {
		const bool res = this->get(pos);
		this->set(pos, false);
		return res;
	}
};


/* forward declaration */
inline void waitForPendingCallResults();
inline void removePendingCallResults();


/** Helper class for finer object/data destruction control. */
struct DelayedDestruction {
	tThread conThread[1];
	tThread commThread[1];
	tSerial * remoteConn;
	ScopedGuard * framesGuard;
	ScopedGuard * ferrGuard;
	ScopedGuard * flogGuard;
	bool highResTimer;
	FILE * flog;
	uint8_t * _DIGITAL_PIN_TO_PORT;
	uint8_t * _DIGITAL_PIN_TO_INTERRUPT;
	bool * _DIGITAL_PIN_PWM_SUPPORT;
	char * device;
	
	explicit DelayedDestruction():
		conThread{static_cast<tThread>(0)},
		commThread{static_cast<tThread>(0)},
		remoteConn(NULL),
		framesGuard(NULL),
		ferrGuard(NULL),
		flogGuard(NULL),
		highResTimer(false),
		flog(NULL),
		_DIGITAL_PIN_TO_PORT(NULL),
		_DIGITAL_PIN_TO_INTERRUPT(NULL),
		_DIGITAL_PIN_PWM_SUPPORT(NULL),
		device(NULL)
	{}
	
	~DelayedDestruction() {
		signalReceived += 100;
		if (this->conThread[0] != static_cast<tThread>(0)) {
			killThread(this->conThread);
			this->conThread[0] = static_cast<tThread>(0);
		}
		if (this->commThread[0] != static_cast<tThread>(0)) {
			joinThread(this->commThread);
			this->commThread[0] = static_cast<tThread>(0);
		}
		if (this->framesGuard != NULL) {
			removePendingCallResults();
		}
		if (this->remoteConn != NULL) {
			printToLog(_T("DEV\tclose\n"));
			tSerial * tmp = this->remoteConn;
			this->remoteConn = NULL;
			ser_delete(tmp);
		}
		if (this->framesGuard != NULL) {
			ScopedGuard * tmp = this->framesGuard;
			this->framesGuard = NULL;
			delete tmp;
		}
		if (this->ferrGuard != NULL) {
			ScopedGuard * tmp = this->ferrGuard;
			this->ferrGuard = NULL;
			delete tmp;
		}
		if (this->flogGuard != NULL) {
			ScopedGuard * tmp = this->flogGuard;
			this->flogGuard = NULL;
			delete tmp;
		}
		if ( this->highResTimer ) endTimerRes(1000);
		if (this->flog != NULL) {
			FILE * tmp = this->flog;
			this->flog = NULL;
			fclose(tmp);
		}
		if (this->_DIGITAL_PIN_TO_PORT != NULL) {
			free(this->_DIGITAL_PIN_TO_PORT);
			this->_DIGITAL_PIN_TO_PORT = NULL;
		}
		if (this->_DIGITAL_PIN_TO_INTERRUPT != NULL) {
			free(this->_DIGITAL_PIN_TO_INTERRUPT);
			this->_DIGITAL_PIN_TO_INTERRUPT = NULL;
		}
		if (this->_DIGITAL_PIN_PWM_SUPPORT != NULL) {
			free(this->_DIGITAL_PIN_PWM_SUPPORT);
			this->_DIGITAL_PIN_PWM_SUPPORT = NULL;
		}
		if (this->device != NULL) {
			free(this->device);
			this->device = NULL;
		}
	}
};


} /* anonymous namespace */


/**
 * Helper structure to pass the context data to the thread wrapper for createThread().
 */
typedef struct {
	void (* fn)(void * data);
	void * data;
} tThreadWrapper;


/** Interrupt callback handler. */
typedef void (* tInterrupt)(void);


typedef enum {
	MSGT_SUCCESS = 0,
	MSGT_CRIT_EARLY_API,
	MSGT_CRIT_CMD_LINE,
	MSGT_ERR_NO_MEM,
	MSGT_ERR_OPT_NO_ARG,
	MSGT_ERR_OPT_BAD_TIMEOUT,
	MSGT_ERR_OPT_NO_DEVICE,
	MSGT_ERR_OPT_AMB_C,
	MSGT_ERR_OPT_AMB_S,
	MSGT_ERR_OPT_AMB_X,
	MSGT_ERR_LOG_CREATE,
	MSGT_ERR_REMOTE_CONNECT,
	MSGT_ERR_CREATE_THREAD,
	MSGT_ERR_BROKEN_FRAME,
	MSGT_ERR_OP_MISMATCH,
	MSGT_ERR_TIMEOUT,
	MSGT_ERR_REMOTE,
	MSGT_ERR_REMOTE_ERROR,
	MSGT_ERR_REMOTE_FRAME,
	MSGT_ERR_REMOTE_CALL,
	MSGT_ERR_REMOTE_WRITE,
	MSGT_ERR_REMOTE_READ,
	MSGT_ERR_REMOTE_RESULT,
	MSGT_ERR_PROT_VERSION,
	MSGT_ERR_PTR_RESULT,
	MSGT_ERR_REMOTE_INST_COUNT,
	MSGT_WARN_EARLY_API,
	MSGT_WARN_SET_PRIO,
	MSGT_WARN_API_INV_PIN,
	MSGT_WARN_API_INV_INTERRUPT,
	MSGT_WARN_API_INV_VALUE,
	MSGT_WARN_API_INV_MODE,
	MSGT_WARN_API_INV_BITORDER,
	MSGT_WARN_API_INV_CLOCK_DIV,
	MSGT_WARN_API_INV_COL,
	MSGT_WARN_API_INV_COL_COUNT,
	MSGT_WARN_API_INV_ROW,
	MSGT_WARN_API_INV_ROW_COUNT,
	MSGT_WARN_API_INV_CHAR_SIZE,
	MSGT_WARN_API_INV_CHAR_LOCATION,
	MSGT_WARN_API_INV_INSTANCE,
	MSGT_WARN_API_MAX_INTERRUPT,
	MSGT_INFO_SIGTERM,
	MSGT_INFO_OP_UNSUPPORTED,
	MSG_COUNT
} tMessage;


static const void * fmsg[MSG_COUNT] = {
	/* MSGT_SUCCESS                    */ _T(""), /* never used for output */
	/* MSGT_CRIT_EARLY_API             */ _T("Critical: Arduino API function call before setup().\n"),
	/* MSGT_CRIT_CMD_LINE              */ _T("Critical: Failed to get command-line arguments.\n"),
	/* MSGT_ERR_NO_MEM                 */ _T("Error: Failed to allocate memory.\n"),
	/* MSGT_ERR_OPT_NO_ARG             */ _T2("Error: Option argument is missing for '%" PRTCHAR "'.\n"),
	/* MSGT_ERR_OPT_BAD_TIMEOUT        */ _T2("Error: Invalid timeout value. (%" PRTCHAR ")"),
	/* MSGT_ERR_OPT_NO_DEVICE          */ _T("Error: Missing device.\n"),
	/* MSGT_ERR_OPT_AMB_C              */ _T("Error: Unknown or ambiguous option '-%c'.\n"),
	/* MSGT_ERR_OPT_AMB_S              */ _T2("Error: Unknown or ambiguous option '%" PRTCHAR "'.\n"),
	/* MSGT_ERR_OPT_AMB_X              */ _T("Error: Unknown option character '0x%02X'.\n"),
	/* MSGT_ERR_LOG_CREATE             */ _T("Error: Failed to create log file.\n"),
	/* MSGT_ERR_REMOTE_CONNECT         */ _T("Error: Failed to connect to remote device via %s.\n"),
	/* MSGT_ERR_CREATE_THREAD          */ _T("Error: Failed to create a thread.\n"),
	/* MSGT_ERR_BROKEN_FRAME           */ _T("Error: Received broken data frame from remote device.\n"),
	/* MSGT_ERR_OP_MISMATCH            */ _T("Error: OP code of received remote call result does not match.\n"),
	/* MSGT_ERR_TIMEOUT                */ _T("Error: The operation timed out.\n"),
	/* MSGT_ERR_REMOTE                 */ _T("Error: Error on remote device.\n"),
	/* MSGT_ERR_REMOTE_ERROR           */ _T2("Error: Error on remote device (type: %" PRTCHAR ").\n"),
	/* MSGT_ERR_REMOTE_FRAME           */ _T2("Error: Error on remote device (type: %" PRTCHAR ", seq: %u).\n"),
	/* MSGT_ERR_REMOTE_CALL            */ _T2("Error: Function call on remote device failed (type: %" PRTCHAR ", seq: %u, op: %" PRTCHAR ").\n"),
	/* MSGT_ERR_REMOTE_WRITE           */ _T("Error: Failed to write data to remote device.\n"),
	/* MSGT_ERR_REMOTE_READ            */ _T("Error: Failed to read data from remote device.\n"),
	/* MSGT_ERR_REMOTE_RESULT          */ _T("Error: Received an invalid result from remote device.\n"),
	/* MSGT_ERR_PROT_VERSION           */ _T("Error: Incompatible protocol version on remote device (local: 0x%04X, remote: 0x%04X).\n"),
	/* MSGT_ERR_PTR_RESULT             */ _T("Error: Remote functions returning pointer values are not supported.\n"),
	/* MSGT_ERR_REMOTE_INST_COUNT      */ _T2("Error: The maximum number of active remote instances reached for %" PRTCHAR ".\n"),
	/* MSGT_WARN_EARLY_API             */ _T("Warning: Arduino API function call before setup().\n"),
	/* MSGT_WARN_SET_PRIO              */ _T("Warning: Failed to set current process priority to high.\n"),
	/* MSGT_WARN_API_INV_PIN           */ _T2("Warning: An invalid pin number was passed to %" PRTCHAR " (pin: %u).\n"),
	/* MSGT_WARN_API_INV_INTERRUPT     */ _T2("Warning: An invalid interrupt number was passed to %" PRTCHAR " (interrupt: %u).\n"),
	/* MSGT_WARN_API_INV_VALUE         */ _T2("Warning: An invalid value was passed to %" PRTCHAR " (val: %u).\n"),
	/* MSGT_WARN_API_INV_MODE          */ _T2("Warning: An invalid mode was passed to %" PRTCHAR " (mode: %u).\n"),
	/* MSGT_WARN_API_INV_BITORDER      */ _T2("Warning: An invalid bit order was passed to %" PRTCHAR " (bit order: %u).\n"),
	/* MSGT_WARN_API_INV_CLOCK_DIV     */ _T2("Warning: An invalid clock divider was passed to %" PRTCHAR " (clock divider: %u).\n"),
	/* MSGT_WARN_API_INV_COL           */ _T2("Warning: An invalid column was passed to %" PRTCHAR " (col: %i).\n"),
	/* MSGT_WARN_API_INV_COL_COUNT     */ _T2("Warning: An invalid column count was passed to %" PRTCHAR " (cols: %u).\n"),
	/* MSGT_WARN_API_INV_ROW           */ _T2("Warning: An invalid row was passed to %" PRTCHAR " (row: %i).\n"),
	/* MSGT_WARN_API_INV_ROW_COUNT     */ _T2("Warning: An invalid row count was passed to %" PRTCHAR " (rows: %u).\n"),
	/* MSGT_WARN_API_INV_CHAR_SIZE     */ _T2("Warning: An invalid character size was passed to %" PRTCHAR " (charSize: 0x%02X).\n"),
	/* MSGT_WARN_API_INV_CHAR_LOCATION */ _T2("Warning: An invalid character location was passed to %" PRTCHAR " (location: %u).\n"),
	/* MSGT_WARN_API_INV_INSTANCE      */ _T2("Warning: An invalid instance was accessed in %" PRTCHAR " (instance: %u).\n"),
	/* MSGT_WARN_API_MAX_INTERRUPT     */ _T2("Warning: The maximum number of active interrupts has been reached in %" PRTCHAR " (interrupt: %u).\n"),
	/* MSGT_INFO_SIGTERM               */ _T("Info: Received signal. Finishing current operation.\n"),
	/* MSGT_INFO_OP_UNSUPPORTED        */ _T2("Info: OP code is not supported by the remote device (op: %" PRTCHAR ").\n")
};


static const TCHAR hexStr[] = _T("0123456789ABCDEF");


static const TCHAR * errStr[] = {
	_T("SUCCESS"),
	_T("UNSUPPORTED_OPCODE"),
	_T("BROKEN_FRAME"),
	_T("SIGNATURE_MISMATCH"),
	_T("INVALID_ARGUMENT"),
	_T("UNKNOWN")
};

static const TCHAR * resStr[] = {
	_T("RESULT"),
	_T("INTERRUPT"),
	_T("ERROR"),
	_T("UNKNOWN")
};

static const TCHAR * opStr[] = {
	_T("RESET"),
	_T("GET_PROT_VERSION"),
	_T("GET_DEV_NAME"),
	_T("GET_CONSTANTS"),
	_T("GET_PIN_MAP_D"),
	_T("GET_PIN_MAP_A"),
	_T("GET_PIN_MAP_S"),
	_T("GET_PIN_MAP_P"),
	_T("GET_PIN_MAP_I"),
	_T("PIN_MODE"),
	_T("DIGITAL_READ"),
	_T("DIGITAL_WRITE"),
	_T("ANALOG_READ"),
	_T("ANALOG_REFERENCE"),
	_T("ANALOG_WRITE"),
	_T("ANALOG_READ_RESOLUTION"),
	_T("ANALOG_WRITE_RESOLUTION"),
	_T("PULSE_IN"),
	_T("PULSE_IN_LONG"),
	_T("SHIFT_IN"),
	_T("SHIFT_OUT"),
	_T("TONE"),
	_T("NO_TONE"),
	_T("ATTACH_INTERRUPT"),
	_T("DETACH_INTERRUPT"),
	_T("HARDWARE_SERIAL_GET_CONSTANTS"),
	_T("HARDWARE_SERIAL_AVAILABLE"),
	_T("HARDWARE_SERIAL_AVAILABLE_FOR_WRITE"),
	_T("HARDWARE_SERIAL_BEGIN1"),
	_T("HARDWARE_SERIAL_BEGIN2"),
	_T("HARDWARE_SERIAL_END"),
	_T("HARDWARE_SERIAL_PEEK"),
	_T("HARDWARE_SERIAL_READ"),
	_T("HARDWARE_SERIAL_FLUSH"),
	_T("HARDWARE_SERIAL_WRITE"),
	_T("SOFTWARE_SERIAL"),
	_T("SOFTWARE_SERIAL_AVAILABLE"),
	_T("SOFTWARE_SERIAL_BEGIN"),
	_T("SOFTWARE_SERIAL_END"),
	_T("SOFTWARE_SERIAL_IS_LISTENING"),
	_T("SOFTWARE_SERIAL_STOP_LISTENING"),
	_T("SOFTWARE_SERIAL_OVERFLOW"),
	_T("SOFTWARE_SERIAL_PEEK"),
	_T("SOFTWARE_SERIAL_READ"),
	_T("SOFTWARE_SERIAL_LISTEN"),
	_T("SOFTWARE_SERIAL_WRITE"),
	_T("EEPROM_LENGTH"),
	_T("EEPROM_READ"),
	_T("EEPROM_WRITE"),
	_T("EEPROM_UPDATE"),
	_T("LIQUID_CRYSTAL1"),
	_T("LIQUID_CRYSTAL2"),
	_T("LIQUID_CRYSTAL3"),
	_T("LIQUID_CRYSTAL4"),
	_T("LIQUID_CRYSTAL_BEGIN"),
	_T("LIQUID_CRYSTAL_CLEAR"),
	_T("LIQUID_CRYSTAL_HOME"),
	_T("LIQUID_CRYSTAL_NO_DISPLAY"),
	_T("LIQUID_CRYSTAL_DISPLAY"),
	_T("LIQUID_CRYSTAL_NO_BLINK"),
	_T("LIQUID_CRYSTAL_BLINK"),
	_T("LIQUID_CRYSTAL_NO_CURSOR"),
	_T("LIQUID_CRYSTAL_CURSOR"),
	_T("LIQUID_CRYSTAL_SCROLL_DISPLAY_LEFT"),
	_T("LIQUID_CRYSTAL_SCROLL_DISPLAY_RIGHT"),
	_T("LIQUID_CRYSTAL_LEFT_TO_RIGHT"),
	_T("LIQUID_CRYSTAL_RIGHT_TO_LEFT"),
	_T("LIQUID_CRYSTAL_NO_AUTOSCROLL"),
	_T("LIQUID_CRYSTAL_AUTOSCROLL"),
	_T("LIQUID_CRYSTAL_SET_ROW_OFFSETS"),
	_T("LIQUID_CRYSTAL_CREATE_CHAR"),
	_T("LIQUID_CRYSTAL_SET_CURSOR"),
	_T("LIQUID_CRYSTAL_COMMAND"),
	_T("LIQUID_CRYSTAL_WRITE"),
	_T("SPI_GET_CONSTANTS"),
	_T("SPI_BEGIN"),
	_T("SPI_END"),
	_T("SPI_BEGIN_TRANSACTION"),
	_T("SPI_END_TRANSACTION"),
	_T("SPI_SET_CLOCK_DIVIDER"),
	_T("SPI_SET_BIT_ORDER"),
	_T("SPI_SET_DATA_MODE"),
	_T("SPI_TRANSFER"),
	_T("SPI_TRANSFER16"),
	_T("SPI_USING_INTERRUPT"),
	_T("SPI_NOT_USING_INTERRUPT"),
	_T("WIRE_BEGIN1"),
	_T("WIRE_BEGIN2"),
	_T("WIRE_END"),
	_T("WIRE_SET_CLOCK"),
	_T("WIRE_REQUEST_FROM"),
	_T("WIRE_BEGIN_TRANSMISSION"),
	_T("WIRE_END_TRANSMISSION"),
	_T("WIRE_AVAILABLE"),
	_T("WIRE_PEEK"),
	_T("WIRE_READ"),
	_T("WIRE_FLUSH"),
	_T("WIRE_WRITE"),
	_T("WIRE_ON_RECEIVE"),
	_T("WIRE_ON_REQUEST"),
	_T("UNKNOWN")
};


static DelayedDestruction INIT_PRIO_ATTR(999) delayedDtor; /* destroyed after the following instances */
static int verbose = 1; /* 0 = critical, 1 = error, 2 = warn, 3 = info, 4 = debug */
static char * device = NULL;
static TCHAR * logFile = NULL;
static size_t commTimeout = DEFAULT_TIMEOUT; /* communication timeout in milliseconds */
static bool useLazyCallResult = false;
static int64_t startTime = 0; /* microseconds since the program was started */
#if defined(PCF_IS_WIN)
static LARGE_INTEGER perfFreq;
#endif /* PCF_IS_WIN */
static tSerial * remoteConn;
static ScopedGuard * framesGuard = NULL;
static Seq INIT_PRIO_ATTR(1000) frameSeq;
static ::adde::_RC<::adde::_FVI> pendingCallResults[256];
static volatile uint8_t pendingCallResultCount = 0;
static volatile tInterrupt ints[MAX_NUM_INTERRUPTS] = {0};
static FILE * fin = NULL;
static FILE * fout = NULL;
static FILE * ferr = NULL;
static FILE * flog = NULL;
static ScopedGuard * ferrGuard = NULL;
static ScopedGuard * flogGuard = NULL;
static volatile int conPeekBuffer = -1;
static volatile bool initializedMain = false;
static Framing<MAX_FRAME_SIZE> INIT_PRIO_ATTR(1000) framing(writeToRemoteHandler);


/* API instances */
static uint8_t hwSerCount, swSerCount, eepromCount, lcdCount, spiCount, wireCount;
static RemoteInstance INIT_PRIO_ATTR(1000) swSerInst;
static RemoteInstance INIT_PRIO_ATTR(1000) lcdInst;
ConsoleSerial INIT_PRIO_ATTR(1000) Serial;
EEPROMClass INIT_PRIO_ATTR(1000) EEPROM(0);
HardwareSerial INIT_PRIO_ATTR(1000) Serial1(0);
HardwareSerial INIT_PRIO_ATTR(1000) Serial2(1);
HardwareSerial INIT_PRIO_ATTR(1000) Serial3(2);
HardwareSerial INIT_PRIO_ATTR(1000) Serial4(3);
HardwareSerial INIT_PRIO_ATTR(1000) Serial5(4);
HardwareSerial INIT_PRIO_ATTR(1000) Serial6(5);
HardwareSerial INIT_PRIO_ATTR(1000) Serial7(6);
HardwareSerial INIT_PRIO_ATTR(1000) Serial8(7);
SPIClass INIT_PRIO_ATTR(1000) SPI(0);
TwoWire INIT_PRIO_ATTR(1000) Wire(0);
TwoWire INIT_PRIO_ATTR(1000) Wire1(1);


#undef atexit
extern "C" {
static void __empty() {}
void yield(void) __attribute__((weak, alias("__empty")));
int _atexit(void (* fn)()) { return atexit(fn); }
void init(void) __attribute__((weak));
void init(void) {}
void initVariant(void) __attribute__((weak));
void initVariant(void) {}
} /* extern "C" */


/* forward declarations */
static int addeMain(int argc, TCHAR ** argv);
static void checkInitializedMain(const bool warn = true);
static void printHelp(void);
static void handleSignal(int signum);
#ifndef UNICODE
static char * strndupInternal(const char * str, const size_t n);
#endif /* !UNICODE */
static int printToLog(const TCHAR * fmt, ...) PRINTF_ATTR(1, 2);
static int printToLogWithMore(const TCHAR * fmt, ...) PRINTF_ATTR(1, 2);
static int printToLogV(const TCHAR * fmt, va_list ap, const bool fau) PRINTF_ATTR(1, 0);
static int addeLogV(const char * fmt, va_list ap, const bool fau) PRINTF_ATTR(1, 0);
static int printToErr(const TCHAR * fmt, ...) PRINTF_ATTR(1, 2);
static int printToErrV(const TCHAR * fmt, va_list ap) PRINTF_ATTR(1, 0);
static void checkDigitalPin(const TCHAR * ref, const uint8_t pin);
static int beginTimerRes(const unsigned int resolution);
static int endTimerRes(const unsigned int resolution);
static int createThread(tThread * handle, void (* fn)(void * data), void * data);
static int joinThread(tThread * handle);
static void killThread(tThread * handle);
static bool writeToRemoteHandler(const uint8_t val, const bool eof);
static void readFromRemoteHandler(const uint8_t seq, uint8_t * buf, const size_t len, const bool err);
static void readFromRemoteThread(void * data);
static void readConsoleThread(void * data);
static size_t getSpinMilli(void);
static uint64_t getCpuInterval(void);
template <typename T> static bool spinWait(volatile T & var, const T match, const size_t timeout);
template <typename T> static bool spinWaitNot(volatile T & var, const T match, const size_t timeout);
template <typename T> static bool spinWaitDelay(volatile T & var, const T match, const size_t shortTimeout, const size_t longTimeout);
template <typename T> static bool spinWaitNotDelay(volatile T & var, const T match, const size_t shortTimeout, const size_t longTimeout);
static int64_t getAbsTime(void);
static void ntodt(tDateTime * dt, uint32_t num);


namespace {


/** Pointer argument type with fixed size. */
template <
	typename T,
	typename B,
	typename enable_if<is_arithmetic<T>::value>::type * = nullptr
>
struct PtrArg {
	typedef B type;
	const T * ptr;
	size_t size;
	explicit PtrArg(const T * p, const size_t s): ptr(p), size(s) {}
	int printTo(FILE * fd) const {
		int ret = _ftprintf(fd, _T("{"));
		if (ret <= 0) return ret;
		for (size_t i = 0; i < this->size; i++) {
			if (i != 0) {
				const int res = _ftprintf(fd, _T(", "));
				if (res <= 0) return res;
				ret += res;
			}
			const int res = ::adde::_V<T>(ptr[i]).printTo(fd);
			if (res <= 0) return res;
			ret += res;
		}
		const int res = _ftprintf(fd, _T("}"));
		if (res <= 0) return res;
		ret += res;
		return ret;
	}
};


/** Converts the given pointer to a pointer argument type with the given base type. */
template <typename B, typename T>
PtrArg<T, B> makePtrArg(const T * p, const size_t s) {
	return PtrArg<T, B>(p, s);
}


/** Null-terminated C string argument type. */
struct CStrArg {
	typedef adde::_char type;
	const char * ptr;
	size_t size;
	explicit CStrArg(const char * p): ptr(p), size(strlen(ptr) + 1) {}
	int printTo(FILE * fd) const {
		int ret = _ftprintf(fd, _T("\""));
		if (ret <= 0) return ret;
		{
			TCHAR * str = _tfromUtf8N(this->ptr, this->size);
			if (str == NULL) return -1;
			const int res = _ftprintf(fd, _T2("%" PRTCHAR), str);
			if (str != reinterpret_cast<const TCHAR *>(this->ptr)) free(str);
			if (res <= 0) return res;
			ret += res;
		}
		const int res = _ftprintf(fd, _T("\""));
		if (res <= 0) return res;
		ret += res;
		return ret;
	}
};


template <typename T> struct IsPtrArg : false_type {};
template <typename T, typename B> struct IsPtrArg<PtrArg<T, B>> : true_type {};
template <> struct IsPtrArg<CStrArg> : true_type {};


template <typename T, typename U>
bool checkAndWrite(const OpCode::Type op, const size_t arg, const T mapped, const U raw) {
	if (sizeof(T) != sizeof(U) && mapped != raw) {
		printToLog(_T2("DEV\tout\ttrunc type\t%" PRTCHAR "\t%u\t%u -> %u\n"), opStr[op], static_cast<unsigned int>(arg), static_cast<unsigned int>(sizeof(U)), static_cast<unsigned int>(sizeof(T)));
	}
	return framing.write(mapped);
}


template <typename T>
typename enable_if<is_integral<typename T::type>::value && is_unsigned<typename T::type>::value, bool>::type writeRemoteType(const OpCode::Type op, const size_t arg, const T val) {
	switch (val.size()) {
	case 1: return checkAndWrite(op, arg, static_cast<uint8_t>(val.value), val.value);
	case 2: return checkAndWrite(op, arg, static_cast<uint16_t>(val.value), val.value);
	case 4: return checkAndWrite(op, arg, static_cast<uint32_t>(val.value), val.value);
	case 8: return checkAndWrite(op, arg, static_cast<uint64_t>(val.value), val.value);
	default:
		printToLog(_T2("DEV\tout\tinvalid type\t%" PRTCHAR "\t%u\t%u\n"), opStr[op], static_cast<unsigned int>(arg), static_cast<unsigned int>(val.size()));
		return false;
	}
}


template <typename T>
typename enable_if<is_integral<typename T::type>::value && is_signed<typename T::type>::value, bool>::type writeRemoteType(const OpCode::Type op, const size_t arg, const T val) {
	switch (val.size()) {
	case 1: return checkAndWrite(op, arg, static_cast<int8_t>(val.value), val.value);
	case 2: return checkAndWrite(op, arg, static_cast<int16_t>(val.value), val.value);
	case 4: return checkAndWrite(op, arg, static_cast<int32_t>(val.value), val.value);
	case 8: return checkAndWrite(op, arg, static_cast<int64_t>(val.value), val.value);
	default:
		printToLog(_T2("DEV\tout\tinvalid type\t%" PRTCHAR "\t%u\t%u\n"), opStr[op], static_cast<unsigned int>(arg), static_cast<unsigned int>(val.size()));
		return false;
	}
}


template <typename T>
typename enable_if<is_floating_point<typename T::type>::value, bool>::type writeRemoteType(const OpCode::Type op, const size_t arg, const T val) {
	switch (val.size()) {
	case 4: return checkAndWrite(op, arg, static_cast<float>(val.value), val.value);
	case 8: return checkAndWrite(op, arg, static_cast<double>(val.value), val.value);
	case 16: return checkAndWrite(op, arg, static_cast<long double>(val.value), val.value);
	default:
		printToLog(_T2("DEV\tout\tinvalid type\t%" PRTCHAR "\t%u\t%u\n"), opStr[op], static_cast<unsigned int>(arg), static_cast<unsigned int>(val.size()));
		return false;
	}
}


template <typename T>
typename enable_if<IsPtrArg<T>::value, bool>::type writeRemoteType(const OpCode::Type op, const size_t arg, const T val) {
	const uint16_t size = static_cast<uint16_t>(val.size);
	if (size != val.size) {
		printToLog(_T2("DEV\tout\ttrunc arr\t%" PRTCHAR "\t%u\t%u -> %u\n"), opStr[op], static_cast<unsigned int>(arg), static_cast<unsigned int>(val.size), static_cast<unsigned int>(size));
	}
	if ( ! checkAndWrite(op, arg, size, val.size) ) return false;
	for (uint16_t i = 0; i < size; i++) {
		if ( ! writeRemoteType(op, arg, typename T::type(val.ptr[i])) ) return false;
	}
	return true;
}


bool writeRemoteTypes(const OpCode::Type /* op */, const size_t /* arg */) {
	return true;
}


template <typename T0, typename ...Args>
bool writeRemoteTypes(const OpCode::Type op, const size_t arg /* = 0 */, const T0 val, Args... args) {
	return writeRemoteType(op, arg, val) && writeRemoteTypes(op, arg + 1, args...);
}


/**
 * Argument list termination function which prints a line-feed.
 * 
 * @return same as _ftprintf
 */
int printArgsToLog() {
	return _ftprintf(flog, _T("\n"));
}


/**
 * Prints out the given argument list to flog with a line-feed at the end.
 * 
 * @param[in] val - current element to print
 * @param[in] args - remaining elements to print
 * @return same as _ftprintf
 */
template <typename T0, typename ...Args>
int printArgsToLog(const T0 val, Args... args) {
	int ret = _ftprintf(flog, _T("\t"));
	if (ret <= 0) return ret;
	int res = val.printTo(flog);
	if (res < 0) return res;
	ret += res;
	res = printArgsToLog(args...);
	if (res < 0) return res;
	ret += res;
	return ret;
};


/**
 * Helper function to read back a given type from the passed buffer.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @param[out] out - variable to set
 * @return true on success, false on out-of-boundary error
 */
template <typename T, typename enable_if<is_arithmetic<T>::value>::type * = nullptr>
bool readFrameValue(uint8_t * & buf, size_t & len, T & out) {
	if (sizeof(T) > len) return false;
	if (sizeof(T) > 1) {
#if __FRAMING_HPP__IS_BIG_ENDIAN
		memcpy(reinterpret_cast<uint8_t *>(&out), buf, sizeof(T));
#else /* reverse little endian to big endian */
		uint8_t * ptr = reinterpret_cast<uint8_t *>(&out) + sizeof(T) - 1;
		for (size_t i = 0; i < sizeof(T); i++) {
			*ptr-- = buf[i];
		}
#endif
	} else {
		out = *buf;
	}
	buf += sizeof(T);
	len = size_t(len - sizeof(T));
	return true;
}


/**
 * Helper function to read back a given type from the passed buffer.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @param[out] out - variable to set
 * @return true on success, false on out-of-boundary error
 */
template <
	typename T,
	typename B = typename remove_pointer<T>::type,
	typename enable_if<is_pointer<T>::value && is_arithmetic<B>::value>::type * = nullptr
>
bool readFrameValue(uint8_t * & buf, size_t & len, T & out) {
	if (sizeof(uint16_t) > len) return false;
	uint16_t outLen;
	if ( ! readFrameValue(buf, len, outLen) ) return false;
	if (size_t(outLen) > len) return false;
#if !__FRAMING_HPP__IS_BIG_ENDIAN
	/* reverse little endian to big endian */
	if (sizeof(B) > 1) {
		for (uint16_t i = 0; i < outLen; i++) {
			reverse(buf + (i * sizeof(B)), buf +  + ((i + 1) * sizeof(B)));
		}
	}
#endif
	out = buf;
	buf += outLen;
	len = size_t(len - outLen);
	return true;
}


template <typename T, typename U>
bool checkAndReadFrameValue(const OpCode::Type op, uint8_t * & buf, size_t & len, T & mapped, U & raw) {
	const bool res = readFrameValue(buf, len, mapped);
	raw = static_cast<U>(mapped);
	if (sizeof(T) != sizeof(U) && mapped != raw) {
		printToLog(_T2("DEV\tin\ttrunc type\t%" PRTCHAR "\t%u -> %u\n"), opStr[op], static_cast<unsigned int>(sizeof(T)), static_cast<unsigned int>(sizeof(U)));
	}
	if (flogGuard != NULL && flog != NULL) {
		printToLogWithMore(_T2("DEV\tin\tcallRes\t%" PRTCHAR ""), opStr[op]);
		printArgsToLog(::adde::_V<U>(raw));
		fflush(flog);
		flogGuard->unlock();
	}
	return res;
}


/**
 * Extracts the call result from the given buffer.
 * 
 * @param[in] op - remote call OP code for logging output
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @param[out] out - ::adde::_V variable to set
 * @return true on success, else false
 */
template <typename R, typename enable_if<::adde::_IsV<R>::value && is_integral<typename R::type>::value && is_unsigned<typename R::type>::value>::type * = nullptr>
bool extractCallResult(const OpCode::Type op, uint8_t * & buf, size_t & len, R & out) {
	switch (out.size()) {
	case 1: { uint8_t u8 = 0; return checkAndReadFrameValue(op, buf, len, u8, out.value); }
	case 2: { uint16_t u16 = 0; return checkAndReadFrameValue(op, buf, len, u16, out.value); }
	case 4: { uint32_t u32 = 0; return checkAndReadFrameValue(op, buf, len, u32, out.value); }
	case 8: { uint64_t u64 = 0; return checkAndReadFrameValue(op, buf, len, u64, out.value); }
	default:
		printToLog(_T2("DEV\tin\tinvalid type\t%" PRTCHAR "\t%u\n"), opStr[op], static_cast<unsigned int>(out.size()));
		return false;
	}
}


/**
 * Extracts the call result from the given buffer.
 * 
 * @param[in] op - remote call OP code for logging output
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @param[out] out - ::adde::_V variable to set
 * @return true on success, else false
 */
template <typename R, typename enable_if<::adde::_IsV<R>::value && is_integral<typename R::type>::value && is_signed<typename R::type>::value>::type * = nullptr>
bool extractCallResult(const OpCode::Type op, uint8_t * & buf, size_t & len, R & out) {
	switch (out.size()) {
	case 1: { int8_t i8 = 0; return checkAndReadFrameValue(op, buf, len, i8, out.value); }
	case 2: { int16_t i16 = 0; return checkAndReadFrameValue(op, buf, len, i16, out.value); }
	case 4: { int32_t i32 = 0; return checkAndReadFrameValue(op, buf, len, i32, out.value); }
	case 8: { int64_t i64 = 0; return checkAndReadFrameValue(op, buf, len, i64, out.value); }
	default:
		printToLog(_T2("DEV\tin\tinvalid type\t%" PRTCHAR "\t%u\n"), opStr[op], static_cast<unsigned int>(out.size()));
		return false;
	}
}


/**
 * Extracts the call result from the given buffer.
 * 
 * @param[in] op - remote call OP code for logging output
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @param[out] out - ::adde::_V variable to set
 * @return true on success, else false
 */
template <typename R, typename enable_if<::adde::_IsV<R>::value && is_floating_point<typename R::type>::value>::type * = nullptr>
bool extractCallResult(const OpCode::Type op, uint8_t * & buf, size_t & len, R & out) {
	switch (out.size()) {
	case 4: { float f32 = 0.0f; return checkAndReadFrameValue(op, buf, len, f32, out.value); }
	case 8: { double f64 = 0.0; return checkAndReadFrameValue(op, buf, len, f64, out.value); }
	case 16: { long double f80 = 0.0L; return checkAndReadFrameValue(op, buf, len, f80, out.value); }
	default:
		printToLog(_T2("DEV\tin\tinvalid type\t%" PRTCHAR "\t%u\n"), opStr[op], static_cast<unsigned int>(out.size()));
		return false;
	}
}


/**
 * Extracts the call result from the given buffer.
 * 
 * @param[in] op - remote call OP code for logging output
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @param[out] out - ::adde::_V variable to set
 * @return true on success, else false
 */
template <typename R, typename enable_if<is_same<R, ::adde::_void>::value>::type * = nullptr>
bool extractCallResult(const OpCode::Type op, uint8_t * & /* buf */, size_t & /* len */, R & /* out */) {
	return true;
}


/**
 * Calls a function on the remote device with the given arguments.
 * 
 * @param[in] timeout - function timeout added to commTimeout in milliseconds
 * @param[in] op - function OP code
 * @param[in] args - function arguments
 * @return call result and function result
 * @tparam R - remote function result type
 * @tparam ...Args - function argument types
 */
template <typename R, typename ...Args>
::adde::_F<R> callWith(const size_t timeout, const OpCode::Type op, Args... args) {
	checkInitializedMain();
	const unsigned long start = millis();
	uint8_t seq;
	::adde::_RC<::adde::_FV<R>> callResultFuture = ::adde::_make_rc<::adde::_FV<R>>(start, commTimeout + timeout, static_cast<int>(op));
	bool locked = false;
	/* prepare call result future element */
	for (;;) {
		if (signalReceived != 0) goto onError;
		framesGuard->lock();
		locked = true;
		if (pendingCallResultCount < 255) break;
		framesGuard->unlock();
		locked = false;
	};
	seq = frameSeq.next();
	pendingCallResults[seq] = ::adde::_RC<::adde::_FVI>(callResultFuture);
	pendingCallResultCount++;
	/* perform remote call */
	if ( ! framing.beginTransmission(seq) ) goto onError;
	if ( ! framing.write(uint8_t(op)) ) goto onError;
	if ( ! writeRemoteTypes(op, 0, args...) ) goto onError;
	if ( ! framing.endTransmission() ) goto onError;
	framesGuard->unlock();
	locked = false;
	if (flogGuard != NULL && flog != NULL) {
		printToLogWithMore(_T2("DEV\tout\tcall\t%" PRTCHAR), opStr[op]);
		printArgsToLog(args...);
		fflush(flog);
		flogGuard->unlock();
	}
	/* wait for call result if lazy call results are disabled */
	if ( ! useLazyCallResult ) {
		callResultFuture->waitForResult();
		if ( ! callResultFuture->valid ) {
			if (verbose > 0) printToErr(MSGT(MSGT_ERR_TIMEOUT));
			signalReceived += 100;
			exit(EXIT_FAILURE); /* timeout */
		}
		if ( ! callResultFuture->success ) {
			switch (op) {
			case OpCode::HARDWARE_SERIAL_GET_CONSTANTS:
			case OpCode::SPI_GET_CONSTANTS:
				break;
			default:
				delay(1);
				signalReceived += 100;
				exit(EXIT_FAILURE); /* remote call failed; error message is written in readFromRemoteHandler() */
				break;
			}
		}
	}
onError:
	if ( locked ) framesGuard->unlock();
	return ::adde::_F<R>(callResultFuture);
}


/**
 * Calls a function on the remote device with the given arguments and no additional timeout.
 * 
 * @param[in] op - function OP code
 * @param[in] args - function arguments
 * @return call result and function result
 * @tparam R - remote function result type
 * @tparam ...Args - function argument types
 */
template <typename R, typename ...Args>
inline ::adde::_F<R> callWith(const OpCode::Type op, Args... args) {
	return callWith<R>(0, op, args...);
}


/**
 * Waits for outstanding remote call results.
 */
inline void waitForPendingCallResults() {
	for (size_t i = 0; i < 256; i++) {
		framesGuard->lock();
		::adde::_RC<::adde::_FVI> future = pendingCallResults[i];
		framesGuard->unlock();
		if ( future ) future->waitForResult();
	}
}


/**
 * Removes outstanding remote call results.
 */
inline void removePendingCallResults() {
	framesGuard->lock();
	for (size_t i = 0; i < 256; i++) {
		pendingCallResults[i] = ::adde::_RC<::adde::_FVI>();
	}
	pendingCallResultCount = 0;
	framesGuard->unlock();
}


} /* anonymous namespace */


/**
 * Main entry point.
 */
int _tmain(int argc, TCHAR ** argv) {
	int ret = addeMain(argc, argv);
	if (ret != EXIT_SUCCESS) goto onError;
	if (signalReceived != 0) goto onSuccess;
	
	init();
	initVariant();
	
	setup();
	while (signalReceived == 0) loop();
	
	signalReceived++;
onSuccess:
	ret = EXIT_SUCCESS;
onError:
	return ret;
};


/**
 * ADDE initialization entry point. May be called before C main().
 * 
 * @param[in] argc - argument count
 * @param[in] argv - argument value array
 * @return exit code
 */
static int addeMain(int argc, TCHAR ** argv) {
	if ( initializedMain ) return EXIT_SUCCESS;
	initializedMain = true;
	enum {
		GETOPT_LICENSE = 1,
		GETOPT_UTF8 = 2,
		GETOPT_VERSION = 3
	};
	char POSIXLY_CORRECT[] = "POSIXLY_CORRECT=";
	int ret = EXIT_FAILURE;
	TCHAR * strNum;
	struct option longOptions[] = {
		{_T("license"),      no_argument,       NULL, GETOPT_LICENSE},
		{_T("utf8"),         no_argument,       NULL,    GETOPT_UTF8},
		{_T("version"),      no_argument,       NULL, GETOPT_VERSION},
		{_T("coarse-times"), no_argument,       NULL,        _T('c')},
		{_T("fast"),         no_argument,       NULL,        _T('f')},
		{_T("help"),         no_argument,       NULL,        _T('h')},
		{_T("log"),          required_argument, NULL,        _T('l')},
		{_T("priority"),     no_argument,       NULL,        _T('p')},
		{_T("reset"),        no_argument,       NULL,        _T('r')},
		{_T("timeout"),      required_argument, NULL,        _T('t')},
		{_T("verbose"),      no_argument,       NULL,        _T('v')},
		{NULL, 0, NULL, 0}
	};
#ifdef UNICODE
	bool utf8Out = false;
#endif /* UNICODE */
	bool appendLog = false;
	bool resetRemote = false;
	bool useHighPriority = false;
	bool useHighResTimer = true;

	/* ensure that the environment does not change the argument parser behavior */
	putenv(POSIXLY_CORRECT);
	
	/* set the file descriptors */
	fin  = stdin;
	fout = stdout;
	ferr = stderr;
	flog = NULL;
	
	_DIGITAL_PIN_TO_PORT = NULL;
	_DIGITAL_PIN_TO_INTERRUPT = NULL;
	
#if defined(PCF_IS_WIN) && !defined(__CYGWIN__)
	_setmode(_fileno(fin), _O_BINARY);
	_setmode(_fileno(fout), _O_BINARY);
#ifdef UNICODE
	_setmode(_fileno(ferr), _O_U16TEXT);
#endif /* UNICODE */
#endif /* PCF_IS_WIN */

	if (argc < 2) {
		printHelp();
		return EXIT_FAILURE;
	}
	
	while (1) {
		const int res = getopt_long(argc, argv, _T(":cfhl:prt:v"), longOptions, NULL);

		if (res == -1) break;
		switch (res) {
		case GETOPT_LICENSE:
#ifdef UNICODE
			{
				TCHAR * text = _tfromUtf8N(licenseText, sizeof(licenseText));
				if (text == NULL) {
					_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
					goto onError;
				}
				_ftprintf(ferr, _T2("%" PRTCHAR), text);
				if (text != reinterpret_cast<const TCHAR *>(licenseText)) free(text);
			}
#else /* !UNICODE */
			_ftprintf(ferr, _T2("%" PRTCHAR), licenseText);
#endif /* UNICODE */
			signalReceived++;
			exit(EXIT_SUCCESS);
			break;
		case GETOPT_UTF8:
#ifdef UNICODE
			utf8Out = true;
			_setmode(_fileno(ferr), _O_U8TEXT);
#endif /* UNICODE */
			break;
		case GETOPT_VERSION:
			_ftprintf(fout, _T("%u.%u.%u"), ADDE);
			signalReceived++;
			exit(EXIT_SUCCESS);
			break;
		case _T('c'):
			useHighResTimer = false;
			break;
		case _T('f'):
			useLazyCallResult = true;
			break;
		case _T('h'):
			printHelp();
			signalReceived++;
			exit(EXIT_SUCCESS);
			break;
		case _T('l'):
			logFile = optarg;
			break;
		case _T('p'):
			useHighPriority = true;
			break;
		case _T('r'):
			resetRemote = true;
			break;
		case _T('t'):
			commTimeout = static_cast<size_t>(_tcstol(optarg, &strNum, 10));
			if (commTimeout < TIMEOUT_RESOLUTION || strNum == NULL || *strNum != 0) {
				_ftprintf(ferr, MSGT(MSGT_ERR_OPT_BAD_TIMEOUT), optarg);
				goto onError;
			}
			break;
		case _T('v'):
			verbose++;
			break;
		case _T(':'):
			_ftprintf(ferr, MSGT(MSGT_ERR_OPT_NO_ARG), argv[optind - 1]);
			goto onError;
			break;
		case _T('?'):
			if (_istprint(static_cast<TINT>(optopt)) != 0) {
				_ftprintf(ferr, MSGT(MSGT_ERR_OPT_AMB_C), optopt);
			} else if (optopt == 0) {
				_ftprintf(ferr, MSGT(MSGT_ERR_OPT_AMB_S), argv[optind - 1]);
			} else {
				_ftprintf(ferr, MSGT(MSGT_ERR_OPT_AMB_X), static_cast<int>(optopt));
			}
			goto onError;
			break;
		default:
			abort();
		}
	}
	
	if (optind >= argc) {
		_ftprintf(ferr, MSGT(MSGT_ERR_OPT_NO_DEVICE));
		goto onError;
	}
	device = _ttoUtf8(argv[optind]);
	if (device == NULL) {
		_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
		goto onError;
	}
	delayedDtor.device = device;
	
	if (logFile != NULL) {
		if (logFile[0] == _T('+')) {
			logFile++;
			appendLog = true;
		}
#ifdef UNICODE
		if ( utf8Out ) {
			flog = _tfopen(logFile, appendLog ? _T("at,ccs=UTF-8") : _T("wt,ccs=UTF-8"));
		} else {
			flog = _tfopen(logFile, appendLog ? _T("at,ccs=UTF-16LE") : _T("wt,ccs=UTF-16LE"));
		}
#else /* UNICODE */
		flog = _tfopen(logFile, appendLog ? _T("a") : _T("w"));
#endif /* !UNICODE */
		if (flog == NULL) {
			_ftprintf(ferr, MSGT(MSGT_ERR_LOG_CREATE));
			goto onError;
		}
		delayedDtor.flog = flog;
	}
	
#ifdef PCF_IS_WIN
	/* initialize early to enable delay/millis/micros functions */
	QueryPerformanceFrequency(&perfFreq);
#endif /* PCF_IS_WIN */
	
	/* install signal handlers */
	signal(SIGINT, handleSignal);
	signal(SIGTERM, handleSignal);
	
	/* set process priority to high */
	if ( useHighPriority ) {
#if defined(PCF_IS_WIN)
		if (SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS) != TRUE) {
			_ftprintf(ferr, MSGT(MSGT_WARN_SET_PRIO));
		}
#elif defined(PCF_IS_LINUX)
		if (setpriority(PRIO_PROCESS, 0, -20) != 0) {
			_ftprintf(ferr, MSGT(MSGT_WARN_SET_PRIO));
		}
#endif
	}
	
	/* request 1ms timer resolution */
	if ( useHighResTimer ) {
		beginTimerRes(1000);
		delayedDtor.highResTimer = true;
	}
	
	/* initialize time reference for millis() and micros() */
#if defined(PCF_IS_WIN)
	{
		LARGE_INTEGER counter;
		QueryPerformanceCounter(&counter);
		startTime = static_cast<int64_t>(counter.QuadPart);
	}
#elif defined(PCF_IS_LINUX)
	startTime = getAbsTime();
#endif
	
	/* initialize timing variables */
	getSpinMilli();
	getCpuInterval();
	
	/* initialize guard variables */
	framesGuard = new ScopedGuard(getSpinMilli());
	if (framesGuard == NULL) {
		_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
		goto onError;
	}
	delayedDtor.framesGuard = framesGuard;
	ferrGuard = new ScopedGuard(getSpinMilli());
	if (ferrGuard == NULL) {
		_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
		goto onError;
	}
	delayedDtor.ferrGuard = ferrGuard;
	flogGuard = new ScopedGuard(getSpinMilli());
	if (flogGuard == NULL) {
		_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
		goto onError;
	}
	delayedDtor.flogGuard = flogGuard;
	
	if (createThread(delayedDtor.conThread, readConsoleThread, NULL) != 1) {
		_ftprintf(ferr, MSGT(MSGT_ERR_CREATE_THREAD));
		goto onError;
	}
	
onConnect:
	/* connect to remote device */
	printToLog(_T2("DEV\topen\t%s\n"), device);
	remoteConn = ser_create(device, 115200, SFR_8N1, SFC_NONE);
	if (remoteConn == NULL) {
		printToErr(MSGT(MSGT_ERR_REMOTE_CONNECT), device);
		goto onError;
	}
	delayedDtor.remoteConn = remoteConn;
	
	/* wait for remote device to connect */
	delay(1000);
	ser_clear(remoteConn);
	
	/* start receiving data handling thread */
	if (createThread(delayedDtor.commThread, readFromRemoteThread, NULL) != 1) {
		_ftprintf(ferr, MSGT(MSGT_ERR_CREATE_THREAD));
		goto onError;
	}
	delay(100);
	
	if ( resetRemote ) {
		resetRemote = false;
		/* perform remote reset */
		printToLog(_T("DEV\treset\n"));
		if ( ! callWith<::adde::_void>(OpCode::RESET).success() ) goto onError;
		delay(3000);
		/* stop receiving data handling thread */
		signalReceived++;
		joinThread(delayedDtor.commThread);
		delayedDtor.commThread[0] = static_cast<tThread>(0);
		if (signalReceived == 1) signalReceived = 0;
		/* close connection to remote device */
		printToLog(_T("DEV\tclose\n"));
		tSerial * tmp = remoteConn;
		remoteConn = NULL;
		delayedDtor.remoteConn = NULL;
		ser_delete(tmp);
		/* re-connecting to remote device */
		goto onConnect;
	}
	
	{ /* get and check remote protocol version */
		const uint16_t protVer = callWith<::adde::_uint16_t>(OpCode::GET_PROT_VERSION);
		if (verbose > 1) printToLog(_T("DEV\tprot\t0x%04X\n"), static_cast<unsigned int>(protVer));
		if ((protVer & 0xFF00) != (ADDE_PROT_VERSION & 0xFF00)) {
			if (verbose > 0) {
				printToErr(MSGT(MSGT_ERR_PROT_VERSION), static_cast<unsigned int>(ADDE_PROT_VERSION), static_cast<unsigned int>(protVer));
			}
			goto onError;
		}
	}
	
	if (flog != NULL && verbose > 1) {
		/* get the device name for the logs */
		callWith<::adde::_GET_DEV_NAME>(OpCode::GET_DEV_NAME);
	}
	
	/* get constant and pin values (NonDelayVoid cannot be used here) */
	if ( ! callWith<::adde::_GET_CONSTANTS>(OpCode::GET_CONSTANTS).success() ) goto onError;
	callWith<::adde::_GET_PIN_MAP_D>(OpCode::GET_PIN_MAP_D);
	callWith<::adde::_GET_PIN_MAP_A>(OpCode::GET_PIN_MAP_A);
	callWith<::adde::_GET_PIN_MAP_S>(OpCode::GET_PIN_MAP_S);
	callWith<::adde::_GET_PIN_MAP_P>(OpCode::GET_PIN_MAP_P);
	callWith<::adde::_GET_PIN_MAP_I>(OpCode::GET_PIN_MAP_I);
	/* may not be supported by the remote device */
	callWith<::adde::_HARDWARE_SERIAL_GET_CONSTANTS>(OpCode::HARDWARE_SERIAL_GET_CONSTANTS);
	/* may not be supported by the remote device */
	callWith<::adde::_SPI_GET_CONSTANTS>(OpCode::SPI_GET_CONSTANTS);
	
	/* ensure all global constants are received before running user code which may depend on those */
	waitForPendingCallResults();
	ret = EXIT_SUCCESS;
onError:
	if (ret != EXIT_SUCCESS) signalReceived++;
	return ret;
};


/**
 * Checks whether main was already called. Program execution is aborted if this is not the case.
 */
static void checkInitializedMain(const bool warn) {
	if ( initializedMain ) return;
#if defined(PCF_IS_WIN)
	if ( warn ) {
		_ftprintf(stderr, MSGT(MSGT_WARN_EARLY_API));
		fflush(stderr);
	}
	TCHAR ** enpv, ** argv;
	int argc, si = 0;
	__tgetmainargs(&argc, &argv, &enpv, _CRT_glob, &si);
	const int ret = addeMain(argc, argv);
	if (ret != EXIT_SUCCESS) {
		signalReceived += 100;
		exit(ret);
	}
#elif defined(PCF_IS_LINUX)
	if ( warn ) {
		_ftprintf(stderr, MSGT(MSGT_WARN_EARLY_API));
		fflush(stderr);
	}
	int fd = open("/proc/self/cmdline", O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		_ftprintf(stderr, MSGT(MSGT_CRIT_CMD_LINE));
		signalReceived += 100;
		exit(4);
	}
	int ret = EXIT_FAILURE;
	char * args = NULL;
	size_t argsLen = 0;
	size_t argsRead = 0;
	int argc = 0, n;
	char ** argv = NULL;
	for (;;) {
		argsLen += 4096;
		args = static_cast<char *>(realloc(args, argsLen));
		if (args == NULL) {
			_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
			goto onError;
		}
		const ssize_t len = read(fd, args + argsRead, 4096);
		if (len > 0) argsRead += static_cast<size_t>(len);
		if (len < 4096) break;
	}
	close(fd);
	fd = -1;
	if (argsRead <= 0) {
		_ftprintf(stderr, MSGT(MSGT_CRIT_CMD_LINE));
		ret = 3;
		goto onError;
	}
	for (size_t i = 0; i < argsRead; i++) {
		if (args[i] == 0) argc++;
	}
	if (argc <= 0) {
		_ftprintf(stderr, MSGT(MSGT_CRIT_CMD_LINE));
		ret = 2;
		goto onError;
	}
	argv = static_cast<char **>(malloc(sizeof(char *) * static_cast<size_t>(argc)));
	if (argv == NULL) {
		_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
		goto onError;
	}
	n = 0;
	argv[0] = args;
	for (size_t i = 0; i < argsRead; i++) {
		if (args[i] == 0) {
			n++;
			if (n < argc) argv[n] = args + i + 1;
		}
	}
	ret = addeMain(argc, argv);
onError:
	if (fd >= 0) close(fd);
	if (ret != EXIT_SUCCESS) {
		if (args != NULL) free(args);
		if (argv != NULL) free(argv);
		signalReceived += 100;
		exit(ret);
	}
#else
	_ftprintf(stderr, MSGT(MSGT_CRIT_EARLY_API));
	fflush(stderr);
	signalReceived += 100;
	exit(5);
#endif
}


/**
 * Write the help for this application to standard out.
 */
static void printHelp(void) {
	_ftprintf(ferr,
	_T("adde [options] <device>\n")
	_T("\n")
	_T("-c, --coarse-times\n")
	_T("      Use coarse times for time functions. High resolution times are used by\n")
	_T("      default if available.\n")
	_T("-f, --fast\n")
	_T("      Do not actively wait for remote call results until actually needed. This\n")
	_T("      reduces latency and increases remote call throughput in some cases.\n")
	_T("-h, --help\n")
	_T("      Print short usage instruction.\n")
	_T("    --license\n")
	_T("      Displays the licenses for this program.\n")
	_T("-l, --log <file>\n")
	_T("      Write logs to this file. No logs are written by default.\n")
	_T("      The output is appended if the log file has a '+' prefix.\n")
	_T("-p, --priority\n")
	_T("      Runs the process with high execution priority.\n")
#ifdef UNICODE
	_T("    --utf8\n")
	_T("      Sets the encoding for error console and log file to UTF-8.\n")
	_T("      The default is UTF-16.\n")
#endif /* UNICODE */
	_T("-r, --reset\n")
	_T("      Performs a reset on the remote device before starting 3 seconds later.\n")
	_T("-t, --timeout <number>\n")
	_T("      Timeout for remote function calls in milliseconds. Default: %u\n")
	_T("-v\n")
	_T("      Increases verbosity.\n")
	_T("    --version\n")
	_T("      Outputs the program version.\n")
	_T("\n")
	_T("adde %u.%u.%u\n")
	_T("https://github.com/daniel-starke/adde\n")
	, static_cast<unsigned int>(commTimeout)
	, ADDE);
}


/**
 * Handles external signals.
 * 
 * @param[in] signum - received signal number
 */
static void handleSignal(int /* signum */) {
	if (signalReceived == 0 && verbose > 2) printToErr(MSGT(MSGT_INFO_SIGTERM));
	signalReceived++;
}


#ifndef UNICODE
/**
 * Duplicates the given string by taking at most the number of characters given. The result is
 * guaranteed to be null-terminated.
 * 
 * @param[in] str - string to duplicate
 * @param[in] n - max number of characters to copy
 * @return Newly allocated string or NULL on error
 */
static char * strndupInternal(const char * str, const size_t n) {
	if (str == NULL) return NULL;
	size_t len = 0;
	for (const char * ptr = str; len < n && *ptr != 0; len++, ptr++);
	char * res = (char *)malloc(len + 1);
	if (res == NULL) return NULL;
	if (len > 0) memcpy(res, str, len);
	res[len] = 0;
	return res;
}
#endif /* !UNICODE */


/**
 * Like _ftprintf() but locked so that only one thread can write at a time. This writes to flog if
 * defined and adds a time stamp at the beginning.
 * 
 * @param[in] fmt - format string
 * @param[in] ... - format string arguments
 * @return same as _ftprintf
 */
static int printToLog(const TCHAR * fmt, ...) {
	if (flogGuard == NULL || flog == NULL) return -1;
	va_list ap;
	va_start(ap, fmt);
	const int res = printToLogV(fmt, ap, true);
	va_end(ap);
	return res;
}


/**
 * Like _ftprintf() but locked so that only one thread can write at a time. This writes to flog if
 * defined and adds a time stamp at the beginning. The lock is not taken and the output is not
 * flushed at the end. The calling function needs to ensure that both is performed.
 * 
 * @param[in] fmt - format string
 * @param[in] ... - format string arguments
 * @return same as _ftprintf
 */
static int printToLogWithMore(const TCHAR * fmt, ...) {
	if (flogGuard == NULL || flog == NULL) return -1;
	va_list ap;
	va_start(ap, fmt);
	const int res = printToLogV(fmt, ap, false);
	va_end(ap);
	return res;
}


/**
 * Like _vftprintf() but locked so that only one thread can write at a time. This writes to flog if
 * defined and adds a time stamp at the beginning.
 * 
 * @param[in] fmt - format string
 * @param[in,out] ap - argument list
 * @param[in] fau - flush and unlock at the end
 * @return same as _vftprintf
 */
static int printToLogV(const TCHAR * fmt, va_list ap, const bool fau) {
	if (flogGuard == NULL || flog == NULL) return -1;
	const uint64_t now = getAbsTime();
	const unsigned long microsNow = micros();
	tDateTime dt;
	ntodt(&dt, static_cast<uint32_t>(now / UINT64_C(1000000)));
	flogGuard->lock();
	const int res1 = _ftprintf(
		flog,
		_T("%04u-%02u-%02u\t%02u:%02u:%02u.%03u\t%lu\t"),
		static_cast<unsigned int>(dt.year),
		static_cast<unsigned int>(dt.month),
		static_cast<unsigned int>(dt.day),
		static_cast<unsigned int>(dt.hour),
		static_cast<unsigned int>(dt.min),
		static_cast<unsigned int>(dt.sec),
		static_cast<unsigned int>((now / UINT64_C(1000)) % UINT64_C(1000)),
		microsNow
	);
	if (res1 < 27) {
		flogGuard->unlock();
		return res1;
	}
	const int res2 = _vftprintf(flog, fmt, ap);
	if ( fau ) {
		fflush(flog);
		flogGuard->unlock();
	}
	return (res2 < 0) ? res2 : res1 + res2;
}


/**
 * Like fprintf() but locked so that only one thread can write at a time. This writes to flog if
 * defined and adds a time stamp and a USER tag at the beginning.
 * 
 * @param[in] fmt - format string
 * @param[in] ... - format string arguments
 * @return same as fprintf
 */
int addeLog(const char * fmt, ...) {
	if (flogGuard == NULL || flog == NULL) return -1;
	va_list ap;
	va_start(ap, fmt);
	const int res = addeLogV(fmt, ap, true);
	va_end(ap);
	return res;
}


/**
 * Like vfprintf() but locked so that only one thread can write at a time. This writes to flog if
 * defined and adds a time stamp and a USER tag at the beginning.
 * 
 * @param[in] fmt - format string
 * @param[in,out] ap - argument list
 * @param[in] fau - flush and unlock at the end
 * @return same as vfprintf
 */
static int addeLogV(const char * fmt, va_list ap, const bool fau) {
	if (flogGuard == NULL || flog == NULL) return -1;
	const uint64_t now = getAbsTime();
	const unsigned long microsNow = micros();
	tDateTime dt;
	ntodt(&dt, static_cast<uint32_t>(now / UINT64_C(1000000)));
	flogGuard->lock();
	const int res1 = fprintf(
		flog,
		"%04u-%02u-%02u\t%02u:%02u:%02u.%03u\t%lu\tUSER\t",
		static_cast<unsigned int>(dt.year),
		static_cast<unsigned int>(dt.month),
		static_cast<unsigned int>(dt.day),
		static_cast<unsigned int>(dt.hour),
		static_cast<unsigned int>(dt.min),
		static_cast<unsigned int>(dt.sec),
		static_cast<unsigned int>((now / UINT64_C(1000)) % UINT64_C(1000)),
		microsNow
	);
	if (res1 < 27) {
		flogGuard->unlock();
		return res1;
	}
	const int res2 = vfprintf(flog, fmt, ap);
	if ( fau ) {
		fflush(flog);
		flogGuard->unlock();
	}
	return (res2 < 0) ? res2 : res1 + res2;
}


/**
 * Like _ftprintf() but locked so that only one thread can write at a time. This writes to ferr if
 * defined and also duplicated the output using printToLog().
 * 
 * @param[in] fmt - format string
 * @param[in] ... - format string arguments
 * @return same as _ftprintf
 */
static int printToErr(const TCHAR * fmt, ...) {
	if (ferrGuard == NULL || ferr == NULL) return -1;
	va_list ap;
	va_start(ap, fmt);
	const int res = printToErrV(fmt, ap);
	va_end(ap);
	return res;
}


/**
 * Like _vftprintf() but locked so that only one thread can write at a time. This writes to ferr if
 * defined and also duplicated the output using printToLog().
 * 
 * @param[in] fmt - format string
 * @param[in,out] ap - argument list
 * @return same as _vftprintf
 */
static int printToErrV(const TCHAR * fmt, va_list ap) {
	if (flogGuard == NULL || flog == NULL) return -1;
	va_list ap2;
	va_copy(ap2, ap);
	printToLogV(fmt, ap2, true);
	va_end(ap2);
	ferrGuard->lock();
	const int res = _vftprintf(ferr, fmt, ap);
	fflush(ferr);
	ferrGuard->unlock();
	return res;
}


/**
 * Prints a warning if the given digital pin is invalid.
 * 
 * @param[in] ref - reference string to output (e.g. function name where the check was performed)
 * @param[in] pin - digital pin to check
 */
static void checkDigitalPin(const TCHAR * ref, const uint8_t pin) {
	if (pin >= _NUM_DIGITAL_PINS) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_PIN), ref, static_cast<unsigned int>(pin));
	}
}


/**
 * Starts a section with a different timer resolution.
 * 
 * @param[in] resolution - new timer resolution in microseconds
 * @return 1 on success, else 0
 * @remarks Call endTimerRes() as soon as the timed operation finished.
 */
static int beginTimerRes(const unsigned int resolution) {
#if defined(PCF_IS_WIN)
	const int res = (timeBeginPeriod((UINT)(resolution / 1000)) == TIMERR_NOERROR) ? 1 : 0;
	delay(100); /* let the system perform a context switch to set the new timer resolution */
	return res;
#else
	PCF_UNUSED(resolution)
	return 0;
#endif /* PCF_IS_WIN */
}


/**
 * Ends a section with a different timer resolution.
 * 
 * @param[in] resolution - timer resolution of the current section in microseconds
 * @return 1 on success, else 0
 * @see beginTimerRes()
 */
static int endTimerRes(const unsigned int resolution) {
#if defined(PCF_IS_WIN)
	return (timeEndPeriod((UINT)(resolution / 1000)) == TIMERR_NOERROR) ? 1 : 0;
#else
	PCF_UNUSED(resolution)
	return 0;
#endif /* PCF_IS_WIN */
}


#if defined(PCF_IS_WIN)
/**
 * Helper function to wrap the LPTHREAD_START_ROUTINE around the used thread function signature.
 * 
 * @param[in,out] arg - arguments for the thread function which shall be called (will be freed)
 * @return 0
 */
static DWORD __stdcall newThreadWrapper(void * arg) {
	tThreadWrapper * wrap = static_cast<tThreadWrapper *>(arg);
	void (* fn)(void * data) = wrap->fn;
	void * data = wrap->data;
	free(arg);
	fn(data);
	return 0;
}
#elif defined(PCF_IS_LINUX)
/**
 * Helper function to wrap the pthread function signature around the used thread function signature.
 * 
 * @param[in,out] arg - arguments for the thread function which shall be called (will be freed)
 * @return NULL
 */
static void * newThreadWrapper(void * arg) {
	tThreadWrapper * wrap = static_cast<tThreadWrapper *>(arg);
	void (* fn)(void * data) = wrap->fn;
	void * data = wrap->data;
	free(arg);
	fn(data);
	return NULL;
}
#endif /* PCF_IS_LINUX */


/**
 * Creates a new thread with the given function.
 * 
 * @param[out] handle - returns the handle to the created thread
 * @param[in] fn - function to run
 * @param[in] data - pointer to user data
 * @return 1 on success, 0 on error
 */
static int createThread(tThread * handle, void (* fn)(void * data), void * data) {
	if (handle == NULL || fn == NULL) return 0;
	tThreadWrapper * wrap = static_cast<tThreadWrapper *>(malloc(sizeof(tThreadWrapper)));
	if (wrap == NULL) return 0;
	wrap->fn = fn;
	wrap->data = data;
#if defined(PCF_IS_WIN)
	*handle = CreateThread(NULL, 0, newThreadWrapper, wrap, 0, NULL);
	if (*handle == NULL) {
		free(wrap);
		return 0;
	}
#else /* !PCF_IS_WIN */
	if (pthread_create(handle, NULL, newThreadWrapper, wrap) != 0) {
		free(wrap);
		return 0;
	}
#endif /* PCF_IS_WIN */
	return 1;
}


/**
 * Joins the given thread and closes its handle on success.
 * 
 * @param[in,out] handle - thread to join and close
 * @return 1 on success, 0 on error
 */
static int joinThread(tThread * handle) {
	if (handle == NULL) return 0;
#if defined(PCF_IS_WIN)
	if (*handle == NULL) return 0;
	WaitForSingleObject(*handle, INFINITE);
	CloseHandle(*handle);
#else /* !PCF_IS_WIN */
	if (pthread_join(*handle, NULL) != 0) return 0;
#endif /* PCF_IS_WIN */
	memset(handle, 0, sizeof(*handle));
	return 1;
}


/**
 * Kills and closes the given thread without handling its current execution state.
 * 
 * @param[in,out] handle - thread to kill and close
 */
static void killThread(tThread * handle) {
	if (handle == NULL) return;
#if defined(PCF_IS_WIN)
	TerminateThread(*handle, 0);
	CloseHandle(*handle);
#else /* !PCF_IS_WIN */
	pthread_kill(*handle, SIGKILL);
#endif /* PCF_IS_WIN */
	memset(handle, 0, sizeof(*handle));
}


/**
 * Write a single byte to the remote device.
 * 
 * @param[in] val - byte to write
 * @param[in] eof - end-of-frame mark
 * @return true on success, else false
 */
static bool writeToRemoteHandler(const uint8_t val, const bool eof) {
	static size_t fillSize = 0;
	static uint8_t buf[MAX_FRAME_SIZE];
	static TCHAR hexBuf[MAX_FRAME_SIZE * 3];
	if (fillSize < MAX_FRAME_SIZE) {
		buf[fillSize++] = val;
	} else {
		return false;
	}
	if (verbose > 2) printToLog(_T("DEV\tout\thandle\t%02X\n"), val);
	if (fillSize >= MAX_FRAME_SIZE || eof) {
		const ssize_t res = ser_write(remoteConn, buf, fillSize, commTimeout);
		if (verbose > 1 && flog != NULL) {
			printToLog(_T("DEV\tout\tres\t%li\n"), static_cast<long>(res));
			if (res > 0) {
				for (ssize_t i = 0; i < res; i++) {
					const uint8_t b = buf[i];
					hexBuf[(3 * i)] = hexStr[(b >> 4) & 0x0F];
					hexBuf[(3 * i) + 1] = hexStr[b & 0x0F];
					hexBuf[(3 * i) + 2] = _T(' ');
				}
				hexBuf[(res * 3) - 1] = 0;
				printToLog(_T2("DEV\tout\tdump\t%" PRTCHAR "\n"), hexBuf);
			}
		}
		if (res > 0) {
			if (static_cast<size_t>(res) < fillSize) {
				memmove(buf, buf + res, static_cast<size_t>(fillSize - res));
				fillSize = static_cast<size_t>(fillSize - res);
			} else {
				fillSize = 0;
			}
		} else if (res < 0) {
			if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_WRITE));
			return false;
		}
	}
	return true;
}


/**
 * Handle frame which was completely received from remote device.
 * 
 * @param[in] seq - frame sequence number
 * @param[in,out] buf - frame data
 * @param[in] len - frame data length
 * @param[in] err - true on error, else false
 */
static void readFromRemoteHandler(const uint8_t seq, uint8_t * buf, const size_t len, const bool err) {
	if (verbose > 1 && flog != NULL) {
		printToLog(
			_T2("DEV\tin\tframe\t%" PRTCHAR "\t%u\t%u\t%" PRTCHAR "\t%" PRTCHAR "\n"),
			err ? _T("err") : _T("ok"),
			static_cast<unsigned int>(len),
			static_cast<unsigned int>(seq),
			(len > 0) ? resStr[PCF_MIN(buf[0], static_cast<uint8_t>(ResultCode::COUNT))] : resStr[ResultCode::COUNT],
			(len <= 1) ? _T("-")
				: (buf[0] == ResultCode::RESULT) ? opStr[PCF_MIN(buf[1], static_cast<uint8_t>(OpCode::COUNT))]
				: (buf[0] == ResultCode::INTERRUPT) ? _T("-")
				: (buf[0] == ResultCode::ERROR) ? errStr[PCF_MIN(buf[1], static_cast<uint8_t>(ErrorCode::COUNT))]
				: _T("-")
		);
	}
	if (len <= 0 || err) return;
	const uint8_t * ptr = buf;
	const uint8_t * endPtr = ptr + len;
	if (ptr[0] == ResultCode::INTERRUPT) {
		/* process interrupts from remote device */
		for (size_t i = 0; i < MAX_NUM_INTERRUPTS; i++) {
			const size_t bitOffset = i & 0x7;
			if (bitOffset == 0) {
				ptr++;
				if (ptr >= endPtr) break;
			}
			const bool triggered = bool(((*ptr) >> bitOffset) & 1);
			if ( triggered ) {
				printToLog(_T("DEV\tin\tint\t%u\n"), static_cast<unsigned int>(i));
				if (_INTERRUPTS && ints[i] != NULL) ints[i]();
			}
		}
	} else if (ptr[0] == ResultCode::RESULT) {
		/* process the remote call result */
		if (len < 2) {
			if (verbose > 0) printToErr(MSGT(MSGT_ERR_BROKEN_FRAME));
			printToLog(_T("DEV\tin\terr\n"));
			return;
		}
		uint8_t * remBuf = buf + 2;
		size_t remLen = static_cast<size_t>(len - 2);
		framesGuard->lock();
		::adde::_RC<::adde::_FVI> future = pendingCallResults[seq];
		if ( future ) {
			if (future->reference != static_cast<int>(ptr[1])) {
				if (verbose > 0) printToErr(MSGT(MSGT_ERR_OP_MISMATCH));
			}
			future->success = future->set(remBuf, remLen);
			future->valid = true;
			pendingCallResults[seq] = ::adde::_RC<::adde::_FVI>();
			if (pendingCallResultCount > 0) pendingCallResultCount--;
		}
		framesGuard->unlock();
	} else if (ptr[0] == ResultCode::ERROR) {
		/* process the remote call result for remote error cases */
		ptr++;
		if (verbose > 0) {
			if (ptr == endPtr) {
				printToErr(MSGT(MSGT_ERR_REMOTE));
			} else if ((ptr + 1) == endPtr) {
				printToErr(MSGT(MSGT_ERR_REMOTE_ERROR), errStr[PCF_MIN(ptr[0], static_cast<uint8_t>(ErrorCode::COUNT))]);
			} else if ((ptr + 2) == endPtr) {
				printToErr(MSGT(MSGT_ERR_REMOTE_FRAME), errStr[PCF_MIN(ptr[0], static_cast<uint8_t>(ErrorCode::COUNT))], static_cast<unsigned int>(ptr[1]));
			} else {
				switch (static_cast<OpCode::Type>(ptr[2])) {
				case OpCode::HARDWARE_SERIAL_GET_CONSTANTS:
				case OpCode::SPI_GET_CONSTANTS:
					/* special handling for HardwareSerial and SPI constants (@see ::adde::_FV<>::set() and callWith()) */
					return;
				default:
					printToErr(MSGT(MSGT_ERR_REMOTE_CALL),
						errStr[PCF_MIN(ptr[0], static_cast<uint8_t>(ErrorCode::COUNT))],
						static_cast<unsigned int>(ptr[1]),
						opStr[PCF_MIN(ptr[2], static_cast<uint8_t>(OpCode::COUNT))]
					);
					break;
				}
			}
		}
		if (flog != NULL) {
			if (ptr == endPtr) {
				printToLog(_T("DEV\tin\terr\n"));
			} else if ((ptr + 1) == endPtr) {
				printToLog(_T2("DEV\tin\terr\t%" PRTCHAR "\n"), errStr[PCF_MIN(ptr[0], static_cast<uint8_t>(ErrorCode::COUNT))]);
			} else if ((ptr + 2) == endPtr) {
				printToLog(_T2("DEV\tin\terr\t%" PRTCHAR "\t%u\n"), errStr[PCF_MIN(ptr[0], static_cast<uint8_t>(ErrorCode::COUNT))], static_cast<unsigned int>(ptr[1]));
			} else {
				printToLog(_T2("DEV\tin\terr\t%" PRTCHAR "\t%u\t%" PRTCHAR "\n"),
					errStr[PCF_MIN(ptr[0], static_cast<uint8_t>(ErrorCode::COUNT))],
					static_cast<unsigned int>(ptr[1]),
					opStr[PCF_MIN(ptr[2], static_cast<uint8_t>(OpCode::COUNT))]
				);
			}
		}
		framesGuard->lock();
		::adde::_RC<::adde::_FVI> future = pendingCallResults[seq];
		if ( future ) {
			if (len > 2 && future->reference != static_cast<int>(ptr[2])) {
				if (verbose > 0) printToErr(MSGT(MSGT_ERR_OP_MISMATCH));
			}
			future->success = false;
			future->valid = true;
			pendingCallResults[seq] = ::adde::_RC<::adde::_FVI>();
			if (pendingCallResultCount > 0) pendingCallResultCount--;
		}
		framesGuard->unlock();
		signalReceived++;
	} else {
		printToLog(_T("DEV\tin\tukn\t%u\n"), static_cast<unsigned int>(ptr[0]));
	}
}


/**
 * Thread which reads data received from the remote device and calls the appropriate internal
 * functions.
 * 
 * @param[in] data - unused
 */
static void readFromRemoteThread(void * /* data */) {
	static uint8_t buf[MAX_FRAME_SIZE]; /* static to ensure the buffer exists during termination sequence */
	static TCHAR hexBuf[MAX_FRAME_SIZE * 3]; /* static to ensure the buffer exists during termination sequence */
	while (signalReceived == 0) {
		const ssize_t res = ser_read(remoteConn, buf, sizeof(buf), TIMEOUT_RESOLUTION);
		if (verbose > 1 && flog != NULL) {
			printToLog(_T("DEV\tin\tres\t%li\n"), static_cast<long>(res));
			if (res > 0) {
				for (ssize_t i = 0; i < res; i++) {
					const uint8_t b = buf[i];
					hexBuf[(3 * i)] = hexStr[(b >> 4) & 0x0F];
					hexBuf[(3 * i) + 1] = hexStr[b & 0x0F];
					hexBuf[(3 * i) + 2] = _T(' ');
				}
				hexBuf[(res * 3) - 1] = 0;
				printToLog(_T2("DEV\tin\tdump\t%" PRTCHAR "\n"), hexBuf);
			}
		}
		if (res == -1) {
			if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_READ));
			signalReceived++;
			break;
		}
		for (ssize_t i = 0; i < res; i++) {
			if ( ! framing.read(buf[i], readFromRemoteHandler) ) {
				printToLog(_T2("DEV\tin\terr\t%" PRTCHAR "\n"), errStr[ErrorCode::BROKEN_FRAME]);
				if (verbose > 0) printToErr(MSGT(MSGT_ERR_BROKEN_FRAME));
			}
		}
	}
}


/**
 * Thread which reads data from fin to handle asynchronous data reception for ConsoleSerial.
 * 
 * @param[in] data - unused
 */
static void readConsoleThread(void * /* data */) {
	uint8_t buffer[1];
#if defined(PCF_IS_WIN)
	const bool isConsole = (GetFileType(reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(fin)))) == FILE_TYPE_CHAR);
	if ( isConsole ) {
		while (signalReceived == 0) {
			if ( spinWaitDelay(conPeekBuffer, -1, 1, TIMEOUT_RESOLUTION) ) {
				while (signalReceived == 0 && _kbhit() != 0) {
					if (fin == NULL || feof(fin) != 0) break;
					const int c = _getch();
					if (fin == NULL || feof(fin) != 0) break;
					if (c == 0x00 || c == 0xE0) _getch(); /* ignore control sequences */
					conPeekBuffer = c;
					break;
				}
			}
		}
	} else
#endif /* PCF_IS_WIN */
	while (signalReceived == 0) {
		if ( spinWaitDelay(conPeekBuffer, -1, 1, TIMEOUT_RESOLUTION) ) {
			if (fin == NULL || feof(fin) != 0) break;
			if (fread(buffer, 1, 1, fin) > 0) {
				conPeekBuffer = static_cast<int>(*buffer);
			}
		}
	}
}


/**
 * Returns the number of busy-wait iterations until one millisecond passed.
 * 
 * @return iterations per 1 millisecond
 */
static size_t getSpinMilli(void) {
	static size_t cachedValue = 0;
	if (cachedValue != 0) return cachedValue;
	int64_t current, last = getAbsTime();
	volatile size_t res = 100;
	volatile int testVar = 0;
	do {
		res *= 10;
		/* wait for next time slice */
		while ((current = getAbsTime()) == last);
		last = current;
		const size_t count = res;
		/* busy wait */
		for (size_t i = 0; i < count; i++) {
			testVar = ~testVar;
			if (testVar == 10) break; /* never matched, just for time prediction */
		}
		current = getAbsTime();
	} while (static_cast<uint64_t>(current - last) < UINT64_C(1000000));
	/* calculate how many iterations 1ms would had taken */
	const uint64_t count = static_cast<uint64_t>(res);
	const uint64_t diff = static_cast<uint64_t>(current - last);
	cachedValue = static_cast<size_t>((count * UINT64_C(1000)) / diff);
	return cachedValue;
}


/**
 * Returns the number of microseconds per CPU time slice.
 * 
 * @return microseconds per CPU time slice
 */
static uint64_t getCpuInterval(void) {
	static uint64_t cachedValue = 0;
	if (cachedValue != 0) return cachedValue;
	int64_t first, current, last = getAbsTime();
	/* wait for next time slice */
	while ((current = getAbsTime()) == last);
	first = current;
	last = current;
	/* wait for next time slice */
	while ((current = getAbsTime()) == last);
	/* time difference between start and end of a time slice */
	cachedValue = static_cast<uint64_t>(current - first);
	return cachedValue;
}


/**
 * Busy waits for the variable to match the given value or timeouts.
 * 
 * @param[in] var - variable to monitor
 * @param[in] match - target value to reach
 * @param[in] timeout - timeout in milliseconds
 * @return true on match, false on timeout
 * @remarks getSpinMilli() should be called once before using this function
 */
template <typename T>
static bool spinWait(volatile T & var, const T match, const size_t timeout) {
	if (var == match) return true;
	const size_t spinMilli = getSpinMilli();
	for (size_t t = 0; t < timeout; t++) {
		for (size_t i = 0; i < spinMilli; i++) {
			if (var == match) return true;
		}
	}
	return false;
}


/**
 * Busy waits for the variable to not match the given value or timeouts.
 * 
 * @param[in] var - variable to monitor
 * @param[in] match - target value to reach
 * @param[in] timeout - timeout in milliseconds
 * @return true on match, false on timeout
 * @remarks getSpinMilli() should be called once before using this function
 */
template <typename T>
static bool spinWaitNot(volatile T & var, const T match, const size_t timeout) {
	if (var != match) return true;
	const size_t spinMilli = getSpinMilli();
	for (size_t t = 0; t < timeout; t++) {
		for (size_t i = 0; i < spinMilli; i++) {
			if (var != match) return true;
		}
	}
	return false;
}


/**
 * Busy waits for the variable to match the given value or delay one time slice to allow context
 * switching and before busy waiting again. The longTimeout sets absolute timeout, whereas the
 * shortTimeout sets how long to busy wait.
 * 
 * @param[in] var - variable to monitor
 * @param[in] match - target value to reach
 * @param[in] shortTimeout - timeout in milliseconds to busy wait
 * @param[in] longTimeout - timeout in milliseconds to wait
 * @return true on match, false on timeout
 * @remarks getSpinMilli() and getCpuInterval() should be called once before using this function
 */
template <typename T>
static bool spinWaitDelay(volatile T & var, const T match, const size_t shortTimeout, const size_t longTimeout) {
	for (size_t i = 0; i < longTimeout; i += shortTimeout) {
		if ( spinWait(var, match, shortTimeout) ) return true;
		const unsigned long start = micros();
		delay(1);
		i += static_cast<size_t>((micros() - start) / 1000UL);
	}
	return false;
}


/**
 * Busy waits for the variable to not match the given value or delay one time slice to allow context
 * switching and before busy waiting again. The longTimeout sets absolute timeout, whereas the
 * shortTimeout sets how long to busy wait.
 * 
 * @param[in] var - variable to monitor
 * @param[in] match - target value to reach
 * @param[in] shortTimeout - timeout in milliseconds to busy wait
 * @param[in] longTimeout - timeout in milliseconds to wait
 * @return true on match, false on timeout
 * @remarks getSpinMilli() and getCpuInterval() should be called once before using this function
 */
template <typename T>
static bool spinWaitNotDelay(volatile T & var, const T match, const size_t shortTimeout, const size_t longTimeout) {
	for (size_t i = 0; i < longTimeout; i += shortTimeout) {
		if ( spinWaitNot(var, match, shortTimeout) ) return true;
		const unsigned long start = micros();
		delay(1);
		i += static_cast<size_t>((micros() - start) / 1000UL);
	}
	return false;
}


/**
 * Returns the current absolute time in microseconds since 2000-01-01 00:00:00.000 UTC.
 * 
 * @return 0 on error else the UTC time in microseconds
 */
static int64_t getAbsTime(void) {
#if defined(PCF_IS_WIN)
	SYSTEMTIME systemTime;
	FILETIME fileTime;
	GetSystemTime(&systemTime);
	if (SystemTimeToFileTime(&systemTime, &fileTime) == 0) return 0;
	return (int64_t)((((uint64_t)(fileTime.dwHighDateTime) << 32) | ((uint64_t)(fileTime.dwLowDateTime))) / UINT64_C(10)) - INT64_C(12591158400000000);
#elif defined(PCF_IS_LINUX)
	struct timeval tv;
	if (gettimeofday(&tv, NULL) != 0) return 0;
	return (((int64_t)(tv.tv_sec)) * INT64_C(1000000)) + ((int64_t)(tv.tv_usec)) - INT64_C(946684800000000);
#endif
}


/**
 * Converts the given point in time in seconds since 2000-01-01 00:00:00 into a date time structure.
 * 
 * @param[out] dt - output date time structure
 * @param[in] num - seconds since 2000-01-01 00:00:00
 * @remarks Changing num from uint32_t to something else or the reference date time is not trivial.
 * Several optimizations were introduced here that are not generic.
 */
static void ntodt(tDateTime * dt, uint32_t num) {
	static const uint8_t normalMonths[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	static const uint8_t leapMonths[]   = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	dt->sec = static_cast<uint8_t>(num % 60);
	num /= 60;
	dt->min = static_cast<uint8_t>(num % 60);
	num /= 60;
	dt->hour = static_cast<uint8_t>(num % 24);
	num /= 24;
	dt->year = static_cast<uint16_t>((num + ((num > UINT32_C(36525)) ? 1 : 0)) * UINT32_C(400) / UINT32_C(146100)); /* 365.25 days per year with correct rounding */
	const uint8_t leap4 = static_cast<uint8_t>(dt->year >> 2); /* number of leap years passed */
	const uint8_t leap100 = (dt->year >= 100) ? 1 : 0; /* number of non-leaping leap years passed */
	const uint8_t * mDays = (dt->year != 100 && (dt->year & 3) == 0) ? leapMonths : normalMonths;
	num = static_cast<uint32_t>(num - static_cast<uint32_t>(365 * dt->year) - leap4 + leap100); /* days of this year starting at 1 */
	if (mDays == leapMonths) num++; /* leap years have one additional day */
	uint8_t month = 0;
	for (; num > mDays[month] && month < 12; num = static_cast<uint32_t>(num - mDays[month]), month++);
	dt->day = static_cast<uint8_t>(num);
	dt->month = static_cast<uint8_t>(month + 1);
	dt->year = static_cast<uint16_t>(dt->year + 2000);
}


namespace adde {


size_t _bool::size() { return static_cast<size_t>(_sizeof_bool); }
size_t _signed_char::size() { return static_cast<size_t>(_sizeof_signed_char); }
size_t _unsigned_char::size() { return static_cast<size_t>(_sizeof_unsigned_char); }
size_t _signed_short::size() { return static_cast<size_t>(_sizeof_signed_short); }
size_t _unsigned_short::size() { return static_cast<size_t>(_sizeof_unsigned_short); }
size_t _signed_int::size() { return static_cast<size_t>(_sizeof_signed_int); }
size_t _unsigned_int::size() { return static_cast<size_t>(_sizeof_unsigned_int); }
size_t _signed_long::size() { return static_cast<size_t>(_sizeof_signed_long); }
size_t _unsigned_long::size() { return static_cast<size_t>(_sizeof_unsigned_long); }
size_t _signed_long_long::size() { return static_cast<size_t>(_sizeof_signed_long_long); }
size_t _unsigned_long_long::size() { return static_cast<size_t>(_sizeof_unsigned_long_long); }
size_t _float::size() { return static_cast<size_t>(_sizeof_float); }
size_t _double::size() { return static_cast<size_t>(_sizeof_double); }
size_t _long_double::size() { return static_cast<size_t>(_sizeof_long_double); }
size_t _void_ptr::size() { return static_cast<size_t>(_sizeof_void_ptr); }
size_t _ptrdiff_t::size() { return static_cast<size_t>(_sizeof_ptrdiff_t); }
size_t _wchar_t::size() { return static_cast<size_t>(_sizeof_wchar_t); }
size_t _size_t::size() { return static_cast<size_t>(_sizeof_size_t); }


template <> int _V<bool>::printTo(FILE * fd) const { return _ftprintf(fd, this->value ? _T("true") : _T("false")); }
template <> int _V<signed char>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%i"), static_cast<int>(this->value)); }
template <> int _V<unsigned char>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%u"), static_cast<unsigned int>(this->value)); }
template <> int _V<signed short>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%i"), static_cast<int>(this->value)); }
template <> int _V<unsigned short>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%u"), static_cast<unsigned int>(this->value)); }
template <> int _V<signed int>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%i"), this->value); }
template <> int _V<unsigned int>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%u"), this->value); }
template <> int _V<signed long>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%li"), this->value); }
template <> int _V<unsigned long>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%lu"), this->value); }
template <> int _V<signed long long>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%lli"), this->value); }
template <> int _V<unsigned long long>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%llu"), this->value); }
template <> int _V<float>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%f"), static_cast<double>(this->value)); }
template <> int _V<double>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%f"), this->value); }
template <> int _V<long double>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%Lf"), this->value); }
template <> int _V<void *>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%p"), this->value); }
template <> int _V<wchar_t>::printTo(FILE * fd) const { return _ftprintf(fd, _T("%u"), static_cast<unsigned int>(this->value)); }


/**
 * Creates a new lock for the reference counter.
 */
void * _createRcLock() {
	return static_cast<void *>(new ScopedGuard(getSpinMilli()));
}


/**
 * Destroy the given reference counter lock.
 */
void _destroyRcLock(void * lock) {
	if (lock == NULL) return;
	delete static_cast<ScopedGuard *>(lock);
}


/**
 * Locks the reference counter lock.
 */
void _lockRcLock(void * lock) {
	if (lock == NULL) return;
	static_cast<ScopedGuard *>(lock)->lock();
}


/**
 * Unlocks the reference counter lock.
 */
void _unlockRcLock(void * lock) {
	if (lock == NULL) return;
	static_cast<ScopedGuard *>(lock)->unlock();
}


/**
 * Waits for the future result value. Program execution is aborted on timeout.
 */
void _FVI::waitForResult() {
	/* early out if the result is already available */
	if ( this->valid ) return;
	/* wait for result */
	const unsigned long timePassed = static_cast<unsigned long>(millis() - this->start);
	const uint64_t cpuIntvl = getCpuInterval();
	const size_t shortTimeout = static_cast<size_t>(PCF_MAX(UINT64_C(1), cpuIntvl / UINT64_C(1000)));
	if (timePassed >= static_cast<unsigned long>(this->timeout) || ( ! spinWaitDelay(this->valid, true, shortTimeout, static_cast<unsigned long>(this->timeout - timePassed)) )) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_TIMEOUT));
		signalReceived += 100;
		exit(EXIT_FAILURE); /* timeout */
	}
}


/**
 * Destructor.
 * 
 * @remarks This may abort program execution if useLazyCallResult is false and the remote call
 * result handler times out.
 */
_FVI::~_FVI() {
	if (signalReceived != 0) return;
	if ( ! useLazyCallResult ) this->waitForResult();
}


bool _FV<_void>::set(uint8_t * & /* buf */, size_t & /* len */) {
	return true;
}
template <> bool _FV<_bool>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_signed_char>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_unsigned_char>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_signed_short>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_unsigned_short>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_signed_int>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_unsigned_int>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_signed_long>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_unsigned_long>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_signed_long_long>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_unsigned_long_long>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_float>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_double>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_long_double>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_void_ptr>::set(uint8_t * & /* buf */, size_t & /* len */) {
	/* pointer return types are not supported */
	if (verbose > 0) printToErr(MSGT(MSGT_ERR_PTR_RESULT));
	return false;
}
template <> bool _FV<_ptrdiff_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_wchar_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
	return false;
}
template <> bool _FV<_size_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_int8_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_uint8_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_int16_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_uint16_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_int32_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_uint32_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_int64_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}
template <> bool _FV<_uint64_t>::set(uint8_t * & buf, size_t & len) {
	return extractCallResult(static_cast<OpCode::Type>(this->reference), buf, len, this->value);
}


/**
 * Special handling for call results with OP code GET_DEV_NAME.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_GET_DEV_NAME>::set(uint8_t * & buf, size_t & len) {
	if (len < 1) return true;
	/* remove invalid characters */
	size_t d = 0;
	for (size_t s = 0; s < len; s++) {
		if (buf[s] == 9) {
			buf[d++] = ' ';
		} else if (buf[s] > 32) {
			buf[d++] = buf[s];
		}
	}
	/* convert UTF-8 to native string format */
	TCHAR * str = _tfromUtf8N(reinterpret_cast<const char *>(buf), d);
	if (str == NULL) {
		_ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
		return false;
	}
	/* output string to log */
	printToLog(_T2("DEV\tname\t%" PRTCHAR "\n"), str);
	if (static_cast<void *>(str) != static_cast<void *>(buf)) free(str);
	return true;
}


/**
 * Special handling for call results with OP code GET_CONSTANTS.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_GET_CONSTANTS>::set(uint8_t * & buf, size_t & len) {
	if (len < 46) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	readFrameValue(buf, len, _sizeof_bool);
	readFrameValue(buf, len, _sizeof_signed_char);
	readFrameValue(buf, len, _sizeof_unsigned_char);
	readFrameValue(buf, len, _sizeof_signed_short);
	readFrameValue(buf, len, _sizeof_unsigned_short);
	readFrameValue(buf, len, _sizeof_signed_int);
	readFrameValue(buf, len, _sizeof_unsigned_int);
	readFrameValue(buf, len, _sizeof_signed_long);
	readFrameValue(buf, len, _sizeof_unsigned_long);
	readFrameValue(buf, len, _sizeof_signed_long_long);
	readFrameValue(buf, len, _sizeof_unsigned_long_long);
	readFrameValue(buf, len, _sizeof_float);
	readFrameValue(buf, len, _sizeof_double);
	readFrameValue(buf, len, _sizeof_long_double);
	readFrameValue(buf, len, _sizeof_void_ptr);
	readFrameValue(buf, len, _sizeof_ptrdiff_t);
	readFrameValue(buf, len, _sizeof_wchar_t);
	readFrameValue(buf, len, _sizeof_size_t);
	readFrameValue(buf, len, _HIGH);
	readFrameValue(buf, len, _LOW);
	readFrameValue(buf, len, _INPUT);
	readFrameValue(buf, len, _INPUT_PULLUP);
	readFrameValue(buf, len, _OUTPUT);
	readFrameValue(buf, len, _SERIAL);
	readFrameValue(buf, len, _DISPLAY);
	readFrameValue(buf, len, _LSBFIRST);
	readFrameValue(buf, len, _MSBFIRST);
	readFrameValue(buf, len, _CHANGE);
	readFrameValue(buf, len, _FALLING);
	readFrameValue(buf, len, _RISING);
	readFrameValue(buf, len, _DEFAULT);
	readFrameValue(buf, len, _EXTERNAL);
	readFrameValue(buf, len, _INTERNAL1V1);
	readFrameValue(buf, len, _INTERNAL);
	readFrameValue(buf, len, _INTERNAL2V56);
	readFrameValue(buf, len, _INTERNAL2V56_EXTCAP);
	readFrameValue(buf, len, hwSerCount);
	readFrameValue(buf, len, swSerCount);
	readFrameValue(buf, len, eepromCount);
	readFrameValue(buf, len, lcdCount);
	readFrameValue(buf, len, spiCount);
	readFrameValue(buf, len, wireCount);
	readFrameValue(buf, len, _F_CPU);
	swSerInst = RemoteInstance(static_cast<size_t>(swSerCount));
	lcdInst = RemoteInstance(static_cast<size_t>(lcdCount));
	if (verbose > 1) {
		printToLog(_T("DEV\tvar\tsizeof(bool)\t%u\n"), static_cast<unsigned int>(_sizeof_bool));
		printToLog(_T("DEV\tvar\tsizeof(signed char)\t%u\n"), static_cast<unsigned int>(_sizeof_signed_char));
		printToLog(_T("DEV\tvar\tsizeof(unsigned char)\t%u\n"), static_cast<unsigned int>(_sizeof_unsigned_char));
		printToLog(_T("DEV\tvar\tsizeof(signed short)\t%u\n"), static_cast<unsigned int>(_sizeof_signed_short));
		printToLog(_T("DEV\tvar\tsizeof(unsigned short)\t%u\n"), static_cast<unsigned int>(_sizeof_unsigned_short));
		printToLog(_T("DEV\tvar\tsizeof(signed int)\t%u\n"), static_cast<unsigned int>(_sizeof_signed_int));
		printToLog(_T("DEV\tvar\tsizeof(unsigned int)\t%u\n"), static_cast<unsigned int>(_sizeof_unsigned_int));
		printToLog(_T("DEV\tvar\tsizeof(signed long)\t%u\n"), static_cast<unsigned int>(_sizeof_signed_long));
		printToLog(_T("DEV\tvar\tsizeof(unsigned long)\t%u\n"), static_cast<unsigned int>(_sizeof_unsigned_long));
		printToLog(_T("DEV\tvar\tsizeof(signed long long)\t%u\n"), static_cast<unsigned int>(_sizeof_signed_long_long));
		printToLog(_T("DEV\tvar\tsizeof(unsigned long long)\t%u\n"), static_cast<unsigned int>(_sizeof_unsigned_long_long));
		printToLog(_T("DEV\tvar\tsizeof(float)\t%u\n"), static_cast<unsigned int>(_sizeof_float));
		printToLog(_T("DEV\tvar\tsizeof(double)\t%u\n"), static_cast<unsigned int>(_sizeof_double));
		printToLog(_T("DEV\tvar\tsizeof(long double)\t%u\n"), static_cast<unsigned int>(_sizeof_long_double));
		printToLog(_T("DEV\tvar\tsizeof(void *)\t%u\n"), static_cast<unsigned int>(_sizeof_void_ptr));
		printToLog(_T("DEV\tvar\tsizeof(ptrdiff_t)\t%u\n"), static_cast<unsigned int>(_sizeof_ptrdiff_t));
		printToLog(_T("DEV\tvar\tsizeof(wchar_t)\t%u\n"), static_cast<unsigned int>(_sizeof_wchar_t));
		printToLog(_T("DEV\tvar\tsizeof(size_t)\t%u\n"), static_cast<unsigned int>(_sizeof_size_t));
		printToLog(_T("DEV\tvar\tHIGH\t%u\n"), static_cast<unsigned int>(_HIGH));
		printToLog(_T("DEV\tvar\tLOW\t%u\n"), static_cast<unsigned int>(_LOW));
		printToLog(_T("DEV\tvar\tINPUT\t%u\n"), static_cast<unsigned int>(_INPUT));
		printToLog(_T("DEV\tvar\tINPUT_PULLUP\t%u\n"), static_cast<unsigned int>(_INPUT_PULLUP));
		printToLog(_T("DEV\tvar\tOUTPUT\t%u\n"), static_cast<unsigned int>(_OUTPUT));
		printToLog(_T("DEV\tvar\tSERIAL\t%u\n"), static_cast<unsigned int>(_SERIAL));
		printToLog(_T("DEV\tvar\tDISPLAY\t%u\n"), static_cast<unsigned int>(_DISPLAY));
		printToLog(_T("DEV\tvar\tLSBFIRST\t%u\n"), static_cast<unsigned int>(_LSBFIRST));
		printToLog(_T("DEV\tvar\tMSBFIRST\t%u\n"), static_cast<unsigned int>(_MSBFIRST));
		printToLog(_T("DEV\tvar\tCHANGE\t%u\n"), static_cast<unsigned int>(_CHANGE));
		printToLog(_T("DEV\tvar\tFALLING\t%u\n"), static_cast<unsigned int>(_FALLING));
		printToLog(_T("DEV\tvar\tRISING\t%u\n"), static_cast<unsigned int>(_RISING));
		printToLog(_T("DEV\tvar\tDEFAULT\t%u\n"), static_cast<unsigned int>(_DEFAULT));
		printToLog(_T("DEV\tvar\tEXTERNAL\t%u\n"), static_cast<unsigned int>(_EXTERNAL));
		printToLog(_T("DEV\tvar\tINTERNAL1V1\t%u\n"), static_cast<unsigned int>(_INTERNAL1V1));
		printToLog(_T("DEV\tvar\tINTERNAL\t%u\n"), static_cast<unsigned int>(_INTERNAL));
		printToLog(_T("DEV\tvar\tINTERNAL2V56\t%u\n"), static_cast<unsigned int>(_INTERNAL2V56));
		printToLog(_T("DEV\tvar\tINTERNAL2V56_EXTCAP\t%u\n"), static_cast<unsigned int>(_INTERNAL2V56_EXTCAP));
		printToLog(_T("DEV\tvar\t#HARDWARE_SERIAL\t%u\n"), static_cast<unsigned int>(hwSerCount));
		printToLog(_T("DEV\tvar\t#SOFTWARE_SERIAL\t%u\n"), static_cast<unsigned int>(swSerCount));
		printToLog(_T("DEV\tvar\t#EEPROM\t%u\n"), static_cast<unsigned int>(eepromCount));
		printToLog(_T("DEV\tvar\t#LCD\t%u\n"), static_cast<unsigned int>(lcdCount));
		printToLog(_T("DEV\tvar\t#SPI\t%u\n"), static_cast<unsigned int>(spiCount));
		printToLog(_T("DEV\tvar\t#WIRE\t%u\n"), static_cast<unsigned int>(wireCount));
		printToLog(_T("DEV\tvar\tF_CPU\t%u\n"), static_cast<unsigned int>(_F_CPU));
	}
	return true;
}


/**
 * Special handling for call results with OP code GET_PIN_MAP_D.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_GET_PIN_MAP_D>::set(uint8_t * & buf, size_t & len) {
	if (len < 1) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	readFrameValue(buf, len, _NUM_DIGITAL_PINS);
	if (verbose > 1) printToLog(_T("DEV\tvar\tNUM_DIGITAL_PINS\t%u\n"), static_cast<unsigned int>(_NUM_DIGITAL_PINS));
	if (len < _NUM_DIGITAL_PINS) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	if (_DIGITAL_PIN_TO_PORT != NULL) {
		free(_DIGITAL_PIN_TO_PORT);
		_DIGITAL_PIN_TO_PORT = NULL;
		delayedDtor._DIGITAL_PIN_TO_PORT = NULL;
	}
	if (_DIGITAL_PIN_TO_INTERRUPT != NULL) {
		free(_DIGITAL_PIN_TO_INTERRUPT);
		_DIGITAL_PIN_TO_INTERRUPT = NULL;
		delayedDtor._DIGITAL_PIN_TO_INTERRUPT = NULL;
	}
	if (_DIGITAL_PIN_PWM_SUPPORT != NULL) {
		free(_DIGITAL_PIN_PWM_SUPPORT);
		_DIGITAL_PIN_PWM_SUPPORT = NULL;
		delayedDtor._DIGITAL_PIN_PWM_SUPPORT = NULL;
	}
	if (_NUM_DIGITAL_PINS > 0) {
		if (_DIGITAL_PIN_TO_PORT == NULL) {
			_DIGITAL_PIN_TO_PORT = static_cast<uint8_t *>(malloc(_NUM_DIGITAL_PINS));
			delayedDtor._DIGITAL_PIN_TO_PORT = _DIGITAL_PIN_TO_PORT;
		}
		if (_DIGITAL_PIN_TO_INTERRUPT == NULL) {
			_DIGITAL_PIN_TO_INTERRUPT = static_cast<uint8_t *>(malloc(_NUM_DIGITAL_PINS));
			delayedDtor._DIGITAL_PIN_TO_INTERRUPT = _DIGITAL_PIN_TO_INTERRUPT;
		}
		if (_DIGITAL_PIN_PWM_SUPPORT == NULL) {
			_DIGITAL_PIN_PWM_SUPPORT = static_cast<bool *>(malloc(_NUM_DIGITAL_PINS));
			delayedDtor._DIGITAL_PIN_PWM_SUPPORT = _DIGITAL_PIN_PWM_SUPPORT;
		}
		if (_DIGITAL_PIN_TO_PORT != NULL) {
			for (uint8_t i = 0; i < _NUM_DIGITAL_PINS; i++) {
				readFrameValue(buf, len, _DIGITAL_PIN_TO_PORT[i]);
				if (verbose > 1) printToLog(_T("DEV\tvar\tDIGITAL_PIN_TO_PORT[%u]\t%u\n"), static_cast<unsigned int>(i), static_cast<unsigned int>(_DIGITAL_PIN_TO_PORT[i]));
			}
		}
	}
	return true;
}


/**
 * Special handling for call results with OP code GET_PIN_MAP_A.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_GET_PIN_MAP_A>::set(uint8_t * & buf, size_t & len) {
	if (len < 1) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	readFrameValue(buf, len, _NUM_ANALOG_INPUTS);
	if (verbose > 1) printToLog(_T("DEV\tvar\tNUM_ANALOG_INPUTS\t%u\n"), static_cast<unsigned int>(_NUM_ANALOG_INPUTS));
	if (len < _NUM_ANALOG_INPUTS) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	static uint8_t * analogPins[] = {
		 &_A0,  &_A1,  &_A2,  &_A3,  &_A4,  &_A5,  &_A6,  &_A7,
		 &_A8,  &_A9, &_A10, &_A11, &_A12, &_A13, &_A14, &_A15,
		&_A16, &_A17, &_A18, &_A19, &_A20, &_A21, &_A22, &_A23,
		&_A24, &_A25, &_A26, &_A27, &_A28, &_A29, &_A30, &_A31,
		&_A32, &_A33, &_A34, &_A35, &_A36, &_A37, &_A38, &_A39,
		&_A40, &_A41, &_A42, &_A43, &_A44, &_A45, &_A46, &_A47,
		&_A48, &_A49, &_A50, &_A51, &_A52, &_A53, &_A54, &_A55,
		&_A56, &_A57, &_A58, &_A59, &_A60, &_A61, &_A62, &_A63
	};
	for (uint8_t i = 0; i < _NUM_ANALOG_INPUTS; i++) {
		readFrameValue(buf, len, *(analogPins[i]));
		if (verbose > 1) printToLog(_T("DEV\tvar\tA%u\t%u\n"), static_cast<unsigned int>(i), static_cast<unsigned int>(*(analogPins[i])));
	}
	return true;
}


/**
 * Special handling for call results with OP code GET_PIN_MAP_S.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_GET_PIN_MAP_S>::set(uint8_t * & buf, size_t & len) {
	if (len < 14) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	readFrameValue(buf, len, _LED_BUILTIN);
	readFrameValue(buf, len, _LED_BUILTIN_RX);
	readFrameValue(buf, len, _LED_BUILTIN_TX);
	readFrameValue(buf, len, _PIN_SPI_SS0);
	readFrameValue(buf, len, _PIN_SPI_SS1);
	readFrameValue(buf, len, _PIN_SPI_SS2);
	readFrameValue(buf, len, _PIN_SPI_SS3);
	readFrameValue(buf, len, _PIN_SPI_MOSI);
	readFrameValue(buf, len, _PIN_SPI_MISO);
	readFrameValue(buf, len, _PIN_SPI_SCK);
	readFrameValue(buf, len, _PIN_WIRE_SDA);
	readFrameValue(buf, len, _PIN_WIRE_SCL);
	readFrameValue(buf, len, _PIN_WIRE1_SDA);
	readFrameValue(buf, len, _PIN_WIRE1_SCL);
	if (verbose > 1) {
		printToLog(_T("DEV\tvar\tLED_BUILTIN\t%u\n"), static_cast<unsigned int>(_LED_BUILTIN));
		printToLog(_T("DEV\tvar\tLED_BUILTIN_RX\t%u\n"), static_cast<unsigned int>(_LED_BUILTIN_RX));
		printToLog(_T("DEV\tvar\tLED_BUILTIN_TX\t%u\n"), static_cast<unsigned int>(_LED_BUILTIN_TX));
		printToLog(_T("DEV\tvar\tPIN_SPI_SS0\t%u\n"), static_cast<unsigned int>(_PIN_SPI_SS0));
		printToLog(_T("DEV\tvar\tPIN_SPI_SS1\t%u\n"), static_cast<unsigned int>(_PIN_SPI_SS1));
		printToLog(_T("DEV\tvar\tPIN_SPI_SS2\t%u\n"), static_cast<unsigned int>(_PIN_SPI_SS2));
		printToLog(_T("DEV\tvar\tPIN_SPI_SS3\t%u\n"), static_cast<unsigned int>(_PIN_SPI_SS3));
		printToLog(_T("DEV\tvar\tPIN_SPI_MOSI\t%u\n"), static_cast<unsigned int>(_PIN_SPI_MOSI));
		printToLog(_T("DEV\tvar\tPIN_SPI_MISO\t%u\n"), static_cast<unsigned int>(_PIN_SPI_MISO));
		printToLog(_T("DEV\tvar\tPIN_SPI_SCK\t%u\n"), static_cast<unsigned int>(_PIN_SPI_SCK));
		printToLog(_T("DEV\tvar\tPIN_WIRE_SDA\t%u\n"), static_cast<unsigned int>(_PIN_WIRE_SDA));
		printToLog(_T("DEV\tvar\tPIN_WIRE_SCL\t%u\n"), static_cast<unsigned int>(_PIN_WIRE_SCL));
		printToLog(_T("DEV\tvar\tPIN_WIRE1_SDA\t%u\n"), static_cast<unsigned int>(_PIN_WIRE1_SDA));
		printToLog(_T("DEV\tvar\tPIN_WIRE1_SCL\t%u\n"), static_cast<unsigned int>(_PIN_WIRE1_SCL));
	}
	return true;
}


/**
 * Special handling for call results with OP code GET_PIN_MAP_P.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_GET_PIN_MAP_P>::set(uint8_t * & buf, size_t & len) {
	if (len < 1) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	buf++;
	len = static_cast<size_t>(len - 1);
	if (_DIGITAL_PIN_PWM_SUPPORT != NULL) {
		const uint8_t num = static_cast<uint8_t>(len);
		for (uint8_t i = 0; i < _NUM_DIGITAL_PINS; i++) _DIGITAL_PIN_PWM_SUPPORT[i] = false;
		for (uint8_t i = 0; i < num; i++) {
			uint8_t val;
			readFrameValue(buf, len, val);
			if (val >= _NUM_DIGITAL_PINS) {
				if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
				return false;
			}
			_DIGITAL_PIN_PWM_SUPPORT[val] = true;
			if (verbose > 1) printToLog(_T("DEV\tvar\tDIGITAL_PIN_PWM_SUPPORT[%u]\t1\n"), static_cast<unsigned int>(val));
		}
	}
	return true;
}


/**
 * Special handling for call results with OP code GET_PIN_MAP_I.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_GET_PIN_MAP_I>::set(uint8_t * & buf, size_t & len) {
	if (len < 1) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	buf++;
	len = static_cast<size_t>(len - 1);
	if (len < _NUM_DIGITAL_PINS) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	if (_DIGITAL_PIN_TO_INTERRUPT != NULL) {
		for (uint8_t i = 0; i < _NUM_DIGITAL_PINS; i++) {
			readFrameValue(buf, len, _DIGITAL_PIN_TO_INTERRUPT[i]);
			if (verbose > 1) printToLog(_T("DEV\tvar\tDIGITAL_PIN_TO_INTERRUPT[%u]\t%u\n"), static_cast<unsigned int>(i), static_cast<unsigned int>(_DIGITAL_PIN_TO_INTERRUPT[i]));
		}
	}
	return true;
}


/**
 * Special handling for call results with OP code HARDWARE_SERIAL_GET_CONSTANTS.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_HARDWARE_SERIAL_GET_CONSTANTS>::set(uint8_t * & buf, size_t & len) {
	if (len < 24) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	readFrameValue(buf, len, _SERIAL_5N1);
	readFrameValue(buf, len, _SERIAL_6N1);
	readFrameValue(buf, len, _SERIAL_7N1);
	readFrameValue(buf, len, _SERIAL_8N1);
	readFrameValue(buf, len, _SERIAL_5N2);
	readFrameValue(buf, len, _SERIAL_6N2);
	readFrameValue(buf, len, _SERIAL_7N2);
	readFrameValue(buf, len, _SERIAL_8N2);
	readFrameValue(buf, len, _SERIAL_5E1);
	readFrameValue(buf, len, _SERIAL_6E1);
	readFrameValue(buf, len, _SERIAL_7E1);
	readFrameValue(buf, len, _SERIAL_8E1);
	readFrameValue(buf, len, _SERIAL_5E2);
	readFrameValue(buf, len, _SERIAL_6E2);
	readFrameValue(buf, len, _SERIAL_7E2);
	readFrameValue(buf, len, _SERIAL_8E2);
	readFrameValue(buf, len, _SERIAL_5O1);
	readFrameValue(buf, len, _SERIAL_6O1);
	readFrameValue(buf, len, _SERIAL_7O1);
	readFrameValue(buf, len, _SERIAL_8O1);
	readFrameValue(buf, len, _SERIAL_5O2);
	readFrameValue(buf, len, _SERIAL_6O2);
	readFrameValue(buf, len, _SERIAL_7O2);
	readFrameValue(buf, len, _SERIAL_8O2);
	if (verbose > 1) {
		printToLog(_T("DEV\tvar\tSERIAL_5N1\t%u\n"), static_cast<unsigned int>(_SERIAL_5N1));
		printToLog(_T("DEV\tvar\tSERIAL_6N1\t%u\n"), static_cast<unsigned int>(_SERIAL_6N1));
		printToLog(_T("DEV\tvar\tSERIAL_7N1\t%u\n"), static_cast<unsigned int>(_SERIAL_7N1));
		printToLog(_T("DEV\tvar\tSERIAL_8N1\t%u\n"), static_cast<unsigned int>(_SERIAL_8N1));
		printToLog(_T("DEV\tvar\tSERIAL_5N2\t%u\n"), static_cast<unsigned int>(_SERIAL_5N2));
		printToLog(_T("DEV\tvar\tSERIAL_6N2\t%u\n"), static_cast<unsigned int>(_SERIAL_6N2));
		printToLog(_T("DEV\tvar\tSERIAL_7N2\t%u\n"), static_cast<unsigned int>(_SERIAL_7N2));
		printToLog(_T("DEV\tvar\tSERIAL_8N2\t%u\n"), static_cast<unsigned int>(_SERIAL_8N2));
		printToLog(_T("DEV\tvar\tSERIAL_5E1\t%u\n"), static_cast<unsigned int>(_SERIAL_5E1));
		printToLog(_T("DEV\tvar\tSERIAL_6E1\t%u\n"), static_cast<unsigned int>(_SERIAL_6E1));
		printToLog(_T("DEV\tvar\tSERIAL_7E1\t%u\n"), static_cast<unsigned int>(_SERIAL_7E1));
		printToLog(_T("DEV\tvar\tSERIAL_8E1\t%u\n"), static_cast<unsigned int>(_SERIAL_8E1));
		printToLog(_T("DEV\tvar\tSERIAL_5E2\t%u\n"), static_cast<unsigned int>(_SERIAL_5E2));
		printToLog(_T("DEV\tvar\tSERIAL_6E2\t%u\n"), static_cast<unsigned int>(_SERIAL_6E2));
		printToLog(_T("DEV\tvar\tSERIAL_7E2\t%u\n"), static_cast<unsigned int>(_SERIAL_7E2));
		printToLog(_T("DEV\tvar\tSERIAL_8E2\t%u\n"), static_cast<unsigned int>(_SERIAL_8E2));
		printToLog(_T("DEV\tvar\tSERIAL_5O1\t%u\n"), static_cast<unsigned int>(_SERIAL_5O1));
		printToLog(_T("DEV\tvar\tSERIAL_6O1\t%u\n"), static_cast<unsigned int>(_SERIAL_6O1));
		printToLog(_T("DEV\tvar\tSERIAL_7O1\t%u\n"), static_cast<unsigned int>(_SERIAL_7O1));
		printToLog(_T("DEV\tvar\tSERIAL_8O1\t%u\n"), static_cast<unsigned int>(_SERIAL_8O1));
		printToLog(_T("DEV\tvar\tSERIAL_5O2\t%u\n"), static_cast<unsigned int>(_SERIAL_5O2));
		printToLog(_T("DEV\tvar\tSERIAL_6O2\t%u\n"), static_cast<unsigned int>(_SERIAL_6O2));
		printToLog(_T("DEV\tvar\tSERIAL_7O2\t%u\n"), static_cast<unsigned int>(_SERIAL_7O2));
		printToLog(_T("DEV\tvar\tSERIAL_8O2\t%u\n"), static_cast<unsigned int>(_SERIAL_8O2));
	}
	return true;
}


/**
 * Special handling for call results with OP code SPI_GET_CONSTANTS.
 * 
 * @param[in,out] buf - buffer to read from
 * @param[in,out] len - length of the buffer
 * @return true on success, else false
 */
template <>
bool _FV<_SPI_GET_CONSTANTS>::set(uint8_t * & buf, size_t & len) {
	if (len < 14) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_RESULT));
		return false;
	}
	readFrameValue(buf, len, _SPI_CLOCK_DIV2);
	readFrameValue(buf, len, _SPI_CLOCK_DIV4);
	readFrameValue(buf, len, _SPI_CLOCK_DIV8);
	readFrameValue(buf, len, _SPI_CLOCK_DIV16);
	readFrameValue(buf, len, _SPI_CLOCK_DIV32);
	readFrameValue(buf, len, _SPI_CLOCK_DIV64);
	readFrameValue(buf, len, _SPI_CLOCK_DIV128);
	readFrameValue(buf, len, _SPI_CLOCK_DIV256);
	readFrameValue(buf, len, _SPI_CLOCK_DIV512);
	readFrameValue(buf, len, _SPI_CLOCK_DIV1024);
	readFrameValue(buf, len, _SPI_MODE0);
	readFrameValue(buf, len, _SPI_MODE1);
	readFrameValue(buf, len, _SPI_MODE2);
	readFrameValue(buf, len, _SPI_MODE3);
	if (verbose > 1) {
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV2\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV2));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV4\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV4));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV8\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV8));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV16\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV16));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV32\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV32));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV64\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV64));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV128\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV128));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV256\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV256));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV512\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV512));
		printToLog(_T("DEV\tvar\tSPI_CLOCK_DIV1024\t%u\n"), static_cast<unsigned int>(_SPI_CLOCK_DIV1024));
		printToLog(_T("DEV\tvar\tSPI_MODE0\t%u\n"), static_cast<unsigned int>(_SPI_MODE0));
		printToLog(_T("DEV\tvar\tSPI_MODE1\t%u\n"), static_cast<unsigned int>(_SPI_MODE1));
		printToLog(_T("DEV\tvar\tSPI_MODE2\t%u\n"), static_cast<unsigned int>(_SPI_MODE2));
		printToLog(_T("DEV\tvar\tSPI_MODE3\t%u\n"), static_cast<unsigned int>(_SPI_MODE3));
	}
	return true;
}


} /* namespace adde */


/**
 * Changes the mode of the given pin.
 * 
 * @param[in] pin - pin to change
 * @param[in] mode - new pin settings
 */
void pinMode(uint8_t pin, uint8_t mode) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("pinMode()"), pin);
	if (mode != _INPUT && mode != _INPUT_PULLUP && mode != _OUTPUT) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_MODE), _T("pinMode()"), static_cast<unsigned int>(mode));
	}
	callWith<_void>(OpCode::PIN_MODE, _uint8_t(pin), _uint8_t(mode));
}


/**
 * Sets the digital pin value to HIGH or LOW.
 * 
 * @param[in] pin - digital pin to set
 * @param[in] val - new pin value
 */
void digitalWrite(uint8_t pin, uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("digitalWrite()"), pin);
	if (val != _LOW && val != _HIGH) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_VALUE), _T("digitalWrite()"), static_cast<unsigned int>(val));
	}
	callWith<_void>(OpCode::DIGITAL_WRITE, _uint8_t(pin), _uint8_t(val));
}


/**
 * Returns the current digital pin value.
 * 
 * @param[in] pin - digital pin requested
 * @return digital pin value
 */
int digitalRead(uint8_t pin) {
	return _digitalRead(pin);
}


/**
 * Returns the current digital pin value.
 * 
 * @param[in] pin - digital pin requested
 * @return digital pin value
 */
ADDE_F(_int) _digitalRead(uint8_t pin) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("digitalRead()"), pin);
	return callWith<_int>( OpCode::DIGITAL_READ, _uint8_t(pin));
}


/**
 * Returns the current analog pin value.
 * 
 * @param[in] pin - analog pin requested
 * @return analog pin value
 */
int analogRead(uint8_t pin) {
	return _analogRead(pin);
}


/**
 * Returns the current analog pin value.
 * 
 * @param[in] pin - analog pin requested
 * @return analog pin value
 */
ADDE_F(_int) _analogRead(uint8_t pin) {
	checkInitializedMain();
	using namespace ::adde;
	if (pin == INVALID_PIN) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_PIN), _T("analogRead()"), static_cast<unsigned int>(pin));
	}
	return callWith<_int>(OpCode::ANALOG_READ, _uint8_t(pin));
}


/**
 * Sets the analog reference voltage source for analog pins.
 * 
 * @param[in] mode - reference voltage source
 */
void analogReference(uint8_t mode) {
	checkInitializedMain();
	using namespace ::adde;
	if (mode == INVALID_REFERENCE || (mode != _EXTERNAL && mode != _INTERNAL1V1 && mode != _INTERNAL && mode != _INTERNAL2V56 && mode != _INTERNAL2V56_EXTCAP)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_MODE), _T("analogReference()"), static_cast<unsigned int>(mode));
	}
	callWith<_void>(OpCode::ANALOG_REFERENCE, _uint8_t(mode));
}


/**
 * Sets the analog pin value as given.
 * 
 * @param[in] pin - analog pin to set
 * @param[in] val - new pin value
 */
void analogWrite(uint8_t pin, uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (pin == INVALID_PIN) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_PIN), _T("analogWrite()"), static_cast<unsigned int>(pin));
	}
	callWith<_void>(OpCode::ANALOG_WRITE, _uint8_t(pin), _uint8_t(val));
}


/**
 * Sets the analog read resolution.
 * 
 * @param[in] val - analog read resolution (number of bits)
 */
void analogReadResolution(int val) {
	checkInitializedMain();
	using namespace ::adde;
	if (val > static_cast<int>(_int::size() * 8)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_VALUE), _T("analogReadResolution()"), static_cast<unsigned int>(val));
	}
	callWith<_void>(OpCode::ANALOG_READ_RESOLUTION, _int(val));
}


/**
 * Sets the analog write resolution.
 * 
 * @param[in] val - analog write resolution (number of bits)
 */
void analogWriteResolution(int val) {
	checkInitializedMain();
	using namespace ::adde;
	if (val > static_cast<int>(_int::size() * 8)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_VALUE), _T("analogWriteResolution()"), static_cast<unsigned int>(val));
	}
	callWith<_void>(OpCode::ANALOG_WRITE_RESOLUTION, _int(val));
}


/**
 * Reads a pulse (i.e. change from HIGH to LOW or LOW to HIGH; the later given as val) on the given pin.
 * 
 * @param[in] pin - digital pin requested
 * @param[in] val - type of pulse to read (i.e. value change to HIGH or LOW)
 * @param[in] timeout - optional timeout in microseconds
 * @return length of the pulse in microseconds
 */
unsigned long pulseIn(uint8_t pin, uint8_t val, unsigned long timeout) {
	return _pulseIn(pin, val, timeout);
}


/**
 * Reads a pulse (i.e. change from HIGH to LOW or LOW to HIGH; the later given as val) on the given pin.
 * 
 * @param[in] pin - digital pin requested
 * @param[in] val - type of pulse to read (i.e. value change to HIGH or LOW)
 * @param[in] timeout - optional timeout in microseconds
 * @return length of the pulse in microseconds
 */
ADDE_F(_unsigned_long) _pulseIn(uint8_t pin, uint8_t val, unsigned long timeout) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("pulseIn()"), pin);
	if (val != _LOW && val != _HIGH) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_VALUE), _T("pulseIn()"), static_cast<unsigned int>(val));
	}
	return callWith<_unsigned_long>(static_cast<size_t>(timeout / 1000UL), OpCode::PULSE_IN, _uint8_t(pin), _uint8_t(val), _unsigned_long(timeout));
}


/**
 * Reads a pulse (i.e. change from HIGH to LOW or LOW to HIGH; the later given as val) on the given pin.
 * 
 * @param[in] pin - digital pin requested
 * @param[in] val - type of pulse to read (i.e. value change to HIGH or LOW)
 * @param[in] timeout - optional timeout in microseconds
 * @return length of the pulse in microseconds
 */
unsigned long pulseInLong(uint8_t pin, uint8_t val, unsigned long timeout) {
	return _pulseInLong(pin, val, timeout);
}


/**
 * Reads a pulse (i.e. change from HIGH to LOW or LOW to HIGH) on the given pin.
 * 
 * @param[in] pin - digital pin requested
 * @param[in] val - type of pulse to read (i.e. value change to HIGH or LOW)
 * @param[in] timeout - optional timeout in microseconds
 * @return length of the pulse in microseconds
 */
ADDE_F(_unsigned_long) _pulseInLong(uint8_t pin, uint8_t val, unsigned long timeout) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("pulseInLong()"), pin);
	if (val != _LOW && val != _HIGH) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_VALUE), _T("pulseInLong()"), static_cast<unsigned int>(val));
	}
	return callWith<_unsigned_long>(static_cast<size_t>(timeout / 1000UL), OpCode::PULSE_IN_LONG, _uint8_t(pin), _uint8_t(val), _unsigned_long(timeout));
}


/**
 * Writes out a byte as a pulsed sequence of bits in the given bit order.
 * This is a software implementation of SPI.
 * 
 * @param[in] dataPin - pin to write the data to
 * @param[in] clockPin - pin to write the clock signal to
 * @param[in] bitOrder - bit order (LSBFIRST or MSBFIRST)
 * @param[in] val - byte value to write out
 */
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("shiftOut() arg dataPin"), dataPin);
	checkDigitalPin(_T("shiftOut() arg clockPin"), clockPin);
	if (bitOrder != _LSBFIRST && bitOrder != _MSBFIRST) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_BITORDER), _T("shiftOut()"), static_cast<unsigned int>(bitOrder));
	}
	callWith<_void>(OpCode::SHIFT_OUT, _uint8_t(dataPin), _uint8_t(clockPin), _uint8_t(bitOrder), _uint8_t(val));
}


/**
 * Reads in a byte as a pulsed sequence of bits in the given bit order.
 * This is a software implementation of SPI.
 * 
 * @param[in] dataPin - pin to write the data to
 * @param[in] clockPin - pin to write the clock signal to
 * @param[in] bitOrder - bit order (LSBFIRST or MSBFIRST)
 * @return received byte value
 */
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
	return _shiftIn(dataPin, clockPin, bitOrder);
}


/**
 * Reads in a byte as a pulsed sequence of bits in the given bit order.
 * This is a software implementation of SPI.
 * 
 * @param[in] dataPin - pin to write the data to
 * @param[in] clockPin - pin to write the clock signal to
 * @param[in] bitOrder - bit order (LSBFIRST or MSBFIRST)
 * @return received byte value
 */
ADDE_F(_uint8_t) _shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("shiftIn() arg dataPin"), dataPin);
	checkDigitalPin(_T("shiftIn() arg clockPin"), clockPin);
	if (bitOrder != _LSBFIRST && bitOrder != _MSBFIRST) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_BITORDER), _T("shiftOut()"), static_cast<unsigned int>(bitOrder));
	}
	return callWith<_uint8_t>(OpCode::SHIFT_IN, _uint8_t(dataPin), _uint8_t(clockPin), _uint8_t(bitOrder));
}


/**
 * Plays a tone in the given frequency on the passed pin. Arduino uses Timer2 to generate the tone,
 * thus only one tone can be generated at a time. Use noTone() to stop the tone from playing.
 * 
 * @param[in] pin - pin to play a tone on
 * @param[in] frequency - frequency of the tone in Hz
 * @param[in] duration - optional duration of the tone
 */
void tone(uint8_t pin, unsigned int frequency, unsigned long duration) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("tone()"), pin);
	callWith<_void>(OpCode::TONE, _uint8_t(pin), _unsigned_int(frequency), _unsigned_long(duration));
}


/**
 * Stops playing a tone on the given pin.
 * 
 * @param[in] pin - pin to stop playing a tone on
 */
void noTone(uint8_t pin) {
	checkInitializedMain();
	using namespace ::adde;
	checkDigitalPin(_T("noTone()"), pin);
	callWith<_void>(OpCode::NO_TONE, _uint8_t(pin));
}


/**
 * Attaches an callback to the given interrupt. The trigger type is given with mode. Use
 * digitalPinToInterrupt() to obtain the interrupt from a given pin.
 * 
 * @param[in] interrupt - interrupt to attach
 * @param[in] isr - callback routine upon interrupt
 * @param[in] mode - trigger mode
 */
void attachInterrupt(uint8_t interrupt, void (* isr)(void), int mode) {
	checkInitializedMain();
	using namespace ::adde;
	if (mode != _LOW && mode != _CHANGE && mode != _RISING && mode != _FALLING && mode != _HIGH) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_MODE), _T("attachInterrupt()"), static_cast<unsigned int>(mode));
	}
	for (uint8_t i = 0, j = 0; i < _NUM_DIGITAL_PINS; i++) {
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == static_cast<uint8_t>(NOT_AN_INTERRUPT)) continue;
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == interrupt) {
			ints[j] = isr;
			callWith<_void>(OpCode::ATTACH_INTERRUPT, _uint8_t(interrupt), _uint8_t(j), _int(mode));
			return;
		}
		j++;
		if (j >= MAX_NUM_INTERRUPTS) {
			if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_MAX_INTERRUPT), _T("attachInterrupt()"), static_cast<unsigned int>(interrupt));
			return;
		}
	}
	if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INTERRUPT), _T("attachInterrupt()"), static_cast<unsigned int>(interrupt));
}


/**
 * Detaches the callback associated to the given interrupt. Use digitalPinToInterrupt() to obtain
 * the interrupt from a given pin.
 * 
 * @param[in] interrupt - interrupt to detach
 */
void detachInterrupt(uint8_t interrupt) {
	checkInitializedMain();
	using namespace ::adde;
	for (uint8_t i = 0, j = 0; i < _NUM_DIGITAL_PINS; i++) {
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == static_cast<uint8_t>(NOT_AN_INTERRUPT)) continue;
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == interrupt) {
			ints[j] = NULL;
			callWith<_void>(OpCode::DETACH_INTERRUPT, _uint8_t(interrupt));
			return;
		}
		j++;
		if (j >= MAX_NUM_INTERRUPTS) {
			if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_MAX_INTERRUPT), _T("detachInterrupt()"), static_cast<unsigned int>(interrupt));
			return;
		}
	}
	if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INTERRUPT), _T("detachInterrupt()"), static_cast<unsigned int>(interrupt));
}


/**
 * Returns the number of milliseconds passed since the program was started.
 * 
 * @return milliseconds since program start
 */
unsigned long millis(void) {
	checkInitializedMain();
#ifdef PCF_IS_WIN
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return static_cast<unsigned long>((static_cast<int64_t>(counter.QuadPart) - startTime) * INT64_C(1000) / static_cast<int64_t>(perfFreq.QuadPart));
#elif defined(PCF_IS_LINUX)
	return static_cast<unsigned long>((getAbsTime() - startTime) / UINT64_C(1000));
#endif
}


/**
 * Returns the number of microseconds passed since the program was started.
 * 
 * @return microseconds since program start
 */
unsigned long micros(void) {
	checkInitializedMain();
#ifdef PCF_IS_WIN
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return static_cast<unsigned long>((static_cast<int64_t>(counter.QuadPart) - startTime) * INT64_C(1000000) / static_cast<int64_t>(perfFreq.QuadPart));
#elif defined(PCF_IS_LINUX)
	return static_cast<unsigned long>(getAbsTime() - startTime);
#endif
}


/**
 * The function lets the current thread sleep for the number
 * of milliseconds passed to it.
 *
 * @param[in] duration - time duration to sleep
 * @remarks The duration can get incorrect results if less than 10 milliseconds.
 * @remarks yield() is given a multiple of 
 */
void delay(const unsigned long duration) {
	checkInitializedMain();
	if (duration == 0) return;
	const unsigned long start = millis();
	const unsigned long end = start + duration;
	const uint64_t cpuIntvl = getCpuInterval();
	const unsigned long interval = static_cast<unsigned long>(PCF_MAX(UINT64_C(1), cpuIntvl / UINT64_C(1000)));
	for (;;) {
		yield(); /* call user defined function */
		const unsigned long now = millis();
		if (static_cast<unsigned long>(now - start) >= duration) return;
		const unsigned long rem = static_cast<unsigned long>(end - now);
		const unsigned long next = (rem >= interval) ? interval : rem;
		if (next == 0) break;
#if defined(PCF_IS_WIN)
		Sleep(static_cast<DWORD>(next));
#elif defined(PCF_IS_LINUX)
		usleep(static_cast<useconds_t>(next * 1000));
#endif
	}
}


/**
 * The function lets the current thread sleep for the number
 * of microseconds passed to it.
 *
 * @param[in] duration - time duration to sleep
 * @remarks The duration can get incorrect results if less than 1 millisecond.
 */
void delayMicroseconds(const unsigned int duration) {
	checkInitializedMain();
#if defined(PCF_IS_WIN)
	LARGE_INTEGER ft;
	HANDLE timer = CreateWaitableTimer(NULL, TRUE, NULL);
	if (timer != NULL) {
		ft.QuadPart = -((LONGLONG)(duration * 10));
		SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
		WaitForSingleObject(timer, INFINITE);
		CloseHandle(timer);
	}
#elif defined(PCF_IS_LINUX)
	struct timespec ts;
	ts.tv_sec = duration / 1000000;
	ts.tv_nsec = (duration % 1000000) * 1000;
	errno = 0;
	while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
#endif
}


/**
 * Constructor.
 */
ConsoleSerial::ConsoleSerial() {}


/**
 * Destructor.
 */
ConsoleSerial::~ConsoleSerial() {}


/**
 * Starts the console serial port at the given Baud rate.
 * 
 * @param[in] speed - Baud rate of the port
 */
void ConsoleSerial::begin(unsigned long speed) {
	checkInitializedMain();
	printToLog(_T2("HOST\tout\t%" PRTCHAR "\t%lu\n"), opStr[OpCode::HARDWARE_SERIAL_BEGIN1], speed);
}


/**
 * Starts the console serial port at the given Baud rate and mode setting.
 * 
 * @param[in] speed - Baud rate of the port
 * @param[in] mode - data transmission and framing mode (e.g. SERIAL_8N1)
 */
void ConsoleSerial::begin(unsigned long speed, uint8_t mode) {
	checkInitializedMain();
	printToLog(_T2("HOST\tout\t%" PRTCHAR "\t%lu\t%u\n"), opStr[OpCode::HARDWARE_SERIAL_BEGIN2], speed, static_cast<unsigned int>(mode));
}


/**
 * Stops the console serial instance.
 */
void ConsoleSerial::end() {
	checkInitializedMain();
	printToLog(_T2("HOST\tout\t%" PRTCHAR "\n"), opStr[OpCode::HARDWARE_SERIAL_END]);
}


/**
 * Returns the number of bytes available for reading.
 * 
 * @return number of bytes available in input buffer
 */
int ConsoleSerial::available() {
	checkInitializedMain();
	const int res = (conPeekBuffer >= 0) ? 1 : 0;
	printToLog(_T2("HOST\tin\t%" PRTCHAR "\t%i\n"), opStr[OpCode::HARDWARE_SERIAL_AVAILABLE], res);
	return res;
}


/**
 * Returns the number of bytes available for writing.
 * 
 * @return number of free bytes available in output buffer
 */
int ConsoleSerial::availableForWrite() {
	checkInitializedMain();
	const int res = (fout != NULL && feof(fout) == 0 && ferror(fout) == 0) ? 256 : 0;
	printToLog(_T2("HOST\tin\t%" PRTCHAR "\t%i\n"), opStr[OpCode::HARDWARE_SERIAL_AVAILABLE_FOR_WRITE], res);
	return res;
}


/**
 * Returns the next received byte without removing it from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int ConsoleSerial::peek() {
	checkInitializedMain();
	const int res = conPeekBuffer;
	printToLog(_T2("HOST\tin\t%" PRTCHAR "\t%i\n"), opStr[OpCode::HARDWARE_SERIAL_PEEK], res);
	return res;
}


/**
 * Returns the next received byte from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int ConsoleSerial::read() {
	checkInitializedMain();
	int res = conPeekBuffer;
	conPeekBuffer = -1;
	if (res >= 0) return res;
	while (signalReceived == 0) {
		if ( spinWaitNotDelay(conPeekBuffer, -1, 1, TIMEOUT_RESOLUTION) ) {
			res = conPeekBuffer;
			conPeekBuffer = -1;
			break;
		}
	}
	printToLog(_T2("HOST\tin\t%" PRTCHAR "\t%i\n"), opStr[OpCode::HARDWARE_SERIAL_READ], res);
	return res;
}


/**
 * Flush the output buffer and block until all remaining bytes are transmitted.
 */
void ConsoleSerial::flush() {
	checkInitializedMain();
	if (fout != NULL) fflush(fout);
	printToLog(_T2("HOST\tout\t%" PRTCHAR "\n"), opStr[OpCode::HARDWARE_SERIAL_FLUSH]);
}


/**
 * Transmits the given byte.
 * 
 * @param[in] val - byte value to send
 * @return number of bytes transmitted
 */
size_t ConsoleSerial::write(uint8_t val) {
	checkInitializedMain();
	if (fout == NULL) return 0;
	printToLog(_T2("HOST\tout\t%" PRTCHAR "\t%u\n"), opStr[OpCode::HARDWARE_SERIAL_WRITE], static_cast<unsigned int>(val));
	const size_t res = static_cast<size_t>(fwrite(&val, 1, 1, fout));
	printToLog(_T2("HOST\tin\t%" PRTCHAR "\t%u\n"), opStr[OpCode::HARDWARE_SERIAL_WRITE], static_cast<unsigned int>(res));
	return res;
}


/**
 * Returns true if the serial console is available.
 * 
 * @return true if set, else false
 */
ConsoleSerial::operator bool() {
	checkInitializedMain();
	return fin != NULL && fout != NULL;
}


/**
 * Constructor.
 * 
 * @param[in] instance - instance number
 */
HardwareSerial::HardwareSerial(const uint8_t instance):
	inst(instance)
{}


/**
 * Destructor.
 */
HardwareSerial::~HardwareSerial() {
	if (signalReceived >= 100) return;
	checkInitializedMain();
	if (this->inst < hwSerCount) this->end();
}


/**
 * Starts the serial port at the given Baud rate.
 * 
 * @param[in] speed - Baud rate of the port
 */
void HardwareSerial::begin(unsigned long speed) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::begin(speed)"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::HARDWARE_SERIAL_BEGIN1, _uint8_t(this->inst), _unsigned_long(speed));
}


/**
 * Starts the serial port at the given Baud rate and mode setting.
 * 
 * @param[in] speed - Baud rate of the port
 * @param[in] mode - data transmission and framing mode (e.g. SERIAL_8N1)
 */
void HardwareSerial::begin(unsigned long speed, uint8_t mode) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::begin(speed, mode)"), static_cast<unsigned int>(this->inst));
	}
	if (mode != _SERIAL_5N1 && mode != _SERIAL_6N1 && mode != _SERIAL_7N1 && mode != _SERIAL_8N1 &&
		mode != _SERIAL_5N2 && mode != _SERIAL_6N2 && mode != _SERIAL_7N2 && mode != _SERIAL_8N2 &&
		mode != _SERIAL_5E1 && mode != _SERIAL_6E1 && mode != _SERIAL_7E1 && mode != _SERIAL_8E1 &&
		mode != _SERIAL_5E2 && mode != _SERIAL_6E2 && mode != _SERIAL_7E2 && mode != _SERIAL_8E2 &&
		mode != _SERIAL_5O1 && mode != _SERIAL_6O1 && mode != _SERIAL_7O1 && mode != _SERIAL_8O1 &&
		mode != _SERIAL_5O2 && mode != _SERIAL_6O2 && mode != _SERIAL_7O2 && mode != _SERIAL_8O2) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_MODE), _T("HardwareSerial::begin()"), static_cast<unsigned int>(mode));
	}
	callWith<_void>(OpCode::HARDWARE_SERIAL_BEGIN2, _uint8_t(this->inst), _unsigned_long(speed), _uint8_t(mode));
}


/**
 * Stops the hardware serial instance to free the pins for other operations.
 */
void HardwareSerial::end() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::end()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::HARDWARE_SERIAL_END, _uint8_t(this->inst));
}


/**
 * Returns the number of bytes available for reading.
 * 
 * @return number of bytes available in input buffer
 */
int HardwareSerial::available() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::available()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_int>(OpCode::HARDWARE_SERIAL_AVAILABLE, _uint8_t(this->inst));
}


/**
 * Returns the number of bytes available for writing.
 * 
 * @return number of free bytes available in output buffer
 */
int HardwareSerial::availableForWrite() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::availableForWrite()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_int>(OpCode::HARDWARE_SERIAL_AVAILABLE_FOR_WRITE, _uint8_t(this->inst));
}


/**
 * Returns the next received byte without removing it from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int HardwareSerial::peek() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::peek()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_int>(OpCode::HARDWARE_SERIAL_PEEK, _uint8_t(this->inst));
}


/**
 * Returns the next received byte from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int HardwareSerial::read() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::read()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_int>(OpCode::HARDWARE_SERIAL_READ, _uint8_t(this->inst));
}


/**
 * Flush the output buffer and block until all remaining bytes are transmitted.
 */
void HardwareSerial::flush() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::flush()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::HARDWARE_SERIAL_FLUSH, _uint8_t(this->inst));
}


/**
 * Transmits the given byte.
 * 
 * @param[in] val - byte value to send
 * @return number of bytes transmitted
 */
size_t HardwareSerial::write(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= hwSerCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("HardwareSerial::write()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_size_t>(OpCode::HARDWARE_SERIAL_WRITE, _uint8_t(this->inst), _uint8_t(val));
}


/**
 * Constructor.
 * 
 * @param[in] receivePin - RX pin (incoming)
 * @param[in] transmitPin - TX pin (outgoing)
 * @param[in] inverseLogic - invert bit values
 */
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverseLogic) {
	checkInitializedMain(false);
	using namespace ::adde;
	this->inst = swSerInst.add();
	if (this->inst == RemoteInstance::npos) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_INST_COUNT), _T("SoftwareSerial"));
		signalReceived += 100;
		exit(EXIT_FAILURE);
	}
	checkDigitalPin(_T("SoftwareSerial::SoftwareSerial() arg receivePin"), receivePin);
	checkDigitalPin(_T("SoftwareSerial::SoftwareSerial() arg transmitPin"), transmitPin);
	callWith<_void>(OpCode::SOFTWARE_SERIAL, _uint8_t(this->inst), _uint8_t(receivePin), _uint8_t(transmitPin), _bool(inverseLogic));
}


/**
 * Destructor.
 */
SoftwareSerial::~SoftwareSerial() {
	if (signalReceived >= 100) return;
	checkInitializedMain();
	this->end();
	swSerInst.erase(this->inst);
}


/**
 * Starts the serial port at the given Baud rate.
 * 
 * @param[in] speed - Baud rate of the port
 */
void SoftwareSerial::begin(long speed) {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::SOFTWARE_SERIAL_BEGIN, _uint8_t(this->inst), _long(speed));
}


/**
 * Sets the current object as the one listening and returns true if this replaces another one.
 * 
 * @return true if other object was replaced, else false
 */
ADDE_F(_bool) SoftwareSerial::listen() {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_bool>(OpCode::SOFTWARE_SERIAL_LISTEN, _uint8_t(this->inst));
}


/**
 * Stops the software serial instance to free the pins for other operations.
 */
void SoftwareSerial::end() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::SOFTWARE_SERIAL_END, _uint8_t(this->inst));
}


/**
 * Returns whether this software serial instance is in listening mode.
 * 
 * @return true if listening, else false
 */
ADDE_F(_bool) SoftwareSerial::isListening() {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_bool>(OpCode::SOFTWARE_SERIAL_IS_LISTENING, _uint8_t(this->inst));
}


/**
 * Returns whether this software serial instance is in listening mode.
 * 
 * @return true if it was listening, else false
 */
ADDE_F(_bool) SoftwareSerial::stopListening() {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_bool>(OpCode::SOFTWARE_SERIAL_STOP_LISTENING, _uint8_t(this->inst));
}


/**
 * Returns whether there was a buffer overflow.
 * 
 * @return true on overflow, else false
 */
ADDE_F(_bool) SoftwareSerial::overflow() {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_bool>(OpCode::SOFTWARE_SERIAL_OVERFLOW, _uint8_t(this->inst));
}


/**
 * Returns the number of bytes available for reading.
 * 
 * @return number of bytes available in input buffer
 */
int SoftwareSerial::available() {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_int>(OpCode::SOFTWARE_SERIAL_AVAILABLE, _uint8_t(this->inst));
}


/**
 * Returns the next received byte without removing it from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int SoftwareSerial::peek() {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_int>(OpCode::SOFTWARE_SERIAL_PEEK, _uint8_t(this->inst));
}


/**
 * Returns the next received byte from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int SoftwareSerial::read() {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_int>(OpCode::SOFTWARE_SERIAL_READ, _uint8_t(this->inst));
}


/**
 * Transmits the given byte.
 * 
 * @param[in] val - byte value to send
 * @return number of bytes transmitted
 */
size_t SoftwareSerial::write(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_size_t>(OpCode::SOFTWARE_SERIAL_WRITE, _uint8_t(this->inst), _uint8_t(val));
}


/**
 * Dereferencing operator. Reads a byte from the referenced position in the EEPROM.
 * 
 * @return byte value from EEPROM
 */
ADDE_F(_uint8_t) EERef::operator* () const {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= eepromCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("EERef::operator* ()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_uint8_t>(OpCode::EEPROM_READ, _uint8_t(0), _int(this->index));
}


/**
 * Assignment operator. Writes a byte to the referenced position in the EEPROM.
 * 
 * @param[in] val - value to write
 * @return reference to this position
 */
EERef & EERef::operator= (uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= eepromCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("EERef::operator= ()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::EEPROM_WRITE, _uint8_t(0), _int(this->index), _uint8_t(val));
	return *this;
}


/**
 * Writes a byte to the referenced position in the EEPROM if the current stored value is different.
 * 
 * @param[in] val - value to write
 * @return reference to this position
 */
EERef & EERef::update(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= eepromCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("EERef::update()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::EEPROM_UPDATE, _uint8_t(0), _int(this->index), _uint8_t(val));
	return *this;
}


/**
 * Constructor.
 * 
 * @param[in] instance - instance number
 */
EEPROMClass::EEPROMClass(const uint8_t instance):
	inst(instance)
{}


/**
 * Return the size of the EEPROM in number of bytes.
 * 
 * @return EEPROM size
 */
ADDE_F(_uint16_t) EEPROMClass::length() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= eepromCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("EEPROMClass::length()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_uint16_t>(OpCode::EEPROM_LENGTH, _uint8_t(0));
}


/**
 * Constructor.
 * 
 * @param[in] rs - reset pin
 * @param[in] enable - data enable pin
 * @param[in] d0 - data pin 0
 * @param[in] d1 - data pin 1
 * @param[in] d2 - data pin 2
 * @param[in] d3 - data pin 3
 * @param[in] d4 - data pin 4
 * @param[in] d5 - data pin 5
 * @param[in] d6 - data pin 6
 * @param[in] d7 - data pin 7
 */
LiquidCrystal::LiquidCrystal(uint8_t rs, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
	checkInitializedMain(false);
	using namespace ::adde;
	this->inst = lcdInst.add();
	if (this->inst == RemoteInstance::npos) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_INST_COUNT), _T("LiquidCrystal"));
		signalReceived += 100;
		exit(EXIT_FAILURE);
	}
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg rs"), rs);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg enable"), enable);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d0"), d0);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d1"), d1);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d2"), d2);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d3"), d3);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d4"), d4);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d5"), d5);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d6"), d6);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d7"), d7);
	callWith<_void>(OpCode::LIQUID_CRYSTAL1, _uint8_t(this->inst), _uint8_t(rs), _uint8_t(enable), _uint8_t(d0), _uint8_t(d1), _uint8_t(d2), _uint8_t(d3), _uint8_t(d4), _uint8_t(d5), _uint8_t(d6), _uint8_t(d7));
}


/**
 * Constructor.
 * 
 * @param[in] rs - reset pin
 * @param[in] rw - data read/write pin
 * @param[in] enable - data enable pin
 * @param[in] d0 - data pin 0
 * @param[in] d1 - data pin 1
 * @param[in] d2 - data pin 2
 * @param[in] d3 - data pin 3
 * @param[in] d4 - data pin 4
 * @param[in] d5 - data pin 5
 * @param[in] d6 - data pin 6
 * @param[in] d7 - data pin 7
 */
LiquidCrystal::LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
	checkInitializedMain(false);
	using namespace ::adde;
	this->inst = lcdInst.add();
	if (this->inst == RemoteInstance::npos) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_INST_COUNT), _T("LiquidCrystal"));
		signalReceived += 100;
		exit(EXIT_FAILURE);
	}
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg rs"), rs);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg rw"), rw);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg enable"), enable);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d0"), d0);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d1"), d1);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d2"), d2);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d3"), d3);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d4"), d4);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d5"), d5);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d6"), d6);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d7"), d7);
	callWith<_void>(OpCode::LIQUID_CRYSTAL2, _uint8_t(this->inst), _uint8_t(rs), _uint8_t(rw), _uint8_t(enable), _uint8_t(d0), _uint8_t(d1), _uint8_t(d2), _uint8_t(d3), _uint8_t(d4), _uint8_t(d5), _uint8_t(d6), _uint8_t(d7));
}


/**
 * Constructor.
 * 
 * @param[in] rs - reset pin
 * @param[in] rw - data read/write pin
 * @param[in] enable - data enable pin
 * @param[in] d0 - data pin 0
 * @param[in] d1 - data pin 1
 * @param[in] d2 - data pin 2
 * @param[in] d3 - data pin 3
 */
LiquidCrystal::LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
	checkInitializedMain(false);
	using namespace ::adde;
	this->inst = lcdInst.add();
	if (this->inst == RemoteInstance::npos) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_INST_COUNT), _T("LiquidCrystal"));
		signalReceived += 100;
		exit(EXIT_FAILURE);
	}
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg rs"), rs);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg rw"), rw);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg enable"), enable);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d0"), d0);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d1"), d1);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d2"), d2);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d3"), d3);
	callWith<_void>(OpCode::LIQUID_CRYSTAL3, _uint8_t(this->inst), _uint8_t(rs), _uint8_t(rw), _uint8_t(enable), _uint8_t(d0), _uint8_t(d1), _uint8_t(d2), _uint8_t(d3));
}


/**
 * Constructor.
 * 
 * @param[in] rs - reset pin
 * @param[in] enable - data enable pin
 * @param[in] d0 - data pin 0
 * @param[in] d1 - data pin 1
 * @param[in] d2 - data pin 2
 * @param[in] d3 - data pin 3
 */
LiquidCrystal::LiquidCrystal(uint8_t rs, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
	checkInitializedMain(false);
	using namespace ::adde;
	this->inst = lcdInst.add();
	if (this->inst == RemoteInstance::npos) {
		if (verbose > 0) printToErr(MSGT(MSGT_ERR_REMOTE_INST_COUNT), _T("LiquidCrystal"));
		signalReceived += 100;
		exit(EXIT_FAILURE);
	}
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg rs"), rs);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg enable"), enable);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d0"), d0);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d1"), d1);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d2"), d2);
	checkDigitalPin(_T("LiquidCrystal::LiquidCrystal() arg d3"), d3);
	callWith<_void>(OpCode::LIQUID_CRYSTAL4, _uint8_t(this->inst), _uint8_t(rs), _uint8_t(enable), _uint8_t(d0), _uint8_t(d1), _uint8_t(d2), _uint8_t(d3));
}


/**
 * Destructor.
 */
LiquidCrystal::~LiquidCrystal() {
	if (signalReceived >= 100) return;
	checkInitializedMain();
	lcdInst.erase(this->inst);
}


/**
 * Starts the liquid crystal interface with the given parameters.
 * 
 * @param[in] cols - number of columns
 * @param[in] rows - number of rows
 * @param[in] charSize - character size (defaults to LCD_5x8DOTS)
 */
void LiquidCrystal::begin(uint8_t cols, uint8_t rows, uint8_t charSize) {
	checkInitializedMain();
	using namespace ::adde;
	if (cols <= 0 || cols > 20) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_COL_COUNT), _T("LiquidCrystal::begin()"), static_cast<unsigned int>(cols));
	}
	if (rows <= 0 || rows > 8) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_ROW_COUNT), _T("LiquidCrystal::begin()"), static_cast<unsigned int>(rows));
	}
	if (charSize > (LCD_4BITMODE | LCD_8BITMODE | LCD_1LINE | LCD_2LINE | LCD_5x8DOTS | LCD_5x10DOTS)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_CHAR_SIZE), _T("LiquidCrystal::begin()"), static_cast<unsigned int>(charSize));
	}
	callWith<_void>(OpCode::LIQUID_CRYSTAL_BEGIN, _uint8_t(this->inst), _uint8_t(cols), _uint8_t(rows), _uint8_t(charSize));
}


/**
 * Clears the display.
 */
void LiquidCrystal::clear() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_CLEAR, _uint8_t(this->inst));
}


/**
 * Returns the cursor to the upper left corner.
 */
void LiquidCrystal::home() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_HOME, _uint8_t(this->inst));
}


/**
 * Turns the display off.
 */
void LiquidCrystal::noDisplay() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_NO_DISPLAY, _uint8_t(this->inst));
}


/**
 * Turns the display on.
 */
void LiquidCrystal::display() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_DISPLAY, _uint8_t(this->inst));
}


/**
 * Do not display a blinking cursor.
 */
void LiquidCrystal::noBlink() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_NO_BLINK, _uint8_t(this->inst));
}


/**
 * Display a blinking cursor.
 */
void LiquidCrystal::blink() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_BLINK, _uint8_t(this->inst));
}


/**
 * Do not display a cursor.
 */
void LiquidCrystal::noCursor() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_NO_CURSOR, _uint8_t(this->inst));
}


/**
 * Display a cursor.
 */
void LiquidCrystal::cursor() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_CURSOR, _uint8_t(this->inst));
}


/**
 * Moves the displayed characters one character to the left.
 */
void LiquidCrystal::scrollDisplayLeft() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_SCROLL_DISPLAY_LEFT, _uint8_t(this->inst));
}


/**
 * Moves the displayed characters one character to the right.
 */
void LiquidCrystal::scrollDisplayRight() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_SCROLL_DISPLAY_RIGHT, _uint8_t(this->inst));
}


/**
 * Sets the text writing direction to write from left to right.
 */
void LiquidCrystal::leftToRight() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_LEFT_TO_RIGHT, _uint8_t(this->inst));
}


/**
 * Sets the text writing direction to write from right to left.
 */
void LiquidCrystal::rightToLeft() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_RIGHT_TO_LEFT, _uint8_t(this->inst));
}


/**
 * Turns automatic scrolling off.
 */
void LiquidCrystal::noAutoscroll() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_NO_AUTOSCROLL, _uint8_t(this->inst));
}


/**
 * Turns automatic scrolling on.
 */
void LiquidCrystal::autoscroll() {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_AUTOSCROLL, _uint8_t(this->inst));
}


/**
 * Sets the row offsets.
 * 
 * @param[in] row1 - character offset at row 1
 * @param[in] row2 - character offset at row 2
 * @param[in] row3 - character offset at row 3
 * @param[in] row4 - character offset at row 4
 */
void LiquidCrystal::setRowOffsets(int row1, int row2, int row3, int row4) {
	checkInitializedMain();
	using namespace ::adde;
	if (row1 < 0 || row1 > 20) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_COL), _T("LiquidCrystal::setRowOffsets() arg row1"), row1);
	}
	if (row2 < 0 || row2 > 20) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_COL), _T("LiquidCrystal::setRowOffsets() arg row2"), row2);
	}
	if (row3 < 0 || row3 > 20) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_COL), _T("LiquidCrystal::setRowOffsets() arg row3"), row3);
	}
	if (row4 < 0 || row4 > 20) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_COL), _T("LiquidCrystal::setRowOffsets() arg row4"), row4);
	}
	callWith<_void>(OpCode::LIQUID_CRYSTAL_SET_ROW_OFFSETS, _uint8_t(this->inst), _int(row1), _int(row2), _int(row3), _int(row4));
}


/**
 * Creates a new custom character from the given bitmap at the passed location.
 * 
 * @param[in] location - location where to store the new character at
 * @param[in] charMap - bitmap of the character
 */
void LiquidCrystal::createChar(uint8_t location, uint8_t * charMap) {
	checkInitializedMain();
	using namespace ::adde;
	if (location > 7) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_CHAR_LOCATION), _T("LiquidCrystal::createChar()"), static_cast<unsigned int>(location));
	}
	callWith<_void>(OpCode::LIQUID_CRYSTAL_CREATE_CHAR, _uint8_t(this->inst), _uint8_t(location), makePtrArg<_uint8_t>(charMap, 8));
}


/**
 * Sets the cursor to the given position.
 * 
 * @param[in] col - column (starting at 0)
 * @param[in] row - row (starting at 0)
 */
void LiquidCrystal::setCursor(uint8_t col, uint8_t row) {
	checkInitializedMain();
	using namespace ::adde;
	if (col > 20) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_COL), _T("LiquidCrystal::setCursor()"), static_cast<int>(col));
	}
	if (row > 4) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_ROW), _T("LiquidCrystal::setCursor()"), static_cast<int>(row));
	}
	callWith<_void>(OpCode::LIQUID_CRYSTAL_SET_CURSOR, _uint8_t(this->inst), _uint8_t(col), _uint8_t(row));
}


/**
 * Writes the given command to the display.
 * 
 * @param[in] val - command to write
 */
void LiquidCrystal::command(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	callWith<_void>(OpCode::LIQUID_CRYSTAL_COMMAND, _uint8_t(this->inst), _uint8_t(val));
}


/**
 * Prints the given character.
 * 
 * @param[in] val - character to print
 * @return number of characters printed
 */
size_t LiquidCrystal::write(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	return callWith<_size_t>(OpCode::LIQUID_CRYSTAL_WRITE, _uint8_t(this->inst), _uint8_t(val));
}


/**
 * Constructor.
 * 
 * @param[in] instance - instance number
 */
SPIClass::SPIClass(const uint8_t instance):
	inst(instance)
{}


/**
 * Destructor.
 */
SPIClass::~SPIClass() {
	if (signalReceived >= 100) return;
	checkInitializedMain();
	if (this->inst < spiCount) this->end();
}


/**
 * Starts the SPI interface.
 */
void SPIClass::begin() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::begin()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::SPI_BEGIN, _uint8_t(this->inst));
}


/**
 * Attaches a dedicated interrupt number to use SPI from within interrupt handlers.
 * 
 * @param[in] interrupt - interrupt number to attach for SPI transactions
 */
void SPIClass::usingInterrupt(uint8_t interrupt) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::usingInterrupt()"), static_cast<unsigned int>(this->inst));
	}
	bool invalidInterrupt = true;
	for (uint8_t i = 0; i < _NUM_DIGITAL_PINS; i++) {
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == static_cast<uint8_t>(NOT_AN_INTERRUPT)) continue;
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == interrupt) {
			invalidInterrupt = true;
			break;
		}
	}
	if (invalidInterrupt == false && verbose > 1) {
		printToErr(MSGT(MSGT_WARN_API_INV_INTERRUPT), _T("SPIClass::usingInterrupt()"), static_cast<unsigned int>(interrupt));
	}
	callWith<_void>(OpCode::SPI_USING_INTERRUPT, _uint8_t(this->inst), _uint8_t(interrupt));
}


/**
 * Detaches the previously attached dedicated interrupt number to use SPI from within interrupt handlers.
 * 
 * @param[in] interrupt - interrupt number to detach for SPI transactions
 */
void SPIClass::notUsingInterrupt(uint8_t interrupt) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::notUsingInterrupt()"), static_cast<unsigned int>(this->inst));
	}
	bool invalidInterrupt = true;
	for (uint8_t i = 0; i < _NUM_DIGITAL_PINS; i++) {
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == static_cast<uint8_t>(NOT_AN_INTERRUPT)) continue;
		if (_DIGITAL_PIN_TO_INTERRUPT[i] == interrupt) {
			invalidInterrupt = true;
			break;
		}
	}
	if (invalidInterrupt == false && verbose > 1) {
		printToErr(MSGT(MSGT_WARN_API_INV_INTERRUPT), _T("SPIClass::notUsingInterrupt()"), static_cast<unsigned int>(interrupt));
	}
	callWith<_void>(OpCode::SPI_NOT_USING_INTERRUPT, _uint8_t(this->inst), _uint8_t(interrupt));
}


/**
 * Starts an SPI transaction with the given set of settings. Call endTransaction() at the end of the
 * transaction.
 * 
 * @param[in] settings - SPI interface settings
 */
void SPIClass::beginTransaction(SPISettings settings) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::beginTransaction()"), static_cast<unsigned int>(this->inst));
	}
	if (settings.clock == INVALID_SPI_CLOCK || (settings.clock != _SPI_CLOCK_DIV2 && settings.clock != _SPI_CLOCK_DIV4 && settings.clock != _SPI_CLOCK_DIV8 &&
		settings.clock != _SPI_CLOCK_DIV16 && settings.clock != _SPI_CLOCK_DIV32 && settings.clock != _SPI_CLOCK_DIV64 && settings.clock != _SPI_CLOCK_DIV128 &&
		settings.clock != _SPI_CLOCK_DIV256 && settings.clock != _SPI_CLOCK_DIV512 && settings.clock != _SPI_CLOCK_DIV1024)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_CLOCK_DIV), _T("SPIClass::setClockDivider()"), static_cast<unsigned int>(settings.clock));
	}
	if (settings.bitOrder != _LSBFIRST && settings.bitOrder != _MSBFIRST) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_BITORDER), _T("SPIClass::setBitOrder()"), static_cast<unsigned int>(settings.bitOrder));
	}
	if (settings.dataMode == INVALID_SPI_MODE || (settings.dataMode != _SPI_MODE0 && settings.dataMode != _SPI_MODE1 &&
		settings.dataMode != _SPI_MODE2 && settings.dataMode != _SPI_MODE3)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_MODE), _T("SPIClass::setDataMode()"), static_cast<unsigned int>(settings.dataMode));
	}
	callWith<_void>(OpCode::SPI_BEGIN_TRANSACTION, _uint8_t(this->inst), _uint32_t(settings.clock), _uint8_t(settings.bitOrder), _uint8_t(settings.dataMode));
}


/**
 * Transfers a data byte on the SPI bus.
 * 
 * @param[in] data - data byte to transfer
 * @return received data byte
 */
ADDE_F(_uint8_t) SPIClass::transfer(uint8_t data) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::transfer()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_uint8_t>(OpCode::SPI_TRANSFER, _uint8_t(this->inst), _uint8_t(data));
}


/**
 * Transfers a data word on the SPI bus.
 * 
 * @param[in] data - data word to transfer
 * @return received data word
 */
ADDE_F(_uint16_t) SPIClass::transfer16(uint16_t data) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::transfer16()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_uint16_t>(OpCode::SPI_TRANSFER16, _uint8_t(this->inst), _uint16_t(data));
}


/**
 * Transfers data on the SPI bus.
 * 
 * @param[in,out] buf - data buffer to transfer (received data overwrites written data)
 * @param[in] size - data buffer size
 */
void SPIClass::transfer(void * buf, size_t size) {
	checkInitializedMain();
	using namespace ::adde;
	if (buf == NULL || size == 0) return;
	uint8_t * buf8 = static_cast<uint8_t *>(buf);
	_RC<_FV<_uint8_t>> * tmp = new _RC<_FV<_uint8_t>>[size];
	if (tmp == NULL) {
		if (verbose > 0) _ftprintf(ferr, MSGT(MSGT_ERR_NO_MEM));
		signalReceived += 100;
		exit(EXIT_FAILURE);
		return;
	}
	/* send out data with deferred results */
	for (size_t i = 0; i < size; i++) {
		tmp[i] = this->transfer(buf8[i]);
	}
	/* wait for results and write data back to given buffer */
	for (size_t i = 0; i < size; i++) {
		buf8[i] = static_cast<uint8_t>(_F<_uint8_t>(tmp[i]));
	}
	delete[] tmp;
}


/**
 * Stops the SPI transaction which was started with beginTransaction().
 */
void SPIClass::endTransaction() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::endTransaction()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::SPI_END_TRANSACTION, _uint8_t(this->inst));
}


/**
 * Stops the SPI instance to free the pins for other operations.
 */
void SPIClass::end() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::end()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::SPI_END, _uint8_t(this->inst));
}


/**
 * Sets the SPI clock divider.
 * 
 * @param[in] val - new SPI clock divider
 * @deprecated Use SPISettings in beginTransaction() instead.
 */
void SPIClass::setClockDivider(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::setClockDivider()"), static_cast<unsigned int>(this->inst));
	}
	if (val == INVALID_SPI_CLOCK || (val != _SPI_CLOCK_DIV2 && val != _SPI_CLOCK_DIV4 && val != _SPI_CLOCK_DIV8 && val != _SPI_CLOCK_DIV16 &&
		val != _SPI_CLOCK_DIV32 && val != _SPI_CLOCK_DIV64 && val != _SPI_CLOCK_DIV128 && val != _SPI_CLOCK_DIV256 && val != _SPI_CLOCK_DIV512 &&
		val != _SPI_CLOCK_DIV1024)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_CLOCK_DIV), _T("SPIClass::setClockDivider()"), static_cast<unsigned int>(val));
	}
	callWith<_void>(OpCode::SPI_SET_CLOCK_DIVIDER, _uint8_t(this->inst), _uint8_t(val));
}


/**
 * Sets the SPI bit order.
 * 
 * @param[in] val - new SPI bit order
 * @deprecated Use SPISettings in beginTransaction() instead.
 */
void SPIClass::setBitOrder(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::setBitOrder()"), static_cast<unsigned int>(this->inst));
	}
	if (val != _LSBFIRST && val != _MSBFIRST) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_BITORDER), _T("SPIClass::setBitOrder()"), static_cast<unsigned int>(val));
	}
	callWith<_void>(OpCode::SPI_SET_BIT_ORDER, _uint8_t(this->inst), _uint8_t(val));
}


/**
 * Sets the SPI data mode.
 * 
 * @param[in] val - new SPI data mode
 * @deprecated Use SPISettings in beginTransaction() instead.
 */
void SPIClass::setDataMode(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= spiCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("SPIClass::setDataMode()"), static_cast<unsigned int>(this->inst));
	}
	if (val == INVALID_SPI_MODE || (val != _SPI_MODE0 && val != _SPI_MODE1 && val != _SPI_MODE2 && val != _SPI_MODE3)) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_MODE), _T("SPIClass::setDataMode()"), static_cast<unsigned int>(val));
	}
	callWith<_void>(OpCode::SPI_SET_DATA_MODE, _uint8_t(this->inst), _uint8_t(val));
}



/**
 * Constructor.
 * 
 * @param[in] instance - instance number
 */
TwoWire::TwoWire(const uint8_t instance):
	inst(instance)
{}


/**
 * Destructor.
 */
TwoWire::~TwoWire() {
	if (signalReceived >= 100) return;
	checkInitializedMain();
	if (this->inst < wireCount) this->end();
}


/**
 * Starts the I2C interface.
 */
void TwoWire::begin() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::begin()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::WIRE_BEGIN1, _uint8_t(this->inst));
}


/**
 * Starts the I2C interface for the given address.
 * 
 * @param[in] address - device address
 */
void TwoWire::begin(uint8_t address) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::begin(address)"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::WIRE_BEGIN2, _uint8_t(this->inst), _uint8_t(address));
}


/**
 * Stops the I2C instance to free the pins for other operations.
 */
void TwoWire::end() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::end()"), static_cast<unsigned int>(this->inst));
	}
	delay(1000);
	callWith<_void>(OpCode::WIRE_END, _uint8_t(this->inst));
}


/**
 * Sets the I2C interface speed in Hz.
 * 
 * @param[in] clock - clock speed in Hz
 */
void TwoWire::setClock(uint32_t clock) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::setClock()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::WIRE_SET_CLOCK, _uint8_t(this->inst), _uint32_t(clock));
}


/**
 * Starts I2C transmission with the given device address. Call endTransmission() at the end of the
 * transmission.
 * 
 * @param[in] address - device address
 */
void TwoWire::beginTransmission(uint8_t address) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::beginTransmission()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::WIRE_BEGIN_TRANSMISSION, _uint8_t(this->inst), _uint8_t(address));
}


/**
 * Stop the I2C transmission which was started with beginTransmission().
 * 
 * @param[in] sendStop - non-zero to send a stop bit, zero otherwise
 * @return 0 - success
 * @return 1 - length to long for buffer
 * @return 2 - address send, NACK received
 * @return 3 - data send, NACK received
 * @return 4 - other I2C error (lost bus arbitration, bus error, etc.)
 */
ADDE_F(_uint8_t) TwoWire::endTransmission(uint8_t sendStop) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::endTransmission()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_uint8_t>(OpCode::WIRE_END_TRANSMISSION, _uint8_t(this->inst), _uint8_t(sendStop));
}


/**
 * Requests data from the slave as a master device.
 * 
 * @param[in] address - slave device address
 * @param[in] size - number of bytes requested
 * @return number of bytes received
 */
ADDE_F(_uint8_t) TwoWire::requestFrom(uint8_t address, uint8_t size) {
	checkInitializedMain();
	return this->requestFrom(address, size, static_cast<uint8_t>(true));
}


/**
 * Requests data from the slave as a master device.
 * 
 * @param[in] address - slave device address
 * @param[in] size - number of bytes requested
 * @param[in] sendStop - non-zero to send a stop bit, zero otherwise
 * @return number of bytes received
 */
ADDE_F(_uint8_t) TwoWire::requestFrom(uint8_t address, uint8_t size, uint8_t sendStop) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::requestFrom()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_uint8_t>(OpCode::WIRE_REQUEST_FROM, _uint8_t(this->inst), _uint8_t(address), _uint8_t(size), _uint8_t(sendStop));
}


/**
 * Returns the number of bytes available for reading.
 * 
 * @return number of bytes available in input buffer
 */
int TwoWire::available() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::available()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_int>(OpCode::WIRE_AVAILABLE, _uint8_t(this->inst));
}


/**
 * Returns the next received byte without removing it from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int TwoWire::peek() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::peek()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_int>(OpCode::WIRE_PEEK, _uint8_t(this->inst));
}


/**
 * Returns the next received byte from the input buffer.
 * 
 * @return next byte or negative value on error
 */
int TwoWire::read() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::read()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_int>(OpCode::WIRE_READ, _uint8_t(this->inst));
}


/**
 * Flush the output buffer and block until all remaining bytes are transmitted.
 */
void TwoWire::flush() {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::flush()"), static_cast<unsigned int>(this->inst));
	}
	callWith<_void>(OpCode::WIRE_FLUSH, _uint8_t(this->inst));
}


/**
 * Transmits the given byte.
 * 
 * @param[in] val - byte value to send
 * @return number of bytes transmitted
 */
size_t TwoWire::write(uint8_t val) {
	checkInitializedMain();
	using namespace ::adde;
	if (this->inst >= wireCount) {
		if (verbose > 1) printToErr(MSGT(MSGT_WARN_API_INV_INSTANCE), _T("TwoWire::write()"), static_cast<unsigned int>(this->inst));
	}
	return callWith<_size_t>(OpCode::WIRE_WRITE, _uint8_t(this->inst), _uint8_t(val));
}


/**
 * Transmits the given bytes.
 * 
 * @param[in] buf - data buffer to send
 * @param[in] size - data buffer size
 * @return number of bytes transmitted
 */
size_t TwoWire::write(const uint8_t * buf, size_t size) {
	checkInitializedMain();
	size_t res = 0;
	for (size_t i = 0; i < size; i++) {
		res += this->write(buf[i]);
	}
	return res;
}
