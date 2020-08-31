How To Add A New Remote Function
================================

The following example demonstrates the needed steps and places to modify to a the function `noTone()` to the remote call API.  

**src/arduino/**
- [Protocol.hpp](../src/arduino/Protocol.hpp): add a new OP code to `OpCode::Type` (e.g. `NO_TONE`)
- [Protocol.hpp](../src/arduino/Protocol.hpp): adjust `ADDE_PROT_VERSION` accordingly to reflect the new API
- [arduino.hpp](../src/arduino/arduino.hpp): declare the OP code handler with `ErrorCode::Type handleNoTone(FrameHandlerArgs & args);`
- [arduino.ino](../src/arduino/arduino.ino): add the handler to the array `handler` in the same order as defined in `OpCode::Type`
- [arduino.ino](../src/arduino/arduino.ino): adjust the device name in `handleGetDevName` if needed
- [arduino.ino](../src/arduino/arduino.ino): define the handler and bind it the to remote function with `DEF_FUNCTION_HANDLER(handleNoTone, noTone)`  
  Note: The signature of the remote function is automatically parsed and used as declared. Ensure that the function declaration is visible here.
- [Target.hpp](../src/arduino/Target.hpp): define the enabler macro for the new OP code handler with `#define HAS_handleNoTone`

**src/**
- [Arduino.h](../src/Arduino.h): make the function visible by adding the original function declaration here or in another file visible to the main application
- [Arduino.cpp](../src/Arduino.cpp): include the header file for the new function declaration (or nothing if added to `Arduino.h`)
- [Arduino.cpp](../src/Arduino.cpp): add the function definition to offload the call on the remote device
```.cpp
    void noTone(uint8_t pin) {
    	checkInitializedMain(); /* needed to ensure the function is available before setup() is called */
    	using namespace ::adde; /* for _void */
    	checkDigitalPin(_T("noTone()"), pin); /* debug helper which checks if the given pin is a digital pin (optional feature) */
    	callWith<_void>(OpCode::NO_TONE, _uint8_t(pin)); /* actual call to the remote device */
    }
```

Remarks
=======

This minimal example depicts the general steps to perform for a new remote function call. It remains
true if the function in question is a class member function. The modifications, however, differ slightly
in such case regarding the OP code handler definition and remote function call handling. Other classes
were already ported and can be found as an example in the source code.  
The function declaration for the main application (e.g. in [src/Arduino.h](../src/Arduino.h)) can be made
using one of the adde return types (e.g. `ADDE_F(_uint8_t)`) to enable the `--fast` option for those.
