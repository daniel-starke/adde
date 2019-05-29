/**
 * @file abi.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-09
 * @version 2019-03-10
 */
#ifndef _MSC_VER
#include <stdlib.h>


extern "C" void __cxa_pure_virtual(void) __attribute__((__noreturn__));
extern "C" void __cxa_deleted_virtual(void) __attribute__((__noreturn__));


void __cxa_pure_virtual(void) {
	abort();
}


void __cxa_deleted_virtual(void) {
	abort();
}
#endif /* !_MSC_VER */
