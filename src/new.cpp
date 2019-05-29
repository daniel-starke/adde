/**
 * @file new.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-09
 * @version 2019-03-09
 */
#include "new.h"


void * operator new(size_t size) {
	return malloc(size);
}


void * operator new[](size_t size) {
	return malloc(size);
}


// void * operator new(size_t, void * ptr) noexcept {
// 	return ptr;
// }


// void * operator new[](size_t, void * ptr) noexcept {
// 	return ptr;
// }


void operator delete(void * ptr) {
	free(ptr);
}


void operator delete[](void * ptr) {
	free(ptr);
}
