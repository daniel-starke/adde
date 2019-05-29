/**
 * @file New.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-04
 * @version 2019-03-09
 */
#include "New.hpp"


void * operator new(size_t, void * ptr) noexcept {
	return ptr;
}


void * operator new[](size_t, void * ptr) noexcept {
	return ptr;
}
