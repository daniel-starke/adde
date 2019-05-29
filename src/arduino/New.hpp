/**
 * @file New.hpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-04
 * @version 2019-03-11
 */
#ifndef __NEW_HPP__
#define __NEW_HPP__

#include <stddef.h>


void * operator new(size_t, void * ptr) noexcept __attribute__((weak));
void * operator new[](size_t, void * ptr) noexcept __attribute__((weak));


#endif /* __NEW_HPP__ */
