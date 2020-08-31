/**
 * @file new.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-09
 * @version 2019-04-22
 */
#ifndef __ADDE_NEW_H__
#define __ADDE_NEW_H__

#include <stdlib.h>


void * operator new(size_t size);
void * operator new[](size_t size);
// void * operator new(size_t, void * ptr) noexcept;
// void * operator new[](size_t, void * ptr) noexcept;
void operator delete(void * ptr);
void operator delete[](void * ptr);


#endif /* __ADDE_NEW_H__ */
