/**
 * @file Printable.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-09
* @version 2019-04-22
 */
#ifndef __ADDE_PRINTABLE_H__
#define __ADDE_PRINTABLE_H__

#include <stddef.h>


/* forward declaration */
class Print;


class Printable {
public:
	virtual size_t printTo(Print &) const = 0;
};


#endif /* __ADDE_PRINTABLE_H__ */
