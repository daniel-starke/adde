/**
 * @file tchar.c
 * @author Daniel Starke
 * @copyright Copyright 2017-2018 Daniel Starke
 * @see tchar.h
 * @date 2017-05-24
 * @version 2018-12-13
 */
#include "tchar.h"


/**
 * Returns the pointer to the last occurrence of a character from the given set.
 * NULL is returned if no set character was found.
 * 
 * @param[in] str1 - search within this string
 * @param[in] str1 - search for a character in this set
 * @return NULL if not found or the pointer to the last occurrence
 */
const TCHAR * _tcsrpbrk(const TCHAR * str1, const TCHAR * str2) {
	const TCHAR * last = NULL;
	const TCHAR * cur = str1;
	while ((cur = _tcspbrk(cur, str2)) != NULL) {
		last = cur;
		cur++;
	}
	return last;
}
