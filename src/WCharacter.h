/**
* @file WCharacter.h
* @author Daniel Starke
* @copyright Copyright 2019 Daniel Starke
* @date 2019-03-11
* @version 2019-04-22
*/
#ifndef __ADDE_WCHARACTER_H__
#define __ADDE_WCHARACTER_H__

#include <ctype.h>


static inline bool isAlphaNumeric(int c) {
	return isalnum(c) != 0;
}


static inline bool isAlpha(int c) {
	return isalpha(c) != 0;
}


static inline bool isAscii(int c) {
	return c <= 0x7F;
}


static inline bool isWhitespace(int c) {
	/* workaround for MinGW where isblank() is not available */
	return c == ' ' || c == '\t';
}


static inline bool isControl(int c) {
	return iscntrl(c) != 0;
}


static inline bool isDigit(int c) {
	return isdigit(c) != 0;
}


static inline bool isGraph(int c) {
	return isgraph(c) != 0;
}


static inline bool isLowerCase(int c) {
	return islower(c) != 0;
}


static inline bool isPrintable(int c) {
	return isprint(c) != 0;
}


static inline bool isPunct(int c) {
	return ispunct(c) != 0;
}


static inline bool isSpace(int c) {
	return isspace(c) != 0;
}


static inline bool isUpperCase(int c) {
	return isupper(c) != 0;
}


static inline bool isHexadecimalDigit(int c) {
	return isxdigit(c) != 0;
}


static inline int toAscii(int c) {
	return c & 0x7F;
}


static inline int toLowerCase(int c) {
	return tolower(c);
}


static inline int toUpperCase(int c) {
	return toupper(c);
}


#endif /* __ADDE_WCHARACTER_H__ */
