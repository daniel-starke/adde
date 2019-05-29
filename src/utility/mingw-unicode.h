/**
 * @file mingw-unicode.h
 * @author Daniel Starke
 * @copyright Copyright 2013-2019 Daniel Starke
 * @date 2013-02-16
 * @version 2019-04-30
 */
#ifndef __MINGW_UNICODE_H__
#define __MINGW_UNICODE_H__


#ifdef __cplusplus
extern "C" {
#endif


#undef _tmain
#ifdef _UNICODE
#define _tmain wmain
#include <stdio.h>
#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>
#else
#define _tmain main
#endif

#if ((defined(__GNUC__) || defined(__TINYC__)) && defined(_UNICODE))

#ifndef __MSVCRT__
#error Unicode main function requires linking to MSVCRT
#endif

#include <wchar.h>
#include <stdlib.h>

#if !defined(__MINGW64__)
# define stat _stat
#endif

#ifdef __TINYC__
int _CRT_glob = 0;
#else
extern int _CRT_glob;
#endif
extern void __wgetmainargs(int *, wchar_t ***, wchar_t ***, int, int *);

#ifdef MAIN_USE_ENVP
int wmain(int argc, wchar_t * argv[], wchar_t * envp[]);
#else
int wmain(int argc, wchar_t * argv[]);
#endif

int main() {
	wchar_t ** enpv, ** argv;
	int argc, si = 0;
	/* this also creates the global variable __wargv */
	__wgetmainargs(&argc, &argv, &enpv, _CRT_glob, &si);
#ifdef MAIN_USE_ENVP
	return wmain(argc, argv, enpv);
#else
	return wmain(argc, argv);
#endif
}


#endif /* defined(__GNUC__) && defined(_UNICODE) */


#ifdef __cplusplus
}
#endif


#endif /* __MINGW_UNICODE_H__ */
