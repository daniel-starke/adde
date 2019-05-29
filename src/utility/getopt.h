/**
 * @file getopt.h
 * @author Daniel Starke
 * @copyright Copyright 2017-2018 Daniel Starke
 * @see getopt.h
 * @date 2017-05-22
 * @version 2018-12-13
 */
#ifndef __LIBPCF_GETOPT_H__
#define __LIBPCF_GETOPT_H__
#define __GETOPT_H__ /* avoid collision with cygwins getopt.h include in sys/unistd.h */

#include "tchar.h"
#include "target.h"
#include "argp.h"
#include "argps.h"
#include "argpus.h"
#ifndef UNICODE /* ASCII */
#define option tArgPES
#else /* UNICODE */
#define option tArgPEUS
#endif /* UNICODE */


#ifdef __cplusplus
extern "C" {
#endif


extern int argps_optind;
extern int argps_opterr;
extern int argps_optopt;
extern char * argps_optarg;


extern int argps_getopt(int argc, char * const argv[], const char * optstring);
extern int argps_getoptLong(int argc, char * const argv[], const char * optstring, const tArgPES * longopts, int * longindex);
extern int argps_getoptLongOnly(int argc, char * const argv[], const char * optstring, const tArgPES * longopts, int * longindex);


extern int argpus_optind;
extern int argpus_opterr;
extern int argpus_optopt;
extern wchar_t * argpus_optarg;


extern int argpus_getopt(int argc, wchar_t * const argv[], const wchar_t * optstring);
extern int argpus_getoptLong(int argc, wchar_t * const argv[], const wchar_t * optstring, const tArgPEUS * longopts, int * longindex);
extern int argpus_getoptLongOnly(int argc, wchar_t * const argv[], const wchar_t * optstring, const tArgPEUS * longopts, int * longindex);


#ifndef UNICODE /* ASCII */
#define optind argps_optind
#define opterr argps_opterr
#define optopt argps_optopt
#define optarg argps_optarg
#define getopt argps_getopt
#define getopt_long argps_getoptLong
#define getopt_long_only argps_getoptLongOnly
#else /* UNICODE */
#define optind argpus_optind
#define opterr argpus_opterr
#define optopt argpus_optopt
#define optarg argpus_optarg
#define getopt argpus_getopt
#define getopt_long argpus_getoptLong
#define getopt_long_only argpus_getoptLongOnly
#endif /* UNICODE */


#ifdef __cplusplus
}
#endif


#endif /* __LIBPCF_GETOPT_H__ */
