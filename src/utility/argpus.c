/**
 * @file argpus.c
 * @author Daniel Starke
 * @copyright Copyright 2017-2018 Daniel Starke
 * @see argpus.h
 * @date 2017-05-22
 * @version 2018-12-13
 */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <wchar.h>
#include "argpus.h"


#define ARGP_FUNC argpus_parse
#define ARGP_CTX tArgPUS
#define ARGP_LOPT tArgPEUS
#define ARGP_UNICODE


/* include template function */
#include "argp.i"
