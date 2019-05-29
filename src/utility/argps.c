/**
 * @file argps.c
 * @author Daniel Starke
 * @copyright Copyright 2017-2018 Daniel Starke
 * @see argps.h
 * @date 2017-05-20
 * @version 2018-12-13
 */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "argps.h"


#define ARGP_FUNC argps_parse
#define ARGP_CTX tArgPS
#define ARGP_LOPT tArgPES
#undef ARGP_UNICODE


/* include template function */
#include "argp.i"
