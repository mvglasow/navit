#include "config.h"
#ifndef HAVE_API_WIN32_BASE
#define USE_POSIX_THREADS 1
#endif
#if USE_POSIX_THREADS
#include <pthread.h>
#endif
#include "debug.h"

#define g_return_if_fail


#define g_assert(expr) dbg_assert (expr)
