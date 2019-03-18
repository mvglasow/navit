#include "config.h"
#ifndef HAVE_API_WIN32_BASE
#define USE_POSIX_THREADS 1
#endif
#if USE_POSIX_THREADS
#include <pthread.h>
#endif
#include "debug.h"

#define g_return_if_fail


char* g_convert               (const char  *str,
				int        len,            
				const char  *to_codeset,
				const char  *from_codeset,
				int        *bytes_read,     
				int        *bytes_written,  
				void      **error);

#define g_assert(expr) dbg_assert (expr)
