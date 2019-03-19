#include "fake.h"

#include <stdlib.h>             /* posix_memalign() */
#include <string.h>
#include <errno.h>
#include "gmem.h"               /* gslice.h */
#include "gthreadprivate.h"
#include "glib.h"
#include "galias.h"
#ifdef HAVE_UNISTD_H
#include <unistd.h>             /* sysconf() */
#endif
#ifdef G_OS_WIN32
#include <windows.h>
#include <process.h>
#endif

#include <stdio.h>              /* fputs/fprintf */


/**
 * g_get_current_time:
 * @result: #GTimeVal structure in which to store current time.
 *
 * Equivalent to the UNIX gettimeofday() function, but portable.
 **/
void
g_get_current_time (GTimeVal *result)
{
#ifndef G_OS_WIN32
  struct timeval r;

  g_return_if_fail (result != NULL);

  /*this is required on alpha, there the timeval structs are int's
    not longs and a cast only would fail horribly*/
  gettimeofday (&r, NULL);
  result->tv_sec = r.tv_sec;
  result->tv_usec = r.tv_usec;
#else
  FILETIME ft;
  guint64 time64;

  g_return_if_fail (result != NULL);

#if defined(HAVE_API_WIN32_CE)
  GetCurrentFT(&ft);
#else
  GetSystemTimeAsFileTime (&ft);
#endif
  memmove (&time64, &ft, sizeof (FILETIME));

  /* Convert from 100s of nanoseconds since 1601-01-01
   * to Unix epoch. Yes, this is Y2038 unsafe.
   */
  time64 -= G_GINT64_CONSTANT (116444736000000000);
  time64 /= 10;

  result->tv_sec = time64 / 1000000;
  result->tv_usec = time64 % 1000000;
#endif
}

// FIXME: should use real utf8-aware function
gchar * g_utf8_casefold(const gchar *s, gssize len) 
{
  return g_ascii_strdown(s,len);
}
