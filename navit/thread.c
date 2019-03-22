/**
 * Navit, a modular navigation system.
 * Copyright (C) 2005-2019 Navit Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this program; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

/** @file
 *
 * @brief Abstraction layer for system-specific thread routines.
 */

#include "thread.h"
#include <glib.h>
#ifndef HAVE_API_WIN32_BASE
#include <errno.h>
#include <time.h>
#endif
#include "debug.h"

#if HAVE_POSIX_THREADS
/**
 * @brief Describes the main function for a thread.
 */
struct thread_main_data {
    int (*main)(void *);       /**< The thread’s main function. */
    void * data;               /**< The argument for the function. */
};

/**
 * @brief A wrapper around the main thread function.
 *
 * This wraps the implementation-neutral main function into a function with the signature expected by the POSIX thread
 * library.
 *
 * @param data Pointer to a `struct thread_main_data` encapsulating the main function and its argument.
 */
static void *thread_main_wrapper(void * data) {
    struct thread_main_data * main_data = (struct thread_main_data *) data;
    void * ret = (void *) (main_data->main(main_data->data));
    return ret;
}
#endif

thread *thread_new(int (*main)(void *), void * data, char * name) {
#if HAVE_POSIX_THREADS
    int err;
    thread * ret = g_new0(thread, 1);
    struct thread_main_data * main_data = g_new0(struct thread_main_data, 1);
    main_data->main = main;
    main_data->data = data;
    err = pthread_create(ret, NULL, thread_main_wrapper, (void *) main_data);
    if (err) {
        dbg(lvl_error, "error %d, thread=%p", err, ret);
        g_free(ret);
        return NULL;
    }
#ifdef __USE_GNU
    if (name) {
        err = pthread_setname_np(*thread, name);
        if (err)
            dbg(lvl_warning, "error %d, thread=%p", err, ret);
    }
#endif
    return ret;
#else
#error "call to thread_new() on a platform without thread support"
#endif
}

void thread_destroy(thread* thread) {
#if HAVE_POSIX_THREADS
    g_free(thread);
#endif
}

void thread_sleep(long msec) {
#ifdef HAVE_API_WIN32_BASE
    Sleep(msec);
#else
    struct timespec req, rem;
    req.tv_sec = (msec /1000);
    req.tv_nsec = (msec % 1000) * 1000000;
    while ((nanosleep(&req, &rem) == -1) && errno == EINTR)
        req = rem;
#endif
}

void thread_exit(int result) {
#if HAVE_POSIX_THREADS
    pthread_exit((void *) result);
#else
    return;
#endif
}

int thread_join(thread * thread) {
#if HAVE_POSIX_THREADS
    void * ret;
    int err = pthread_join(*thread, &ret);
    if (err) {
        dbg(lvl_error, "error %d, thread=%p", err, thread);
        return -1;
    }
    return (int) ret;
#else
    return -1;
#endif
}

thread_lock *thread_lock_new(void) {
#if HAVE_POSIX_THREADS
    thread_lock *ret = g_new0(thread_lock, 1);
    int err = pthread_rwlock_init(ret, NULL);
    if (err) {
        dbg(lvl_error, "error %d, lock=%p", err, ret);
        g_free(ret);
        return NULL;
    }
    return ret;
#else
    return NULL;
#endif
}

void thread_lock_destroy(thread_lock *lock) {
#if HAVE_POSIX_THREADS
    int err = pthread_rwlock_destroy(lock);
    if (err)
        dbg(lvl_error, "error %d, lock=%p", err, lock);
    g_free(lock);
#endif
}

void thread_lock_acquire_read(thread_lock *lock) {
#if HAVE_POSIX_THREADS
    int err = pthread_rwlock_rdlock(lock);
    if (err)
        dbg(lvl_error, "error %d, lock=%p", err, lock);
#endif
}

void thread_lock_release_read(thread_lock *lock) {
#if HAVE_POSIX_THREADS
    int err = pthread_rwlock_unlock(lock);
    if (err)
        dbg(lvl_error, "error %d, lock=%p", err, lock);
#endif
}

void thread_lock_acquire_write(thread_lock *lock) {
#if HAVE_POSIX_THREADS
    int err = pthread_rwlock_wrlock(lock);
    if (err)
        dbg(lvl_error, "error %d, lock=%p", err, lock);
#endif
}

void thread_lock_release_write(thread_lock *lock) {
#if HAVE_POSIX_THREADS
    int err = pthread_rwlock_unlock(lock);
    if (err)
        dbg(lvl_error, "error %d, lock=%p", err, lock);
#endif
}
