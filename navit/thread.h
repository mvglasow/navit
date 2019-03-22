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
 * @brief Thread abstraction layer for Navit.
 *
 * This file provides a cross-platform thread API for Navit. It is not necessarily feature-complete—if Navit doesn’t
 * need a certain feature, it will not be included.
 *
 * It is permitted (and even desirable) to include this header file on platforms which have no support for threads:
 * this will set the `HAVE_NAVIT_THREADS` constant to 0, allowing for the use of preprocessor conditionals to write
 * code which will run in both single-threaded and multi-threaded environments.
 *
 * On platforms for which Navit does not have thread support, functions defined in this header file are no-ops if they
 * are not meaningful in a single-threaded environment. For example, attempting to acquire a lock is not necessary if
 * there are no other threads which might already be holding the lock. Mapping these functions to no-ops allows for
 * better code legibility, as it eliminates the need to wrap these calls into conditionals. Where they occur next to
 * sections wrapped in conditionals, they may be placed either inside or outside the conditional section. Consistency
 * is advised, though.
 *
 * On the other hand, operations such as attempting to spawn a new thread will cause a compiler error in a
 * single-threaded environment, as there is no appropriate “translation” for this operation (the developer would have
 * to choose an appropriate way of running the code on the main thread, possibly with further refactoring to avoid
 * locking up the program, thus attempting to spawn a thread is an error).
 *
 * Types defined in this header are intended to be used as pointers. Constructor functions allocate memory and return
 * pointers, and other functions expect pointers as well. On platforms without thread support, types defined here map
 * to `void` and cannot be instantiated directly.
 *
 * Deadlocks are currently not taken into account. Depending on the lower layer, threads involved in a deadlock may
 * either lock up forever, or log an error and use the locked resource anyway (resulting in inconsistencies later on).
 * This needs to be fixed in a later release and may involve API changes.
 *
 * Use of other thread APIs in Navit is generally discouraged. An exception to this are platform-specific modules:
 * these can use platform-specific APIs to spawn extra threads and synchronize with them, as long as they are entirely
 * contained within the module and no code outside the module ever needs to synchronize with them.
 */

#ifndef NAVIT_THREAD_H
#define NAVIT_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAVE_API_WIN32_BASE
#undef HAVE_NAVIT_THREADS
#warning "threads are not supported on this platform, building a single-threaded version"
#include <windows.h>
#else
#define HAVE_POSIX_THREADS 1
#define HAVE_NAVIT_THREADS 1
#include <pthread.h>
#endif


#if HAVE_POSIX_THREADS
#define thread pthread_t
#define thread_lock pthread_rwlock_t
#else
#define thread void
#define thread_lock void
#endif

/**
 * @brief Creates and spawns a new thread.
 *
 * The new thread will terminate if its `main` function returns or the process exits.
 *
 * This function must be surrounded with an `#if HAVE_NAVIT_THREADS` conditional, else use of this function will result
 * in a compiler error when building for a platform without thread support. An alternative way of running the thread
 * code is usually necessary for these platforms.
 *
 * @param main The main function for the new thread, see description
 * @param data Data passed to the main function
 * @param name A display name for the thread (may not be supported on all platforms)
 *
 * @return The new thread, or NULL if an error occurred.
 */
thread *thread_new(int (*main)(void *), void * data, char * name);

/**
 * @brief Frees all resources associated with the thread.
 *
 * If Navit was built without thread support, this is a no-op.
 */
void thread_destroy(thread* thread);

/**
 * @brief Pauses the current thread for a set amount of time.
 *
 * If Navit was built without thread support, this will make Navit sleep for the amount of time requested.
 *
 * @param msec The number of milliseconds to sleep for
 */
void thread_sleep(long msec);

/**
 * @brief Exits the current thread.
 *
 * The exit code can be obtained by calling `thread_join()` on the thread.
 *
 * If Navit was built without thread support, this is a no-op.
 *
 * @param result The exit code
 */
void thread_exit(int result);

/**
 * @brief Joins a thread, i.e. blocks until the thread has finished.
 *
 * If Navit was built without thread support, this function will return -1 immediately.
 *
 * @return The thread’s exit value, -1 if an error was encountered.
 */
int thread_join(thread * thread);

/**
 * @brief Creates a new lock.
 *
 * The caller is responsible for freeing up the lock with `thread_lock_destroy()` when it is no longer needed.
 *
 * If Navit was built without thread support, this is a no-op and NULL will be returned.
 */
thread_lock *thread_lock_new(void);

/**
 * @brief Frees all resources associated with the lock.
 *
 * If Navit was built without thread support, this is a no-op. If `lock` is NULL on a platform with thread support,
 * the behavior is undefined.
 */
void thread_lock_destroy(thread_lock *lock);

/**
 * @brief Acquires a read lock for the current thread.
 *
 * Read locks are recursive, i.e. one thread can acquire the same read lock multiple times. Each release will undo
 * exactly one lock operation, i.e. if a read lock was acquired n times, it must be released n times before another
 * thread can acquire a write lock.
 *
 * If Navit was built without thread support, this is a no-op.
 */
void thread_lock_acquire_read(thread_lock *lock);

/**
 * @brief Releases a read lock for the current thread.
 *
 * If Navit was built without thread support, this is a no-op.
 */
void thread_lock_release_read(thread_lock *lock);

/**
 * @brief Acquires a write lock for the current thread.
 *
 * Write locks, unlike read locks, are not recursive, i.e. even the same thread cannot acquire the same write lock more
 * than once.
 *
 * If Navit was built without thread support, this is a no-op.
 */
void thread_lock_acquire_write(thread_lock *lock);

/**
 * @brief Releases a write lock for the current thread.
 *
 * If Navit was built without thread support, this is a no-op.
 */
void thread_lock_release_write(thread_lock *lock);


#ifdef __cplusplus
}
#endif

#endif /* NAVIT_THREAD_H */
