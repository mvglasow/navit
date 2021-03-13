/**
 * Navit, a modular navigation system.
 * Copyright (C) 2005-2021 Navit Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

/**
 * @file traffic_traff_http.c
 *
 * @brief The TraFF plugin for HTTP
 *
 * This plugin receives TraFF feeds from a TraFF HTTP server, either on the local device or on a
 * remote system.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _POSIX_C_SOURCE
#include <sys/types.h>
#endif

#include <curl/curl.h>
#include "thread.h"
#include "glib_slice.h"
#include "config.h"
#include "item.h"
#include "attr.h"
#include "coord.h"
#include "map.h"
#include "route_protected.h"
#include "route.h"
#include "transform.h"
#include "event.h"
#include "xmlconfig.h"
#include "traffic.h"
#include "plugin.h"
#include "callback.h"
#include "vehicle.h"
#include "debug.h"
#include "navit.h"
#include "util.h"

/**
 * @brief Default poll interval, in msec.
 *
 * Unless `attr_interval` is set, this interval will be used. 600000 msec = 10 minutes.
 */
#define DEFAULT_INTERVAL 600000

/**
 * @brief Minimum area around the current position for which to retrieve traffic updates.
 *
 * 100000 is equivalent to around 50 km on each side of the current position. The actual subscription area
 * can be larger, allowing for a subscription area to be kept over multiple position updates.
 *
 * The actual subscription area around the current location is stored in
 * {@link struct traffic_priv::position_rect} and updated in
 * {@link traffic_traff_http_position_callback(struct traffic_priv *, struct navit *, struct vehicle *)}.
 */
#define POSITION_RECT_SIZE 100000

/** Name for the worker thread */
#define TRAFF_HTTP_WORKER_THREAD_NAME "traff_http"

/**
 * @brief Stores information about the plugin instance.
 */
struct traffic_priv {
    struct navit * nav;         /**< The navit instance */
    struct traffic * traffic;   /**< The traffic instance */
    int position_valid;         /**< Whether Navit currently has a valid position */
    struct coord_rect * position_rect; /**< Rectangle around last known vehicle position (in `projection_mg`) */
    struct map_selection * route_map_sel; /**< Map selection for the current route */
    thread * worker_thread;     /**< Worker thread for network communication */
    int interval;               /**< Poll interval for the source, in msec */
    char * source;              /**< URL of the TraFF service */
    GList * queue;              /**< Queue of requests to be processed by the worker thread */
    thread_lock * queue_lock;   /**< Lock for the request queue */
    thread_event * queue_event; /**< Event that is signaled when a request is posted to the queue */
    char * subscription_id;     /**< Subscription ID */
    int exiting;                /**< Whether the plugin is shutting down */
};

/**
 * @brief Stores HTTP response data.
 */
struct curl_result {
    char *data;                 /**< Result data */
    size_t size;                /**< Size of data stored in `memory` */
};

void traffic_traff_http_destroy(struct traffic_priv * this_);
struct traffic_message ** traffic_traff_http_get_messages(struct traffic_priv * this_);

/**
 * @brief Destructor.
 */
void traffic_traff_http_destroy(struct traffic_priv * this_) {
    /* tell the worker thread to clean up and exit */
    this_->exiting = 1;
    thread_event_signal(this_->queue_event);
    if (this_->position_rect)
        g_free(this_->position_rect);
    this_->position_rect = NULL;
    if (this_->route_map_sel)
        route_free_selection(this_->route_map_sel);
    this_->route_map_sel = NULL;
    dbg(lvl_debug, "waiting for worker thread to clean up and terminate…");
    thread_join(this_->worker_thread);
    dbg(lvl_debug, "worker thread terminated");
}

/**
 * @brief Returns an empty traffic report.
 *
 * @return Always `NULL`
 */
struct traffic_message ** traffic_traff_http_get_messages(struct traffic_priv * this_) {
    return NULL;
}

/**
 * @brief The methods implemented by this plugin
 */
static struct traffic_methods traffic_traff_http_meth = {
    traffic_traff_http_get_messages,
    traffic_traff_http_destroy,
};


/**
 * @brief Callback function to process HTTP response data.
 *
 * `userp` must point to a `struct curl_result`, which will receive the result data. Data will be
 * null-terminated when this function returns. Data may be received in chunks, with this function called
 * multiple times.
 *
 * @param contents Points to a chunk of result data
 * @param size Number of characters in `contents`
 * @param nmemb Number of bytes per character
 * @param userp Points to the buffer structure as passed in `CURLOPT_WRITEDATA`
 *
 * @return Number of bytes stored, or 0 if an error occurred
 */
static size_t curl_result_callback(void *contents, size_t size, size_t nmemb, void *userp) {
    size_t realsize = size * nmemb;
    struct curl_result *mem = (struct curl_result *)userp;

    char *ptr = g_realloc(mem->data, mem->size + realsize + 1);
    if (ptr == NULL) {
        /* out of memory! */
        dbg(lvl_error, "not enough memory (realloc returned NULL)");
        return 0;
    }

    mem->data = ptr;
    memcpy(&(mem->data[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->data[mem->size] = 0;

    return realsize;
}


/**
 * @brief Sends an HTTP request and returns the result.
 *
 * @param url The URL to request
 * @param postdata Data to be sent as part of the HTTP POST operation
 *
 * @result Pointer to a `struct curl_result` representing the data returned. The caller is responsible
 * for freeing up both the struct and the memory pointed to by its `data` member.
 */
static struct curl_result * curl_post(char * url, char * data) {
    struct curl_result * ret;
    CURL *curl_handle;
    CURLcode curl_res;

    curl_handle = curl_easy_init();
    if (curl_handle) {
        ret = g_new0(struct curl_result, 1);
        ret->data = g_malloc0(1);
        ret->size = 0;

        curl_easy_setopt(curl_handle, CURLOPT_URL, url);

        curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, data);

        /* provide a callback and buffer for result data */
        curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, curl_result_callback);
        curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)ret);

        // FIXME provide a meaningful user agent
        curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");

        /* follow redirects */
        curl_easy_setopt(curl_handle, CURLOPT_FOLLOWLOCATION, 1L);

        curl_res = curl_easy_perform(curl_handle);
        curl_easy_cleanup(curl_handle);
        if (curl_res == CURLE_OK) {
            return ret;
        } else {
            dbg(lvl_error, "curl status: %s", curl_easy_strerror(curl_res));
            g_free(ret->data);
            g_free(ret);
        }
    } else {
        dbg(lvl_error, "curl initialization failed");
    }
    return NULL;
}


/**
 * @brief Called when a new TraFF feed is received.
 *
 * The worker thread posts this function to run on the main thread by registering a timeout event `*ev`
 * with `*cb` as its callback. Both are good for one call only and need to be cleaned up when this
 * function runs.
 *
 * @param this_ Private data for the module instance
 * @param messages Parsed messages
 * @param cb Pointer to the callback used to call this function
 */
static void traffic_traff_http_on_feed_received(struct traffic * traffic,
                                                struct traffic_message ** messages,
                                                struct callback ** cb) {
    dbg(lvl_debug, "enter");
    callback_destroy(*cb);
    g_free(cb);

    traffic_process_messages(traffic, messages);
    g_free(messages);
}

/**
 * @brief Processes a TraFF response.
 *
 * This runs on the worker thread. If messages are received, they are posted to the main thread for
 * processing.
 *
 * @param this_ The plugin instance
 * @param response The parsed response
 *
 * @return True if messages were received, false if not
 */
static int traffic_traff_http_process_response(struct traffic_priv * this_,
                                           struct traffic_response * response) {
    int ret = 0;
    struct callback ** cb;
    if (!strcmp(response->status, "OK") || !strcmp(response->status, "PARTIALLY_COVERED")) {
        if (response->subscription_id) {
            this_->subscription_id = response->subscription_id;
        }
        // TODO subscription timeout
        if (response->messages && *(response->messages)) {
            dbg(lvl_debug, "response contains messages, posting traffic feed");
            cb = g_new0(struct callback *, 1);
            *cb = callback_new_3(callback_cast(traffic_traff_http_on_feed_received), this_->traffic,
                                 response->messages, cb);
            event_add_timeout(1, 0, *cb);
        }
        ret = !!(response->messages);
        g_free(response->status);
        g_free(response);
    } else {
        dbg(lvl_error, "TraFF request failed with status %s", response->status);
    }
    return ret;
}


/**
 * @brief Main function for the worker thread.
 *
 * The worker thread handles all network I/O and, if a feed has been received, notifies the main thread
 * by adding a callback to its message loop.
 *
 * @param this_gpointer Pointer to the `struct traffic_priv` for the plugin instance
 */
static gpointer traffic_traff_http_worker_thread_main(gpointer this_gpointer) {
    struct traffic_priv * this_ = (struct traffic_priv *) this_gpointer;

    /* Whether the current run of the loop should poll the source */
    int poll;

    /* Partial request data (from queue), if any */
    char * rdata;

    /* Data for the request, if any */
    char * request;

    /* Result for the request */
    struct curl_result * chunk;

    /* Decoded response */
    struct traffic_response * response;

    while (1) {
        /* by default, poll the source every time the loop runs, unless we’re exiting */
        poll = !this_->exiting;

        /* if we’re exiting, clean up and exit */
        if (this_->exiting) {

            /* no need for the lock as the main thread is no longer placing requests at this point */
            while (this_->queue) {
                request = this_->queue->data;
                dbg(lvl_error, "discarding request: \n%s", request);
                this_->queue = g_list_remove(this_->queue, request);
                g_free(request);
            }

            thread_event_destroy(this_->queue_event);
            this_->queue_event = NULL;
            thread_lock_destroy(this_->queue_lock);
            this_->queue_lock = NULL;

            /* unsubscribe if we are subscribed */
            if (this_->subscription_id) {
                request = g_strdup_printf("<request operation='UNSUBSCRIBE' subscription_id='%s'/>", this_->subscription_id);
                chunk = curl_post(this_->source, request);
                if (chunk) {
                    g_free(chunk->data);
                    g_free(chunk);
                }
            }

            break;
        }

        /* check if we have any pending requests */
        thread_lock_acquire_write(this_->queue_lock);
        while (this_->queue) {
            /* get data, remove entry and release the lock for the duration of the network request */
            rdata = this_->queue->data;
            this_->queue = g_list_remove(this_->queue, rdata);
            thread_lock_release_write(this_->queue_lock);

            /* send the request and process its results */
            if (this_->subscription_id)
                request = g_strdup_printf("<request operation='CHANGE' subscription_id='%s'>\n%s\n</request>",
                        this_->subscription_id, rdata);
            else
                request = g_strdup_printf("<request operation='SUBSCRIBE'>\n%s\n</request>", rdata);
            dbg(lvl_error, "sending request: \n%s", request);
            chunk = curl_post(this_->source, request);
            if (chunk) {
                response = traffic_get_response_from_xml_string(this_->traffic, chunk->data);
                g_free(chunk->data);
                g_free(chunk);
                // TODO repeat if subscription unknown
                poll &= !traffic_traff_http_process_response(this_, response);
            }
            g_free(request);
            g_free(rdata);

            /* reacquire the lock so the loop condition is protected */
            thread_lock_acquire_write(this_->queue_lock);
        }
        /* the queue is empty, ensure the event is unset before releasing the lock */
        thread_event_reset(this_->queue_event);
        thread_lock_release_write(this_->queue_lock);

        if (this_->subscription_id && poll) {
            /* poll */
            request = g_strdup_printf("<request operation='POLL' subscription_id='%s'/>", this_->subscription_id);
            chunk = curl_post(this_->source, request);
            if (chunk) {
                response = traffic_get_response_from_xml_string(this_->traffic, chunk->data);
                g_free(chunk->data);
                g_free(chunk);
                // TODO handle unknown subscription
                traffic_traff_http_process_response(this_, response);
            }
        }

        /* finally, sleep until the next poll is due or we receive a new request */
        thread_event_wait(this_->queue_event, this_->interval);
    }
}


/**
 * @brief Sets the route map selection
 *
 * @param this_ The instance which will handle the selection update
 */
static void traffic_traff_http_set_selection(struct traffic_priv * this_) {
    struct route * route;
    struct coord_geo lu, rl;
    gchar *filter_list;
    gchar *min_road_class;

    if (this_->route_map_sel)
        route_free_selection(this_->route_map_sel);
    this_->route_map_sel = NULL;
    if (navit_get_destination_count(this_->nav) && (route = (navit_get_route(this_->nav))))
        this_->route_map_sel = route_get_selection(route);

    /* start building the filter list */
    filter_list = g_strconcat_printf(NULL, "<filter_list>\n");
    if (this_->position_rect) {
        transform_to_geo(projection_mg, &this_->position_rect->lu, &lu);
        transform_to_geo(projection_mg, &this_->position_rect->rl, &rl);
        filter_list = g_strconcat_printf(filter_list, "    <filter bbox=\"%.5f %.5f %.5f %.5f\"/>\n",
                                         rl.lat, lu.lng, lu.lat, rl.lng);
    }
    for (struct map_selection * sel = this_->route_map_sel; sel; sel = sel->next) {
        transform_to_geo(projection_mg, &sel->u.c_rect.lu, &lu);
        transform_to_geo(projection_mg, &sel->u.c_rect.rl, &rl);
        min_road_class = order_to_min_road_class(sel->order);
        if (!min_road_class)
            filter_list = g_strconcat_printf(filter_list, "    <filter bbox=\"%.5f %.5f %.5f %.5f\"/>\n",
                                             rl.lat, lu.lng, lu.lat, rl.lng);
        else
            filter_list = g_strconcat_printf(filter_list, "    <filter min_road_class=\"%s\" bbox=\"%.5f %.5f %.5f %.5f\"/>\n",
                                             min_road_class, rl.lat, lu.lng, lu.lat, rl.lng);
    }
    filter_list = g_strconcat_printf(filter_list, "</filter_list>");
    thread_lock_acquire_write(this_->queue_lock);
    this_->queue = g_list_append(this_->queue, filter_list);
    thread_event_signal(this_->queue_event);
    thread_lock_release_write(this_->queue_lock);
}


/**
 * @brief Callback for the traffic attribute
 *
 * This is needed because the traffic instance is not available until our constructor and init methods
 * have returned. To finish initialization, i.e. obtain a reference to the traffic instance and launch
 * the worker thread (which needs that instance), we register a callback when the attribute changes.
 * This happens only once at startup.
 *
 * @param this_ The instance which will handle the update
 */
static void traffic_traff_http_traffic_callback(struct traffic_priv * this_) {
    struct attr * attr;
    struct attr_iter * a_iter;

    attr = g_new0(struct attr, 1);
    a_iter = navit_attr_iter_new(NULL);
    if (navit_get_attr(this_->nav, attr_traffic, attr, a_iter))
        this_->traffic = (struct traffic *) attr->u.navit_object;
    navit_attr_iter_destroy(a_iter);
    g_free(attr);

    if (this_->traffic && !this_->worker_thread) {
        dbg(lvl_error, "traffic module fully initialized, starting worker thread"); // FIXME lvl_debug
        this_->worker_thread = thread_new(traffic_traff_http_worker_thread_main, this_,
                                          TRAFF_HTTP_WORKER_THREAD_NAME);
    }
}


/**
 * @brief Callback for destination changes
 *
 * @param this_ The instance which will handle the destination update
 */
static void traffic_traff_http_destination_callback(struct traffic_priv * this_) {
    traffic_traff_http_set_selection(this_);
}


/**
 * @brief Callback for navigation status changes
 *
 * This callback is necessary to force an update of existing subscriptions when Navit acquires a new
 * position (after not having had valid position information), as the map selection will change when
 * the current position becomes known for the first time.
 *
 * @param this_ The instance which will handle the navigation status update
 * @param status The status of the navigation engine (the value of the {@code nav_status} attribute)
 */
static void traffic_traff_http_status_callback(struct traffic_priv * this_, int status) {
    int new_position_valid = (status != 1);
    if (new_position_valid && !this_->position_valid) {
        this_->position_valid = new_position_valid;
        traffic_traff_http_set_selection(this_);
    } else if (new_position_valid != this_->position_valid)
        this_->position_valid = new_position_valid;
}


/**
 * @brief Callback for position changes
 *
 * This updates {@link struct traffic_priv::position_rect} if the vehicle has moved far enough from its
 * center to be within {@link POSITION_RECT_SIZE} of one of its boundaries. The new rectangle is created
 * with twice that amount of padding, allowing the vehicle to move for at least that distance before the
 * subscription needs to be updated again.
 *
 * @param this_ The instance which will handle the position update
 * @param navit The Navit instance
 * @param vehicle The vehicle which delivered the position update and from which the position can be queried
 */
static void traffic_traff_http_position_callback(struct traffic_priv * this_, struct navit *navit,
        struct vehicle *vehicle) {
    struct attr attr;
    struct coord c;
    struct coord_rect cr;
    if (!vehicle_get_attr(vehicle, attr_position_coord_geo, &attr, NULL))
        return;
    transform_from_geo(projection_mg, attr.u.coord_geo, &c);
    cr.lu = c;
    cr.rl = c;
    cr.lu.x -= POSITION_RECT_SIZE;
    cr.rl.x += POSITION_RECT_SIZE;
    cr.lu.y += POSITION_RECT_SIZE;
    cr.rl.y -= POSITION_RECT_SIZE;
    if (!this_->position_rect)
        this_->position_rect = g_new0(struct coord_rect, 1);
    if (!coord_rect_contains(this_->position_rect, &cr.lu) || !coord_rect_contains(this_->position_rect, &cr.rl)) {
        cr.lu.x -= POSITION_RECT_SIZE;
        cr.rl.x += POSITION_RECT_SIZE;
        cr.lu.y += POSITION_RECT_SIZE;
        cr.rl.y -= POSITION_RECT_SIZE;
        *(this_->position_rect) = cr;
        traffic_traff_http_set_selection(this_);
    }
}


/**
 * @brief Initializes a traff_http plugin
 *
 * @return True on success, false on failure
 */
static int traffic_traff_http_init(struct traffic_priv * this_) {
    struct route * route;
    struct navigation * navigation;

    /* TODO verify event system, accept if thread-safe, warn if functions are missing, else exit
     *
     * Thread-safe and OK to use: glib, android
     * Functions missing, won’t work: null, opengl
     * Probably not thread-safe: win32, qt (for qt_qpainter), qt5
     * Not sure: cocoa, sdl
     */
    if (!strcmp("null", event_system()) || !strcmp("opengl", event_system())) {
        /* null and opengl do not implement functions we require */
        dbg(lvl_error, "event system %s is incomplete, preventing the traff_http plugin from working",
            event_system());
        return 0;
    } else if (strcmp("glib", event_system()) && strcmp("android", event_system())) {
        /*
         * glib and android are known to be thread-safe
         * win32, qt (from qt_painter) and qt5 are not thread-safe and cannot be used
         * cocoa and sdl need in-depth verification
         */
        dbg(lvl_error, "event system %s is not thread-safe and cannot be used with the traff_http plugin",
            event_system());
        return 0;
    }
    /* TODO anything else to do here? */

    /* register callback for traffic module so we can finish setting up */
    navit_add_callback(this_->nav, callback_new_attr_1(callback_cast(traffic_traff_http_traffic_callback),
            attr_traffic, this_));

    /* register callbacks for position and destination changes */
    navit_add_callback(this_->nav, callback_new_attr_1(callback_cast(traffic_traff_http_position_callback),
                       attr_position_coord_geo, this_));
    navit_add_callback(this_->nav, callback_new_attr_1(callback_cast(traffic_traff_http_destination_callback),
                       attr_destination, this_));
    if ((navigation = navit_get_navigation(this_->nav)))
        navigation_register_callback(navigation, attr_nav_status,
                                     callback_new_attr_1(callback_cast(traffic_traff_http_status_callback), attr_nav_status, this_));

    return 1;
}


/**
 * @brief Registers a new traff_http traffic plugin
 *
 * @param nav The navit instance
 * @param meth Receives the traffic methods
 * @param attrs The attributes for the map
 * @param cbl
 *
 * @return A pointer to a `traffic_priv` structure for the plugin instance
 */
static struct traffic_priv * traffic_traff_http_new(struct navit *nav, struct traffic_methods *meth,
        struct attr **attrs, struct callback_list *cbl) {
    struct traffic_priv *ret;
    struct attr * attr;

    dbg(lvl_debug, "enter");

    ret = g_new0(struct traffic_priv, 1);
    ret->nav = nav;
    ret->traffic = NULL;
    ret->position_valid = 0;
    ret->position_rect = NULL;
    ret->route_map_sel = NULL;
    /* worker_thread will be set when we initialize */
    attr = attr_search(attrs, attr_interval);
    if (attr)
        ret->interval = attr->u.num;
    else
        ret->interval = DEFAULT_INTERVAL;
    attr = attr_search(attrs, attr_source);
    if (attr) {
        if (strncmp(attr->u.str, "http://", 7) && strncmp(attr->u.str, "https://", 8)) {
            dbg(lvl_error, "source must be an HTTP(S) URI: %s", attr->u.str);
        } else
            ret->source = attr->u.str;
    }
    ret->queue = NULL;
    ret->queue_lock = thread_lock_new();
    ret->queue_event = thread_event_new();
    ret->subscription_id = NULL;
    ret->exiting = 0;
    *meth = traffic_traff_http_meth;

    traffic_traff_http_init(ret);

    return ret;
}

/**
 * @brief Initializes the traffic plugin.
 *
 * This function is called once on startup.
 */
void plugin_init(void) {
    dbg(lvl_debug, "enter");
    curl_global_init(CURL_GLOBAL_ALL);

    plugin_register_category_traffic("traff_http", traffic_traff_http_new);
}
