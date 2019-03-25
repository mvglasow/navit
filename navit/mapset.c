/**
 * Navit, a modular navigation system.
 * Copyright (C) 2005-2008 Navit Team
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

/** @file
 *
 * @brief Contains code used for loading more than one map
 *
 * The code in this file introduces "mapsets", which are collections of several maps.
 * This enables navit to operate on more than one map at once. See map.c / map.h to learn
 * how maps are handled.
 */

#include <string.h>
#include <glib.h>
#include <glib/gprintf.h>
#include "thread.h"
#include "debug.h"
#include "item.h"
#include "mapset.h"
#include "projection.h"
#include "map.h"
#include "xmlconfig.h"

struct mapset {
    NAVIT_OBJECT
    GList *maps; /**< Linked list of all the maps in the mapset */
    thread_lock *rw_lock; /**< Lock for insertions, deletions or iterations over maps in the mapset */
};

struct attr_iter {
    GList *last;
};

/**
 * @brief Creates a new, empty mapset
 *
 * @return The new mapset
 */
struct mapset *mapset_new(struct attr *parent, struct attr **attrs) {
    struct mapset *ms;

    ms=g_new0(struct mapset, 1);
    ms->func=&mapset_func;
    navit_object_ref((struct navit_object *)ms);
    ms->attrs=attr_list_dup(attrs);
    ms->rw_lock = thread_lock_new();

    return ms;
}

struct mapset *mapset_dup(struct mapset *ms) {
    struct mapset *ret=mapset_new(NULL, ms->attrs);
    thread_lock_acquire_read(ms->rw_lock);
    ret->maps=g_list_copy(ms->maps);
    thread_lock_release_read(ms->rw_lock);
    ret->rw_lock = thread_lock_new();
    return ret;
}


struct attr_iter *
mapset_attr_iter_new(void) {
    return g_new0(struct attr_iter, 1);
}

void mapset_attr_iter_destroy(struct attr_iter *iter) {
    g_free(iter);
}

/**
 * @brief Adds a map to a mapset.
 *
 * Attribute types other than `attr_map` are not supported.
 *
 * If Navit was built with GLib thread support, this call will block as long as any thread still has a mapset handle
 * open.
 *
 * @param ms The mapset to add the map to
 * @param attr The attribute for the map to be added
 */
int mapset_add_attr(struct mapset *ms, struct attr *attr) {
    switch (attr->type) {
    case attr_map:
        thread_lock_acquire_write(ms->rw_lock);
        ms->attrs=attr_generic_add_attr(ms->attrs,attr);
        ms->maps=g_list_append(ms->maps, attr->u.map);
        thread_lock_release_write(ms->rw_lock);
        return 1;
    default:
        return 0;
    }
}

/**
 * @brief Removes a map from a mapset.
 *
 * Attribute types other than `attr_map` are not supported.
 *
 * If Navit was built with GLib thread support, this call will block as long as any thread still has a mapset handle
 * open.
 *
 * @param ms The mapset to remove the map from
 * @param attr The attribute for the map to be removed
 */
int mapset_remove_attr(struct mapset *ms, struct attr *attr) {
    switch (attr->type) {
    case attr_map:
        thread_lock_acquire_write(ms->rw_lock);
        ms->attrs=attr_generic_remove_attr(ms->attrs,attr);
        ms->maps=g_list_remove(ms->maps, attr->u.map);
        thread_lock_release_write(ms->rw_lock);
        return 1;
    default:
        return 0;
    }
}

/**
 * @brief Retrieves a map from a mapset.
 *
 * Attribute types other than `attr_map` are not supported.
 *
 * @param ms The mapset to retrieve the map from
 * @param type The attribute type, only `attr_map` is supported
 * @param attr Points to a buffer which will receive the attribute
 * @param iter An iterator (if NULL, the first map in the set is retrieved)
 */
int mapset_get_attr(struct mapset *ms, enum attr_type type, struct attr *attr, struct attr_iter *iter) {
    GList *map;
    map=ms->maps;
    attr->type=type;
    switch (type) {
    case attr_map:
        thread_lock_acquire_read(ms->rw_lock);
        while (map) {
            if (!iter || iter->last == g_list_previous(map)) {
                attr->u.map=map->data;
                if (iter)
                    iter->last=map;
                thread_lock_release_read(ms->rw_lock);
                return 1;
            }
            map=g_list_next(map);
        }
        thread_lock_release_read(ms->rw_lock);
        break;
    default:
        break;
    }
    return 0;
}

/**
 * @brief Destroys a mapset.
 *
 * This destroys a mapset. Please note that it does not touch the contained maps
 * in any way.
 *
 * @param ms The mapset to be destroyed
 */
void mapset_destroy(struct mapset *ms) {
    thread_lock_destroy(ms->rw_lock);
    g_list_free(ms->maps);
    attr_list_free(ms->attrs);
    g_free(ms);
}

/**
 * @brief Handle for a mapset in use
 *
 * This struct is used for a mapset that is in use. With this it is possible to iterate
 * all maps in a mapset.
 */
struct mapset_handle {
    GList *l;	/**< Pointer to the current (next) map */
    thread_lock * ms_rw_lock; /**< Lock for insertions, deletions or iterations over maps in the mapset */
};

/**
 * @brief Returns a new handle for a mapset.
 *
 * This returns a new handle for an existing mapset. The new handle points to the first
 * map in the set.
 *
 * If Navit was built with GLib thread support, this will obtain a read lock on the mapset, which is released when the
 * mapset is closed again. Until then, no maps can be added to or removed from the mapset.
 *
 * @param ms The mapset to get a handle of
 * @return The new mapset handle
 */
struct mapset_handle *
mapset_open(struct mapset *ms) {
    struct mapset_handle *ret=NULL;
    if(ms) {
        ret=g_new(struct mapset_handle, 1);
        thread_lock_acquire_read(ms->rw_lock);
        ret->ms_rw_lock = ms->rw_lock;
        ret->l=ms->maps;
    }

    return ret;
}

/**
 * @brief Gets the next map from a mapset handle
 *
 * If you set active to true, this function will not return any maps that
 * have the attr_active attribute associated with them and set to false.
 *
 * @param msh The mapset handle to get the next map of
 * @param active Set to true to only get active maps (See description)
 * @return The next map
 */
struct map * mapset_next(struct mapset_handle *msh, int active) {
    struct map *ret;
    struct attr active_attr;

    for (;;) {
        if (!msh || !msh->l)
            return NULL;
        ret=msh->l->data;
        msh->l=g_list_next(msh->l);
        if (!active)
            return ret;
        if (active == 2 && map_get_attr(ret, attr_route_active, &active_attr, NULL)) {
            if (active_attr.u.num)
                return ret;
            else
                continue;
        }
        if (active == 3 && map_get_attr(ret, attr_search_active, &active_attr, NULL)) {
            if (active_attr.u.num)
                return ret;
            else
                continue;
        }
        if (!map_get_attr(ret, attr_active, &active_attr, NULL))
            return ret;
        if (active_attr.u.num)
            return ret;
    }
}

/**
 * @brief Gets a map from the mapset by name
 *
 * @param ms The map
 * @param map_name the map name used by the search
 * @return The next map
 */
struct map *
mapset_get_map_by_name(struct mapset *ms, const char*map_name) {
    struct mapset_handle*msh;
    struct map*curr_map;
    struct attr map_attr;
    if( !ms || !map_name ) {
        return NULL;
    }
    msh=mapset_open(ms);
    while ((curr_map=mapset_next(msh, 1))) {
        //get map name
        if(map_get_attr(curr_map,attr_name, &map_attr,NULL)) {
            if( ! strcmp(map_attr.u.str, map_name)) {
                break;
            }
        }
    }
    mapset_close(msh);
    return curr_map;
}

/**
 * @brief Closes a mapset handle after it is no longer used.
 *
 * If Navit was built with GLib thread support, this will also release the lock on the mapset. Maps can be added to or
 * removed from the mapset when there are no more mapset handles.
 *
 * @param msh Mapset handle to be closed
 */
void mapset_close(struct mapset_handle *msh) {
    if (msh)
        thread_lock_release_read(msh->ms_rw_lock);
    g_free(msh);
}

/**
 * @brief Holds information about a search in a mapset
 *
 * This struct holds information about a search (e.g. for a street) in a mapset.
 *
 * @sa For a more detailed description see the documentation of mapset_search_new().
 */
struct mapset_search {
    GList *map;					/**< The list of maps to be searched within */
    struct map_search *ms;		/**< A map search struct for the map currently active */
    struct item *item;			/**< "Superior" item. */
    struct attr *search_attr;	/**< Attribute to be searched for. */
    int partial;				/**< Indicates if one would like to have partial matches */
    struct mapset *mapset;		/**< reference to current mapset. Set to NULL when all maps are searched */
};

/**
 * @brief Starts a search on a mapset
 *
 * This function starts a search on a mapset. What attributes one can search for depends on the
 * map plugin. See the description of map_search_new() in map.c for details.
 *
 * If you enable partial matches bear in mind that the search matches only the begin of the
 * strings - a search for a street named "street" would match to "streetfoo", but not to
 * "somestreet". Search is case insensitive.
 *
 * The item passed to this function specifies a "superior item" to "search within" - e.g. a town
 * in which we want to search for a street, or a country in which to search for a town.
 *
 * @param ms The mapset that should be searched
 * @param item Specifies a superior item to "search within" (see description)
 * @param search_attr Attribute specifying what to search for. See description.
 * @param partial Set this to true to also have partial matches. See description.
 * @return A new mapset search struct for this search
 */
struct mapset_search *
mapset_search_new(struct mapset *ms, struct item *item, struct attr *search_attr, int partial) {
    struct mapset_search *this;
    dbg(lvl_debug,"enter(%p,%p,%p,%d)", ms, item, search_attr, partial);
    this=g_new0(struct mapset_search,1);
    if(this != NULL && ms!=NULL ) {
        this->mapset=ms;
        this->item=item;
        this->search_attr=search_attr;
        this->partial=partial;
        return this;
    } else {
        return NULL;
    }
}

/**
 * @brief Returns the next found item from a mapset search
 *
 * This function returns the next item from a mapset search or NULL if there are no more items found.
 * It automatically iterates through all the maps in the mapset. Please note that maps which have the
 * attr_active attribute associated with them and set to false are not searched.
 *
 * @param this The mapset search to return an item from
 * @return The next found item or NULL if there are no more items found
 */
struct item *
mapset_search_get_item(struct mapset_search *this_) {
    struct item *ret=NULL;
    struct attr active_attr;
    int country_search=this_->search_attr->type >= attr_country_all && this_->search_attr->type <= attr_country_name;

    thread_lock_acquire_read(this_->mapset->rw_lock);
    while ((this_) && (this_->mapset) && (!this_->ms
                                          || !(ret=map_search_get_item(this_->ms)))) { /* The current map has no more items to be returned */

        /* Use only the first map from the mapset to search for country codes. */
        if (this_->map && country_search)
            break;

        for (;;) {
            if (!this_->map)
                this_->map=this_->mapset->maps;
            else
                this_->map=g_list_next(this_->map);

            if (!this_->map) {
                /* No more maps left, mark this mapset_search as finished */
                this_->mapset=NULL;
                break;
            }

            /* Any map can be used for country search, regardless of it's attr_active value */
            if(country_search)
                break;

            if (map_get_attr(this_->map->data, attr_search_active, &active_attr, NULL)) {
                if (!active_attr.u.num)
                    continue;
            }
            if (!map_get_attr(this_->map->data, attr_active, &active_attr, NULL))
                break;
            if (active_attr.u.num)
                break;
        }
        if(this_->ms) {
            map_search_destroy(this_->ms);
            this_->ms=NULL;
        }
        if (! this_->map)
            break;
        this_->ms=map_search_new(this_->map->data, this_->item, this_->search_attr, this_->partial);
    }
    thread_lock_release_read(this_->mapset->rw_lock);
    return ret;
}

/**
 * @brief Destroys a mapset search
 *
 * @param this The mapset search to be destroyed
 */
void mapset_search_destroy(struct mapset_search *this_) {
    if (this_) {
        map_search_destroy(this_->ms);
        g_free(this_);
    }
}

struct object_func mapset_func = {
    attr_mapset,
    (object_func_new)mapset_new,
    (object_func_get_attr)mapset_get_attr,
    (object_func_iter_new)mapset_attr_iter_new,
    (object_func_iter_destroy)mapset_attr_iter_destroy,
    (object_func_set_attr)NULL,
    (object_func_add_attr)mapset_add_attr,
    (object_func_remove_attr)mapset_remove_attr,
    (object_func_init)NULL,
    (object_func_destroy)mapset_destroy,
    (object_func_dup)mapset_dup,
    (object_func_ref)navit_object_ref,
    (object_func_unref)navit_object_unref,
};



