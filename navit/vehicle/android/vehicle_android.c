/** @file vehicle_android.c
 * @brief android uses dbus signals
 *
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
 *
 * @Author Tim Niemeyer <reddog@mastersword.de>
 * @date 2008-2009
 */

#include <config.h>
#include <string.h>
#include <glib.h>
#include <math.h>
#include <time.h>
#include "debug.h"
#include "callback.h"
#include "plugin.h"
#include "coord.h"
#include "item.h"
#include "android.h"
#include "vehicle.h"

#define RAW_LOCATIONS 2

/**
 * Indices for raw locations in {@code vehicle_priv.raw_loc}
 */
enum raw_index {
	raw_index_gps = 0,
	raw_index_network = 1,
};


/**
 * Flags to describe which members of a {@code struct location} contain valid data.
 */
enum location_flags {
	location_flag_has_geo = 0x1,		/*!< The location supplies coordinates.
										 *   Locations without coordinates may be used for inertial
										 *   navigation or to supplement another location with extra
										 *   data or supplied by the other location. */
	location_flag_has_speed = 0x2,		/*!< The location supplies speed data. */
	location_flag_has_direction = 0x4,	/*!< The location supplies bearing data. */
	location_flag_has_height = 0x8,		/*!< The location supplies altitude data. */
	location_flag_has_radius = 0x10,	/*!< The location supplies accuracy data for its coordinates. */
	location_flag_has_sat_data = 0x20,	/*!< The location supplies satellite data,
										 *   i.e. the number of satellites in view and the number of
										 *   satellites used for position measurement. */
};


/**
 * Describes a location.
 *
 * A location contains data describing the movement of the vehicle, along with associated metadata.
 * It may have been obtained directly from one of the operating system's location providers (such as GPS
 * or network) or calculated by various means.
 */
/* TODO accuracy for speed, bearing (with flags) */
/* TODO preference level for locations (fusion will use only the highest available preference level) */
struct location {
	struct coord_geo geo;      /**< The position of the vehicle **/
	double speed;              /**< Speed in km/h **/
	double direction;          /**< Bearing in degrees **/
	double height;             /**< Altitude in meters **/
	double radius;             /**< Position accuracy in meters **/
	int fix_type;              /**< Type of last fix.
	                            *   On Android, this is either 1 for a fix or 0 if the fix has been lost. **/
	struct timeval fix_time;   /**< Timestamp of last fix.
	                            *   All location sources must use the same reference time (usually
	                            *   system time) to allow comparison and extrapolation.	**/
	char fixiso8601[128];      /**< Timestamp of last fix in ISO 8601 format **/
	int sats;                  /**< Number of satellites in view **/
	int sats_used;             /**< Number of satellites used in fix **/
	int valid;                 /**< Whether the data in this location is valid, and how it was obtained
	                            *   (e.g. through measurement or extrapolation). See
	                            *   {@code enum attr_position_valid} for possible values. Examine
	                            *   {@code flags} to find out what data this location supplies. **/
	int flags;                 /**< Describes the information supplied by this location.
	                            *   Members whose the corresponding flag is not set should be ignored.
	                            *   The flags do not supply any information on how the location data was
	                            *   obtained, or if it is valid. This can be determined by examining
	                            *   {@code valid}, which should always be used in conjunction with the
	                            *   flags. **/
};

struct vehicle_priv {
	struct callback_list *cbl;
	struct location location;  /**< The location of the vehicle.
	                                This is what Navit assumes to be the current location. It can be the
	                                last position obtained from any location provider, or an estimate
	                                based on previous positions, time elapsed and other factors. **/
	struct location **raw_loc; /**< Raw locations used to calculate {@code location}.
	                            *   This is a NULL-terminated pointer array. **/
	struct attr ** attrs;
	struct callback *pcb;      /**< The callback function for position updates **/
	struct callback *scb;      /**< The callback function for status updates **/
	struct callback *fcb;      /**< The callback function for fix status updates **/
	jclass NavitVehicleClass;  /**< The {@code NavitVehicle} class **/
	jobject NavitVehicle;      /**< An instance of {@code NavitVehicle} **/
	jclass LocationClass;      /**< Android's {@code Location} class **/
	jmethodID Location_getLatitude, Location_getLongitude, Location_getSpeed, Location_getBearing, Location_getAltitude, Location_getTime, Location_getAccuracy, Location_getProvider,
	Location_hasSpeed, Location_hasBearing, Location_hasAltitude, Location_hasAccuracy;
};

/**
 * @brief Destroys the vehicle_android instance
 *
 * This methods releases the memory used by the struct itself, as well as the memory referenced by
 * {@code raw_loc}, but not the memory referenced by {@code cbl} and {@code attr}. It is the caller's
 * responsibility to ensure that either a copy of these pointers remains available or to free the memory
 * referenced by them prior to calling this method.
 * 
 * @param priv The instance to destroy.
 */
static void
vehicle_android_destroy(struct vehicle_priv *priv)
{
	int i;
	dbg(lvl_debug,"enter\n");
	for (i = 0; priv->raw_loc[i]; i++)
		g_free(priv->raw_loc[i]);
	g_free(priv->raw_loc);
	g_free(priv);
}

/**
 * @brief Retrieves a vehicle attribute.
 *
 * @param priv vehicle_priv structure for the vehicle
 * @param type The attribute type to retrieve
 * @param attr Points to an attr structure that will receive the attribute data
 * @returns True for success, false for failure
 */
static int
vehicle_android_position_attr_get(struct vehicle_priv *priv,
			       enum attr_type type, struct attr *attr)
{
	dbg(lvl_debug,"enter %s\n",attr_to_name(type));
	switch (type) {
	case attr_position_fix_type:
		attr->u.num = priv->location.fix_type;
		break;
	case attr_position_height:
		attr->u.numd = &priv->location.height;
		break;
	case attr_position_speed:
		attr->u.numd = &priv->location.speed;
		break;
	case attr_position_direction:
		attr->u.numd = &priv->location.direction;
		break;
	case attr_position_radius:
		attr->u.numd = &priv->location.radius;
		break;
	case attr_position_qual:
		attr->u.num = priv->location.sats;
		break;
	case attr_position_sats_used:
		attr->u.num = priv->location.sats_used;
		break;
	case attr_position_coord_geo:
		attr->u.coord_geo = &priv->location.geo;
		if (priv->location.valid == attr_position_valid_invalid)
			return 0;
		break;
	case attr_position_time_iso8601:
		attr->u.str=priv->location.fixiso8601;
		break;
	case attr_position_valid:
		attr->u.num = priv->location.valid;
		break;
	default:
		return 0;
	}
	dbg(lvl_debug,"ok\n");
	attr->type = type;
	return 1;
}

struct vehicle_methods vehicle_android_methods = {
	vehicle_android_destroy,
	vehicle_android_position_attr_get,
};


/**
 * @brief Updates the vehicle position.
 *
 * This method recalculates the position and sets its members accordingly. It is called from the
 * position callback but may be extended in the future to be called by other triggers (events or timers)
 * to extrapolate the current vehicle position.
 *
 * This method will fill the struct passed as {@code out} with the newly calculated location data and
 * trigger callbacks from the list passed as {@code cbl} when one of the respective attributes changes.
 * The decision whether to trigger a callback is made by comparing old and new values of {@code out}. In
 * order for this to work correctly, {@code out} must hold the last location of the vehicle when this
 * method is called, or zeroed out if no previous location is known.
 *
 * For now, this method simply takes the most recent valid fix we received.
 *
 * @param v The {@code struct_vehicle_priv} for the vehicle
 * @param in Raw locations (NULL-terminated pointer array)
 * @param out The last calculated location of the vehicle; this struct will receive the updated location
 * @param cbl Callback list of the vehicle; callbacks from this list will be triggered when one of the
 * respective attributes changes
 */
/* TODO: in the long run, this should become a generic function which other vehicles can use as well */
static void
vehicle_android_update_position(struct location ** in, struct location * out, struct callback_list * cbl) {
	int index = 0;
	int i;
	int validity_changed = 0;	/* Whether position validity has changed. */

	/* having the loop allows for easy extension of the array beyond 2 elements */
	for (i = 1; in[i]; i++)
		if ((in[i]->valid = attr_position_valid_valid)
				&& ((in[i]->fix_time.tv_sec > in[index]->fix_time.tv_sec)
						|| ((in[i]->fix_time.tv_sec == in[index]->fix_time.tv_sec) && (in[i]->fix_time.tv_usec > in[index]->fix_time.tv_usec))))
			index = i;
	dbg(lvl_debug, "index=%d\n", index);
	/* TODO revise validity logic when we introduce extrapolated positions */
	validity_changed = (out->valid != in[index]->valid);
	out->valid = in[index]->valid;
	if (in[index]->valid == attr_position_valid_invalid) {
		if (validity_changed)
			callback_list_call_attr_0(cbl, attr_position_valid);
		return;
	}
	out->geo.lat = in[index]->geo.lat;
	out->geo.lng = in[index]->geo.lng;
	out->speed = in[index]->speed;
	out->direction = in[index]->direction;
	out->height = in[index]->height;
	out->radius = in[index]->radius;
	if (out->fix_type != in[index]->fix_type) {
		out->fix_type = in[index]->fix_type;
		callback_list_call_attr_0(cbl, attr_position_fix_type);
	}
	out->fix_time.tv_sec = in[index]->fix_time.tv_sec;
	out->fix_time.tv_usec = in[index]->fix_time.tv_usec;
	memcpy(out->fixiso8601, in[index]->fixiso8601, sizeof(out->fixiso8601));
	if (out->sats != in[index]->sats) {
		out->sats = in[index]->sats;
		callback_list_call_attr_0(cbl, attr_position_qual);
	}
	if (out->sats_used != in[index]->sats_used) {
		out->sats_used = in[index]->sats_used;
		callback_list_call_attr_0(cbl, attr_position_sats_used);
	}
	out->flags = in[index]->flags;
	dbg(lvl_debug, "lat %f lon %f time %s\n", out->geo.lat, out->geo.lng, out->fixiso8601);

	if (validity_changed)
		callback_list_call_attr_0(cbl, attr_position_valid);
	/* TODO find out if the position actuially has changed before triggering the callback */
	callback_list_call_attr_0(cbl, attr_position_coord_geo);
}


/**
 * @brief Called when a new position has been reported
 *
 * This function is called by {@code NavitLocationListener} upon receiving a new {@code Location}.
 *
 * @param v The {@code struct_vehicle_priv} for the vehicle
 * @param location A {@code Location} object describing the new position
 */
static void
vehicle_android_position_callback(struct vehicle_priv *v, jobject location) {
	time_t tnow;
	struct tm *tm;
	jobject provider;		/* The location provider as a Java String */
	char * provider_chars;	/* The location provider as a C string */
	char * gps = "gps";		/* The string used for the GPS location provider */
	int index;				/* Index into raw_loc where the location will be stored */

	dbg(lvl_debug,"enter\n");

	provider = (*jnienv)->CallObjectMethod(jnienv, location, v->Location_getProvider);
	provider_chars = (*jnienv)->GetStringUTFChars(jnienv, provider, NULL);
	dbg(lvl_debug, "provider=%s\n", provider_chars);
	if (!strcmp(provider_chars, gps)) {
		index = raw_index_gps;
		/* For a GPS location, use system time in order to make fix_time comparable */
		gettimeofday(&(v->raw_loc[index]->fix_time), NULL);
	} else {
		index = raw_index_network;
		v->raw_loc[index]->fix_time.tv_sec = (*jnienv)->CallLongMethod(jnienv, location, v->Location_getTime) / 1000;
		v->raw_loc[index]->fix_time.tv_usec = ((*jnienv)->CallLongMethod(jnienv, location, v->Location_getTime) % 1000) * 1000;
	}
	index = strcmp(provider_chars, gps) ? raw_index_network : raw_index_gps;
	(*jnienv)->ReleaseStringUTFChars(jnienv, provider, provider_chars);

	v->raw_loc[index]->flags = location_flag_has_geo;
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasSpeed))
		v->raw_loc[index]->flags |= location_flag_has_speed;
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasBearing))
		v->raw_loc[index]->flags |= location_flag_has_direction;
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasAltitude))
		v->raw_loc[index]->flags |= location_flag_has_height;
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasAccuracy))
		v->raw_loc[index]->flags |= location_flag_has_radius;
	v->raw_loc[index]->geo.lat = (*jnienv)->CallDoubleMethod(jnienv, location, v->Location_getLatitude);
	v->raw_loc[index]->geo.lng = (*jnienv)->CallDoubleMethod(jnienv, location, v->Location_getLongitude);
	v->raw_loc[index]->speed = (*jnienv)->CallFloatMethod(jnienv, location, v->Location_getSpeed) * 3.6;
	v->raw_loc[index]->direction = (*jnienv)->CallFloatMethod(jnienv, location, v->Location_getBearing);
	v->raw_loc[index]->height = (*jnienv)->CallDoubleMethod(jnienv, location, v->Location_getAltitude);
	v->raw_loc[index]->radius = (*jnienv)->CallFloatMethod(jnienv, location, v->Location_getAccuracy);
	tnow = v->raw_loc[index]->fix_time.tv_sec;
	tm = gmtime(&tnow);
	strftime(v->raw_loc[index]->fixiso8601, sizeof(v->raw_loc[index]->fixiso8601), "%Y-%m-%dT%TZ", tm);
	v->raw_loc[index]->valid = attr_position_valid_valid;
	dbg(lvl_debug,"lat %f lon %f time %s\n", v->raw_loc[index]->geo.lat, v->raw_loc[index]->geo.lng, v->raw_loc[index]->fixiso8601);

	vehicle_android_update_position(v->raw_loc, &(v->location), v->cbl);
}


/**
 * @brief Called when a new GPS status has been reported
 *
 * This function is called by {@code NavitLocationListener} upon receiving a new {@code GpsStatus}.
 *
 * Note that {@code sats_used} should not be used to determine whether the vehicle's position is valid:
 * some devices report non-zero numbers even when they do not have a fix. Position validity should be
 * determined in {@code vehicle_android_fix_callback} (an invalid fix type means we have lost the fix)
 * and {@code vehicle_android_position_callback} (receiving a position means we have a fix).
 *
 * @param v The {@code struct_vehicle_priv} for the vehicle
 * @param sats_in_view The number of satellites in view
 * @param sats_used The number of satellites currently used to determine the position
 */
static void
vehicle_android_status_callback(struct vehicle_priv *v, int sats_in_view, int sats_used) {
	if (v->raw_loc[raw_index_gps]->sats != sats_in_view) {
		v->raw_loc[raw_index_gps]->sats = sats_in_view;
#if 0
		/* Don't trigger callbacks before data is reflected in v->location */
		callback_list_call_attr_0(v->cbl, attr_position_qual);
#endif
	}
	if (v->raw_loc[raw_index_gps]->sats_used != sats_used) {
		v->raw_loc[raw_index_gps]->sats_used = sats_used;
#if 0
		/* Don't trigger callbacks before data is reflected in v->location */
		callback_list_call_attr_0(v->cbl, attr_position_sats_used);
#endif
	}
	v->raw_loc[raw_index_gps]->flags |= location_flag_has_sat_data;
	/* TODO should we update after this?
	 * If we do, we'd fully recalculate the location despite results turning out the same.
	 * If we don't, callbacks related to sat status would be delayed until the location changes. */
}

/**
 * @brief Called when a change in GPS fix status has been reported
 *
 * This function is called by {@code NavitLocationListener} upon receiving a new {@code android.location.GPS_FIX_CHANGE} broadcast.
 *
 * It is also called whenever a fix is received from any location provider, but note that loss of fix is
 * only reported for GPS.
 *
 * @param v The {@code struct_vehicle_priv} for the vehicle
 * @param fix_type The fix type (1 = fix acquired, 0 = fix lost)
 */
static void
vehicle_android_fix_callback(struct vehicle_priv *v, int fix_type) {
	if (v->raw_loc[raw_index_gps]->fix_type != fix_type) {
		v->raw_loc[raw_index_gps]->fix_type = fix_type;
#if 0
		/* Don't trigger callbacks before data is reflected in v->location */
		callback_list_call_attr_0(v->cbl, attr_position_fix_type);
#endif
		if (!fix_type && (v->raw_loc[raw_index_gps]->valid == attr_position_valid_valid)) {
			v->raw_loc[raw_index_gps]->valid = attr_position_valid_extrapolated_time;
#if 0
			/* Don't trigger callbacks before data is reflected in v->location */
			callback_list_call_attr_0(v->cbl, attr_position_valid);
#endif
		}
		/* TODO should we update after this?
		 * If we do, we'd fully recalculate the location despite results turning out the same.
		 * If we don't, fix type/position validity callbacks would be delayed until the location changes. */
	}
}

/**
 * @brief Initializes an Android vehicle
 *
 * @return True on success, false on failure
 */
static int
vehicle_android_init(struct vehicle_priv *ret)
{
	jmethodID cid;

	if (!android_find_class_global("android/location/Location", &ret->LocationClass))
                return 0;
	if (!android_find_method(ret->LocationClass, "getLatitude", "()D", &ret->Location_getLatitude))
                return 0;
	if (!android_find_method(ret->LocationClass, "getLongitude", "()D", &ret->Location_getLongitude))
                return 0;
	if (!android_find_method(ret->LocationClass, "getSpeed", "()F", &ret->Location_getSpeed))
                return 0;
	if (!android_find_method(ret->LocationClass, "getBearing", "()F", &ret->Location_getBearing))
                return 0;
	if (!android_find_method(ret->LocationClass, "getAltitude", "()D", &ret->Location_getAltitude))
                return 0;
	if (!android_find_method(ret->LocationClass, "getTime", "()J", &ret->Location_getTime))
                return 0;
	if (!android_find_method(ret->LocationClass, "getAccuracy", "()F", &ret->Location_getAccuracy))
                return 0;
	if (!android_find_method(ret->LocationClass, "getProvider", "()Ljava/lang/String;", &ret->Location_getProvider))
                return 0;
	if (!android_find_method(ret->LocationClass, "hasSpeed", "()Z", &ret->Location_hasSpeed))
                return 0;
	if (!android_find_method(ret->LocationClass, "hasBearing", "()Z", &ret->Location_hasBearing))
                return 0;
	if (!android_find_method(ret->LocationClass, "hasAltitude", "()Z", &ret->Location_hasAltitude))
                return 0;
	if (!android_find_method(ret->LocationClass, "hasAccuracy", "()Z", &ret->Location_hasAccuracy))
                return 0;
	if (!android_find_class_global("org/navitproject/navit/NavitVehicle", &ret->NavitVehicleClass))
                return 0;
        dbg(lvl_debug,"at 3\n");
        cid = (*jnienv)->GetMethodID(jnienv, ret->NavitVehicleClass, "<init>", "(Landroid/content/Context;III)V");
        if (cid == NULL) {
                dbg(lvl_error,"no method found\n");
                return 0; /* exception thrown */
        }
        dbg(lvl_debug, "at 4 android_activity=%p\n", android_activity);
        ret->NavitVehicle=(*jnienv)->NewObject(jnienv, ret->NavitVehicleClass, cid, android_activity,
                                                  (int) ret->pcb, (int) ret->scb, (int) ret->fcb);
        dbg(lvl_debug,"result=%p\n",ret->NavitVehicle);
	if (!ret->NavitVehicle)
		return 0;
        if (ret->NavitVehicle)
		ret->NavitVehicle = (*jnienv)->NewGlobalRef(jnienv, ret->NavitVehicle);

	return 1;
}

/**
 * @brief Creates a new vehicle_android instance.
 * 
 * @param meth The methods for the vehicle. This structure must be filled in prior to calling this method.
 * @param cbl The callback list for the new instance.
 * @param attrs List of attributes for the new instance
 * @return The new vehicle_android instance
 */
static struct vehicle_priv *
vehicle_android_new_android(struct vehicle_methods *meth,
	       		struct callback_list *cbl,
		       	struct attr **attrs)
{
	struct vehicle_priv *ret;
	int i;

	dbg(lvl_debug, "enter\n");
	ret = g_new0(struct vehicle_priv, 1);
	ret->cbl = cbl;
	ret->pcb = callback_new_1(callback_cast(vehicle_android_position_callback), ret);
	ret->scb = callback_new_1(callback_cast(vehicle_android_status_callback), ret);
	ret->fcb = callback_new_1(callback_cast(vehicle_android_fix_callback), ret);
	ret->location.valid = attr_position_valid_invalid;
	ret->location.sats = 0;
	ret->location.sats_used = 0;
	ret->raw_loc = g_new0(struct location *, RAW_LOCATIONS + 1);
	for (i = 0; i <= RAW_LOCATIONS; i++)
		ret->raw_loc[i] = g_new0(struct location, 1);
	*meth = vehicle_android_methods;
	vehicle_android_init(ret);
	dbg(lvl_debug, "return\n");
	return ret;
}

/**
 * @brief Registers the vehicle_android plugin
 */
void
plugin_init(void)
{
	dbg(lvl_debug, "enter\n");
	plugin_register_vehicle_type("android", vehicle_android_new_android);
}
