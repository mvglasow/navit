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
#include <sys/time.h>
#include "debug.h"
#include "callback.h"
#include "plugin.h"
#include "coord.h"
#include "item.h"
#include "android.h"
#include "vehicle.h"
#include "location.h"

#define RAW_LOCATIONS 2

/**
 * Indices for raw locations in {@code vehicle_priv.raw_loc}
 */
enum raw_index {
	raw_index_gps = 0,
	raw_index_network = 1,
};


struct vehicle_priv {
	struct callback_list *cbl;
	struct location *location; /**< The location of the vehicle.
	                                This is what Navit assumes to be the current location. It can be the
	                                last position obtained from any location provider, or an estimate
	                                based on previous positions, time elapsed and other factors. **/
	struct location **raw_loc; /**< Raw locations used to calculate {@code location}.
	                            *   This is a NULL-terminated pointer array. **/
	double altitude;           /**< Temporary value for returning altitude in an attr, not set until attr is requested **/
	double bearing;            /**< Temporary value for returning bearing in an attr, not set until attr is requested **/
	/* FIXME should we do something similar for fix_iso8601? */
	struct coord_geo position; /**< Temporary value for returning position in an attr, not set until attr is requested **/
	double position_accuracy;  /**< Temporary value for returning positional accuracy in an attr, not set until attr is requested **/
	double speed;              /**< Temporary value for returning speed in an attr, not set until attr is requested **/
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
	location_destroy(priv->location);
	for (i = 0; priv->raw_loc[i]; i++)
		location_destroy(priv->raw_loc[i]);
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
		attr->u.num = location_get_fix_type(priv->location);
		break;
	case attr_position_height:
		if (!location_has_altitude(priv->location))
			return 0;
		priv->altitude = location_get_altitude(priv->location);
		attr->u.numd = &priv->altitude;
		break;
	case attr_position_speed:
		if (!location_has_speed(priv->location))
			return 0;
		priv->speed = location_get_speed(priv->location);
		attr->u.numd = &priv->speed;
		break;
	case attr_position_direction:
		if (!location_has_bearing(priv->location))
			return 0;
		priv->bearing = location_get_bearing(priv->location);
		attr->u.numd = &priv->bearing;
		break;
	case attr_position_radius:
		if (!location_has_position_accuracy(priv->location))
			return 0;
		priv->position_accuracy = location_get_position_accuracy(priv->location);
		attr->u.numd = &priv->position_accuracy;
		break;
	case attr_position_qual:
		if (!location_has_sat_data(priv->location))
			return 0;
		attr->u.num = location_get_sats(priv->location);
		break;
	case attr_position_sats_used:
		if (!location_has_sat_data(priv->location))
			return 0;
		attr->u.num = location_get_sats_used(priv->location);
		break;
	case attr_position_coord_geo:
		location_get_position(priv->location, &priv->position);
		attr->u.coord_geo = &priv->position;
		if (!location_has_position(priv->location) || (location_get_validity(priv->location) == attr_position_valid_invalid))
			return 0;
		break;
	case attr_position_time_iso8601:
		attr->u.str = location_get_fixiso8601(priv->location);
		break;
	case attr_position_valid:
		attr->u.num = location_get_validity(priv->location);
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
 * @brief Called when a new position has been reported
 *
 * This function is called by {@code NavitLocationListener} upon receiving a new {@code Location}.
 *
 * @param v The {@code struct_vehicle_priv} for the vehicle
 * @param location A {@code Location} object describing the new position
 */
static void
vehicle_android_position_callback(struct vehicle_priv *v, jobject location) {
	struct timeval tv;
	struct coord_geo geo;
	jobject provider;		/* The location provider as a Java String */
	char * provider_chars;	/* The location provider as a C string */
	char * gps = "gps";		/* The string used for the GPS location provider */
	int index;				/* Index into raw_loc where the location will be stored */

	dbg(lvl_debug,"enter\n");

	provider = (*jnienv)->CallObjectMethod(jnienv, location, v->Location_getProvider);
	provider_chars = (*jnienv)->GetStringUTFChars(jnienv, provider, NULL);
	if (!strcmp(provider_chars, gps)) {
		index = raw_index_gps;
		location_set_preference(v->raw_loc[index], preference_high);
		/* For a GPS location, use system time in order to make fix_time comparable */
		gettimeofday(&tv, NULL);
		location_set_fix_time(v->raw_loc[index], &tv);
	} else {
		index = raw_index_network;
		location_set_preference(v->raw_loc[index], preference_medium);
		tv.tv_sec = (*jnienv)->CallLongMethod(jnienv, location, v->Location_getTime) / 1000;
		tv.tv_usec = ((*jnienv)->CallLongMethod(jnienv, location, v->Location_getTime) % 1000) * 1000;
		location_set_fix_time(v->raw_loc[index], &tv);
	}
	index = strcmp(provider_chars, gps) ? raw_index_network : raw_index_gps;
	dbg(lvl_debug, "provider=%s, index=%d\n", provider_chars, index);
	(*jnienv)->ReleaseStringUTFChars(jnienv, provider, provider_chars);

	geo.lat = (*jnienv)->CallDoubleMethod(jnienv, location, v->Location_getLatitude);
	geo.lng = (*jnienv)->CallDoubleMethod(jnienv, location, v->Location_getLongitude);
	location_set_position(v->raw_loc[index], &geo);
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasSpeed))
		location_set_speed(v->raw_loc[index], (*jnienv)->CallFloatMethod(jnienv, location, v->Location_getSpeed) * 3.6);
	else
		location_clear_speed(v->raw_loc[index]);
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasBearing))
		location_set_bearing(v->raw_loc[index], (*jnienv)->CallFloatMethod(jnienv, location, v->Location_getBearing));
	else
		location_clear_bearing(v->raw_loc[index]);
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasAltitude))
		location_set_altitude(v->raw_loc[index], (*jnienv)->CallDoubleMethod(jnienv, location, v->Location_getAltitude));
	else
		location_clear_altitude(v->raw_loc[index]);
	if ((*jnienv)->CallBooleanMethod(jnienv, location, v->Location_hasAccuracy))
		location_set_position_accuracy(v->raw_loc[index], (*jnienv)->CallFloatMethod(jnienv, location, v->Location_getAccuracy));
	else
		location_clear_position_accuracy(v->raw_loc[index]);
	location_set_validity(v->raw_loc[index], attr_position_valid_valid);
	dbg(lvl_debug,"lat %f lon %f time %s\n", geo.lat, geo.lng, location_get_fixiso8601(v->raw_loc[index]));

	location_update(v->raw_loc, v->location, v->cbl);
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
	if (location_has_sat_data(v->raw_loc[raw_index_gps])
			&& (location_get_sats(v->raw_loc[raw_index_gps]) == sats_in_view)
			&& (location_get_sats_used(v->raw_loc[raw_index_gps]) == sats_used))
		return;

	location_set_sat_data(v->raw_loc[raw_index_gps], sats_in_view, sats_used);
	location_update(v->raw_loc, v->location, v->cbl);
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
	if (location_get_fix_type(v->raw_loc[raw_index_gps]) != fix_type) {
		location_set_fix_type(v->raw_loc[raw_index_gps], fix_type);
		location_update(v->raw_loc, v->location, v->cbl);
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
	ret->location = location_new();
	location_set_validity(ret->location, attr_position_valid_invalid);
	location_clear_sat_data(ret->location);
	ret->raw_loc = g_new0(struct location *, RAW_LOCATIONS + 1);
	for (i = 0; i < RAW_LOCATIONS; i++)
		ret->raw_loc[i] = location_new();
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
