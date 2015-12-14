/*
 * Navit, a modular navigation system.
 * Copyright (C) 2005-2009 Navit Team
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
 * @file location.c
 *
 * @brief Logic to work with location data.
 *
 * A location contains data describing the movement of the vehicle, along with associated metadata.
 * It may have been obtained directly from a source such as GPS or nearby networks, or calculated by
 * various other means.
 *
 * @date 2015-11-30
 * @author mvglasow
 */

#include <math.h>
#include <glib.h>
#include "config.h"
#include "debug.h"
#include "item.h"
#include "coord.h"
#include "transform.h"
#include "callback.h"
#include "location.h"


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
 *
 * Three members should be examined to find out if and how the location can be used: the {@code valid}
 * member indicates if the location is valid in general. However, this information applies to the time
 * at which the location was obtained, stored in the {@code fix_time} member, which should be examined
 * in order to determine if the location is still current. Eventually, even a valid location may only
 * supply partial information and not all information (position, altitude, bearing or speed) may be
 * present. This is specified in the {@code flags} member.
 */
/* TODO accuracy for speed, bearing (with flags) */
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
	                            *   {@code flags} to find out what data this location supplies.
	                            *
	                            *   Note that validity of a location refers to the point in time
	                            *   indicated by {@code fix_time}. Both members should therefore be
	                            *   evaluated together to ensure the location is still current. **/
	int flags;                 /**< Describes the information supplied by this location.
	                            *   Members whose the corresponding flag is not set should be ignored.
	                            *   The flags do not supply any information on how the location data was
	                            *   obtained, or if it is valid. This can be determined by examining
	                            *   {@code valid}, which should always be used in conjunction with the
	                            *   flags. **/
	int preference;            /**< The preference level of the location. See {@code enum preference}. **/
};


/**
 * @brief Obtains the effective preference level of a location.
 *
 * The effective preference level is determined by applying a penalty, based on the location's validity,
 * to its base preference level. The result can be used for direct integer comparison.
 *
 * @param plev The preference level of the location, see {@code enum preference}
 * @param valid The validity of the location, see {@code enum attr_position_valid}
 *
 * @return The effective preference level (-INT_MAX for an invalid location)
 */
int
get_effective_preference_level(int plev, int valid) {
	switch (valid) {
		case attr_position_valid_invalid:
			return -INT_MAX;
		case attr_position_valid_valid:
		case attr_position_valid_static:
			return plev;
		case attr_position_valid_extrapolated_spatial:
			return plev - 1;
		default:
		/*case attr_position_valid_extrapolated_time:*/
			return plev - 2;
	}
}


/**
 * @brief Creates a new location.
 *
 * @return A new location. The caller is responsible for destroying the location when it is no longer needed.
 */
struct location *
location_new() {
	struct location * ret = g_new0(struct location, 1);
	ret->valid = attr_position_valid_invalid;
	return ret;
}


/**
 * @brief Clears the altitude of a location.
 *
 * After calling this function, calls to {@link location_has_altitude(struct location *)} will return
 * false.
 *
 * @param this_ The location whose altitude is to be cleared.
 */
void location_clear_altitude(struct location *this_) {
	this_->flags &= ~location_flag_has_height;
}


/**
 * @brief Clears the bearing of a location.
 *
 * After calling this function, calls to {@link location_has_bearing(struct location *)} will return
 * false.
 *
 * @param this_ The location whose bearing is to be cleared.
 */
void location_clear_bearing(struct location *this_) {
	this_->flags &= ~location_flag_has_direction;
}


/**
 * @brief Clears the position of a location.
 *
 * After calling this function, calls to {@link location_has_position(struct location *)} will return
 * false.
 *
 * @param this_ The location whose position is to be cleared.
 */
void location_clear_position(struct location *this_) {
	this_->flags &= ~location_flag_has_geo;
}


/**
 * @brief Clears the positional accuracy of a location.
 *
 * After calling this function, calls to {@link location_has_position_accuracy(struct location *)} will
 * return false.
 *
 * @param this_ The location whose positional accuracy is to be cleared.
 */
void location_clear_position_accuracy(struct location *this_) {
	this_->flags &= ~location_flag_has_radius;
}


/**
 * @brief Clears the satellite data of a location.
 *
 * After calling this function, calls to {@link location_has_sat_data(struct location *)} will return
 * false.
 *
 * @param this_ The location whose satellite data is to be cleared.
 */
void location_clear_sat_data(struct location *this_){
	this_->flags &= ~location_flag_has_sat_data;
}


/**
 * @brief Clears the speed of a location.
 *
 * After calling this function, calls to {@link location_has_speed(struct location *)} will return
 * false.
 *
 * @param this_ The location whose speed is to be cleared.
 */
void location_clear_speed(struct location *this_) {
	this_->flags &= ~location_flag_has_speed;
}


/**
 * @brief Returns the altitude of a location.
 *
 * Altitude is the difference between the position of the vehicle and mean sea level. The altitude of a
 * land vehicle is always equal to terrain elevation.
 *
 * Prior to calling this function, the caller should ensure that the location actually supplies altitude
 * data by calling {@link location_has_altitude(struct location *)}.
 *
 * @param this_ The location
 *
 * @return The altitude, if the location supplies it, else undefined.
 */
double location_get_altitude(struct location *this_) {
	return this_->height;
}


/**
 * @brief Returns the bearing of a location.
 *
 * The bearing is the direction into which the vehicle is facing or moving.
 *
 * Prior to calling this function, the caller should ensure that the location actually supplies bearing
 * data by calling {@link location_has_bearing(struct location *)}.
 *
 * @param this_ The location
 *
 * @return The bearing, if the location supplies it, else undefined.
 */
double location_get_bearing(struct location *this_) {
	return this_->direction;
}


/**
 * @brief Returns the timestamp of a location.
 *
 * The timestamp is the time at which a location was obtained, in GMT and relative to system time.
 *
 * @param this_ The location
 * @param time Points to a {@code struct timeval} which will receive the timestamp.
 */
void location_get_fix_time(struct location *this_, struct timeval *time) {
	*time = this_->fix_time;
}


/**
 * @brief Returns the fix type of a location.
 *
 * This is supported for legacy reasons. The semantics of the fix type may differ between
 * implementations, but generally 0 denotes an invalid fix while nonzero values denote a valid fix,
 * optionally using different values for different quality levels.
 *
 * @param this_ The location
 *
 * @return The fix type
 */
int location_get_fix_type(struct location *this_) {
	return this_->fix_type;
}


/**
 * @brief Returns the timestamp of a location in ISO 8601 format.
 *
 * The timestamp is the time at which the location was obtained, relative to system time.
 * ISO 8601 is the format used, among others, by NMEA. The general format is
 * {@code 2015-10-22T02:28:00.000Z}. This function is agnostic of time zones, thus the result will
 * always be in UTC with a time zone designator of Z.
 *
 * @param this_ The location
 *
 * @return The timestamp in ISO8601 format. The string will be freed when the location is destroyed, and
 * the caller should not attempt to alter or free the data.
 */
/* FIXME should we generate this on the fly and have the caller take care of everything? */
char *location_get_fixiso8601(struct location *this_) {
	return this_->fixiso8601;
}


/**
 * @brief Returns the position of a location.
 *
 * The position refers to the actual coordinates (latitude and longitude) of the vehicle.
 *
 * Prior to calling this function, the caller should ensure that the location actually supplies position
 * data by calling {@link location_has_position(struct location *)}.
 *
 * @param this_ The location
 *
 * @return The position, if the location supplies it, else undefined.
 */
void location_get_position(struct location *this_, struct coord_geo *position) {
	*position = this_->geo;
}


/**
 * @brief Returns the positional accuracy of a location.
 *
 * Positional accuracy is the quality indicator for the position, expressed as a distance in meters
 * between actual and position which has a certain likelihood (usually 95%) of not being exceeded.
 *
 * Prior to calling this function, the caller should ensure that the location actually supplies
 * positional accuracy data by calling {@link location_has_position_accuracy(struct location *)}.
 *
 * @param this_ The location
 *
 * @return The positional accuracy, if the location supplies it, else undefined.
 */
int location_get_position_accuracy(struct location *this_) {
	return this_->radius;
}


/**
 * @brief Returns the preference level of a location.
 *
 * See {@link enum preference} for a description of preference levels.
 *
 * @param this_ The location
 *
 * @return The preference level
 */
int location_get_preference(struct location *this_) {
	return this_->preference;
}


/**
 * @brief Returns the number of sats in view when the location was obtained.
 *
 * Prior to calling this function, the caller should ensure that the location actually supplies
 * satellite data by calling {@link location_has_sat_data(struct location *)}.
 *
 * @param this_ The location
 *
 * @return The number of satellites in view, if the location supplies it, else undefined.
 */
int location_get_sats(struct location *this_) {
	return this_->sats;
}


/**
 * @brief Returns the number of satellites used to obtain a location.
 *
 * Prior to calling this function, the caller should ensure that the location actually supplies
 * satellite data by calling {@link location_has_sat_data(struct location *)}.
 *
 * @param this_ The location
 *
 * @return The number of satellites used, if the location supplies it, else undefined.
 */
int location_get_sats_used(struct location *this_) {
	return this_->sats_used;
}


/**
 * @brief Returns the speed of a location.
 *
 * Speed is in km/h.
 *
 * Prior to calling this function, the caller should ensure that the location actually supplies altitude
 * data by calling {@link location_has_speed(struct location *)}.
 *
 * @param this_ The location
 *
 * @return The speed, if the location supplies it, else undefined.
 */
double location_get_speed(struct location *this_) {
	return this_->speed;
}


/**
 * @brief Returns the validity of a location.
 *
 * See {@link enum attr_position_valid} for possible return values and their meanings.
 *
 * @param this_ The location
 *
 * @return The validity
 */
int location_get_validity(struct location *this_) {
	return this_->valid;
}


/**
 * @brief Returns whether the location supplies altitude data.
 *
 * This function should be called and its return value examined prior to any attempt to retrieve
 * altitude data.
 *
 * @param this_ The location
 *
 * @return True if the location supplies altitude data, false if not.
 */
int location_has_altitude(struct location *this_) {
	return !!(this_->flags & location_flag_has_height);
}


/**
 * @brief Returns whether the location supplies bearing data.
 *
 * This function should be called and its return value examined prior to any attempt to retrieve
 * bearing data.
 *
 * @param this_ The location
 *
 * @return True if the location supplies bearing data, false if not.
 */
int location_has_bearing(struct location *this_) {
	return !!(this_->flags & location_flag_has_direction);
}


/**
 * @brief Returns whether the location supplies position data.
 *
 * This function should be called and its return value examined prior to any attempt to retrieve
 * position data.
 *
 * @param this_ The location
 *
 * @return True if the location supplies position data, false if not.
 */
int location_has_position(struct location *this_) {
	return !!(this_->flags & location_flag_has_geo);
}


/**
 * @brief Returns whether the location supplies positional accuracy data.
 *
 * This function should be called and its return value examined prior to any attempt to retrieve
 * positional accuracy data.
 *
 * @param this_ The location
 *
 * @return True if the location supplies positional accuracy data, false if not.
 */
int location_has_position_accuracy(struct location *this_) {
	return !!(this_->flags & location_flag_has_radius);
}


/**
 * @brief Returns whether the location supplies sattelite data.
 *
 * This function should be called and its return value examined prior to any attempt to retrieve
 * satellite data.
 *
 * @param this_ The location
 *
 * @return True if the location supplies satellite data, false if not.
 */
int location_has_sat_data(struct location *this_) {
	return !!(this_->flags & location_flag_has_sat_data);
}


/**
 * @brief Returns whether the location supplies speed data.
 *
 * This function should be called and its return value examined prior to any attempt to retrieve
 * speed data.
 *
 * @param this_ The location
 *
 * @return True if the location supplies speed data, false if not.
 */
int location_has_speed(struct location *this_) {
	return !!(this_->flags & location_flag_has_speed);
}


/**
 * @brief Sets the altitude of a location.
 *
 * Altitude is the difference between the position of the vehicle and mean sea level. The altitude of a
 * land vehicle is always equal to terrain elevation.
 *
 * After calling this function, {@link location_has_altitude(struct location *)} will return true.
 *
 * @param this_ The location
 * @param altitude The altitude
 */
void location_set_altitude(struct location *this_, double altitude) {
	this_->height = altitude;
	this_->flags |= location_flag_has_height;
}


/**
 * @brief Sets the bearing of a location.
 *
 * The bearing is the direction into which the vehicle is facing or moving.
 *
 * After calling this function, {@link location_has_bearing(struct location *)} will return true.
 *
 * @param this_ The location
 * @param bearing The bearing
 */
void location_set_bearing(struct location *this_, double bearing) {
	this_->direction = bearing;
	this_->flags |= location_flag_has_direction;
}


/**
 * @brief Sets the timestamp of a location.
 *
 * The timestamp is the time at which a location was obtained, in GMT and relative to system time.
 *
 * @param this_ The location
 * @param time The timestamp
 */
void location_set_fix_time(struct location *this_, struct timeval *time) {
	time_t timet;
	struct tm *tm;

	this_->fix_time = *time;
	timet = this_->fix_time.tv_sec;
	tm = gmtime(&timet);
	strftime(this_->fixiso8601, sizeof(this_->fixiso8601), "%Y-%m-%dT%TZ", tm);
}


/**
 * @brief Sets the fix type of a location.
 *
 * This is supported for legacy reasons. The semantics of the fix type may differ between
 * implementations, but generally 0 denotes an invalid fix while nonzero values denote a valid fix,
 * optionally using different values for different quality levels.
 *
 * @param this_ The location
 * @param type The fix type
 */
void location_set_fix_type(struct location *this_, int type) {
	this_->fix_type = type;
}


/**
 * @brief Sets the position of a location.
 *
 * The position refers to the actual coordinates (latitude and longitude) of the vehicle.
 *
 * After calling this function, {@link location_has_position(struct location *)} will return true.
 *
 * @param this_ The location
 * @param position The position
 */
void location_set_position(struct location *this_, struct coord_geo *position) {
	this_->geo = *position;
	this_->flags |= location_flag_has_geo;
}


/**
 * @brief Sets the positional accuracy of a location.
 *
 * Positional accuracy is the quality indicator for the position, expressed as a distance in meters
 * between actual and position which has a certain likelihood (usually 95%) of not being exceeded.
 *
 * After calling this function, {@link location_has_position_accuracy(struct location *)} will return
 * true.
 *
 * @param this_ The location
 * @param accuracy The positional accuracy
 */
void location_set_position_accuracy(struct location *this_, int accuracy) {
	this_->radius = accuracy;
	this_->flags |= location_flag_has_radius;
}


/**
 * @brief Sets the preference level of a location.
 *
 * See {@link enum preference} for a description of preference levels.
 *
 * @param this_ The location
 * @param preference The preference level
 */
void location_set_preference(struct location *this_, enum preference preference) {
	this_->preference = preference;
}

/**
 * @brief Sets the satellite data of a location.
 *
 * After calling this function, {@link location_has_sat_data(struct location *)} will return true.
 *
 * @param this_ The location
 * @param sats The number of satellites in view
 * @param sats_used The number of satellites used to obtain the position
 */
void location_set_sat_data(struct location *this_, int sats, int sats_used) {
	this_->sats = sats;
	this_->sats_used = sats_used;
	this_->flags |= location_flag_has_sat_data;
}


/**
 * @brief Sets the speed of a location.
 *
 * Speed is in km/h.
 *
 * After calling this function, {@link location_has_speed(struct location *)} will return true.
 *
 * @param this_ The location
 * @param speed The speed
 */
void location_set_speed(struct location *this_, double speed) {
	this_->speed = speed;
	this_->flags |= location_flag_has_speed;
}


/**
 * @brief Sets the validity of a location.
 *
 * See {@link enum attr_position_valid} for possible values and their meanings.
 *
 * @param this_ The location
 * @param valid The validity
 */
void location_set_validity(struct location *this_, enum attr_position_valid valid) {
	this_->valid = valid;
}


/**
 * @brief Destroys a location.
 *
 * @param this_ The location to destroy.
 */
void
location_destroy(struct location *this_) {
	g_free(this_);
}


/**
 * @brief Updates a location by fusing multiple input locations together.
 *
 * This method recalculates the position and sets its members accordingly. It is generally called from
 * the position callback but may also be called by other triggers (events or timers) to extrapolate the
 * current vehicle position.
 *
 * The new location is generated by fusing information from the supplied sources together. For now, this
 * function simply takes the most recent valid fix it received.
 *
 * This function will fill the struct passed as {@code out} with the newly calculated location data and
 * trigger callbacks from the list passed as {@code cbl} when one of the respective attributes changes.
 * The decision whether to trigger a callback is made by comparing old and new values of {@code out}. In
 * order for this to work correctly, {@code out} must hold the last location of the vehicle when this
 * method is called, or zeroed out if no previous location is known.
 *
 * If {@code out} supplies a position, its {@code fix_type}, {@code fix_time}, {@code fixiso8601},
 * {@code valid} and {@code preference} members will be taken from the input locations which were used
 * to determine that position, selecting the best fix type, most recent timestamp, best validity and
 * highest preference level. If no position is supplied, all locations which were used to obtain at
 * least one value of the output location will be searched for this information in the same manner.
 *
 * This function is intended to be called approximately once per second (the refresh rate of most GPS
 * devices found in the market) for each location source, with the number of input locations in the
 * order of magnitude of 1 - 10 (put differently, at most 31 different locations). Any future extensions
 * to this function should consider these parameters.
 *
 * @param in Raw locations (NULL-terminated pointer array)
 * @param out The last calculated location of the vehicle; this struct will receive the updated location
 * @param cbl Callback list of the vehicle; callbacks from this list will be triggered when one of the
 * respective attributes changes
 */
/* FIXME: use Kalman filter for speed, bearing and altitude (requires accuracy for each) */
void
location_update(struct location ** in, struct location * out, struct callback_list * cbl) {
	int i, i_eplev, used;
	int fix_type_changed = 0;					/* Whether the fix type has changed. */
	int qual_changed = 0;						/* Whether position quality (number of sats) has changed. */
	int sats_used_changed = 0;					/* Whether the number of sats used has changed. */
	int coord_geo_changed = 0;					/* Whether the position has changed. */
	int valid_changed = 0;						/* Whether position validity has changed. */
	int geo_eplev = -INT_MAX;					/* Effective preference level for position */
	int speed_eplev = -INT_MAX;					/* Effective preference level for speed */
	int direction_eplev = -INT_MAX;				/* Effective preference level for bearing */
	int height_eplev = -INT_MAX;				/* Effective preference level for altitude */
	int geo_count = 0;							/* Number of eligible positions */
	int direction_count = 0;					/* Number of eligible bearings */
	struct coord_geo_cart cart = {0, 0, 0};		/* Cartesian coordinates used to average between positions (result) */
	struct coord_geo_cart cart_i;				/* Cartesian coordinates used to average between positions (input) */
	navit_float geo_k = 0, geo_p = 0;			/* Weight and covariance for Kalman filter */
	navit_float geo_len;						/* Length of Cartesian vector for positions */
	struct location * tmp = g_new0(struct location, 1);	/* Temporary storage for the new location */
	navit_float weight_i;						/* Weight for current location (inverse of radius) */
	navit_float speed_weight = 0;				/* Total weight for speed */
	navit_float direction_weight;				/* Total weight for bearing */
	navit_float height_weight = 0;				/* Total weight for elevation */
	navit_float dirx = 0, diry = 0;				/* Cartesian coordinates used to average between bearings */
	time_t t;									/* Temporary time for formatting */
	struct tm *tm;								/* Temporary time for formatting */

	dbg(lvl_debug, "enter\n");

	/* First pass: determine best preference level for data and set some metadata */
	for (i = 0; in[i]; i++) {
		i_eplev = get_effective_preference_level(in[i]->preference, in[i]->valid);
		/* ignore if eplev is -INT_MAX */
		if (i_eplev == -INT_MAX)
			continue;
		if (in[i]->flags & location_flag_has_geo) {
			if (i_eplev > geo_eplev) {
				geo_eplev = i_eplev;
				geo_count = 1;
				tmp->fix_type = in[i]->fix_type;
				tmp->fix_time.tv_sec = in[i]->fix_time.tv_sec;
				tmp->fix_time.tv_usec = in[i]->fix_time.tv_usec;
				tmp->valid = in[i]->valid;
				tmp->preference = in[i]->preference;
			} else if (i_eplev == geo_eplev) {
				geo_count++;
				if (in[i]->fix_type > tmp->fix_type)
					tmp->fix_type = in[i]->fix_type;
				if (timercmp(&(in[i]->fix_time), &(tmp->fix_time), >)) {
					tmp->fix_time.tv_sec = in[i]->fix_time.tv_sec;
					tmp->fix_time.tv_usec = in[i]->fix_time.tv_usec;
				}
				if (attr_position_valid_comp(in[i]->valid, tmp->valid) > 0)
					tmp->valid = in[i]->valid;
				if (in[i]->preference > tmp->preference)
					tmp->preference = in[i]->preference;
			}
		}
		if ((i_eplev > speed_eplev) && (in[i]->flags & location_flag_has_speed))
			speed_eplev = i_eplev;
		if (in[i]->flags & location_flag_has_direction) {
			if (i_eplev > direction_eplev) {
				direction_eplev = i_eplev;
				direction_count = 1;
			} else if (i_eplev == direction_eplev)
				direction_count++;
		}
		if ((i_eplev > height_eplev) && (in[i]->flags & location_flag_has_height))
			height_eplev = i_eplev;
	}

	dbg(lvl_debug, "found %d locations, %d of which will be used for position data\n", i, geo_count);

	for (i = 0; in[i]; i++) {
		i_eplev = get_effective_preference_level(in[i]->preference, in[i]->valid);
		/* ignore if eplev is -INT_MAX */
		if (i_eplev == -INT_MAX)
			continue;
		used = 0;
		weight_i = 1.0 / in[i]->radius;
		if ((in[i]->flags & location_flag_has_geo) && (i_eplev == geo_eplev)) {
			dbg(lvl_debug, "using position data from index %d\n", i);
			used = 1;
			if (geo_count == 1) {
				/* Skip expensive averaging if we have just one position */
				tmp->geo.lat = in[i]->geo.lat;
				tmp->geo.lng = in[i]->geo.lng;
				tmp->radius = in[i]->radius;
				tmp->flags |= location_flag_has_geo;
			} else if (geo_count > 1) {
				if (geo_p == 0) {
					/* first round: use first value as initial estimate */
					geo_p = in[i]->radius * in[i]->radius;
					/* For simplification, assume a unit sphere for Cartesian coords */
					transform_geo_to_cart(&(in[i]->geo), 1, 1, &cart);
				} else {
					geo_k = geo_p / (geo_p + in[i]->radius * in[i]->radius);
					geo_p *= (1.0 - geo_k);
					transform_geo_to_cart(&(in[i]->geo), 1, 1, &cart_i);
					cart.x = (1.0 - geo_k) * cart.x + geo_k * cart_i.x;
					cart.y = (1.0 - geo_k) * cart.y + geo_k * cart_i.y;
					cart.z = (1.0 - geo_k) * cart.z + geo_k * cart_i.z;
				}
			}
		}
		if ((in[i]->flags & location_flag_has_speed) && (i_eplev == speed_eplev)) {
			used = 1;
			speed_weight += weight_i;
			tmp->speed += in[i]->speed * weight_i;
		}
		if ((in[i]->flags & location_flag_has_direction) && (i_eplev == direction_eplev)) {
			used = 1;
			if (direction_count == 1) {
				/* Skip averaging if we have just one bearing */
				tmp->direction = in[i]->direction;
				tmp->flags |= location_flag_has_direction;
			} else if (direction_count > 1) {
				/* Sum up Cartesian coords, using weight as radius */
				dirx += weight_i * navit_cos(in[i]->direction);
				diry += weight_i * navit_sin(in[i]->direction);
			}
		}
		if ((in[i]->flags & location_flag_has_height) && (i_eplev == height_eplev)) {
			used = 1;
			height_weight += weight_i;
			tmp->height += in[i]->height * weight_i;
		}
		if (used) {
			if (!geo_count) {
				/* set fix type, time, validity and preference if we had no locations to collect them from */
				if (in[i]->fix_type > tmp->fix_type)
					tmp->fix_type = in[i]->fix_type;
				if (timercmp(&(in[i]->fix_time), &(tmp->fix_time), >)) {
					tmp->fix_time.tv_sec = in[i]->fix_time.tv_sec;
					tmp->fix_time.tv_usec = in[i]->fix_time.tv_usec;
				}
				if (attr_position_valid_comp(in[i]->valid, tmp->valid) > 0)
					tmp->valid = in[i]->valid;
				if (in[i]->preference > tmp->preference)
					tmp->preference = in[i]->preference;
			}
			if (in[i]->flags & location_flag_has_sat_data) {
				tmp->sats = in[i]->sats;
				tmp->sats_used = in[i]->sats_used;
				tmp->flags |= location_flag_has_sat_data;
			}
		}
	}

	/* Note that we're not setting tmp->timeiso8601 here, it will be generated from scratch as we copy
	 * tmp to out. */

	if (geo_count > 1) {
		/* convert Cartesian vector back to lat/lon, using its length as radius */
		geo_len = sqrtf(cart.x * cart.x + cart.y * cart.y + cart.z * cart.z);
		if (geo_len > 0) {
			/* TODO this has a potentially expensive iteration, can we rewrite the func for our special case? */
			transform_cart_to_geo(&cart, geo_len, geo_len, &(tmp->geo));
			tmp->radius = sqrtf(geo_p);
			tmp->flags |= location_flag_has_geo;
		} else
			/* TODO: should we reuse the old position with an appropriate validity in this case? */
			dbg(lvl_error, "can't update position: input positions cancel each other out\n");
	}
	if (speed_weight != 0) {
		tmp->speed /= speed_weight;
		tmp->flags |= location_flag_has_speed;
	}
	if (direction_count > 1) {
		direction_weight = sqrtf(dirx + dirx + diry + diry);
		if (direction_weight > 0) {
			if (diry >= 0)
				tmp->direction = navit_acos(dirx / direction_weight);
			else
				tmp->direction = 360.0 - navit_acos(dirx / direction_weight);
			/* TODO: do we need to normalize the angle to a particular range? */
			tmp->flags |= location_flag_has_direction;
		} else
			/* TODO: should we reuse the old bearing in that case? */
			dbg(lvl_error, "can't update direction: input directions cancel each other out\n");
	}
	if (height_weight != 0) {
		tmp->height /= height_weight;
		tmp->flags |= location_flag_has_height;
	}

	/* copy tmp to out */
	if ((out->geo.lat != tmp->geo.lat) || (out->geo.lng != tmp->geo.lng)) {
		coord_geo_changed = 1;
		out->geo.lat = tmp->geo.lat;
		out->geo.lng = tmp->geo.lng;
	}
	out->speed = tmp->speed;
	out->direction = tmp->direction;
	out->height = tmp->height;
	out->radius = tmp->radius;
	if (out->fix_type != tmp->fix_type) {
		fix_type_changed = 1;
		out->fix_type = tmp->fix_type;
	}
	if ((out->fix_time.tv_sec != tmp->fix_time.tv_sec) || (out->fix_time.tv_usec != tmp->fix_time.tv_usec)) {
		out->fix_time.tv_sec = tmp->fix_time.tv_sec;
		out->fix_time.tv_usec = tmp->fix_time.tv_usec;
		t = out->fix_time.tv_sec;
		tm = gmtime(&t);
		strftime(out->fixiso8601, sizeof(out->fixiso8601), "%Y-%m-%dT%TZ", tm);
	}
	if (out->sats != tmp->sats) {
		qual_changed = 1;
		out->sats = tmp->sats;
	}
	if (out->sats_used != tmp->sats_used) {
		sats_used_changed = 1;
		out->sats_used = tmp->sats_used;
	}
	if (out->valid != tmp->valid) {
		valid_changed = 1;
		out->valid = tmp->valid;
	}
	out->flags = tmp->flags;

	if (valid_changed || fix_type_changed || qual_changed || sats_used_changed || coord_geo_changed) {
		dbg(lvl_debug, "Attributes changed:%s%s%s%s%s\n",
				valid_changed ? " position_valid" : "",
				coord_geo_changed ? " position_coord_geo" : "",
				fix_type_changed ? " position_fix_type ": "",
				qual_changed ? " position_qual" : "",
				sats_used_changed ? " position_sats_used" : "");
	} else {
		dbg(lvl_debug, "No attributes changed\n");
	}

	/* Trigger callbacks only after we've fully updated the location */
	if (valid_changed)
		callback_list_call_attr_0(cbl, attr_position_valid);
	if (out->valid == attr_position_valid_invalid)
		return;
	if (fix_type_changed)
		callback_list_call_attr_0(cbl, attr_position_fix_type);
	if (qual_changed)
		callback_list_call_attr_0(cbl, attr_position_qual);
	if (sats_used_changed)
		callback_list_call_attr_0(cbl, attr_position_sats_used);
	if (coord_geo_changed)
		callback_list_call_attr_0(cbl, attr_position_coord_geo);
	dbg(lvl_debug, "lat %f lon %f time %s\n", out->geo.lat, out->geo.lng, out->fixiso8601);
}

/*
 * @brief Init location code.
 *
 * Currently it's a dummy function, needed only to reference location code from the navit core so it's linked and its functions are
 * available to navit modules.
 *
 */
void 
location_init(void)
{
}
