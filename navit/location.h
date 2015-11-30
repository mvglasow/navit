/*
 * Navit, a modular navigation system.
 * Copyright (C) 2005-2009 Navit Team
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

/**
 * @file location.h
 *
 * @brief Interface for logic to work with location data.
 *
 * A location contains data describing the movement of the vehicle, along with associated metadata.
 * It may have been obtained directly from a source such as GPS or nearby networks, or calculated by
 * various other means.
 *
 * @date 2015-11-30
 * @author mvglasow
 */

#ifndef NAVIT_LOCATION_H_
#define NAVIT_LOCATION_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <sys/time.h>
#include "coord.h"

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
 * The preference level for raw locations.
 *
 * The preference level essentially represents how much Navit "trusts" locations from different
 * providers. When fusing multiple raw locations, data from more "trustworthy" providers is given
 * preference over others.
 *
 * Generally, when multiple raw locations of different preference levels are fused together, only data
 * of the highest preference level will be used and corresponding data of lower levels will be ignored.
 * Exceptions apply when the locations have a different validity, in which case the preference level and
 * validity are considered together and may level each other out.
 *
 * Example: Of two raw locations, the first one has a high preference level and supplies a position, but
 * no bearing. The second one has a lower preference level and supplies both a location and a bearing.
 * Both have the same validity.  When the locations are fused together, the position of the first one is
 * used while the position of the second location is ignored. However, the final location will use the
 * bearing of the second location as no bearing of a higher preference level is available.
 *
 * Second example: The first of two raw locations has a high preference level but was extrapolated. The
 * second one was measured directly but has a lower preference level. The fuser may consider these two
 * locations equal and use their weighted average for the final location.
 */
enum preference {
	preference_low = 0,
	preference_medium = 1,
	preference_high = 2,
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


/* prototypes */
void vehicle_update_position(struct location ** in, struct location * out, struct callback_list * cbl);
/* end of prototypes */


#ifdef __cplusplus
}
#endif

#endif /* NAVIT_LOCATION_H_ */
