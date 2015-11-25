/**
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

#ifndef NAVIT_VEHICLE_H
#define NAVIT_VEHICLE_H

#ifdef __cplusplus
extern "C" {
#endif
#include "coord.h"

struct point;
struct vehicle_priv;

struct vehicle_methods {
	void (*destroy)(struct vehicle_priv *priv);
	int (*position_attr_get)(struct vehicle_priv *priv, enum attr_type type, struct attr *attr);
	int (*set_attr)(struct vehicle_priv *priv, struct attr *attr);
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


/* prototypes */
enum attr_type;
struct attr;
struct attr_iter;
struct cursor;
struct graphics;
struct point;
struct vehicle;
struct vehicle *vehicle_new(struct attr *parent, struct attr **attrs);
void vehicle_destroy(struct vehicle *this_);
struct attr_iter *vehicle_attr_iter_new(void);
void vehicle_attr_iter_destroy(struct attr_iter *iter);
int vehicle_get_attr(struct vehicle *this_, enum attr_type type, struct attr *attr, struct attr_iter *iter);
int vehicle_set_attr(struct vehicle *this_, struct attr *attr);
int vehicle_add_attr(struct vehicle *this_, struct attr *attr);
int vehicle_remove_attr(struct vehicle *this_, struct attr *attr);
void vehicle_set_cursor(struct vehicle *this_, struct cursor *cursor, int overwrite);
void vehicle_draw(struct vehicle *this_, struct graphics *gra, struct point *pnt, int angle, int speed);
int vehicle_get_cursor_data(struct vehicle *this_, struct point *pnt, int *angle, int *speed);
void vehicle_log_gpx_add_tag(char *tag, char **logstr);
struct vehicle * vehicle_ref(struct vehicle *this_);
void vehicle_unref(struct vehicle *this_);
void vehicle_update_position(struct location ** in, struct location * out, struct callback_list * cbl);
/* end of prototypes */

#ifdef __cplusplus
}
#endif

#endif

