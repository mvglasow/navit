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


/* prototypes */
void location_init(void);
struct location * location_new();
void location_destroy(struct location *this_);

void location_clear_altitude(struct location *this_);
void location_clear_bearing(struct location *this_);
void location_clear_position(struct location *this_);
void location_clear_position_accuracy(struct location *this_);
void location_clear_sat_data(struct location *this_);
void location_clear_speed(struct location *this_);

double location_get_altitude(struct location *this_);
double location_get_bearing(struct location *this_);
void location_get_fix_time(struct location *this_, struct timeval *time);
int location_get_fix_type(struct location *this_);
char *location_get_fixiso8601(struct location *this_);
void location_get_position(struct location *this_, struct coord_geo *position);
int location_get_position_accuracy(struct location *this_);
int location_get_preference(struct location *this_);
int location_get_sats(struct location *this_);
int location_get_sats_used(struct location *this_);
double location_get_speed(struct location *this_);
int location_get_validity(struct location *this_);

int location_has_altitude(struct location *this_);
int location_has_bearing(struct location *this_);
int location_has_position(struct location *this_);
int location_has_position_accuracy(struct location *this_);
int location_has_sat_data(struct location *this_);
int location_has_speed(struct location *this_);

void location_set_altitude(struct location *this_, double altitude);
void location_set_bearing(struct location *this_, double bearing);
void location_set_fix_time(struct location *this_, struct timeval *time);
void location_set_fix_type(struct location *this_, int type);
void location_set_position(struct location *this_, struct coord_geo *position);
void location_set_position_accuracy(struct location *this_, int accuracy);
void location_set_preference(struct location *this_, enum preference preference);
void location_set_sat_data(struct location *this_, int sats, int sats_used);
void location_set_speed(struct location *this_, double speed);
void location_set_validity(struct location *this_, enum attr_position_valid valid);

int location_extrapolate(struct location *in, struct location *out, struct navit *navit, double aspeed, int flags);
void location_update(struct location ** in, struct location * out, struct callback_list * cbl);
/* end of prototypes */


#ifdef __cplusplus
}
#endif

#endif /* NAVIT_LOCATION_H_ */
