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

#include <glib.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "debug.h"
#include "coord.h"
#include "item.h"
#include "xmlconfig.h"
#include "navit.h"
#include "map.h"
#include "navit.h"
#include "route.h"
#include "navigation.h"
#include "callback.h"
#include "transform.h"
#include "plugin.h"
#include "vehicle.h"
#include "vehicleprofile.h"
#include "roadprofile.h"
#include "location.h"
#include "event.h"
#include "util.h"

/** The presumed accuracy for the demo vehicle (3 meters, approximately one lane width) */
#define DEMO_ACCURACY 3

/**
 * The presumed speed for off-road segments.
 * This is a hardcoded value as these segments do not have a corresponding street item, which is a
 * prerequisite for inferring speed information.
 */
#define OFFROAD_SPEED 5

struct vehicle_priv {
	int interval;
	int position_set;           /**< True if the current position was set manually **/
	struct callback_list *cbl;
	struct location *location;  /**< The location of the vehicle.
	                                 For the demo vehicle the location is periodically updated based on
	                                 where the vehicle would be if it had followed the route from its
	                                 last location during the time elapsed. **/
	double altitude;           /**< Temporary value for returning altitude in an attr, not set until attr is requested **/
	double bearing;            /**< Temporary value for returning bearing in an attr, not set until attr is requested **/
	/* FIXME should we do something similar for fix_iso8601? */
	struct coord_geo position; /**< Temporary value for returning position in an attr, not set until attr is requested **/
	double position_accuracy;  /**< Temporary value for returning positional accuracy in an attr, not set until attr is requested **/
	double speed;              /**< Temporary value for returning speed in an attr, not set until attr is requested **/
	struct navit *navit;
	struct route *route;
	struct coord last;
	double config_speed;
	struct callback *timer_callback;
	struct event_timeout *timer;
	struct callback *nav_set_cb;  /**< Callback to call when the navigation object changes **/
	struct callback *nav_done_cb; /**< Callback to call after a new route has been calculated **/
	char *timep;
	char *nmea;

};

static void
vehicle_demo_destroy(struct vehicle_priv *priv)
{
	if (priv->timer)
		event_remove_timeout(priv->timer);
	callback_destroy(priv->timer_callback);
	g_free(priv->timep);
	location_destroy(priv->location);
	g_free(priv);
}

static void
nmea_chksum(char *nmea)
{
	int i;
	if (nmea && strlen(nmea) > 3) {
		unsigned char csum=0;
		for (i = 1 ; i < strlen(nmea)-4 ; i++) 
			csum^=(unsigned char)(nmea[i]);
		sprintf(nmea+strlen(nmea)-3,"%02X\n",csum);
	}
}

static int
vehicle_demo_position_attr_get(struct vehicle_priv *priv,
			       enum attr_type type, struct attr *attr)
{
	char ns='N',ew='E',*timep,*rmc,*gga;
	int hr,min,sec,year,mon,day;
	double lat,lng;
	switch (type) {
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
	case attr_position_coord_geo:
		location_get_position(priv->location, &priv->position);
		attr->u.coord_geo = &priv->position;
		break;
	case attr_position_time_iso8601:
#if 0
		attr->u.str = location_get_fixiso8601(priv->location);
		break;
#endif
		g_free(priv->timep);
		priv->timep=current_to_iso8601();
		attr->u.str=priv->timep;
		break;
       case attr_position_fix_type:
                attr->u.num = 2;
                break;
        case attr_position_sats_used:
                attr->u.num = 9;
                break;
	case attr_position_nmea:
		location_get_position(priv->location, &priv->position);
		lat = priv->position.lat;
		if (lat < 0) {
			lat=-lat;
			ns='S';
		}
		lng = priv->position.lng;
		if (lng < 0) {
			lng=-lng;
			ew='W';
		}
		timep=current_to_iso8601();
		/* FIXME timep = location_get_fix_iso8601(priv->location); */
		sscanf(timep,"%d-%d-%dT%d:%d:%d",&year,&mon,&day,&hr,&min,&sec);
		g_free(timep);
		gga=g_strdup_printf("$GPGGA,%02d%02d%02d,%02.0f%07.4f,%c,%03.0f%07.4f,%c,1,08,2.5,0,M,,,,0000*  \n",hr,min,sec,floor(lat),(lat-floor(lat))*60.0,ns,floor(lng),(lng-floor(lng))*60,ew);
		nmea_chksum(gga);
		rmc=g_strdup_printf("$GPRMC,%02d%02d%02d,A,%02.0f%07.4f,%c,%03.0f%07.4f,%c,%3.1f,%3.1f,%02d%02d%02d,,*  \n",hr,min,sec,floor(lat),(lat-floor(lat))*60.0,ns,floor(lng),(lng-floor(lng))*60,ew,location_get_speed(priv->location)/1.852,(double)location_get_bearing(priv->location),day,mon,year%100);
		nmea_chksum(rmc);
		g_free(priv->nmea);
		priv->nmea=g_strdup_printf("%s%s",gga,rmc);
		g_free(gga);
		g_free(rmc);
		attr->u.str=priv->nmea;
		break;
	case attr_position_valid:
		attr->u.num = location_get_validity(priv->location);
		break;
	default:
		return 0;
	}
	attr->type = type;
	return 1;
}


static int
vehicle_demo_set_attr_do(struct vehicle_priv *priv, struct attr *attr)
{
	struct timeval tv;
	struct navigation *nav;

	dbg(lvl_debug,"enter, attribute %s\n",attr_to_name(attr->type));

	switch(attr->type) {
	case attr_navit:
		priv->navit = attr->u.navit;
		break;
	case attr_route:
		priv->route = attr->u.route;
		break;
	case attr_speed:
		priv->config_speed=attr->u.num;
		break;
	case attr_interval:
		priv->interval=attr->u.num;
		if (priv->timer)
			event_remove_timeout(priv->timer);
		priv->timer=event_add_timeout(priv->interval, 1, priv->timer_callback);
		break;
	case attr_position_coord_geo:
		gettimeofday(&tv, NULL);
		location_set_position(priv->location, attr->u.coord_geo);
		location_set_position_accuracy(priv->location, DEMO_ACCURACY);
		location_set_fix_time(priv->location, &tv);
		if (location_get_validity(priv->location) != attr_position_valid_valid) {
			location_set_validity(priv->location, attr_position_valid_valid);
			callback_list_call_attr_0(priv->cbl, attr_position_valid);
		}
		priv->position_set=1;
		dbg(lvl_debug,"position_set %f %f %s\n", attr->u.coord_geo->lat, attr->u.coord_geo->lng, location_get_fixiso8601(priv->location));
		callback_list_call_attr_0(priv->cbl, attr_position_coord_geo);
		break;
	case attr_profilename:
	case attr_source:
	case attr_name:
		// Ignore; used by Navit's infrastructure, but not relevant for this vehicle.
		break;
	default:
		dbg(lvl_error,"unsupported attribute %s\n",attr_to_name(attr->type));
		return 0;
	}
	return 1;
}

static int
vehicle_demo_set_attr(struct vehicle_priv *priv, struct attr *attr)
{
	return vehicle_demo_set_attr_do(priv, attr);
}

struct vehicle_methods vehicle_demo_methods = {
	vehicle_demo_destroy,
	vehicle_demo_position_attr_get,
	vehicle_demo_set_attr,
};

static void
vehicle_demo_timer(struct vehicle_priv *priv)
{
	struct coord c, c2, pos, ci;
	struct coord_geo geo;
	int slen;							/* Length of current segment */
	int dx, dy;							/* Two-dimensional length of current segment */
	struct route *route=NULL;
	struct map *route_map=NULL;
	struct map_rect *mr=NULL;
	struct item *item=NULL;
	struct timeval tv_old;				/* timestamp for previous location */
	struct timeval tv_new;				/* timestamp for new location */
	int timespan;						/* timespan for which to simulate movement, in tenths of seconds */
	int stime;							/* time needed to follow entire length of current segment, in tenths of seconds */
	struct navigation *nav = NULL;      /* navigation object */
	struct attr status_attr;			/* route status attr */
	struct vehicleprofile *vp = NULL;	/* vehicleprofile to determine default speed */
	struct attr sitem_attr; 			/* street_item attr */
	struct item *sitem = NULL;			/* street item associated with current route map item */
	struct roadprofile *rp = NULL;		/* roadprofile for sitem */
	struct attr maxspeed_attr;			/* maxspeed attr of sitem */
	double item_speed;					/* maxspeed of sitem */
	double vehicle_speed;				/* vehicle speed determined from roadprofile */
	double speed = priv->config_speed;  /* simulated speed */

	if (priv->navit) {
		nav = navit_get_navigation(priv->navit);
		vp = navit_get_vehicleprofile(priv->navit);
	}
	location_get_fix_time(priv->location, &tv_old);
	if (!tv_old.tv_sec && !tv_old.tv_usec) {
		/* invalid timestamp (most likely because no position has ever been set), cannot calculate timespan */
		return;
	}

	/* default in case we can't (yet) retrieve the status attribute, mostly cosmetic */
	status_attr.u.num = status_no_destination;
	if (!nav || !navigation_get_attr(nav, attr_nav_status, &status_attr, NULL) || (status_attr.u.num != status_routing)) {
		/* not yet initialized, not routing or still calculating */
		dbg(lvl_debug, "no route or route not ready (nav=%p, status %d), exiting\n", nav, status_attr.u.num);

		/* make sure the position's timestamp will be reset (see below) when starting a new route */
		priv->position_set = 1;
		return;
	}

	gettimeofday(&tv_new, NULL);

	if (priv->position_set) {
		/* The timespan since the last fix includes the calculation time for the route, which can cause
		 * a huge leap at the start of a long/complex route. To avoid this, reset the timestamp.
		 * Position updates will begin with the subsequent call to this function.
		 */
		location_set_fix_time(priv->location, &tv_new);
		priv->position_set = 0;
		return;
	}

	/* calculate difference in 1/10 s, rounding microseconds */
	timespan = (tv_new.tv_sec - tv_old.tv_sec) * 10 + (tv_new.tv_usec - tv_old.tv_usec + 50000) / 100000;
	dbg(lvl_debug, "timespan=%d (%d.%d - %d.%d)\n", timespan, tv_new.tv_sec, tv_new.tv_usec, tv_old.tv_sec, tv_old.tv_usec);
	if (timespan <= 0) {
		dbg(lvl_error, "last location has an invalid timestamp, aborting\n");
		return;
	}
	dbg(lvl_debug, "###### Entering simulation loop\n");
	if (priv->route)
		route=priv->route;
	else if (priv->navit) 
		route=navit_get_route(priv->navit);
	if (route)
		route_map=route_get_map(route);
	if (route_map)
		mr=map_rect_new(route_map, NULL);
	if (mr)
		item=map_rect_get_item(mr);
	if (item) { /* TODO debug code */
		if (item_coord_get(item, &c, 1))
			transform_to_geo(projection_mg, &c, &geo);
		else {
			geo.lat=360;
			geo.lng=360;
		}
		dbg(lvl_debug, "first item (%d, %d), type=%s, lat=%.6f, lng=%.6f\n", item->id_hi, item->id_lo, item_to_name(item->type), geo.lat, geo.lng);
	}
	if (item && item->type == type_route_start) {
		dbg(lvl_debug, "discarding item (%d, %d), type=%s\n", item->id_hi, item->id_lo, item_to_name(item->type));
		item=map_rect_get_item(mr);
	}
	while(item && item->type!=type_street_route) {
		dbg(lvl_debug, "discarding item (%d, %d), type=%s\n", item->id_hi, item->id_lo, item_to_name(item->type));
		item=map_rect_get_item(mr);
	}
	if (item && item_coord_get(item, &pos, 1)) {
		priv->position_set=0;
		dbg(lvl_debug, "current pos=0x%x,0x%x\n", pos.x, pos.y);
		dbg(lvl_debug, "last pos=0x%x,0x%x\n", priv->last.x, priv->last.y);
		if (priv->last.x == pos.x && priv->last.y == pos.y) {
			dbg(lvl_warning, "endless loop\n");
		}
		priv->last = pos;
		while (item) {
			if (!item_coord_get(item, &c, 1)) {
				dbg(lvl_debug, "discarding item (%d, %d), type=%s (no coords)\n", item->id_hi, item->id_lo, item_to_name(item->type));
				item=map_rect_get_item(mr);
				continue;
			}

			/* debug code */
			if (item_attr_get(item, attr_street_item, &sitem_attr))
				sitem = sitem_attr.u.item;
			else
				sitem = NULL;
			transform_to_geo(projection_mg, &c, &geo);
			dbg(lvl_debug, "examining item (%d, %d), type=%s, sitem=%p, start at (lat=%.6f, lng=%.6f)\n", item->id_hi, item->id_lo, item_to_name(item->type), sitem, geo.lat, geo.lng);

			if (!priv->config_speed) {
				/* if speed is not fixed, determine speed for the segment */
				vehicle_speed = 0;
				if (vp && (vp->maxspeed_handling != maxspeed_ignore)) {
					if (item_attr_get(item, attr_street_item, &sitem_attr)) {
						sitem = sitem_attr.u.item;
						if (sitem) {
							rp = vehicleprofile_get_roadprofile(vp, sitem->type);
							if (rp)
								vehicle_speed = rp->route_weight;
						} else {
							dbg(lvl_warning, "street item is NULL\n");
						}
					} else {
						dbg(lvl_warning, "could not get street item\n");
					}
				}
				if (item_attr_get(item, attr_maxspeed, &maxspeed_attr)) {
					item_speed = maxspeed_attr.u.num;
				} else
					item_speed = 0;
				if (!item_speed)
					speed = vehicle_speed;
				else if (vp->maxspeed_handling == maxspeed_enforce)
					speed = item_speed;
				else
					speed = (vehicle_speed < item_speed) ? vehicle_speed : item_speed;
				if (!speed)
					speed = OFFROAD_SPEED;
				dbg(lvl_debug, "speed=%.0f: %s, item_speed=%.0f, vehicle_speed=%.0f, vp=%p, rp=%p, vp->maxspeed_handling=%d\n",
						speed, sitem ? item_to_name(sitem->type) : "(none)", item_speed, vehicle_speed, vp, rp, vp?vp->maxspeed_handling:0);
			}

			dbg(lvl_debug, "next pos=0x%x,0x%x\n", c.x, c.y);
			slen = transform_distance(projection_mg, &pos, &c);
			stime = slen * 36 / speed;
			dbg(lvl_debug, "timespan=%d stime=%d slen=%d speed=%.0f\n", timespan, stime, slen, speed);
			if (stime < timespan) {
				timespan -= stime;
				pos = c;
			} else {
				if (item_coord_get(item, &c2, 1) || map_rect_get_item(mr)) {
					dx = c.x - pos.x;
					dy = c.y - pos.y;

					/* The following two actually use (len_traveled / slen) as a factor. Since we never
					 * calculate len_traveled, we use (timespan / stime) instead, which is directly
					 * proportional to the above. */
					ci.x = pos.x + dx * timespan / stime;
					ci.y = pos.y + dy * timespan / stime;

					location_set_bearing(priv->location,
					    transform_get_angle_delta(&pos, &c, 0));
					location_set_speed(priv->location, speed);
				} else {
					ci.x = pos.x;
					ci.y = pos.y;
					location_set_speed(priv->location, 0);
					dbg(lvl_debug,"destination reached\n");
				}
				transform_to_geo(projection_mg, &ci, &geo);
				dbg(lvl_debug, "ci=0x%x,0x%x lat=%.6f lng=%.6f\n", ci.x, ci.y, geo.lat, geo.lng);
				location_set_position(priv->location, &geo);
				location_set_position_accuracy(priv->location, DEMO_ACCURACY);
				location_set_fix_time(priv->location, &tv_new);
				if (location_get_validity(priv->location) != attr_position_valid_valid) {
					location_set_validity(priv->location, attr_position_valid_valid);
					callback_list_call_attr_0(priv->cbl, attr_position_valid);
				}
				callback_list_call_attr_0(priv->cbl, attr_position_coord_geo);
				break;
			}
		}
	}
	if (mr)
		map_rect_destroy(mr);
}


static struct vehicle_priv *
vehicle_demo_new(struct vehicle_methods
		 *meth, struct callback_list
		 *cbl, struct attr **attrs)
{
	struct vehicle_priv *ret;

	dbg(lvl_debug, "enter\n");
	ret = g_new0(struct vehicle_priv, 1);
	ret->cbl = cbl;
	ret->interval=1000;
	ret->timer_callback=callback_new_1(callback_cast(vehicle_demo_timer), ret);
	ret->location = location_new();
	ret->nav_set_cb = NULL;
	ret->nav_done_cb = NULL;
	location_set_validity(ret->location, attr_position_valid_invalid);
	location_set_preference(ret->location, preference_high);
	*meth = vehicle_demo_methods;
	while (attrs && *attrs) 
		vehicle_demo_set_attr_do(ret, *attrs++);
	if (!ret->timer)
		ret->timer=event_add_timeout(ret->interval, 1, ret->timer_callback);
	return ret;
}

void
plugin_init(void)
{
	dbg(lvl_debug, "enter\n");
	plugin_register_vehicle_type("demo", vehicle_demo_new);
}
