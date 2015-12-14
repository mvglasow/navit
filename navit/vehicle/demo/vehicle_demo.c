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
#include "navit.h"
#include "map.h"
#include "route.h"
#include "callback.h"
#include "transform.h"
#include "plugin.h"
#include "vehicle.h"
#include "location.h"
#include "event.h"
#include "util.h"

struct vehicle_priv {
	int interval;
	int position_set;
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
		location_set_position(priv->location, attr->u.coord_geo);
		if (location_get_validity(priv->location) != attr_position_valid_valid) {
			location_set_validity(priv->location, attr_position_valid_valid);
			callback_list_call_attr_0(priv->cbl, attr_position_valid);
		}
		priv->position_set=1;
		dbg(lvl_debug,"position_set %f %f\n", attr->u.coord_geo->lat, attr->u.coord_geo->lng);
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
	/* FIXME set missing (but required) members of location:
	 * radius
	 * fix_time and fixiso8601
	 * flags
	 * preference
	 */
	struct coord c, c2, pos, ci;
	struct coord_geo geo;
	int slen, len, dx, dy;
	struct route *route=NULL;
	struct map *route_map=NULL;
	struct map_rect *mr=NULL;
	struct item *item=NULL;

	len = (priv->config_speed * priv->interval / 1000)/ 3.6;
	dbg(lvl_debug, "###### Entering simulation loop\n");
	if (!priv->config_speed)
		return;
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
	if (item && item->type == type_route_start)
		item=map_rect_get_item(mr);
	while(item && item->type!=type_street_route)
		item=map_rect_get_item(mr);
	if (item && item_coord_get(item, &pos, 1)) {
		priv->position_set=0;
		dbg(lvl_debug, "current pos=0x%x,0x%x\n", pos.x, pos.y);
		dbg(lvl_debug, "last pos=0x%x,0x%x\n", priv->last.x, priv->last.y);
		if (priv->last.x == pos.x && priv->last.y == pos.y) {
			dbg(lvl_warning, "endless loop\n");
		}
		priv->last = pos;
		while (item && priv->config_speed) {
			if (!item_coord_get(item, &c, 1)) {
				item=map_rect_get_item(mr);
				continue;
			}
			dbg(lvl_debug, "next pos=0x%x,0x%x\n", c.x, c.y);
			slen = transform_distance(projection_mg, &pos, &c);
			dbg(lvl_debug, "len=%d slen=%d\n", len, slen);
			if (slen < len) {
				len -= slen;
				pos = c;
			} else {
				if (item_coord_get(item, &c2, 1) || map_rect_get_item(mr)) {
					dx = c.x - pos.x;
					dy = c.y - pos.y;
					ci.x = pos.x + dx * len / slen;
					ci.y = pos.y + dy * len / slen;
					location_set_bearing(priv->location,
					    transform_get_angle_delta(&pos, &c, 0));
					location_set_speed(priv->location, priv->config_speed);
				} else {
					ci.x = pos.x;
					ci.y = pos.y;
					/* FIXME or should we clear the speed instead? */
					location_set_speed(&priv->location, 0);
					dbg(lvl_debug,"destination reached\n");
				}
				dbg(lvl_debug, "ci=0x%x,0x%x\n", ci.x, ci.y);
				transform_to_geo(projection_mg, &ci, &geo);
				location_set_position(priv->location, &geo);
				if (location_get_validity(priv->location) != attr_position_valid_valid) {
					location_set_validity(priv->location, attr_position_valid_valid);
					callback_list_call_attr_0(priv->cbl, attr_position_valid);
				}
				callback_list_call_attr_0(priv->cbl, attr_position_coord_geo);
				break;
			}
		}
	} else {
		if (priv->position_set) 
			callback_list_call_attr_0(priv->cbl, attr_position_coord_geo);
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
	ret->config_speed=40;
	ret->timer_callback=callback_new_1(callback_cast(vehicle_demo_timer), ret);
	ret->location = location_new();
	location_set_validity(ret->location, attr_position_valid_invalid);
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
