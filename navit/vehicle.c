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
 
/** @file vehicle.c
 * @brief Generic components of the vehicle object.
 * 
 * This file implements the generic vehicle interface, i.e. everything which is
 * not specific to a single data source.
 *
 * @author Navit Team
 * @date 2005-2014
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <glib.h>
#include "config.h"
#include "debug.h"
#include "coord.h"
#include "item.h"
#include "xmlconfig.h"
#include "log.h"
#include "plugin.h"
#include "transform.h"
#include "util.h"
#include "event.h"
#include "coord.h"
#include "transform.h"
#include "projection.h"
#include "point.h"
#include "graphics.h"
#include "callback.h"
#include "color.h"
#include "layout.h"
#include "vehicle.h"
#include "navit_nls.h"

struct vehicle {
	NAVIT_OBJECT
	struct vehicle_methods meth;
	struct vehicle_priv *priv;
	struct callback_list *cbl;
	struct log *nmea_log, *gpx_log;
	char *gpx_desc;

	// cursor
	struct cursor *cursor;
	int cursor_fixed;
	struct callback *animate_callback;
	struct event_timeout *animate_timer;
	struct point cursor_pnt;
	struct graphics *gra;
	struct graphics_gc *bg;
	struct transformation *trans;
	int angle;
	int speed;
	int sequence;
	GHashTable *log_to_cb;
};

struct object_func vehicle_func;

static void vehicle_set_default_name(struct vehicle *this);
static void vehicle_draw_do(struct vehicle *this_);
static void vehicle_log_nmea(struct vehicle *this_, struct log *log);
static void vehicle_log_gpx(struct vehicle *this_, struct log *log);
static void vehicle_log_textfile(struct vehicle *this_, struct log *log);
static void vehicle_log_binfile(struct vehicle *this_, struct log *log);
static int vehicle_add_log(struct vehicle *this_, struct log *log);



/**
 * @brief Creates a new vehicle
 *
 * @param parent
 * @param attrs Points to a null-terminated array of pointers to the attributes
 * for the new vehicle type.
 *
 * @return The newly created vehicle object
 */
struct vehicle *
vehicle_new(struct attr *parent, struct attr **attrs)
{
	struct vehicle *this_;
	struct attr *source;
	struct vehicle_priv *(*vehicletype_new) (struct vehicle_methods *
						 meth,
						 struct callback_list *
						 cbl,
						 struct attr ** attrs);
	char *type, *colon;
	struct pcoord center;

	dbg(lvl_debug, "enter\n");
	source = attr_search(attrs, NULL, attr_source);
	if (!source) {
		dbg(lvl_error, "incomplete vehicle definition: missing attribute 'source'\n");
		return NULL;
	}

	type = g_strdup(source->u.str);
	colon = strchr(type, ':');
	if (colon)
		*colon = '\0';
	dbg(lvl_debug, "source='%s' type='%s'\n", source->u.str, type);

	vehicletype_new = plugin_get_vehicle_type(type);
	if (!vehicletype_new) {
		dbg(lvl_error, "invalid source '%s': unknown type '%s'\n", source->u.str, type);
		g_free(type);
		return NULL;
	}
	g_free(type);
	this_ = g_new0(struct vehicle, 1);
	this_->func=&vehicle_func;
	navit_object_ref((struct navit_object *)this_);
	this_->cbl = callback_list_new();
	this_->priv = vehicletype_new(&this_->meth, this_->cbl, attrs);
	if (!this_->priv) {
		dbg(lvl_error, "vehicletype_new failed\n");
		callback_list_destroy(this_->cbl);
		g_free(this_);
		return NULL;
	}
	this_->attrs=attr_list_dup(attrs);

	center.pro=projection_screen;
	center.x=0;
	center.y=0;
	this_->trans=transform_new(&center, 16, 0);
	vehicle_set_default_name(this_);

	dbg(lvl_debug, "leave\n");
	this_->log_to_cb=g_hash_table_new(NULL,NULL);
	return this_;
}

/**
 * @brief Destroys a vehicle
 * 
 * @param this_ The vehicle to destroy
 */
void
vehicle_destroy(struct vehicle *this_)
{
	dbg(lvl_debug,"enter\n");
	if (this_->animate_callback) {
		callback_destroy(this_->animate_callback);
		event_remove_timeout(this_->animate_timer);
	}
	transform_destroy(this_->trans);
	this_->meth.destroy(this_->priv);
	callback_list_destroy(this_->cbl);
	attr_list_free(this_->attrs);
	if (this_->bg)
		graphics_gc_destroy(this_->bg);
	if (this_->gra)
		graphics_free(this_->gra);
	g_free(this_);
}

/**
 * Creates an attribute iterator to be used with vehicles
 */
struct attr_iter *
vehicle_attr_iter_new(void)
{
	return (struct attr_iter *)g_new0(void *,1);
}

/**
 * Destroys a vehicle attribute iterator
 *
 * @param iter a vehicle attr_iter
 */
void
vehicle_attr_iter_destroy(struct attr_iter *iter)
{
	g_free(iter);
}



/**
 * Generic get function
 *
 * @param this_ Pointer to a vehicle structure
 * @param type The attribute type to look for
 * @param attr Pointer to a {@code struct attr} to store the attribute
 * @param iter A vehicle attr_iter. This is only used for generic attributes; for attributes specific to the vehicle object it is ignored.
 * @return True for success, false for failure
 */
int
vehicle_get_attr(struct vehicle *this_, enum attr_type type, struct attr *attr, struct attr_iter *iter)
{
	int ret;
	if (type == attr_log_gpx_desc) {
		attr->u.str = this_->gpx_desc;
		return 1;
	}
	if (this_->meth.position_attr_get) {
		ret=this_->meth.position_attr_get(this_->priv, type, attr);
		if (ret)
			return ret;
	}
	return attr_generic_get_attr(this_->attrs, NULL, type, attr, iter);
}

/**
 * Generic set function
 *
 * @param this_ A vehicle
 * @param attr The attribute to set
 * @return False on success, true on failure
 */
int
vehicle_set_attr(struct vehicle *this_, struct attr *attr)
{
	int ret=1;
	if (attr->type == attr_log_gpx_desc) {
		g_free(this_->gpx_desc);
		this_->gpx_desc = g_strdup(attr->u.str);
	} else if (this_->meth.set_attr)
		ret=this_->meth.set_attr(this_->priv, attr);
	/* attr_profilename probably is never used by vehicle itself but it's used to control the
	  routing engine. So any vehicle should allow to set and read it. */
	if(attr->type == attr_profilename)
		ret=1;
	if (ret == 1 && attr->type != attr_navit && attr->type != attr_pdl_gps_update)
		this_->attrs=attr_generic_set_attr(this_->attrs, attr);
	return ret != 0;
}

/**
 * Generic add function
 *
 * @param this_ A vehicle
 * @param attr The attribute to add
 *
 * @return true if the attribute was added, false if not.
 */
int
vehicle_add_attr(struct vehicle *this_, struct attr *attr)
{
	int ret=1;
	switch (attr->type) {
	case attr_callback:
		callback_list_add(this_->cbl, attr->u.callback);
		break;
	case attr_log:
		ret=vehicle_add_log(this_, attr->u.log);
		break;
	// currently supporting oldstyle cursor config.
	case attr_cursor:
		this_->cursor_fixed=1;
		vehicle_set_cursor(this_, attr->u.cursor, 1);
		break;
	default:
		break;
	}
	if (ret)
		this_->attrs=attr_generic_add_attr(this_->attrs, attr);
	return ret;
}

/**
 * @brief Generic remove function.
 *
 * Used to remove a callback from the vehicle.
 * @param this_ A vehicle
 * @param attr
 */
int
vehicle_remove_attr(struct vehicle *this_, struct attr *attr)
{
	struct callback *cb;
	switch (attr->type) {
	case attr_callback:
		callback_list_remove(this_->cbl, attr->u.callback);
		break;
	case attr_log:
		cb=g_hash_table_lookup(this_->log_to_cb, attr->u.log);
		if (!cb)
			return 0;
		g_hash_table_remove(this_->log_to_cb, attr->u.log);
		callback_list_remove(this_->cbl, cb);
		break;
	default:
		this_->attrs=attr_generic_remove_attr(this_->attrs, attr);
		return 0;
	}
	return 1;
}



/**
 * Sets the cursor of a vehicle.
 *
 * @param this_ A vehicle
 * @param cursor A cursor
 * @author Ralph Sennhauser (10/2009)
 */ 
void
vehicle_set_cursor(struct vehicle *this_, struct cursor *cursor, int overwrite)
{
	struct point sc;
	if (this_->cursor_fixed && !overwrite)
		return;
	if (this_->animate_callback) {
		event_remove_timeout(this_->animate_timer);
		this_->animate_timer=NULL;		// dangling pointer! prevent double freeing.
		callback_destroy(this_->animate_callback);
		this_->animate_callback=NULL;	// dangling pointer! prevent double freeing.
	}
	if (cursor && cursor->interval) {
		this_->animate_callback=callback_new_2(callback_cast(vehicle_draw_do), this_, 0);
		this_->animate_timer=event_add_timeout(cursor->interval, 1, this_->animate_callback);
	}

	if (cursor && this_->gra && this_->cursor) {
		this_->cursor_pnt.x+=(this_->cursor->w - cursor->w)/2;
		this_->cursor_pnt.y+=(this_->cursor->h - cursor->h)/2;
		graphics_overlay_resize(this_->gra, &this_->cursor_pnt, cursor->w, cursor->h, 0);
	}

	if (cursor) { 
		sc.x=cursor->w/2;
		sc.y=cursor->h/2;
		if (!this_->cursor && this_->gra)
			graphics_overlay_disable(this_->gra, 0);
	} else {
		sc.x=sc.y=0;
		if (this_->cursor && this_->gra)
			graphics_overlay_disable(this_->gra, 1);
	}
	transform_set_screen_center(this_->trans, &sc);

	this_->cursor=cursor;
}

/**
 * Draws a vehicle on top of a graphics.
 *
 * @param this_ The vehicle
 * @param gra The graphics
 * @param pnt Screen coordinates of the vehicle.
 * @param angle The angle relative to the map.
 * @param speed The speed of the vehicle.
 */
void
vehicle_draw(struct vehicle *this_, struct graphics *gra, struct point *pnt, int angle, int speed)
{
	if (angle < 0)
		angle+=360;
	dbg(lvl_debug,"enter this=%p gra=%p pnt=%p dir=%d speed=%d\n", this_, gra, pnt, angle, speed);
	dbg(lvl_debug,"point %d,%d\n", pnt->x, pnt->y);
	this_->cursor_pnt=*pnt;
	this_->angle=angle;
	this_->speed=speed;
	if (!this_->cursor)
		return;
	this_->cursor_pnt.x-=this_->cursor->w/2;
	this_->cursor_pnt.y-=this_->cursor->h/2;
	if (!this_->gra) {
		struct color c;
		this_->gra=graphics_overlay_new(gra, &this_->cursor_pnt, this_->cursor->w, this_->cursor->h, 0);
		if (this_->gra) {
			graphics_init(this_->gra);
			this_->bg=graphics_gc_new(this_->gra);
			c.r=0; c.g=0; c.b=0; c.a=0;
			graphics_gc_set_foreground(this_->bg, &c);
			graphics_background_gc(this_->gra, this_->bg);
		}
	}
	vehicle_draw_do(this_);
}

int
vehicle_get_cursor_data(struct vehicle *this, struct point *pnt, int *angle, int *speed)
{
	*pnt=this->cursor_pnt;
	*angle=this->angle;
	*speed=this->speed;
	return 1;
}

static void vehicle_set_default_name(struct vehicle *this_)
{
	struct attr default_name;
	if (!attr_search(this_->attrs, NULL, attr_name)) {
		default_name.type=attr_name;
		// Safe cast: attr_generic_set_attr does not modify its parameter.
		default_name.u.str=(char*)_("Unnamed vehicle");
		this_->attrs=attr_generic_set_attr(this_->attrs, &default_name);
		dbg(lvl_error, "Incomplete vehicle definition: missing attribute 'name'. Default name set.\n");
	}
}


static void
vehicle_draw_do(struct vehicle *this_)
{
	struct point p;
	struct cursor *cursor=this_->cursor;
	int speed=this_->speed;
	int angle=this_->angle;
	int sequence=this_->sequence;
	struct attr **attr;
	char *label=NULL;
	int match=0;

	if (!this_->cursor || !this_->cursor->attrs || !this_->gra)
		return;

	attr=this_->attrs;
	while (attr && *attr) {
		if ((*attr)->type == attr_name) 
			label=(*attr)->u.str;
		attr++;
	}
	transform_set_yaw(this_->trans, -this_->angle);
	graphics_draw_mode(this_->gra, draw_mode_begin);
	p.x=0;
	p.y=0;
	graphics_draw_rectangle(this_->gra, this_->bg, &p, cursor->w, cursor->h);
	attr=cursor->attrs;
	while (*attr) {
		if ((*attr)->type == attr_itemgra) {
			struct itemgra *itm=(*attr)->u.itemgra;
			dbg(lvl_debug,"speed %d-%d %d\n", itm->speed_range.min, itm->speed_range.max, speed);
			if (speed >= itm->speed_range.min && speed <= itm->speed_range.max &&  
			    angle >= itm->angle_range.min && angle <= itm->angle_range.max &&  
			    sequence >= itm->sequence_range.min && sequence <= itm->sequence_range.max) {
				graphics_draw_itemgra(this_->gra, itm, this_->trans, label);
			}
			if (sequence < itm->sequence_range.max)
				match=1;
		}
		++attr;
	}
	graphics_draw_drag(this_->gra, &this_->cursor_pnt);
	graphics_draw_mode(this_->gra, draw_mode_end);
	if (this_->animate_callback) {
		++this_->sequence;
		if (cursor->sequence_range && cursor->sequence_range->max < this_->sequence)
			this_->sequence=cursor->sequence_range->min;
		if (! match && ! cursor->sequence_range)
			this_->sequence=0;
	}
}

/**
 * @brief Writes to an NMEA log.
 *
 * @param this_ The vehicle supplying data
 * @param log The log to write to
 */
static void
vehicle_log_nmea(struct vehicle *this_, struct log *log)
{
	struct attr pos_attr;
	if (!this_->meth.position_attr_get)
		return;
	if (!this_->meth.position_attr_get(this_->priv, attr_position_nmea, &pos_attr))
		return;
	log_write(log, pos_attr.u.str, strlen(pos_attr.u.str), 0);
}

/**
 * Add a tag to the extensions section of a GPX trackpoint.
 *
 * @param tag The tag to add
 * @param logstr Pointer to a pointer to a string to be inserted into the log.
 * When calling this function, {@code *logstr} must point to the substring into which the new tag is
 * to be inserted. If {@code *logstr} is NULL, a new string will be created for the extensions section.
 * Upon returning, {@code *logstr} will point to the new string with the additional tag inserted.
 */
void
vehicle_log_gpx_add_tag(char *tag, char **logstr)
{
	char *ext_start="\t<extensions>\n";
	char *ext_end="\t</extensions>\n";
	char *trkpt_end="</trkpt>";
	char *start=NULL,*end=NULL;
	if (!*logstr) {
		start=g_strdup(ext_start);
		end=g_strdup(ext_end);
	} else {
		char *str=strstr(*logstr, ext_start);
		int len;
		if (str) {
			len=str-*logstr+strlen(ext_start);
			start=g_strdup(*logstr);
			start[len]='\0';
			end=g_strdup(str+strlen(ext_start));
		} else {
			str=strstr(*logstr, trkpt_end);
			len=str-*logstr;
			end=g_strdup_printf("%s%s",ext_end,str);
			str=g_strdup(*logstr);
			str[len]='\0';
			start=g_strdup_printf("%s%s",str,ext_start);
			g_free(str);
		}
	}
	*logstr=g_strdup_printf("%s%s%s",start,tag,end);
	g_free(start);
	g_free(end);
}

/**
 * @brief Writes a trackpoint to a GPX log.
 *
 * @param this_ The vehicle supplying data
 * @param log The log to write to
 */
static void
vehicle_log_gpx(struct vehicle *this_, struct log *log)
{
	struct attr attr,*attrp, fix_attr;
	enum attr_type *attr_types;
	char *logstr;
	char *extensions="\t<extensions>\n";

	if (!this_->meth.position_attr_get)
		return;
	if (log_get_attr(log, attr_attr_types, &attr, NULL))
		attr_types=attr.u.attr_types;
	else
		attr_types=NULL;
	if (this_->meth.position_attr_get(this_->priv, attr_position_fix_type, &fix_attr)) {
		if ( fix_attr.u.num == 0 ) 
			return; 
	}
	if (!this_->meth.position_attr_get(this_->priv, attr_position_coord_geo, &attr))
		return;
	logstr=g_strdup_printf("<trkpt lat=\"%f\" lon=\"%f\">\n",attr.u.coord_geo->lat,attr.u.coord_geo->lng);
	if (attr_types && attr_types_contains_default(attr_types, attr_position_time_iso8601, 0)) {
		if (this_->meth.position_attr_get(this_->priv, attr_position_time_iso8601, &attr)) {
			logstr=g_strconcat_printf(logstr,"\t<time>%s</time>\n",attr.u.str);
		} else {
			char *timep = current_to_iso8601();
			logstr=g_strconcat_printf(logstr,"\t<time>%s</time>\n",timep);
			g_free(timep);
		}
	}
	if (this_->gpx_desc) {
		logstr=g_strconcat_printf(logstr,"\t<desc>%s</desc>\n",this_->gpx_desc);
		g_free(this_->gpx_desc);
		this_->gpx_desc = NULL;
	}
	if (attr_types_contains_default(attr_types, attr_position_height,0) && this_->meth.position_attr_get(this_->priv, attr_position_height, &attr))
		logstr=g_strconcat_printf(logstr,"\t<ele>%.6f</ele>\n",*attr.u.numd);
	// <magvar> magnetic variation in degrees; we might use position_magnetic_direction and position_direction to figure it out
	// <geoidheight> Height (in meters) of geoid (mean sea level) above WGS84 earth ellipsoid. As defined in NMEA GGA message (field 11, which vehicle_wince.c ignores)
	// <name> GPS name (arbitrary)
	// <cmt> comment
	// <src> Source of data
	// <link> Link to additional information (URL)
	// <sym> Text of GPS symbol name
	// <type> Type (classification)
	// <fix> Type of GPS fix {'none'|'2d'|'3d'|'dgps'|'pps'}, leave out if unknown. Similar to position_fix_type but more detailed.
	if (attr_types_contains_default(attr_types, attr_position_sats_used,0) && this_->meth.position_attr_get(this_->priv, attr_position_sats_used, &attr))
		logstr=g_strconcat_printf(logstr,"\t<sat>%d</sat>\n",attr.u.num);
	if (attr_types_contains_default(attr_types, attr_position_hdop,0) && this_->meth.position_attr_get(this_->priv, attr_position_hdop, &attr))
		logstr=g_strconcat_printf(logstr,"\t<hdop>%.6f</hdop>\n",*attr.u.numd);
	// <vdop>, <pdop> Vertical and position dilution of precision, no corresponding attribute
	if (attr_types_contains_default(attr_types, attr_position_direction,0) && this_->meth.position_attr_get(this_->priv, attr_position_direction, &attr))
		logstr=g_strconcat_printf(logstr,"\t<course>%.1f</course>\n",*attr.u.numd);
	if (attr_types_contains_default(attr_types, attr_position_speed, 0) && this_->meth.position_attr_get(this_->priv, attr_position_speed, &attr))
		logstr=g_strconcat_printf(logstr,"\t<speed>%.2f</speed>\n",(*attr.u.numd / 3.6));
	if (attr_types_contains_default(attr_types, attr_profilename, 0) && (attrp=attr_search(this_->attrs, NULL, attr_profilename))) {
		logstr=g_strconcat_printf(logstr,"%s\t\t<navit:profilename>%s</navit:profilename>\n",extensions,attrp->u.str);
		extensions="";
	}
	if (attr_types_contains_default(attr_types, attr_position_radius, 0) && this_->meth.position_attr_get(this_->priv, attr_position_radius, &attr)) {
		logstr=g_strconcat_printf(logstr,"%s\t\t<navit:radius>%.2f</navit:radius>\n",extensions,*attr.u.numd);
		extensions="";
	}
	if (!strcmp(extensions,"")) {
		logstr=g_strconcat_printf(logstr,"\t</extensions>\n");
	}
	logstr=g_strconcat_printf(logstr,"</trkpt>\n");
	callback_list_call_attr_1(this_->cbl, attr_log_gpx, &logstr);
	log_write(log, logstr, strlen(logstr), 0);
	g_free(logstr);
}

/**
 * @brief Writes to a text log.
 *
 * @param this_ The vehicle supplying data
 * @param log The log to write to
 */
static void
vehicle_log_textfile(struct vehicle *this_, struct log *log)
{
	struct attr pos_attr,fix_attr;
	char *logstr;
	if (!this_->meth.position_attr_get)
		return;
	if (this_->meth.position_attr_get(this_->priv, attr_position_fix_type, &fix_attr)) {
		if (fix_attr.u.num == 0) 
			return; 
	}
	if (!this_->meth.position_attr_get(this_->priv, attr_position_coord_geo, &pos_attr))
		return;
	logstr=g_strdup_printf("%f %f type=trackpoint\n", pos_attr.u.coord_geo->lng, pos_attr.u.coord_geo->lat);
	callback_list_call_attr_1(this_->cbl, attr_log_textfile, &logstr);
	log_write(log, logstr, strlen(logstr), 0);
}

/**
 * @brief Writes to a binary log.
 *
 * @param this_ The vehicle supplying data
 * @param log The log to write to
 */
static void
vehicle_log_binfile(struct vehicle *this_, struct log *log)
{
	struct attr pos_attr, fix_attr;
	int *buffer;
	int *buffer_new;
	int len,limit=1024,done=0,radius=25;
	struct coord c;
	enum log_flags flags;

	if (!this_->meth.position_attr_get)
		return;
	if (this_->meth.position_attr_get(this_->priv, attr_position_fix_type, &fix_attr)) {
		if (fix_attr.u.num == 0) 
			return; 
	}
	if (!this_->meth.position_attr_get(this_->priv, attr_position_coord_geo, &pos_attr))
		return;
	transform_from_geo(projection_mg, pos_attr.u.coord_geo, &c);
	if (!c.x || !c.y)
		return;
	while (!done) {
		buffer=log_get_buffer(log, &len);
		if (! buffer || !len) {
			buffer_new=g_malloc(5*sizeof(int));
			buffer_new[0]=2;
			buffer_new[1]=type_track;
			buffer_new[2]=0;
		} else {
			buffer_new=g_malloc((buffer[0]+3)*sizeof(int));
			memcpy(buffer_new, buffer, (buffer[0]+1)*sizeof(int));
		}
		dbg(lvl_debug,"c=0x%x,0x%x\n",c.x,c.y);
		buffer_new[buffer_new[0]+1]=c.x;
		buffer_new[buffer_new[0]+2]=c.y;
		buffer_new[0]+=2;
		buffer_new[2]+=2;
		if (buffer_new[2] > limit) {
			int count=buffer_new[2]/2;
			struct coord *out=g_alloca(sizeof(struct coord)*(count));
			struct coord *in=(struct coord *)(buffer_new+3);
			int count_out=transform_douglas_peucker(in, count, radius, out);
			memcpy(in, out, count_out*2*sizeof(int));
			buffer_new[0]+=(count_out-count)*2;	
			buffer_new[2]+=(count_out-count)*2;	
			flags=log_flag_replace_buffer|log_flag_force_flush|log_flag_truncate;
		} else {
			flags=log_flag_replace_buffer|log_flag_keep_pointer|log_flag_keep_buffer|log_flag_force_flush;
			done=1;
		}
		log_write(log, (char *)buffer_new, (buffer_new[0]+1)*sizeof(int), flags);
	}
}

/**
 * @brief Registers a new log to receive data.
 *
 * @param this_ The vehicle supplying data
 * @param log The log to write to
 *
 * @return False if the log is of an unknown type, true otherwise (including when {@code attr_type} is missing).
 */
static int
vehicle_add_log(struct vehicle *this_, struct log *log)
{
	struct callback *cb;
	struct attr type_attr;
	if (!log_get_attr(log, attr_type, &type_attr, NULL))
                return 1;

	if (!strcmp(type_attr.u.str, "nmea")) {
		cb=callback_new_attr_2(callback_cast(vehicle_log_nmea), attr_position_coord_geo, this_, log);
	} else if (!strcmp(type_attr.u.str, "gpx")) {
		char *header = "<?xml version='1.0' encoding='UTF-8'?>\n"
			"<gpx version='1.1' creator='Navit http://navit.sourceforge.net'\n"
			"     xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'\n"
			"     xmlns:navit='http://www.navit-project.org/schema/navit'\n"
			"     xmlns='http://www.topografix.com/GPX/1/1'\n"
			"     xsi:schemaLocation='http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd'>\n"
			"<trk>\n"
			"<trkseg>\n";
		char *trailer = "</trkseg>\n</trk>\n</gpx>\n";
		log_set_header(log, header, strlen(header));
		log_set_trailer(log, trailer, strlen(trailer));
		cb=callback_new_attr_2(callback_cast(vehicle_log_gpx), attr_position_coord_geo, this_, log);
	} else if (!strcmp(type_attr.u.str, "textfile")) {
		char *header = "type=track\n";
		log_set_header(log, header, strlen(header));
		cb=callback_new_attr_2(callback_cast(vehicle_log_textfile), attr_position_coord_geo, this_, log);
	} else if (!strcmp(type_attr.u.str, "binfile")) {
		cb=callback_new_attr_2(callback_cast(vehicle_log_binfile), attr_position_coord_geo, this_, log);
	} else
		return 0;
	g_hash_table_insert(this_->log_to_cb, log, cb);
	callback_list_add(this_->cbl, cb);
	return 1;
}


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
 * @brief Updates the vehicle position.
 *
 * This method recalculates the position and sets its members accordingly. It is generally called from
 * the position callback but may be extended in the future to be called by other triggers (events or timers)
 * to extrapolate the current vehicle position.
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
 * @param v The {@code struct_vehicle_priv} for the vehicle
 * @param in Raw locations (NULL-terminated pointer array)
 * @param out The last calculated location of the vehicle; this struct will receive the updated location
 * @param cbl Callback list of the vehicle; callbacks from this list will be triggered when one of the
 * respective attributes changes
 */
/* FIXME: use Kalman filter for speed, bearing and altitude (requires accuracy for each) */
void
vehicle_update_position(struct location ** in, struct location * out, struct callback_list * cbl) {
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


struct object_func vehicle_func = {
	attr_vehicle,
	(object_func_new)vehicle_new,
	(object_func_get_attr)vehicle_get_attr,
	(object_func_iter_new)vehicle_attr_iter_new,
	(object_func_iter_destroy)vehicle_attr_iter_destroy,
	(object_func_set_attr)vehicle_set_attr,
	(object_func_add_attr)vehicle_add_attr,
	(object_func_remove_attr)vehicle_remove_attr,
	(object_func_init)NULL,
	(object_func_destroy)vehicle_destroy,
	(object_func_dup)NULL,
	(object_func_ref)navit_object_ref,
	(object_func_unref)navit_object_unref,
};
