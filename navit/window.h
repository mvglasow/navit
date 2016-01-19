/**
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
 * Describes areas at each edge of the application window which may be obstructed by the system UI.
 *
 * This allows the map to use all available space, including areas which may be obscured by system UI
 * elements, while constraining other elements such as OSDs or UI controls to an area that is guaranteed
 * to be visible as long as Navit is in the foreground.
 */
struct padding {
	int left;
	int top;
	int right;
	int bottom;
};

/**
 * Describes the Navit application window or equivalent.
 */
struct window {
	void *priv;                                    /**< Private data of the graphics implementation */
	int (*fullscreen)(struct window *win, int on); /**< Method to toggle fullscreen mode */
	void (*disable_suspend)(struct window *win);   /**< Method to disable suspend mode or screen savers */
	struct padding padding;                       /**< Padding for UI controls around window edges */
};
