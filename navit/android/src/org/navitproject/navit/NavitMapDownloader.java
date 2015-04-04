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

package org.navitproject.navit;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLConnection;

import android.location.Location;
import android.os.Bundle;
import android.os.Message;
import android.os.StatFs;
import android.util.Log;

/**
 * @author rikky
 *
 */
public class NavitMapDownloader extends Thread
{
	public static class osm_map_values
	{
		String  lon1;
		String  lat1;
		String  lon2;
		String  lat2;
		String  map_name                     = "";
		long    est_size_bytes               = 0;
		int     level                        = 0;


		public osm_map_values(String mapname, String lon_1, String lat_1, String lon_2, String lat_2,
				long bytes_est, int level)
		{
			this.map_name = mapname;
			this.lon1 = lon_1;
			this.lat1 = lat_1;
			this.lon2 = lon_2;
			this.lat2 = lat_2;
			this.est_size_bytes = bytes_est;
			this.level = level;
		}
		
		public boolean isInMap(Location location) {
			double longitude_1 = Double.valueOf(this.lon1);
			double latitude_1 = Double.valueOf(this.lat1);
			double longitude_2 = Double.valueOf(this.lon2);
			double latitude_2 = Double.valueOf(this.lat2);

			if (location.getLongitude() < longitude_1)
				return false;
			if (location.getLongitude() > longitude_2)
				return false;
			if (location.getLatitude() < latitude_1)
				return false;
			if (location.getLatitude() > latitude_2)
				return false;

			return true;
		}
	}
	
	//
	// define the maps here
	//
	public static final osm_map_values[] osm_maps = {
		new osm_map_values(Navit._("Whole Planet"), "-180", "-90", "180", "90", 17142215673L, 0),
		new osm_map_values(Navit._("Africa"), "-30.89", "-36.17", "61.68", "38.40", 978759015L, 0),
		new osm_map_values(Navit._("Angola"), "11.4", "-18.1", "24.2", "-5.3", 91248521L, 1),
		new osm_map_values(Navit._("Burundi"), "28.9", "-4.5", "30.9", "-2.2", 90190335L, 1),
		new osm_map_values(Navit._("Canary Islands"), "-18.69", "26.52", "-12.79", "29.99", 98499593L, 1),
		new osm_map_values(Navit._("Congo, Democratic Republic of the"), "11.7", "-13.6", "31.5", "5.7", 142546118L, 1),
		new osm_map_values(Navit._("Ethiopia"), "32.89", "3.33", "48.07", "14.97", 104750098L, 1),
		new osm_map_values(Navit._("Guinea"), "-15.47", "7.12", "-7.58", "12.74", 142811547L, 1),
		new osm_map_values(Navit._("Cote d'Ivoire"), "-8.72", "4.09", "-2.43", "10.80", 96781332L, 1),
		new osm_map_values(Navit._("Kenya"), "33.8", "-5.2", "42.4", "4.9", 99336192L, 1),
		new osm_map_values(Navit._("Lesotho"), "26.9", "-30.7", "29.6", "-28.4", 100726025L, 1),
		new osm_map_values(Navit._("Liberia"), "-15.00", "-0.73", "-7.20", "8.65", 124911153L, 1),
		new osm_map_values(Navit._("Libya"), "9.32", "19.40", "25.54", "33.63", 92073435L, 1),
		new osm_map_values(Navit._("Madagascar"), "42.25", "-26.63", "51.20", "-11.31", 106061047L, 1),
		new osm_map_values(Navit._("Namibia")+"+"+Navit._("Botswana"), "11.4", "-29.1", "29.5", "-16.9", 114970822L, 1),
		new osm_map_values(Navit._("Reunion"), "55.2", "-21.4", "55.9", "-20.9", 98431755L, 1),
		new osm_map_values(Navit._("Rwanda"), "28.8", "-2.9", "30.9", "-1.0", 94150938L, 1),
		new osm_map_values(Navit._("South Africa")+"+"+Navit._("Lesotho"), "15.93", "-36.36", "33.65", "-22.08", 156152979L, 1),
		new osm_map_values(Navit._("Tanzania, United Republic of"), "29.19", "-11.87", "40.74", "-0.88", 106768648L, 1),
		new osm_map_values(Navit._("Uganda"), "29.3", "-1.6", "35.1", "4.3", 100022833L, 1),
		new osm_map_values(Navit._("Asia"), "23.8", "0.1", "195.0", "82.4", 3253177355L, 0),
		new osm_map_values(Navit._("Azerbaijan"), "44.74", "38.34", "51.69", "42.37", 102293296L, 1),
		new osm_map_values(Navit._("China"), "67.3", "5.3", "135.0", "54.5", 903999878L, 1),
		new osm_map_values(Navit._("Cyprus"), "32.0", "34.5", "34.9", "35.8", 90703681L, 1),
		new osm_map_values(Navit._("India")+"+"+Navit._("Nepal"), "67.9", "5.5", "89.6", "36.0", 256058660L, 1),
		new osm_map_values(Navit._("Indonesia"), "93.7", "-17.3", "155.5", "7.6", 228766382L, 1),
		new osm_map_values(Navit._("Iran, Islamic Republic of"), "43.5", "24.4", "63.6", "40.4", 154094880L, 1),
		new osm_map_values(Navit._("Iraq"), "38.7", "28.5", "49.2", "37.4", 107058534L, 1),
		new osm_map_values(Navit._("Israel"), "33.99", "29.8", "35.95", "33.4", 119569846L, 1),
		new osm_map_values(Navit._("Japan")+"+"+Navit._("Korea")+"+"+Navit._("Taiwan"), "117.6", "20.5", "151.3", "47.1", 766362093L, 1),
		new osm_map_values(Navit._("Kazakhstan"), "46.44", "40.89", "87.36", "55.45", 280438037L, 1),
		new osm_map_values(Navit._("Kyrgyzstan"), "69.23", "39.13", "80.33", "43.29", 110465464L, 1),
		new osm_map_values(Navit._("Malaysia")+"+"+Navit._("Singapore"), "94.3", "-5.9", "108.6", "6.8", 119187612L, 1),
		new osm_map_values(Navit._("Mongolia"), "87.5", "41.4", "120.3", "52.7", 107398401L, 1),
		new osm_map_values(Navit._("Pakistan"), "60.83", "23.28", "77.89", "37.15", 136107911L, 1),
		new osm_map_values(Navit._("Philippines"), "115.58", "4.47", "127.85", "21.60", 166151582L, 1),
		new osm_map_values(Navit._("Saudi Arabia"), "33.2", "16.1", "55.9", "33.5", 170037602L, 1),
		new osm_map_values(Navit._("Thailand"), "97.5", "5.7", "105.2", "19.7", 134166813L, 1),
		new osm_map_values(Navit._("Turkey"), "25.1", "35.8", "46.4", "42.8", 232347346L, 1),
		new osm_map_values(Navit._("Turkmenistan"), "51.78", "35.07", "66.76", "42.91", 95588384L, 1),
		new osm_map_values(Navit._("UAE+Other"), "51.5", "22.6", "56.7", "26.5", 95709627L, 1),
		new osm_map_values(Navit._("Australia")+"+"+Navit._("Oceania"), "89.84", "-57.39", "179.79", "7.26", 482408675L, 0),
		new osm_map_values(Navit._("Australia"), "110.5", "-44.2", "154.9", "-9.2", 244410808L, 0),
		new osm_map_values(Navit._("Tasmania"), "144.0", "-45.1", "155.3", "-24.8", 176390644L, 1),
		new osm_map_values(Navit._("Victoria")+"+"+Navit._("New South Wales"), "140.7", "-39.4", "153.7", "-26.9", 166905296L, 1),
		new osm_map_values(Navit._("New Caledonia"), "157.85", "-25.05", "174.15", "-16.85", 88175405L, 1),
		new osm_map_values(Navit._("New Zealand"), "165.2", "-47.6", "179.1", "-33.7", 180469055L, 1),
		new osm_map_values(Navit._("Europe"), "-12.97", "33.59", "34.15", "72.10", 9046728786L, 0),
		new osm_map_values(Navit._("Western Europe"), "-17.6", "34.5", "42.9", "70.9", 9512395121L, 1),
		new osm_map_values(Navit._("Austria"), "9.4", "46.32", "17.21", "49.1", 662454467L, 1),
		new osm_map_values(Navit._("Azores"), "-31.62", "36.63", "-24.67", "40.13", 84310373L, 1),
		new osm_map_values(Navit._("BeNeLux"), "2.08", "48.87", "7.78", "54.52", 1520394547L, 1),
		new osm_map_values(Navit._("Netherlands"), "3.07", "50.75", "7.23", "53.73", 1055308779L, 1),
		new osm_map_values(Navit._("Denmark"), "7.65", "54.32", "15.58", "58.07", 290170098L, 1),
		new osm_map_values(Navit._("Faroe Islands"), "-7.8", "61.3", "-6.1", "62.5", 83188591L, 1),
		new osm_map_values(Navit._("France"), "-5.45", "42.00", "8.44", "51.68", 3249655126L, 1),
		new osm_map_values(Navit._("Alsace"), "6.79", "47.27", "8.48", "49.17", 288424319L, 2),
		new osm_map_values(Navit._("Aquitaine"), "-2.27", "42.44", "1.50", "45.76", 372741849L, 2),
		new osm_map_values(Navit._("Auvergne"), "2.01", "44.57", "4.54", "46.85", 248344580L, 2),
		new osm_map_values(Navit._("Basse-Normandie"), "-2.09", "48.13", "1.03", "49.98", 212920166L, 2),
		new osm_map_values(Navit._("Bourgogne"), "2.80", "46.11", "5.58", "48.45", 254169749L, 2),
		new osm_map_values(Navit._("Bretagne"), "-5.58", "46.95", "-0.96", "48.99", 344216282L, 2),
		new osm_map_values(Navit._("Centre"), "0.01", "46.29", "3.18", "48.99", 406182417L, 2),
		new osm_map_values(Navit._("Champagne-Ardenne"), "3.34", "47.53", "5.94", "50.28", 229584275L, 2),
		new osm_map_values(Navit._("Corse"), "8.12", "41.32", "9.95", "43.28", 100145697L, 2),
		new osm_map_values(Navit._("Franche-Comte"), "5.20", "46.21", "7.83", "48.07", 262855868L, 2),
		new osm_map_values(Navit._("Haute-Normandie"), "-0.15", "48.62", "1.85", "50.18", 161154198L, 2),
		new osm_map_values(Navit._("Ile-de-France"), "1.40", "48.07", "3.61", "49.29", 161154198L, 2),
		new osm_map_values(Navit._("Languedoc-Roussillon"), "1.53", "42.25", "4.89", "45.02", 327632080L, 2),
		new osm_map_values(Navit._("Limousin"), "0.58", "44.87", "2.66", "46.50", 183489689L, 2),
		new osm_map_values(Navit._("Lorraine"), "4.84", "47.77", "7.72", "49.73", 269812559L, 2),
		new osm_map_values(Navit._("Midi-Pyrenees"), "-0.37", "42.18", "3.50", "45.10", 383558262L, 2),
		new osm_map_values(Navit._("Nord-pas-de-Calais"), "1.42", "49.92", "4.49", "51.31", 302919146L, 2),
		new osm_map_values(Navit._("Pays-de-la-Loire"), "-2.88", "46.20", "0.97", "48.62", 450193605L, 2),
		new osm_map_values(Navit._("Picardie"), "1.25", "48.79", "4.31", "50.43", 307070563L, 2),
		new osm_map_values(Navit._("Poitou-Charentes"), "-1.69", "45.04", "1.26", "47.23", 313113043L, 2),
		new osm_map_values(Navit._("Provence-Alpes-Cote-d-Azur"), "4.21", "42.91", "7.99", "45.18", 317595783L, 2),
		new osm_map_values(Navit._("Rhone-Alpes"), "3.65", "44.07", "7.88", "46.64", 407877163L, 2),
		new osm_map_values(Navit._("Germany"), "5.18", "46.84", "15.47", "55.64", 2837673591L, 1),
		new osm_map_values(Navit._("Baden-Wuerttemberg"), "7.32", "47.14", "10.57", "49.85", 527492836L, 2),
		new osm_map_values(Navit._("Bayern"), "8.92", "47.22", "13.90", "50.62", 662940459L, 2),
		new osm_map_values(Navit._("Mittelfranken"), "9.86", "48.78", "11.65", "49.84", 160155291L, 2),
		new osm_map_values(Navit._("Niederbayern"), "11.55", "47.75", "14.12", "49.42", 239575482L, 2),
		new osm_map_values(Navit._("Oberbayern"), "10.67", "47.05", "13.57", "49.14", 301207029L, 2),
		new osm_map_values(Navit._("Oberfranken"), "10.31", "49.54", "12.49", "50.95", 183767024L, 2),
		new osm_map_values(Navit._("Oberpfalz"), "11.14", "48.71", "13.47", "50.43", 199582448L, 2),
		new osm_map_values(Navit._("Schwaben"), "9.27", "47.10", "11.36", "49.09", 248955385L, 2),
		new osm_map_values(Navit._("Unterfranken"), "8.59", "49.16", "10.93", "50.67", 236009244L, 2),
		new osm_map_values(Navit._("Berlin"), "13.03", "52.28", "13.81", "52.73", 236009244L, 2),
		new osm_map_values(Navit._("Brandenburg"), "11.17", "51.30", "14.83", "53.63", 248263976L, 2),
		new osm_map_values(Navit._("Bremen"), "8.43", "52.96", "9.04", "53.66", 116469818L, 2),
		new osm_map_values(Navit._("Hamburg"), "9.56", "53.34", "10.39", "53.80", 122614777L, 2),
		new osm_map_values(Navit._("Hessen"), "7.72", "49.34", "10.29", "51.71", 334364378L, 2),
		new osm_map_values(Navit._("Mecklenburg-Vorpommern"), "10.54", "53.05", "14.48", "55.05", 164819613L, 2),
		new osm_map_values(Navit._("Niedersachsen"), "6.40", "51.24", "11.69", "54.22", 662782848L, 2),
		new osm_map_values(Navit._("Nordrhein-westfalen"), "5.46", "50.26", "9.52", "52.59", 796972268L, 2),
		new osm_map_values(Navit._("Rheinland-Pfalz"), "6.06", "48.91", "8.56", "51.00", 334191707L, 2),
		new osm_map_values(Navit._("Saarland"), "6.30", "49.06", "7.46", "49.69", 120534914L, 2),
		new osm_map_values(Navit._("Sachsen-Anhalt"), "10.50", "50.88", "13.26", "53.11", 221889143L, 2),
		new osm_map_values(Navit._("Sachsen"), "11.82", "50.11", "15.10", "51.73", 267277236L, 2),
		new osm_map_values(Navit._("Schleswig-Holstein"), "7.41", "53.30", "11.98", "55.20", 219653495L, 2),
		new osm_map_values(Navit._("Thueringen"), "9.81", "50.15", "12.72", "51.70", 208223042L, 2),
		new osm_map_values(Navit._("Germany")+"+"+Navit._("Austria")+"+"+Navit._("Switzerland"), "3.4", "44.5", "18.6", "55.1", 4492224830L, 1),
		new osm_map_values(Navit._("Iceland"), "-25.3", "62.8", "-11.4", "67.5", 94548231L, 1),
		new osm_map_values(Navit._("Ireland"), "-11.17", "51.25", "-5.23", "55.9", 165550114L, 1),
		new osm_map_values(Navit._("Italy"), "6.52", "36.38", "18.96", "47.19", 1062662974L, 1),
		new osm_map_values(Navit._("Spain")+"+"+Navit._("Portugal"), "-11.04", "34.87", "4.62", "44.41", 812986937L, 1),
		new osm_map_values(Navit._("Mallorca"), "2.2", "38.8", "4.7", "40.2", 134412997L, 2),
		new osm_map_values(Navit._("Galicia"), "-10.0", "41.7", "-6.3", "44.1", 151155417L, 2),
		new osm_map_values(Navit._("Scandinavia"), "4.0", "54.4", "32.1", "71.5", 957345953L, 1),
		new osm_map_values(Navit._("Finland"), "18.6", "59.2", "32.3", "70.3", 326001389L, 1),
		new osm_map_values(Navit._("Denmark"), "7.49", "54.33", "13.05", "57.88", 255631813L, 1),
		new osm_map_values(Navit._("Switzerland"), "5.79", "45.74", "10.59", "47.84", 439384941L, 1),
		new osm_map_values(Navit._("United Kingdom"), "-9.7", "49.6", "2.2", "61.2", 679063758L, 1),
		new osm_map_values(Navit._("England"), "-7.80", "48.93", "2.41", "56.14", 724085648L, 1),
		new osm_map_values(Navit._("Buckinghamshire"), "-1.19", "51.44", "-0.43", "52.25", 109563143L, 2),
		new osm_map_values(Navit._("Cambridgeshire"), "-0.55", "51.96", "0.56", "52.79", 108935775L, 2),
		new osm_map_values(Navit._("Cumbria"), "-3.96", "53.85", "-2.11", "55.24", 108479102L, 2),
		new osm_map_values(Navit._("East yorkshire with hull"), "-1.16", "53.50", "0.54", "54.26", 109066505L, 2),
		new osm_map_values(Navit._("Essex"), "-0.07", "51.40", "1.36", "52.14", 126883293L, 2),
		new osm_map_values(Navit._("Herefordshire"), "-3.19", "51.78", "-2.29", "52.45", 97777992L, 2),
		new osm_map_values(Navit._("Kent"), "-0.02", "50.81", "1.65", "51.53", 112579242L, 2),
		new osm_map_values(Navit._("Lancashire"), "-3.20", "53.43", "-2.00", "54.29", 112845974L, 2),
		new osm_map_values(Navit._("Leicestershire"), "-1.65", "52.34", "-0.61", "53.03", 117179765L, 2),
		new osm_map_values(Navit._("Norfolk"),  "0.10", "52.30", "2.04", "53.41", 108306587L, 2),
		new osm_map_values(Navit._("Nottinghamshire"), "-1.39", "52.73", "-0.62", "53.55", 112023678L, 2),
		new osm_map_values(Navit._("Oxfordshire"), "-1.77", "51.41", "-0.82", "52.22", 109834780L, 2),
		new osm_map_values(Navit._("Shropshire"), "-3.29", "52.26", "-2.18", "53.05", 103532414L, 2),
		new osm_map_values(Navit._("Somerset"), "-3.89", "50.77", "-2.20", "51.40", 109975069L, 2),
		new osm_map_values(Navit._("South yorkshire"), "-1.88", "53.25", "-0.80", "53.71", 110488840L, 2),
		new osm_map_values(Navit._("Suffolk"), "0.29", "51.88", "1.81", "52.60", 109788040L, 2),
		new osm_map_values(Navit._("Surrey"), "-0.90", "51.02", "0.10", "51.52", 121727148L, 2),
		new osm_map_values(Navit._("Wiltshire"),  "-2.41", "50.90", "-1.44", "51.76", 106375888L, 2),
		new osm_map_values(Navit._("Scotland"), "-8.13", "54.49", "-0.15", "61.40", 189176250L, 2),
		new osm_map_values(Navit._("Wales"), "-5.56", "51.28", "-2.60", "53.60", 144572638L, 2),
		new osm_map_values(Navit._("Albania"), "19.09", "39.55", "21.12", "42.72", 106939331L, 1),
		new osm_map_values(Navit._("Belarus"), "23.12", "51.21", "32.87", "56.23", 229648760L, 1),
		new osm_map_values(Navit._("Russian Federation"), "27.9", "41.5", "190.4", "77.6", 1479983473L, 1),
		new osm_map_values(Navit._("Bulgaria"), "24.7", "42.1", "24.8", "42.1", 83696190L, 1),
		new osm_map_values(Navit._("Bosnia and Herzegovina"), "15.69", "42.52", "19.67", "45.32", 144196241L, 1),
		new osm_map_values(Navit._("Czech Republic"), "11.91", "48.48", "19.02", "51.17", 711304334L, 1),
		new osm_map_values(Navit._("Croatia"), "13.4", "42.1", "19.4", "46.9", 251673871L, 1),
		new osm_map_values(Navit._("Estonia"), "21.5", "57.5", "28.2", "59.6", 142340828L, 1),
		new osm_map_values(Navit._("Greece"), "28.9", "37.8", "29.0", "37.8", 83257661L, 1),
		new osm_map_values(Navit._("Crete"), "23.3", "34.5", "26.8", "36.0", 87834598L, 1),
		new osm_map_values(Navit._("Hungary"), "16.08", "45.57", "23.03", "48.39", 256779608L, 1),
		new osm_map_values(Navit._("Latvia"), "20.7", "55.6", "28.3", "58.1", 144857068L, 1),
		new osm_map_values(Navit._("Lithuania"), "20.9", "53.8", "26.9", "56.5", 156811735L, 1),
		new osm_map_values(Navit._("Poland"), "13.6", "48.8", "24.5", "55.0", 1018629889L, 1),
		new osm_map_values(Navit._("Romania"), "20.3", "43.5", "29.9", "48.4", 265402066L, 1),
		new osm_map_values(Navit._("Ukraine"),  "22.0", "44.3", "40.4", "52.4", 524768547L, 1),
		new osm_map_values(Navit._("North America"), "-178.1", "6.5", "-10.4", "84.0", 4526641449L, 0),
		new osm_map_values(Navit._("Alaska"), "-179.5", "49.5", "-129", "71.6", 174207438L, 1),
		new osm_map_values(Navit._("Canada"), "-141.3", "41.5", "-52.2", "70.2", 2273900574L, 1),
		new osm_map_values(Navit._("Hawaii"), "-161.07", "18.49", "-154.45", "22.85", 87476921L, 1),
		new osm_map_values(Navit._("USA")+Navit._(" (except Alaska and Hawaii)"), "-125.4", "24.3", "-66.5", "49.3", 3389691841L, 1),
		new osm_map_values(Navit._("Midwest"), "-104.11", "35.92", "-80.46", "49.46", 1030850300L, 2),
		new osm_map_values(Navit._("Michigan"), "-90.47", "41.64", "-79.00", "49.37", 495136827L, 2),
		new osm_map_values(Navit._("Ohio"), "-84.87", "38.05", "-79.85", "43.53", 226499873L, 2),
		new osm_map_values(Navit._("Northeast"), "-80.58", "38.72", "-66.83", "47.53", 848767227L, 2),
		new osm_map_values(Navit._("Massachusetts"), "-73.56", "40.78", "-68.67", "42.94", 284272912L, 2),
		new osm_map_values(Navit._("Vermont"), "-73.49", "42.68", "-71.41", "45.07", 110911886L, 2),
		new osm_map_values(Navit._("Pacific"), "-180.05", "15.87", "-129.75", "73.04", 172206612L, 2),
		new osm_map_values(Navit._("South"),  "-106.70", "23.98", "-71.46", "40.70", 1481026457L, 2),
		new osm_map_values(Navit._("Arkansas"), "-94.67", "32.95", "-89.59", "36.60", 125115656L, 2),
		new osm_map_values(Navit._("District of Columbia"), "-77.17", "38.74", "-76.86", "39.05", 101152353L, 2),
		new osm_map_values(Navit._("Florida"), "-88.75", "23.63", "-77.67", "31.05", 174378547L, 2),
		new osm_map_values(Navit._("Louisiana"), "-94.09", "28.09", "-88.62", "33.07", 177996551L, 2),
		new osm_map_values(Navit._("Maryland"), "-79.54", "37.83", "-74.99", "40.22", 213559925L, 2),
		new osm_map_values(Navit._("Mississippi"), "-91.71", "29.99", "-88.04", "35.05", 146620850L, 2),
		new osm_map_values(Navit._("Oklahoma"), "-103.41", "33.56", "-94.38", "37.38", 167396836L, 2),
		new osm_map_values(Navit._("Texas"), "-106.96", "25.62", "-92.97", "36.58", 320259945L, 2),
		new osm_map_values(Navit._("Virginia"), "-83.73", "36.49", "-74.25", "39.52", 306054196L, 2),
		new osm_map_values(Navit._("West Virginia"), "-82.70", "37.15", "-77.66", "40.97", 181531617L, 2),
		new osm_map_values(Navit._("West"), "-133.11", "31.28", "-101.99", "49.51", 837597071L, 2),
		new osm_map_values(Navit._("Arizona"), "-114.88", "30.01", "-108.99", "37.06", 140190223L, 2),
		new osm_map_values(Navit._("California"), "-125.94", "32.43", "-114.08", "42.07", 384857326L, 2),
		new osm_map_values(Navit._("Colorado"), "-109.11", "36.52", "-100.41", "41.05", 185388342L, 2),
		new osm_map_values(Navit._("Idaho"), "-117.30", "41.93", "-110.99", "49.18", 137642365L, 2),
		new osm_map_values(Navit._("Montana"), "-116.10", "44.31", "-102.64", "49.74", 140601787L, 2),
		new osm_map_values(Navit._("New Mexico"), "-109.10", "26.98", "-96.07", "37.05", 263105357L, 2),
		new osm_map_values(Navit._("Nevada"), "-120.2", "35.0", "-113.8", "42.1", 164712473L, 2),
		new osm_map_values(Navit._("Oregon"),  "-124.8", "41.8", "-116.3", "46.3", 153169398L, 2),
		new osm_map_values(Navit._("Utah"), "-114.11", "36.95", "-108.99", "42.05", 115653638L, 2),
		new osm_map_values(Navit._("Washington State"), "-125.0", "45.5", "-116.9", "49.0", 167427046L, 2),
		new osm_map_values(Navit._("South+Middle America"), "-83.5", "-56.3", "-30.8", "13.7", 562230441L, 0),
		new osm_map_values(Navit._("Argentina"), "-73.9", "-57.3", "-51.6", "-21.0", 244901827L, 1),
		new osm_map_values(Navit._("Argentina")+"+"+Navit._("Chile"), "-77.2", "-56.3", "-52.7", "-16.1", 269594088L, 1),
		new osm_map_values(Navit._("Bolivia"), "-70.5", "-23.1", "-57.3", "-9.3", 121759213L, 1),
		new osm_map_values(Navit._("Brazil"), "-71.4", "-34.7", "-32.8", "5.4", 388405363L, 1),
		new osm_map_values(Navit._("Chile"), "-81.77", "-58.50", "-65.46", "-17.41", 160742590L, 1),
		new osm_map_values(Navit._("Cuba"), "-85.3", "19.6", "-74.0", "23.6", 95014897L, 1),
		new osm_map_values(Navit._("Colombia"), "-79.1", "-4.0", "-66.7", "12.6", 151545547L, 1),
		new osm_map_values(Navit._("Ecuador"), "-82.6", "-5.4", "-74.4", "2.3", 105488624L, 1),
		new osm_map_values(Navit._("Guyana")+"+"+Navit._("Suriname")+"+"+Navit._("Guyane Francaise"), "-62.0", "1.0", "-51.2", "8.9", 92083881L, 1),
		new osm_map_values(Navit._("Haiti")+"+"+Navit._("Dominican Republic"), "-74.8", "17.3", "-68.2", "20.1", 103797756L, 1),
		new osm_map_values(Navit._("Jamaica"), "-78.6", "17.4", "-75.9", "18.9", 85053924L, 1),
		new osm_map_values(Navit._("Mexico"), "-117.6", "14.1", "-86.4", "32.8", 384891270L, 1),
		new osm_map_values(Navit._("Paraguay"), "-63.8", "-28.1", "-53.6", "-18.8", 116798539L, 1),
		new osm_map_values(Navit._("Peru"), "-82.4", "-18.1", "-67.5", "0.4", 137292365L, 1),
		new osm_map_values(Navit._("Uruguay"), "-59.2", "-36.5", "-51.7", "-29.7", 113817110L, 1),
		new osm_map_values(Navit._("Venezuela"), "-73.6", "0.4", "-59.7", "12.8", 124001905L, 1)
	};

	public  static final String         MAP_FILENAME_PATH                       = Navit.MAP_FILENAME_PATH;
	
	public static NavitMap[] getAvailableMaps() {
		class filterMaps implements FilenameFilter {
			public boolean accept(File dir, String filename) {
				if (filename.endsWith(".bin"))
					return true;
				return false;
			}
		}
		NavitMap maps[] = new NavitMap[0];
		File map_dir = new File(MAP_FILENAME_PATH);
		String map_file_names[] = map_dir.list(new filterMaps());
		if (map_file_names != null) {
			maps = new NavitMap[map_file_names.length];
			for (int map_file_index = 0; map_file_index < map_file_names.length; map_file_index++) {
				maps[map_file_index] = new NavitMap(MAP_FILENAME_PATH, map_file_names[map_file_index]);
			}
		}
		return maps;
	}
	private Boolean                     stop_me                                 = false;
	private osm_map_values              map_values;
	private long                        uiLastUpdated                           = -1;

	private Boolean                     retryDownload                           = false; //Download failed, but
	                                                                                     //we should try to resume
	private static final int            SOCKET_CONNECT_TIMEOUT                  = 60000;          // 60 secs.
	private static final int            SOCKET_READ_TIMEOUT                     = 120000;         // 120 secs.
	private static final int            MAP_WRITE_FILE_BUFFER                   = 1024 * 64;
	private static final int            MAP_WRITE_MEM_BUFFER                    = 1024 * 64;
	private static final int            MAP_READ_FILE_BUFFER                    = 1024 * 64;
	private static final int            UPDATE_PROGRESS_TIME_NS                 = 1000 * 1000000; // 1ns=1E-9s
	private static final int            MAX_RETRIES                             = 5;
	private static final String         TAG                                     = "NavitMapDownloader";

	protected int                       retry_counter                           = 0;

	public NavitMapDownloader(int map_id) {
		this.map_values = osm_maps[map_id];
	}

	public void run() {
		stop_me = false;
		retry_counter = 0;

		Log.v(TAG, "start download " + map_values.map_name);
		updateProgress(0, map_values.est_size_bytes, Navit._("downloading") + ": " + map_values.map_name);
		
		boolean success;
		do {
			try {
				Thread.sleep(10 + retry_counter * 1000);
			} catch (InterruptedException e1) {}
			retryDownload = false;
			success = download_osm_map();
		} while ( !success
				&& retryDownload
				&& retry_counter < MAX_RETRIES
				&& !stop_me);

		if (success) {
			toast(map_values.map_name + " " + Navit._("ready"));
			Log.d(TAG, "success");
		}

		if (success || stop_me ) {
			NavitDialogs.sendDialogMessage( NavitDialogs.MSG_MAP_DOWNLOAD_FINISHED
					, MAP_FILENAME_PATH + map_values.map_name + ".bin", null, -1, success ? 1 : 0 , 0 );
		}
	}
	
	public void stop_thread() {
		stop_me = true;
		Log.d(TAG, "stop_me -> true");
	}

	protected boolean checkFreeSpace(long needed_bytes) {
		long free_space = getFreeSpace();
	
		if ( needed_bytes <= 0 )
			needed_bytes = MAP_WRITE_FILE_BUFFER;

		if (free_space < needed_bytes ) {
			Log.e(TAG, "Not enough free space. Please free at least " + needed_bytes / 1024 /1024 + "Mb.");
			updateProgress(free_space, needed_bytes, Navit._("Error downloading map!") + "\n" + Navit._("Not enough free space"));
			return false;
		}
		return true;
	}

	protected boolean deleteMap() {
		File finalOutputFile = getMapFile();

		if (finalOutputFile.exists()) {
			Message msg =
			        Message.obtain(Navit.N_NavitGraphics.callback_handler,
			                NavitGraphics.msg_type.CLB_DELETE_MAP.ordinal());
			Bundle b = new Bundle();
			b.putString("title", finalOutputFile.getAbsolutePath());
			msg.setData(b);
			msg.sendToTarget();
			return true;
		}
		return false;
	}

	/**
	 * @param map_values
	 * @return
	 */
	protected boolean download_osm_map() {
		long already_read = 0;
		long real_size_bytes = 0;
		boolean resume = true;

		File outputFile = getDestinationFile();
		long old_download_size = outputFile.length();

		URL url = null;
		if (old_download_size > 0) {
			url = readFileInfo();
		}

		if (url == null) {
			resume = false;
			url = getDownloadURL();
		}

		// URL url = new URL("http://192.168.2.101:8080/zweibruecken.bin");
		URLConnection c = initConnection(url);
		if (c != null) {

			if (resume) {
				c.setRequestProperty("Range", "bytes=" + old_download_size + "-");
				already_read = old_download_size;
			}
			try {
				real_size_bytes=Long.parseLong(c.getHeaderField("Content-Length")) + already_read;
			} catch(Exception e) {
				real_size_bytes=-1;
			}

			long fileTime = c.getLastModified();

			if (!resume) {
				outputFile.delete();
				writeFileInfo(c, real_size_bytes);
			}

			if (real_size_bytes <= 0)
				real_size_bytes = map_values.est_size_bytes;

			Log.d(TAG, "size: " + real_size_bytes + ", read: " + already_read + ", timestamp: " + fileTime
			        + ", Connection ref: " + c.getURL());

			if (checkFreeSpace(real_size_bytes - already_read)
			        && downloadData(c, already_read, real_size_bytes, resume, outputFile)) {

				File finalOutputFile = getMapFile();
				// delete an already existing file first
				finalOutputFile.delete();
				// rename file to its final name
				outputFile.renameTo(finalOutputFile);
				return true;
			}
		}
		return false;
	}

	protected File getDestinationFile() {
		File outputFile = new File(MAP_FILENAME_PATH, map_values.map_name + ".tmp");
		outputFile.getParentFile().mkdir();
		return outputFile;
	}

	protected boolean downloadData(URLConnection c, long already_read, long real_size_bytes
	     , boolean resume,File outputFile) {
		boolean success = false;
		BufferedOutputStream buf = getOutputStream(outputFile, resume);
		BufferedInputStream bif = getInputStream(c);

		if (buf != null && bif != null) {
			success = readData(buf, bif, already_read, real_size_bytes);
			// always cleanup, as we might get errors when trying to resume
			try {
				buf.flush();
				buf.close();

				bif.close();
			} catch (IOException e) {
			}
		}
		return success;
	}

	protected URL getDownloadURL() {
		URL url = null;
		try {
			url =
			        new URL("http://maps.navit-project.org/api/map/?bbox=" + map_values.lon1 + "," + map_values.lat1
			                + "," + map_values.lon2 + "," + map_values.lat2);
		} catch (MalformedURLException e) {
			Log.e(TAG, "We failed to create a URL to " + map_values.map_name);
			e.printStackTrace();
			return null;
		}
		Log.v(TAG, "connect to " + url.toString());
		return url;
	}

	protected long getFreeSpace() {
		StatFs fsInfo = new StatFs(MAP_FILENAME_PATH);
		return (long)fsInfo.getAvailableBlocks() * fsInfo.getBlockSize();
	}

	protected BufferedInputStream getInputStream(URLConnection c) {
		BufferedInputStream bif = null;
		try {
			bif = new BufferedInputStream(c.getInputStream(), MAP_READ_FILE_BUFFER);
		} catch (FileNotFoundException e) {
			Log.e(TAG, "File not found on server: " + e);
			if (retry_counter > 0) {
				getMapInfoFile().delete();
			}
			enableRetry();
			bif = null;
		} catch (IOException e) {
			Log.e(TAG, "Error reading from server: " + e);
			enableRetry();
			bif = null;
		}
		return bif;
	}

	protected File getMapFile() {
		return new File(MAP_FILENAME_PATH, map_values.map_name + ".bin");
	}

	protected File getMapInfoFile() {
		return new File(MAP_FILENAME_PATH, map_values.map_name + ".tmp.info");
	}

	protected BufferedOutputStream getOutputStream(File outputFile, boolean resume) {
		BufferedOutputStream buf = null;
		try {
			buf = new BufferedOutputStream(new FileOutputStream(outputFile, resume), MAP_WRITE_FILE_BUFFER);
		} catch (FileNotFoundException e) {
			Log.e(TAG, "Could not open output file for writing: " + e);
			buf = null;
		}
		return buf;
	}
	
	protected URLConnection initConnection(URL url) {
		HttpURLConnection c = null;
		try {
			c = (HttpURLConnection) url.openConnection();
			c.setRequestMethod("GET");
		} catch (Exception e) {
			Log.e(TAG, "Failed connecting server: " + e);
			enableRetry();
			return null;
		}

		c.setReadTimeout(SOCKET_READ_TIMEOUT);
		c.setConnectTimeout(SOCKET_CONNECT_TIMEOUT);
		return c;
	}

	protected boolean readData(OutputStream buf, InputStream bif, long already_read, long real_size_bytes) {
		long start_timestamp = System.nanoTime();
		byte[] buffer = new byte[MAP_WRITE_MEM_BUFFER];
		int len1 = 0;
		long mapFileSize = real_size_bytes;
		long startOffset = already_read;
		boolean success = false;
		
		try {
			while (!stop_me && (len1 = bif.read(buffer)) != -1) {
				already_read += len1;
				updateProgress(start_timestamp, startOffset, already_read, mapFileSize);

				try {
					buf.write(buffer, 0, len1);
				} catch (IOException e) {
					Log.d(TAG, "Error: " + e);
					if ( !checkFreeSpace(real_size_bytes - already_read + MAP_WRITE_FILE_BUFFER)) {
						if (deleteMap()) {
							enableRetry();
						} else {
							updateProgress(already_read, real_size_bytes, Navit._("Error downloading map!") + "\n"
							        + Navit._("Not enough free space"));
						}
					} else {
						updateProgress(already_read, real_size_bytes, Navit._("Error writing map!"));
					}
					
					return false;
				}
			}

			if (stop_me) {
				toast(Navit._("Map download aborted!"));
			} else if ( already_read < real_size_bytes ) {
				Log.d(TAG, "Server send only " + already_read + " bytes of " + real_size_bytes);
				enableRetry();
			} else {
				success = true;
			}
		} catch (IOException e) {
			Log.d(TAG, "Error: " + e);

			enableRetry();
			updateProgress(already_read, real_size_bytes, Navit._("Error downloading map!"));
		}
		
		return success;
	}

	protected URL readFileInfo() {
		URL url = null;
		try {
			ObjectInputStream infoStream = new ObjectInputStream(new FileInputStream(getMapInfoFile()));
			String resume_proto = infoStream.readUTF();
			infoStream.readUTF(); // read the host name (unused for now)
			String resume_file = infoStream.readUTF();
			infoStream.close();
			// looks like the same file, try to resume
			Log.v(TAG, "Try to resume download");
			url = new URL(resume_proto + "://" + "maps.navit-project.org" + resume_file);
		} catch (Exception e) {
			getMapInfoFile().delete();
		}
		return url;
	}

	protected void toast(String message) {
		NavitDialogs.sendDialogMessage(NavitDialogs.MSG_TOAST, null, message, -1, 0, 0);
	}
	
	protected void updateProgress(long startTime, long offsetBytes, long readBytes, long maxBytes) {
		long currentTime = System.nanoTime();

		if ((currentTime > uiLastUpdated + UPDATE_PROGRESS_TIME_NS) && startTime!=currentTime) {
			float per_second_overall = (readBytes - offsetBytes) / ((currentTime - startTime) / 1000000000f);
			long bytes_remaining = maxBytes - readBytes;
			int eta_seconds = (int) (bytes_remaining / per_second_overall);

			String eta_string;
			if (eta_seconds > 60) {
				eta_string = (int) (eta_seconds / 60f) + " m";
			} else {
				eta_string = eta_seconds + " s";
			}
			String info =
			        String.format("%s: %s\n %dMb / %dMb\n %.1f kb/s %s: %s", Navit._("downloading")
			                , map_values.map_name, readBytes / 1024 / 1024, maxBytes / 1024 / 1024,
			                per_second_overall / 1024f, Navit._("ETA"), eta_string);

			if (retry_counter > 0) {
				info += "\n Retry " + retry_counter + "/" + MAX_RETRIES;
			}
			Log.e(TAG, "info: " + info);

			updateProgress(readBytes, maxBytes, info);
			uiLastUpdated = currentTime;
		}
	}
	
	protected void updateProgress(long positionBytes, long maximumBytes, String infoText) {
		NavitDialogs.sendDialogMessage(NavitDialogs.MSG_PROGRESS_BAR, Navit._("Map download"), infoText
		        , NavitDialogs.DIALOG_MAPDOWNLOAD, (int) (maximumBytes / 1024),
		        (int) (positionBytes / 1024));
	}

	protected void writeFileInfo(URLConnection c, long sizeInBytes) {
		ObjectOutputStream infoStream;
		try {
			infoStream = new ObjectOutputStream(new FileOutputStream(getMapInfoFile()));
			infoStream.writeUTF(c.getURL().getProtocol());
			infoStream.writeUTF(c.getURL().getHost());
			infoStream.writeUTF(c.getURL().getFile());
			infoStream.writeLong(sizeInBytes);
			infoStream.close();
		} catch (Exception e) {
			Log.e(TAG, "Could not write info file for map download. Resuming will not be possible. (" + e.getMessage() + ")");
		}
	}

	void enableRetry() {
		retryDownload = true;
		retry_counter++;
	}
	// dialog send helper methods
}
