/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_blob_locator.h"
 * @author C. De Wagter
 * Find a colored item and track its geo-location and update a waypoint to it
 */

#ifndef CV_BLOB_LOCATOR_H
#define CV_BLOB_LOCATOR_H

#include <stdint.h>
extern uint8_t color_lum_min; //declaration of lower upper middle filter minimum range value
extern uint8_t color_lum_max; //declaration of lower upper middle filter maximum range value

extern uint8_t color_cb_min; //declaration of minimum range value blue chromiance
extern uint8_t color_cb_max; //declaration of maximum range value blue chromiance

extern uint8_t color_cr_min; //declaration of minimum range value red chromiance
extern uint8_t color_cr_max; //declaration of maximum range value red chromiance

extern uint8_t cv_blob_locator_reset; //declaration of reset option for blob locator
extern uint8_t cv_blob_locator_type; //declaration of blob locator type to be identified in image

extern int marker_size; //declaration of the size of the marker
extern int geofilter_length; //declaration of the length of time the geofilter will be active
extern int record_video; //declaration of the size of video 

/*intialize the filter colors, blob_locator_reset, geolocation with deafult values and add listeners to blob events */
extern void cv_blob_locator_init(void);
/*Empty methods*/
extern void cv_blob_locator_periodic(void);
/*Based on the locator_types the attributes needed for blob locator changed and test blob_locator
	set condition. if it is set attributes of the camera frame is changed.
 */
extern void cv_blob_locator_event(void);
/*geolocation intialize with default values*/
extern void cv_blob_locator_start(void);
/*Empty methods*/
extern void cv_blob_locator_stop(void);

// This is reset the location by start capturing present location using cs_blob_locator_start
#define cv_blob_locator_GeoReset(_v) {       \
    cv_blob_locator_start();                 \
  }

#define StartVision(X) { start_vision(); false; }
#define StartVisionLand(X) { start_vision_land(); false; }
#define StopVision(X) { stop_vision(); false; }


extern void start_vision(void); /*function declaration to record the video via camera */
extern void start_vision_land(void); /*function declaration to continue the video recording and mark the regions occupied by any objects*/
extern void stop_vision(void); /*function declaration to stop the video once the blob regions identified*/


#endif

