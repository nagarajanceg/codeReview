/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/cv.h
 *
 * Computer vision framework for onboard processing
 */


#ifndef CV_H_
#define CV_H_

#include <pthread.h>

#include "std.h"
#include "peripherals/video_device.h"

#include BOARD_CONFIG

typedef struct image_t *(*cv_function)(struct image_t *img);

struct cv_async {
  pthread_t thread_id; /*hold a generated Pthread id*/
  volatile bool thread_running; /*shows the thread_running status, volatile is used to avoid compiler simplifiaction of the variable*/
  volatile int thread_priority; /*holds th priority of the thread*/
  pthread_mutex_t img_mutex; /* Mutex for process synchronization and concurrency handling*/
  pthread_cond_t img_available;/*pthread condition variable to show status of image availabilty*/
  volatile bool img_processed; /*to denote the modification in the recorded image*/
  struct image_t img_copy; /*to hold the copy of the image*/
};

struct video_listener {
  struct video_listener *next; /* self reference pointer points to the address of the next listerner of same type*/
  struct cv_async *async;  /* pointer points to pthread struct along with image copy*/
  struct timeval ts;
  cv_function func; /*Pointer to image raw structure*/

  // Can be set by user
  uint16_t maximum_fps; /*frames per second*/
  volatile bool active; /*video listener state*/
};
/*New video device add to the threads exisiting*/
extern bool add_video_device(struct video_config_t *device);

// register a processing function and initialize the video device if necessary
extern struct video_listener *cv_add_to_device(struct video_config_t *device, cv_function func, uint16_t fps);
// register a processing function and initialize the video device if necessary
// Add asynchronous structure to override default synchronous behavior along with mutex and conditional variable
// initialized. Then deliberate to a pthread as a listener  
extern struct video_listener *cv_add_to_device_async(struct video_config_t *device, cv_function func, int nice_level, uint16_t fps);

//Check for listener state , send images to async thread if async is set or execute the listener associated with device 
extern void cv_run_device(struct video_config_t *device, struct image_t *img);

#endif /* CV_H_ */
