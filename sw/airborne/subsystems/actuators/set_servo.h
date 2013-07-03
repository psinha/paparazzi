/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file set_servo.h
 *  Set Servos.
 *  Handles the setting of servo outputs based on actuator commands 
 *  transmitted via the telemetry interface.
 */

#ifndef SET_SERVO_H
#define SET_SERVO_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

struct SetServo {
  int32_t commands[SET_SERVO_NB_SERVO];
  int32_t trim[SET_SERVO_NB_SERVO];
};

extern struct SetServo set_servo;

extern void set_servo_init(void);
extern void set_servo_run(pprz_t in_cmd[]);

#endif /* SET_SERVO_H */
