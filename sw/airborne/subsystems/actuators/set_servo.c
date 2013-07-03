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

/** @file set_servo.c
 *  Set Servos.
 *  Handles the setting of servo outputs based on actuator commands 
 *  transmitted via the telemetry interface.
 */

#include "subsystems/actuators/set_servo.h"
#include "paparazzi.h"

//#include <stdint.h>

struct SetServo set_servo;
static const int32_t servo_trim[SET_SERVO_NB_SERVO] = SET_SERVO_TRIM;

/**************************************************
ADD VARIABLES TO SETUP READIN FROM SPI OR UART HERE
**************************************************/

void set_servo_init(void) {
  uint8_t i;
  for (i=0; i<SET_SERVO_NB_SERVO; i++) {
    set_servo.commands[i] = 0;
    set_servo.trim[i] = servo_trim[i];
  }
/**************************************************
DO INIT FOR READIN FROM SPI OR UART HERE
**************************************************/
}

__attribute__ ((always_inline)) static inline void bound_commands(void) {
  uint8_t j;
  for (j=0; j<SET_SERVO_NB_SERVO; j++)
    Bound(set_servo.commands[j],
          SET_SERVO_MIN_SERVO, SET_SERVO_MAX_SERVO);
}


/**************************************************
DEFINE THE FUNCTION FOR READIN FROM SPI OR UART HERE */
void get_values_from_spi_or_uart(void){
  //setting everything to zero for now. in reality, you need to set each servo to the commanded
  //value from the ground station in the for loop below
  for (i=0; i<SET_SERVO_NB_SERVO; i++) {
    set_servo.commands[i] = 0;
  }
}
/**************************************************/

void set_servo_run(pprz_t in_cmd[]) {

  get_values_from_spi_or_uart();
  bound_commands();

}

