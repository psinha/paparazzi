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

/** @file servo_mixing.c
 *  Servo Mixing.
 *  Handles the mapping of roll/pitch/yaw commands
 *  to actual servo commands after filtering.
 */

#include "subsystems/actuators/servo_mixing.h"
#include "paparazzi.h"


//#include <stdint.h>

struct ServoMixing servo_mixing;
struct ServoMixing servo_mixing_old;
int32_t aileron_rcscaler;
int32_t aileron_scaler;
int32_t elevator_rcscaler;
int32_t elevator_scaler;
int32_t sample_size;

void servo_mixing_init(void) {
  uint8_t i;
  for (i=0; i<SERVO_MIXING_NB_SERVO; i++) {
    servo_mixing.commands[i] = 0;
  }
  servo_mixing.trim[0] = SERVO_MIXING_TRIM_FLAPERON_LEFT;
  servo_mixing.trim[1] = SERVO_MIXING_TRIM_FLAPERON_RIGHT;
  servo_mixing.trim[2] = SERVO_MIXING_TRIM_ELEVATOR;

  for (i=0; i<SERVO_MIXING_NB_SERVO; i++) {
    servo_mixing_old.commands[i] = 0;
  }
  servo_mixing_old.trim[0] = SERVO_MIXING_TRIM_FLAPERON_LEFT;
  servo_mixing_old.trim[1] = SERVO_MIXING_TRIM_FLAPERON_RIGHT;
  servo_mixing_old.trim[2] = SERVO_MIXING_TRIM_ELEVATOR;

  sample_size=STABILIZATION_ATTITUDE_SAMPLE_SIZE;
  aileron_rcscaler=STABILIZATION_ATTITUDE_AILERON_RCSCALER;
  aileron_scaler=STABILIZATION_ATTITUDE_AILERON_SCALER;
  elevator_rcscaler=STABILIZATION_ATTITUDE_ELEVATOR_RCSCALER;
  elevator_scaler=STABILIZATION_ATTITUDE_ELEVATOR_SCALER;
}


void servo_mixing_run(pprz_t in_cmd[]) {
  int32_t flaps;
  float iir_mult;
  iir_mult=1/sample_size;
  if (in_cmd[COMMAND_RCFLAPS]>2000)
    flaps=2880;
  else
    flaps=0;
  if(in_cmd[COMMAND_RCMODE]>2000){
    servo_mixing.commands[0] = ((1-iir_mult)*servo_mixing_old.commands[0] + iir_mult*(servo_mixing.trim[0] + aileron_rcscaler * in_cmd[COMMAND_RCAILERON] + flaps))/MAX_PPRZ;
    servo_mixing.commands[1] = ((1-iir_mult)*servo_mixing_old.commands[1] + iir_mult*(servo_mixing.trim[1] + aileron_rcscaler * in_cmd[COMMAND_RCAILERON] - flaps))/MAX_PPRZ;
    servo_mixing.commands[2] = ((1-iir_mult)*servo_mixing_old.commands[2] + iir_mult*(servo_mixing.trim[2] + elevator_rcscaler * in_cmd[COMMAND_RCELEVATOR]))/MAX_PPRZ;
    }
  else{
    servo_mixing.commands[0] = ((1-iir_mult)*servo_mixing_old.commands[0] + iir_mult*(servo_mixing.trim[0] + aileron_scaler * in_cmd[COMMAND_ROLL] + flaps))/MAX_PPRZ;
    servo_mixing.commands[1] = ((1-iir_mult)*servo_mixing_old.commands[1] + iir_mult*(servo_mixing.trim[1] + aileron_scaler * in_cmd[COMMAND_ROLL] - flaps))/MAX_PPRZ;
    servo_mixing.commands[2] = ((1-iir_mult)*servo_mixing_old.commands[2] + iir_mult*(servo_mixing.trim[2] + elevator_scaler * in_cmd[COMMAND_PITCH]))/MAX_PPRZ;
    }
  servo_mixing_old.commands[0] = servo_mixing.commands[0];
  servo_mixing_old.commands[1] = servo_mixing.commands[1];
  servo_mixing_old.commands[2] = servo_mixing.commands[2];
}
