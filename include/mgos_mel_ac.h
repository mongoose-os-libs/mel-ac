/*
 * Copyright 2020 d4rkmen <darkmen@i.ua>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MGOS_MEL_AC_EV_BASE MGOS_EVENT_BASE('M', 'E', 'L')
#define MGOS_EVENT_GRP_MEL_AC MGOS_MEL_AC_EV_BASE

enum mgos_mel_ac_event {
  MGOS_MEL_AC_EV_INITIALIZED = MGOS_MEL_AC_EV_BASE,
  MGOS_MEL_AC_EV_CONNECTED,
  MGOS_MEL_AC_EV_PACKET_WRITE,
  MGOS_MEL_AC_EV_PACKET_READ,
  MGOS_MEL_AC_EV_PACKET_READ_ERROR,
  MGOS_MEL_AC_EV_PARAMS_CHANGED,
  MGOS_MEL_AC_EV_ROOMTEMP_CHANGED,
  MGOS_MEL_AC_EV_TIMERS_CHANGED,
  MGOS_MEL_AC_EV_OPERATING_CHANGED,
  MGOS_MEL_AC_EV_PARAMS_SET,
  MGOS_MEL_AC_EV_PARAMS_NOT_SET,
  MGOS_MEL_AC_EV_TIMER,
  MGOS_MEL_AC_EV_RX_COUNT,
  MGOS_MEL_AC_EV_CONNECT_ERROR
};

enum mgos_mel_ac_param_power {
  MGOS_MEL_AC_PARAM_POWER_OFF = 0,
  MGOS_MEL_AC_PARAM_POWER_ON = 1
};

enum mgos_mel_ac_param_mode {
  MGOS_MEL_AC_PARAM_MODE_HEAT = 1,
  MGOS_MEL_AC_PARAM_MODE_DRY = 2,
  MGOS_MEL_AC_PARAM_MODE_COOL = 3,
  MGOS_MEL_AC_PARAM_MODE_FAN = 7,
  MGOS_MEL_AC_PARAM_MODE_AUTO = 8
};

enum mgos_mel_ac_param_fan {
  MGOS_MEL_AC_PARAM_FAN_AUTO = 0,
  MGOS_MEL_AC_PARAM_FAN_QUIET = 1,
  MGOS_MEL_AC_PARAM_FAN_LOW = 2,
  MGOS_MEL_AC_PARAM_FAN_MED = 3,
  // MGOS_MEL_AC_PARAM_FAN_MEDHI = 4,
  MGOS_MEL_AC_PARAM_FAN_HIGH = 5,
  MGOS_MEL_AC_PARAM_FAN_TURBO = 6
};

enum mgos_mel_ac_param_vane_horiz {
  MGOS_MEL_AC_PARAM_VANE_HORIZ_AUTO = 0,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_1 = 1,  // 15 %
  MGOS_MEL_AC_PARAM_VANE_HORIZ_2 = 2,  // 30 %
  MGOS_MEL_AC_PARAM_VANE_HORIZ_3 = 3,  // 45 %
  MGOS_MEL_AC_PARAM_VANE_HORIZ_4 = 4,  // 60 %
  MGOS_MEL_AC_PARAM_VANE_HORIZ_5 = 5,  // 75 %
  MGOS_MEL_AC_PARAM_VANE_HORIZ_SWING = 7
};

enum mgos_mel_ac_param_vane_vert {
  MGOS_MEL_AC_PARAM_VANE_VERT_AUTO = 0,
  MGOS_MEL_AC_PARAM_VANE_VERT_LEFTEST = 1,
  MGOS_MEL_AC_PARAM_VANE_VERT_LEFT = 2,
  MGOS_MEL_AC_PARAM_VANE_VERT_CENTER = 3,
  MGOS_MEL_AC_PARAM_VANE_VERT_RIGHT = 4,
  MGOS_MEL_AC_PARAM_VANE_VERT_RIGHTEST = 5,
  MGOS_MEL_AC_PARAM_VANE_VERT_LEFTRIGHT = 8,
  MGOS_MEL_AC_PARAM_VANE_VERT_SWING = 12
};

enum mgos_mel_ac_param_isee {
  MGOS_MEL_AC_PARAM_ISEE_OFF = 0,
  MGOS_MEL_AC_PARAM_ISEE_ON = 1
};

enum mgos_mel_ac_timer_mode {
  MGOS_MEL_AC_TIMER_MODE_NONE = 0,
  MGOS_MEL_AC_TIMER_MODE_OFF = 1,
  MGOS_MEL_AC_TIMER_MODE_ON = 2,
  MGOS_MEL_AC_TIMER_MODE_BOTH = 3
};

enum mgos_mel_ac_packet_type {
  MGOS_MEL_AC_PACKET_TYPE_SET_PARAMS = 1,  // Set HVAC params
  MGOS_MEL_AC_PACKET_TYPE_GET_PARAMS = 2,  // Request current params from HVAC
  MGOS_MEL_AC_PACKET_TYPE_GET_TEMP = 3,    // Get room temperature
  MGOS_MEL_AC_PACKET_TYPE_UNKNOWN = 4,     // Set HVAC timers?
  MGOS_MEL_AC_PACKET_TYPE_GET_TIMERS = 5,  // Get timer settings from HVAC
  MGOS_MEL_AC_PACKET_TYPE_GET_OPERATING =
      6,  // HVAC is idle or operatingt to reach setpoint
  MGOS_MEL_AC_PACKET_TYPE_SET_TEMP =
      7,  // Report room temp from external sensor
  MGOS_MEL_AC_PACKET_TYPE_CONNECT = 0xCA  // Handshake packet
};

struct mgos_mel_ac_timers {
  enum mgos_mel_ac_timer_mode mode;
  int on_set;
  int on_left;
  int off_set;
  int off_left;
};

struct mgos_mel_ac_params {
  enum mgos_mel_ac_param_power power;
  enum mgos_mel_ac_param_mode mode;
  float setpoint;
  enum mgos_mel_ac_param_fan fan;
  enum mgos_mel_ac_param_vane_horiz vane_horiz;  // horizontal vane: up-down
  enum mgos_mel_ac_param_vane_vert vane_vert;    // verticalal vane: left-right
  enum mgos_mel_ac_param_isee isee;              // iSee sensor status. readonly
};

void mgos_mel_ac_create();
void mgos_mel_ac_destroy();

void mgos_mel_ac_connect();
void mgos_mel_ac_disconnect();
void mgos_mel_ac_packet_send(uint8_t flags, enum mgos_mel_ac_packet_type type,
                             uint8_t size);
// Setters
bool mgos_mel_ac_set_power(enum mgos_mel_ac_param_power power);
bool mgos_mel_ac_set_mode(enum mgos_mel_ac_param_mode mode);
bool mgos_mel_ac_set_setpoint(float setpoint);
bool mgos_mel_ac_set_ext_temp(float temp);
bool mgos_mel_ac_set_fan(enum mgos_mel_ac_param_fan fan);
bool mgos_mel_ac_set_vane_vert(enum mgos_mel_ac_param_vane_vert vane_vert);
bool mgos_mel_ac_set_vane_horiz(enum mgos_mel_ac_param_vane_horiz vane_horiz);
void mgos_mel_ac_set_params(struct mgos_mel_ac_params *params);
// Getters
enum mgos_mel_ac_param_power mgos_mel_ac_get_power();
enum mgos_mel_ac_param_mode mgos_mel_ac_get_mode();
float mgos_mel_ac_get_setpoint();
enum mgos_mel_ac_param_fan mgos_mel_ac_get_fan();
enum mgos_mel_ac_param_vane_vert mgos_mel_ac_get_vane_vert();
enum mgos_mel_ac_param_vane_horiz mgos_mel_ac_get_vane_horiz();
bool mgos_mel_ac_get_isee();
bool mgos_mel_ac_get_operating();
void mgos_mel_ac_get_params(struct mgos_mel_ac_params *params);
float mgos_mel_ac_get_room_temperature();
bool mgos_mel_ac_get_connected();
// library
bool mgos_mel_ac_init(void);
void mgos_mel_ac_deinit(void);

#ifdef __cplusplus
}
#endif
