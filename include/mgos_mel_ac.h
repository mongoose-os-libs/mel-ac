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

#define MGOS_MEL_AC_OK 0
#define MGOS_MEL_AC_PACKETRECIEVEERR 1
#define MGOS_MEL_AC_TIMEOUT 2
#define MGOS_MEL_AC_ERROR 3

#define MGOS_MEL_AC_EV_NONE 0
#define MGOS_MEL_AC_EV_INITIALIZED 1
#define MGOS_MEL_AC_EV_CONNECTED 2
#define MGOS_MEL_AC_EV_PACKET_WRITE 3
#define MGOS_MEL_AC_EV_PACKET_READ 4
#define MGOS_MEL_AC_EV_PACKET_READ_ERROR 5
#define MGOS_MEL_AC_EV_PARAMS_CHANGED 6
#define MGOS_MEL_AC_EV_ROOMTEMP_CHANGED 7
#define MGOS_MEL_AC_EV_TIMERS_CHANGED 8
#define MGOS_MEL_AC_EV_OPERATING_CHANGED 9
#define MGOS_MEL_AC_EV_PARAMS_SET 10
#define MGOS_MEL_AC_EV_TIMER 11
#define MGOS_MEL_AC_EV_RX_COUNT 12
#define MGOS_MEL_AC_EV_CONNECT_ERROR 13

#define MGOS_MEL_AC_POWER_OFF 0x00
#define MGOS_MEL_AC_POWER_ON 0x01

enum mgos_mel_ac_param {
  MGOS_MEL_AC_PARAM_BAUDRATE = 4,
  MGOS_MEL_AC_SECURITY_LEVEL,
  MGOS_MEL_AC_PARAM_DATAPACKET_LENGTH
};

enum mgos_mel_ac_param_power {
  MGOS_MEL_AC_PARAM_POWER_OFF = 0,
  MGOS_MEL_AC_PARAM_POWER_ON = 1
};

enum mgos_mel_ac_param_mode {
  MGOS_MEL_AC_PARAM_MODE_CURRENT = 0,
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

enum mgos_mel_ac_param_vane_vert {
  MGOS_MEL_AC_PARAM_VANE_VERT_AUTO = 0,
  MGOS_MEL_AC_PARAM_VANE_VERT_1 = 1,  // 15 %
  MGOS_MEL_AC_PARAM_VANE_VERT_2 = 2,  // 30 %
  MGOS_MEL_AC_PARAM_VANE_VERT_3 = 3,  // 45 %
  MGOS_MEL_AC_PARAM_VANE_VERT_4 = 4,  // 60 %
  MGOS_MEL_AC_PARAM_VANE_VERT_5 = 5,  // 75 %
  MGOS_MEL_AC_PARAM_VANE_VERT_SWING = 7
};

enum mgos_mel_ac_param_vane_horiz {
  MGOS_MEL_AC_PARAM_VANE_HORIZ_AUTO = 0,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_LEFTEST = 1,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_LEFT = 2,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_CENTER = 3,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_RIGHT = 4,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_RIGHTEST = 5,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_LEFTRIGHT = 8,
  MGOS_MEL_AC_PARAM_VANE_HORIZ_SWING = 12
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
  MGOS_MEL_AC_PACKET_TYPE_GET_OPERATING = 6,  // HVAC is idle or operatingt to reach setpoint
  MGOS_MEL_AC_PACKET_TYPE_SET_TEMP = 7,   // Report room temp from external sensor
  MGOS_MEL_AC_PACKET_TYPE_CONNECT = 0xCA  // Handshake packet
};

struct mgos_mel_ac;
typedef void (*mgos_mel_ac_ev_handler)(struct mgos_mel_ac *mel, int ev, void *ev_data,
                                    void *user_data);

struct mgos_mel_ac_cfg {
  uint32_t password;
  uint32_t address;
  uint8_t uart_no;
  uint32_t uart_baud_rate;

  // User callback event handler
  mgos_mel_ac_ev_handler handler;
  void *handler_user_data;

  int timeout_secs;
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
  enum mgos_mel_ac_param_vane_vert vane_vert;    // verticalal vane: up-down
  enum mgos_mel_ac_param_vane_horiz vane_horiz;  // horizontal vane: left-right
  enum mgos_mel_ac_param_isee isee;              // iSee sensor status. readonly
};

void mgos_mel_ac_config_set_defaults(struct mgos_mel_ac_cfg *cfg);

struct mgos_mel_ac *mgos_mel_ac_create(struct mgos_mel_ac_cfg *cfg);
void mgos_mel_ac_destroy(struct mgos_mel_ac **mel);

void mgos_mel_ac_connect(struct mgos_mel_ac *mel);
void mgos_mel_ac_disconnect(struct mgos_mel_ac *mel);
void mgos_mel_ac_packet_send(struct mgos_mel_ac *mel, uint8_t flags,
                          enum mgos_mel_ac_packet_type type, uint8_t size);
// Setters
bool mgos_mel_ac_set_power(struct mgos_mel_ac *mel, enum mgos_mel_ac_param_power power);
bool mgos_mel_ac_set_mode(struct mgos_mel_ac *mel, enum mgos_mel_ac_param_mode mode);
bool mgos_mel_ac_set_setpoint(struct mgos_mel_ac *mel, float setpoint);
bool mgos_mel_ac_set_ext_temp(struct mgos_mel_ac *mel, float temp);
bool mgos_mel_ac_set_fan(struct mgos_mel_ac *mel, enum mgos_mel_ac_param_fan fan);
bool mgos_mel_ac_set_vane_vert(struct mgos_mel_ac *mel,
                            enum mgos_mel_ac_param_vane_vert vane_vert);
bool mgos_mel_ac_set_vane_horiz(struct mgos_mel_ac *mel,
                             enum mgos_mel_ac_param_vane_horiz vane_horiz);
void mgos_mel_ac_set_params(struct mgos_mel_ac *mel, struct mgos_mel_ac_params *params);
// Getters
enum mgos_mel_ac_param_power mgos_mel_ac_get_power(struct mgos_mel_ac *mel);
enum mgos_mel_ac_param_mode mgos_mel_ac_get_mode(struct mgos_mel_ac *mel);
float mgos_mel_ac_get_setpoint(struct mgos_mel_ac *mel);
enum mgos_mel_ac_param_fan mgos_mel_ac_get_fan(struct mgos_mel_ac *mel);
enum mgos_mel_ac_param_vane_vert mgos_mel_ac_get_vane_vert(struct mgos_mel_ac *mel);
enum mgos_mel_ac_param_vane_horiz mgos_mel_ac_get_vane_horiz(struct mgos_mel_ac *mel);
bool mgos_mel_ac_get_isee(struct mgos_mel_ac *mel);
bool mgos_mel_ac_get_operating(struct mgos_mel_ac *mel);
void mgos_mel_ac_get_params(struct mgos_mel_ac *mel, struct mgos_mel_ac_params *params);
float mgos_mel_ac_get_room_temperature(struct mgos_mel_ac *mel);
bool mgos_mel_ac_connected(struct mgos_mel_ac *mel);
// service
bool mgos_mel_ac_svc_init(struct mgos_mel_ac *mel, uint16_t period_ms);
// library
bool mgos_mel_ac_init(void);

#ifdef __cplusplus
}
#endif