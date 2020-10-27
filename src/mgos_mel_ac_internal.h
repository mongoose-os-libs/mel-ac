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

#include "mgos_mel_ac.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MGOS_MEL_AC_PACKET_MAGIC 0xFC
#define MGOS_MEL_AC_PACKET_FLAGS_CONNECT 0x18
#define MGOS_MEL_AC_PACKET_FLAGS_IN 0x20
#define MGOS_MEL_AC_PACKET_FLAGS_OUT 0x40
#define MGOS_MEL_AC_PACKET_FLAGS_SET 0x01
#define MGOS_MEL_AC_PACKET_FLAGS_GET 0x02
#define MGOS_MEL_AC_PACKET_VER_MAJ 0x01
#define MGOS_MEL_AC_PACKET_VER_MIN 0x30

#define MGOS_MEL_AC_PACKET_MAX_DATA_SIZE 0x100

#define MGOS_MEL_AC_JSON_SIZE 200

struct mgos_mel_ac_packet_header {
  uint8_t magic;
  uint8_t flags;
  uint8_t ver_maj;
  uint8_t ver_min;
  uint8_t data_size;
};

// struct __attribute__((__packed__)) mgos_mel_ac_packet_control {
//   uint16_t control;  // __attribute__((packed));
//   uint8_t power;
//   uint8_t mode;
//   uint8_t temp;
//   uint8_t fan;
//   uint8_t vane_vert;
//   uint8_t reserved0;
//   uint8_t reserved1;
//   uint8_t reserved2;
//   uint8_t reserved3;
//   uint8_t reserved4;
//   uint8_t vane_horiz;
//   uint8_t temp1;
// };

struct mgos_mel_ac_packet {
  struct mgos_mel_ac_packet_header header;
  uint8_t type;
  uint8_t data[MGOS_MEL_AC_PACKET_MAX_DATA_SIZE];
  uint8_t crc;
};

union mgos_mel_ac_buffer {
  struct mgos_mel_ac_packet packet;
  uint8_t bytes[sizeof(struct mgos_mel_ac_packet)];
};

#define MGOS_MEL_AC_CONNECT_DELAY 250
#define MGOS_MEL_AC_PACKET_READ_TIMEOUT 2000
#define MGOS_MEL_AC_PACKET_SEND_DELAY 2000
#define MGOS_MEL_AC_PACKET_DATA_SIZE 15
#define MGOS_MEL_AC_TIMER_MINS 10

#define MGOS_MEL_AC_PACKET_SET_POWER 0x0001
#define MGOS_MEL_AC_PACKET_SET_MODE 0x0002
#define MGOS_MEL_AC_PACKET_SET_SETPOINT 0x0004
#define MGOS_MEL_AC_PACKET_SET_FAN 0x0008
#define MGOS_MEL_AC_PACKET_SET_VANE_HORIZ 0x0010
#define MGOS_MEL_AC_PACKET_SET_VANE_VERT 0x0100

#define MGOS_MEL_AC_PACKET_SET_EXT_TEMP 0x0001

struct mgos_mel_ac {
  uint8_t uart_no;

  bool connected;
  bool set_params;
  bool set_ext_temp;
  bool start_found;
  bool temp_flag;
  int packet_index;
  int64_t last_start;
  int64_t last_send;
  int64_t last_recv;
  uint16_t have_bytes;

  struct mgos_mel_ac_timers timers;
  struct mgos_mel_ac_packet packet;
  struct mgos_mel_ac_params params;
  struct mgos_mel_ac_params new_params;
  float room_temperature;
  float ext_temperature;
  bool operating;  // if true, the MEL is operating to reach the desired
                   // temperature

  // Service
  int svc_timer_id;
  uint16_t svc_period_ms;
};

#ifdef __cplusplus
}
#endif
