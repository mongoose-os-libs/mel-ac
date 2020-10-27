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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "mgos.h"
#include "mgos_mel_ac_internal.h"
#include "mgos_rpc.h"
#include "mgos_uart.h"

static struct mgos_mel_ac *mel = NULL;

void mgos_mel_ac_params_update() {
  if (!mel) return;
  if (!mel->connected) return;

  uint16_t control = 0;
  memset(&mel->packet.data, 0, MGOS_MEL_AC_PACKET_DATA_SIZE);
  if (mel->params.power != mel->new_params.power) {
    mel->packet.data[2] = mel->new_params.power;
    control |= MGOS_MEL_AC_PACKET_SET_POWER;
  }
  if (mel->params.mode != mel->new_params.mode) {
    mel->packet.data[3] = mel->new_params.mode;
    control |= MGOS_MEL_AC_PACKET_SET_MODE;
  }
  if (mel->params.setpoint != mel->new_params.setpoint) {
    float value = mel->new_params.setpoint;
    value = value < 16 ? 16 : value;
    value = value > 31 ? 31 : value;
    if (mel->temp_flag) {
      mel->packet.data[13] = (int) round(value * 2) | 0x80;
    } else {
      mel->packet.data[4] = 31 - value;
    }
    control |= MGOS_MEL_AC_PACKET_SET_SETPOINT;
  }
  if (mel->params.fan != mel->new_params.fan) {
    mel->packet.data[5] = mel->new_params.fan;
    control |= MGOS_MEL_AC_PACKET_SET_FAN;
  }
  if (mel->params.vane_vert != mel->new_params.vane_vert) {
    mel->packet.data[6] = mel->new_params.vane_vert;
    control |= MGOS_MEL_AC_PACKET_SET_VANE_VERT;
  }
  if (mel->params.vane_horiz != mel->new_params.vane_horiz) {
    mel->packet.data[12] = mel->new_params.vane_horiz;
    control |= MGOS_MEL_AC_PACKET_SET_VANE_HORIZ;
  }
  if (!control) return;  // Nothing to set, skipping
  mel->packet.data[0] = (uint8_t) control & 0xFF;
  mel->packet.data[1] = (uint8_t)(control >> 8) & 0xFF;

  mgos_mel_ac_packet_send(MGOS_MEL_AC_PACKET_FLAGS_SET,
                          MGOS_MEL_AC_PACKET_TYPE_SET_PARAMS,
                          MGOS_MEL_AC_PACKET_DATA_SIZE);
}

void mgos_mel_ac_ext_temp_update() {
  if (!mel) return;
  if (!mel->connected) return;

  uint16_t control = MGOS_MEL_AC_PACKET_SET_EXT_TEMP;
  memset(&mel->packet.data, 0, MGOS_MEL_AC_PACKET_DATA_SIZE);

  float value = mel->ext_temperature;
  value = value < 0 ? 0 : value;
  value = value > 41 ? 41 : value;
  mel->packet.data[2] = (int) round(value * 2) | 0x80;

  mel->packet.data[0] = (uint8_t) control & 0xFF;
  mel->packet.data[1] = (uint8_t)(control >> 8) & 0xFF;

  mgos_mel_ac_packet_send(MGOS_MEL_AC_PACKET_FLAGS_SET,
                          MGOS_MEL_AC_PACKET_TYPE_SET_PARAMS,
                          MGOS_MEL_AC_PACKET_DATA_SIZE);
}

#define MGOS_MEL_AC_PACKETS_ORDER_LEN 4
// packets order loop to send to HVAC
static enum mgos_mel_ac_packet_type
    MGOS_MEL_AC_PACKETS_ORDER[MGOS_MEL_AC_PACKETS_ORDER_LEN] = {
        MGOS_MEL_AC_PACKET_TYPE_GET_PARAMS, MGOS_MEL_AC_PACKET_TYPE_GET_TEMP,
        // MGOS_MEL_AC_PACKET_TYPE_GET_UNKNOWN,
        MGOS_MEL_AC_PACKET_TYPE_GET_TIMERS,
        MGOS_MEL_AC_PACKET_TYPE_GET_OPERATING};

static void mgos_mel_ac_svc_timer(void *arg) {
  if (!mel) return;

  mgos_event_trigger(MGOS_MEL_AC_EV_TIMER, NULL);

  int64_t now = mgos_uptime_micros();

  if (now - mel->last_recv > MGOS_MEL_AC_PACKET_READ_TIMEOUT * 10 * 1e3) {
    // No packets for 10 loops? reconnecting
    mel->connected = false;
    mgos_event_trigger(MGOS_MEL_AC_EV_CONNECTED, &mel->connected);
  }

  if (!mel->connected) {
    if (now - mel->last_send < MGOS_MEL_AC_CONNECT_DELAY * 1e3) return;
    mgos_mel_ac_connect();
  } else {
    if (mel->set_params) {
      if (now - mel->last_send < MGOS_MEL_AC_PACKET_SEND_DELAY * 1e3 / 2)
        return;
      mgos_mel_ac_params_update();
    } else {
      if (now - mel->last_send < MGOS_MEL_AC_PACKET_SEND_DELAY * 1e3) return;
      if (mel->set_ext_temp) {
        mgos_mel_ac_ext_temp_update();
        return;
      }

      memset((void *) &mel->packet.data, 0, MGOS_MEL_AC_PACKET_DATA_SIZE);
      mgos_mel_ac_packet_send(MGOS_MEL_AC_PACKET_FLAGS_GET,
                              MGOS_MEL_AC_PACKETS_ORDER[mel->packet_index++],
                              MGOS_MEL_AC_PACKET_DATA_SIZE);
      if (mel->packet_index >= MGOS_MEL_AC_PACKETS_ORDER_LEN)
        mel->packet_index = 0;
    }
  }
  return;
}

void bin_to_hex(char *to, const unsigned char *p, size_t len) {
  static const char *hex = "0123456789abcdef";

  for (; len--; p++) {
    *to++ = hex[p[0] >> 4];
    *to++ = hex[p[0] & 0x0f];
    if (len) *to++ = ' ';
  }
  *to = '\0';
}

static uint8_t mgos_mel_ac_packet_crc(uint8_t *data, uint8_t size) {
  uint8_t crc = 0;
  for (int i = 0; i < size; i++) crc += data[i];
  return (0xFC - crc);
}

void mgos_mel_ac_packet_send(uint8_t flags, enum mgos_mel_ac_packet_type type,
                             uint8_t size) {
  mel->packet.header.magic = MGOS_MEL_AC_PACKET_MAGIC;
  mel->packet.header.flags = MGOS_MEL_AC_PACKET_FLAGS_OUT | flags;
  mel->packet.header.ver_maj = MGOS_MEL_AC_PACKET_VER_MAJ;
  mel->packet.header.ver_min = MGOS_MEL_AC_PACKET_VER_MIN;
  mel->packet.type = type;
  mel->packet.header.data_size = 1 + size;  // type + size
  mel->packet.data[size] = mgos_mel_ac_packet_crc(
      (uint8_t *) &mel->packet,
      sizeof(struct mgos_mel_ac_packet_header) + mel->packet.header.data_size);

  // debug
  char str[64] = {
      0,
  };
  bin_to_hex(str, (unsigned char *) &mel->packet,
             sizeof(struct mgos_mel_ac_packet_header) +
                 mel->packet.header.data_size + 1);
  mgos_event_trigger(MGOS_MEL_AC_EV_PACKET_WRITE, (void *) str);

  mgos_uart_write(mel->uart_no, (uint8_t *) &mel->packet,
                  sizeof(struct mgos_mel_ac_packet_header) +
                      mel->packet.header.data_size + 1);
  mgos_uart_flush(mel->uart_no);
  mel->last_send = mgos_uptime_micros();
}

static bool mgos_mel_ac_params_changed(struct mgos_mel_ac_params *params) {
  return (mel->params.power != params->power ||
          mel->params.mode != params->mode ||
          mel->params.setpoint != params->setpoint ||
          mel->params.fan != params->fan ||
          mel->params.vane_vert != params->vane_vert ||
          mel->params.vane_horiz != params->vane_horiz ||
          mel->params.isee != params->isee);
}

static bool mgos_mel_ac_timers_changed(struct mgos_mel_ac_timers *timers) {
  return (mel->timers.mode != timers->mode ||
          mel->timers.on_set != timers->on_set ||
          mel->timers.off_set != timers->off_set ||
          mel->timers.on_left != timers->on_left ||
          mel->timers.off_left != timers->off_left);
}

static void mgos_mel_ac_packet_handle() {
  // check header
  if ((mel->packet.header.flags & MGOS_MEL_AC_PACKET_FLAGS_IN) == 0) return;
  uint8_t *bytes = (uint8_t *) &mel->packet;
  uint16_t size =
      sizeof(struct mgos_mel_ac_packet_header) + mel->packet.header.data_size;
  uint8_t crc = mgos_mel_ac_packet_crc(bytes, size);

  if (crc != mel->packet.crc) {
    mgos_event_trigger(MGOS_MEL_AC_EV_PACKET_READ_ERROR, NULL);
    return;
  }

  mel->last_recv = mgos_uptime_micros();
  char str[sizeof(struct mgos_mel_ac_packet) * 2] = {
      0,
  };
  bin_to_hex(str, (unsigned char *) &mel->packet, size + 1);
  mgos_event_trigger(MGOS_MEL_AC_EV_PACKET_READ, (void *) str);

  // Got data to parse?
  if (mel->packet.header.flags & MGOS_MEL_AC_PACKET_FLAGS_GET) {
    switch (mel->packet.type) {
      case MGOS_MEL_AC_PACKET_TYPE_SET_PARAMS: {
        mgos_event_trigger(MGOS_MEL_AC_EV_PARAMS_SET, (void *) &mel->packet);
        break;
      }
      case MGOS_MEL_AC_PACKET_TYPE_GET_PARAMS: {
        struct mgos_mel_ac_params params = {MGOS_MEL_AC_PARAM_POWER_OFF,
                                            MGOS_MEL_AC_PARAM_MODE_CURRENT,
                                            0.00,
                                            MGOS_MEL_AC_PARAM_FAN_AUTO,
                                            MGOS_MEL_AC_PARAM_VANE_VERT_AUTO,
                                            MGOS_MEL_AC_PARAM_VANE_HORIZ_AUTO,
                                            MGOS_MEL_AC_PARAM_ISEE_OFF};
        // uint16_t control_mask = mel->packet.data[0] | (mel->packet.data[1] <<
        // 8);
        params.power = mel->packet.data[2];
        params.mode = mel->packet.data[3] & 0x07;
        params.isee = mel->packet.data[3] & 0x08 ? true : false;
        if (mel->packet.data[10] == 0) {
          params.setpoint = (float) (31 - mel->packet.data[4]);
        } else {
          params.setpoint = (float) (mel->packet.data[10] & 0x7F) / 2;
          mel->temp_flag = true;
        }

        params.fan = mel->packet.data[5];
        params.vane_vert = mel->packet.data[6];
        params.vane_horiz = mel->packet.data[9];

        if (mgos_mel_ac_params_changed(&params)) {
          mgos_event_trigger(MGOS_MEL_AC_EV_PARAMS_CHANGED, (void *) &params);
          mel->params = params;
          mel->new_params = params;
        }
        return;
      }
      case MGOS_MEL_AC_PACKET_TYPE_GET_TEMP: {
        // Room temperature reading
        float room_temperature;

        if (mel->packet.data[6] != 0) {
          room_temperature = (float) (mel->packet.data[5] & 0x7F) / 2;
        } else {
          room_temperature = mel->packet.data[2] + 10;
        }

        if (mel->room_temperature != room_temperature) {
          mgos_event_trigger(MGOS_MEL_AC_EV_ROOMTEMP_CHANGED,
                             (void *) &room_temperature);
          mel->room_temperature = room_temperature;
        }
        return;
      }

      case MGOS_MEL_AC_PACKET_TYPE_UNKNOWN: {
        // unknown
        break;
      }

      case MGOS_MEL_AC_PACKET_TYPE_GET_TIMERS: {
        // Timer packet
        struct mgos_mel_ac_timers timers;

        timers.mode = mel->packet.data[2];
        timers.on_set = mel->packet.data[3] * MGOS_MEL_AC_TIMER_MINS;
        timers.on_left = mel->packet.data[5] * MGOS_MEL_AC_TIMER_MINS;
        timers.off_set = mel->packet.data[4] * MGOS_MEL_AC_TIMER_MINS;
        timers.off_left = mel->packet.data[6] * MGOS_MEL_AC_TIMER_MINS;
        // Event for status change
        if (mgos_mel_ac_timers_changed(&timers)) {
          mgos_event_trigger(MGOS_MEL_AC_EV_TIMERS_CHANGED, (void *) &timers);
          mel->timers = timers;
        }
        return;
      }

      case MGOS_MEL_AC_PACKET_TYPE_GET_OPERATING: {
        // Operating
        bool operating = (bool) mel->packet.data[3];
        if (mel->operating != operating) {
          mgos_event_trigger(MGOS_MEL_AC_EV_OPERATING_CHANGED,
                             (void *) &operating);
          mel->operating = operating;
        }
        return;
      }

    }  // switch

    if (mel->packet.header.flags & MGOS_MEL_AC_PACKET_FLAGS_CONNECT) {
      // Checking for HVAC error
      if (mel->packet.type == 0) {
        mel->connected = true;
        mgos_event_trigger(MGOS_MEL_AC_EV_CONNECTED, (void *) &mel->connected);
      } else {
        mel->connected = false;
        mgos_event_trigger(MGOS_MEL_AC_EV_CONNECT_ERROR,
                           (void *) &mel->packet.type);
      }
    }
  }  // if flags
  else if (mel->packet.header.flags & MGOS_MEL_AC_PACKET_FLAGS_SET) {
    // Set params was successfull ?
    mgos_event_trigger(MGOS_MEL_AC_EV_PARAMS_SET, (void *) &mel->new_params);
    // Save new params here
    mel->params = mel->new_params;  // or may be we should wait for a sync?
    mel->set_params = false;
  }
}

void mgos_mel_ac_packet_read() {
  // Handle packet read timeout
  int64_t now = mgos_uptime_micros();
  if ((now - mel->last_start) > MGOS_MEL_AC_PACKET_READ_TIMEOUT * 1e3) {
    mel->start_found = false;
    mel->have_bytes = 0;
  }

  size_t rx_count = mgos_uart_read_avail(mel->uart_no);

  // debug only
  mgos_event_trigger(MGOS_MEL_AC_EV_RX_COUNT, (void *) &rx_count);

  uint8_t data;
  uint8_t *bytes = (uint8_t *) &mel->packet;

  while (rx_count) {
    size_t n = mgos_uart_read(mel->uart_no, &data, 1);
    if (n != 1)  // read error?
      return;
    if (mel->start_found) {
      bytes[mel->have_bytes++] = data;
      if (mel->have_bytes ==
          (sizeof(struct mgos_mel_ac_packet_header) +
           mel->packet.header.data_size + 1)) {  // have full packet already?
                                                 // got header+data+crc
        mel->packet.crc = data;
        mel->start_found = false;
        mel->have_bytes = 0;
        mgos_mel_ac_packet_handle();
        return;
      }
    } else {
      if (MGOS_MEL_AC_PACKET_MAGIC == data) {
        // Starting to read new packet
        mel->start_found = true;
        mel->have_bytes = 1;
        memset((void *) &mel->packet, 0, sizeof(struct mgos_mel_ac_packet));
        mel->packet.header.magic = data;
        mel->last_start = mgos_uptime_micros();
      }
    }
    rx_count--;
  }
}

void mgos_mel_ac_connect() {
  if (!mel) return;
  mel->connected = false;
  mel->set_params = false;
  mel->packet_index = 0;  // starting the loop
  uint8_t size = 0;
  mel->packet.data[size++] = 0x01;
  mgos_mel_ac_packet_send(
      MGOS_MEL_AC_PACKET_FLAGS_CONNECT | MGOS_MEL_AC_PACKET_FLAGS_GET,
      MGOS_MEL_AC_PACKET_TYPE_CONNECT, size);
}

void mgos_mel_ac_disconnect() {
  if (!mel) return;
  mel->connected = false;
}

static void mgos_mel_ac_uart_dispatcher(int uart_no, void *arg) {
  if (!mel) return;
  assert(uart_no == mel->uart_no);
  if (mgos_uart_read_avail(uart_no) == 0) return;
  mgos_mel_ac_packet_read();
}

void mgos_mel_ac_create() {
  LOG(LL_DEBUG, ("Creating MEL-AC object..."));
  struct mgos_uart_config ucfg;
  mel = calloc(1, sizeof(struct mgos_mel_ac));
  if (!mel) return;
  // Init all the structure members
  memset((void *) mel, 0, sizeof(struct mgos_mel_ac));

  mel->last_recv = 0;
  mel->last_send = 0;
  mel->last_start = 0;
  mel->connected = false;
  mel->start_found = false;
  mel->have_bytes = 0;

  mel->uart_no = mgos_sys_config_get_mel_ac_uart_no();

  // Initialize UART
  mgos_uart_config_set_defaults(mel->uart_no, &ucfg);
  ucfg.baud_rate = mgos_sys_config_get_mel_ac_uart_baud_rate();
  ucfg.num_data_bits = 8;
  ucfg.parity = MGOS_UART_PARITY_EVEN;
  ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
  ucfg.rx_buf_size = 128;
  ucfg.tx_buf_size = 128;
  if (!mgos_uart_configure(mel->uart_no, &ucfg)) goto err;
  mgos_uart_set_dispatcher(mel->uart_no, mgos_mel_ac_uart_dispatcher,
                           (void *) mel);
  mgos_uart_set_rx_enabled(mel->uart_no, true);

  LOG(LL_INFO, ("UART%d initialized %u,%d%c%d", mel->uart_no, ucfg.baud_rate,
                ucfg.num_data_bits,
                ucfg.parity == MGOS_UART_PARITY_NONE ? 'N' : ucfg.parity + '0',
                ucfg.stop_bits));

  mgos_event_trigger(MGOS_MEL_AC_EV_INITIALIZED, NULL);
  mel->svc_period_ms = mgos_sys_config_get_mel_ac_period_ms();
  mel->svc_period_ms = mel->svc_period_ms ? mel->svc_period_ms : 250;
  mel->svc_timer_id =
      mgos_set_timer(mel->svc_period_ms, true, mgos_mel_ac_svc_timer, NULL);

  LOG(LL_INFO, ("MEL-AC service running, period=%ums", mel->svc_period_ms));

  return;
err:
  mgos_event_trigger(MGOS_MEL_AC_EV_CONNECTED, (void *) &mel->connected);
  free(mel);
  return;
}

void mgos_mel_ac_destroy() {
  LOG(LL_DEBUG, ("Destroing MEL-AC object...."));
  if (mel) {
    mgos_uart_set_rx_enabled(mel->uart_no, false);
    mgos_clear_timer(mel->svc_timer_id);
  }
  free(mel);
  mel = NULL;
  return;
}

bool mgos_mel_ac_set_power(enum mgos_mel_ac_param_power power) {
  if (!mel) return false;
  switch (power) {
    case MGOS_MEL_AC_PARAM_POWER_OFF:
    case MGOS_MEL_AC_PARAM_POWER_ON:
      mel->new_params.power = power;
      mel->set_params = true;
      return true;
    default:
      return false;
  }
}

bool mgos_mel_ac_set_mode(enum mgos_mel_ac_param_mode mode) {
  if (!mel) return false;
  switch (mode) {
    case MGOS_MEL_AC_PARAM_MODE_CURRENT:
    case MGOS_MEL_AC_PARAM_MODE_AUTO:
    case MGOS_MEL_AC_PARAM_MODE_COOL:
    case MGOS_MEL_AC_PARAM_MODE_DRY:
    case MGOS_MEL_AC_PARAM_MODE_FAN:
    case MGOS_MEL_AC_PARAM_MODE_HEAT:
      mel->new_params.mode = mode;
      mel->set_params = true;
      return true;
    default:
      return false;
  }
}

bool mgos_mel_ac_set_setpoint(float setpoint) {
  if (!mel) return false;
  if (setpoint > 31 || setpoint < 10) return false;
  mel->new_params.setpoint = round(setpoint * 2) / 2;
  mel->set_params = true;
  return true;
}

bool mgos_mel_ac_set_ext_temp(float temp) {
  if (!mel) return false;
  if (temp > 41 || temp < 0) return false;
  mel->ext_temperature = round(temp * 2) / 2;
  mel->set_ext_temp = true;
  return true;
}

bool mgos_mel_ac_set_fan(enum mgos_mel_ac_param_fan fan) {
  if (!mel) return false;
  switch (fan) {
    case MGOS_MEL_AC_PARAM_FAN_AUTO:
    case MGOS_MEL_AC_PARAM_FAN_HIGH:
    case MGOS_MEL_AC_PARAM_FAN_LOW:
    case MGOS_MEL_AC_PARAM_FAN_MED:
    case MGOS_MEL_AC_PARAM_FAN_QUIET:
    case MGOS_MEL_AC_PARAM_FAN_TURBO:
      mel->new_params.fan = fan;
      mel->set_params = true;
      return true;
    default:
      return false;
  }
}

bool mgos_mel_ac_set_vane_vert(enum mgos_mel_ac_param_vane_vert vane_vert) {
  if (!mel) return false;
  switch (vane_vert) {
    case MGOS_MEL_AC_PARAM_VANE_VERT_AUTO:
    case MGOS_MEL_AC_PARAM_VANE_VERT_1:
    case MGOS_MEL_AC_PARAM_VANE_VERT_2:
    case MGOS_MEL_AC_PARAM_VANE_VERT_3:
    case MGOS_MEL_AC_PARAM_VANE_VERT_4:
    case MGOS_MEL_AC_PARAM_VANE_VERT_5:
    case MGOS_MEL_AC_PARAM_VANE_VERT_SWING:
      mel->new_params.vane_vert = vane_vert;
      mel->set_params = true;
      return true;
    default:
      return false;
  }
}

bool mgos_mel_ac_set_vane_horiz(enum mgos_mel_ac_param_vane_horiz vane_horiz) {
  if (!mel) return false;
  switch (vane_horiz) {
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_AUTO:
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_CENTER:
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_LEFT:
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_LEFTEST:
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_LEFTRIGHT:
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_RIGHT:
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_RIGHTEST:
    case MGOS_MEL_AC_PARAM_VANE_HORIZ_SWING:
      mel->new_params.vane_horiz = vane_horiz;
      mel->set_params = true;
      return true;
    default:
      return false;
  }
}

void mgos_mel_ac_set_params(struct mgos_mel_ac_params *params) {
  if (!mel) return;
  mel->new_params = *params;
  mel->set_params = true;
}

enum mgos_mel_ac_param_power mgos_mel_ac_get_power() {
  if (mel)
    return mel->params.power;
  else
    return MGOS_MEL_AC_PARAM_POWER_OFF;
}

enum mgos_mel_ac_param_mode mgos_mel_ac_get_mode() {
  if (mel)
    return mel->params.mode;
  else
    return MGOS_MEL_AC_PARAM_MODE_CURRENT;
}

float mgos_mel_ac_get_setpoint() {
  if (mel)
    return mel->params.setpoint;
  else
    return 0.0;
}

enum mgos_mel_ac_param_fan mgos_mel_ac_get_fan() {
  if (mel)
    return mel->params.fan;
  else
    return MGOS_MEL_AC_PARAM_FAN_AUTO;
}

enum mgos_mel_ac_param_vane_vert mgos_mel_ac_get_vane_vert() {
  if (mel)
    return mel->params.vane_vert;
  else
    return MGOS_MEL_AC_PARAM_VANE_VERT_AUTO;
}

enum mgos_mel_ac_param_vane_horiz mgos_mel_ac_get_vane_horiz() {
  if (mel)
    return mel->params.vane_horiz;
  else
    return MGOS_MEL_AC_PARAM_VANE_HORIZ_AUTO;
}

bool mgos_mel_ac_param_get_isee() {
  if (mel)
    return mel->params.isee;
  else
    return false;
}

void mgos_mel_ac_get_params(struct mgos_mel_ac_params *params) {
  if (!mel || !params) return;
  *params = mel->params;
}

bool mgos_mel_ac_get_operating() {
  if (mel)
    return mel->operating;
  else
    return false;
}

float mgos_mel_ac_get_room_temperature() {
  if (mel)
    return mel->room_temperature;
  else
    return 0.0;
}

bool mgos_mel_ac_get_connected() {
  if (mel)
    return mel->connected;
  else
    return false;
}

// RPC handlers
static void get_params_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                               struct mg_rpc_frame_info *fi,
                               struct mg_str args) {
  struct mbuf fb;
  struct json_out out = JSON_OUT_MBUF(&fb);

  mbuf_init(&fb, MGOS_MEL_AC_JSON_SIZE);

  struct mgos_mel_ac_params params;
  bool connected = mgos_mel_ac_get_connected();
  bool operating = mgos_mel_ac_get_operating();
  float room = mgos_mel_ac_get_room_temperature();
  mgos_mel_ac_get_params(&params);

  json_printf(
      &out,
      "{connected: %B, power: %d, mode: %d, setpoint: %.1f, fan: %d, "
      "vane_vert: %d, vane_horiz: %d, isee: %B, operating: %B, room: %.1f}",
      connected, params.power, params.mode, params.setpoint, params.fan,
      params.vane_vert, params.vane_horiz, params.isee, operating, room);

  mg_rpc_send_responsef(ri, "%.*s", fb.len, fb.buf);
  ri = NULL;

  mbuf_free(&fb);

  (void) cb_arg;
  (void) fi;
}

static void set_params_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                               struct mg_rpc_frame_info *fi,
                               struct mg_str args) {
  struct mbuf fb;
  struct json_out out = JSON_OUT_MBUF(&fb);

  mbuf_init(&fb, MGOS_MEL_AC_JSON_SIZE);

  struct mgos_mel_ac_params params;
  mgos_mel_ac_get_params(&params);

  bool success = true;  // true - if no errors
  if (json_scanf(args.p, args.len, "{power: %d}", &params.power) == 1) {
    if (!mgos_mel_ac_set_power(params.power)) success = false;
  }
  if (json_scanf(args.p, args.len, "{mode: %d}", &params.mode) == 1) {
    if (!mgos_mel_ac_set_mode(params.mode)) success = false;
  }
  if (json_scanf(args.p, args.len, "{setpoint: %f}", &params.setpoint) == 1) {
    if (!mgos_mel_ac_set_setpoint(params.setpoint)) success = false;
  }
  if (json_scanf(args.p, args.len, "{fan: %d}", &params.fan) == 1) {
    if (!mgos_mel_ac_set_fan(params.fan)) success = false;
  }
  if (json_scanf(args.p, args.len, "{vane_vert: %d}", &params.vane_vert) == 1) {
    if (!mgos_mel_ac_set_vane_vert(params.vane_vert)) success = false;
  }
  if (json_scanf(args.p, args.len, "{vane_horiz: %d}", &params.vane_horiz) ==
      1) {
    if (!mgos_mel_ac_set_vane_horiz(params.vane_horiz)) success = false;
  }
  // Not implemented
  // if (json_scanf(args.p, args.len, "{isee: %d}", &params.isee) == 1) {
  // 	mgos_mel_ac_set_isee(mel, params.isee);
  // }
  // Sending back changes made
  json_printf(&out,
              "{success: %B, power: %d, mode: %d, setpoint: %.1f, fan: %d, "
              "vane_vert: %d, vane_horiz: %d}",
              success, params.power, params.mode, params.setpoint, params.fan,
              params.vane_vert, params.vane_horiz);
  mg_rpc_send_responsef(ri, "%.*s", fb.len, fb.buf);
  ri = NULL;

  mbuf_free(&fb);

  (void) cb_arg;
  (void) fi;
}

bool mgos_mel_ac_init(void) {
  if (mgos_sys_config_get_mel_ac_enable()) {
    mgos_mel_ac_create();

    if (mgos_sys_config_get_mel_ac_rpc_enable()) {
      struct mg_rpc *c = mgos_rpc_get_global();
      mg_rpc_add_handler(c, "MEL-AC.GetParams", "{}", get_params_handler, NULL);
      mg_rpc_add_handler(c, "MEL-AC.SetParams",
                         "{power: %d, mode: %d, setpoint: %.1f, fan: %d, "
                         "vane_vert: %d, vane_horiz: %d}",
                         set_params_handler, NULL);
    }
  }
  return true;
}

void mgos_mel_ac_deinit(void) {
  mgos_mel_ac_destroy();
}
