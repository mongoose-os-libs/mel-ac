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

#define MGOS_MEL_AC_PACKETS_ORDER_LEN 4
// packets order loop to send to HVAC
static enum mgos_mel_ac_packet_type
    MGOS_MEL_AC_PACKETS_ORDER[MGOS_MEL_AC_PACKETS_ORDER_LEN] = {
        MGOS_MEL_AC_PACKET_TYPE_GET_PARAMS, MGOS_MEL_AC_PACKET_TYPE_GET_TEMP,
        // MGOS_MEL_AC_PACKET_TYPE_GET_UNKNOWN,
        MGOS_MEL_AC_PACKET_TYPE_GET_TIMERS, MGOS_MEL_AC_PACKET_TYPE_GET_OPERATING};

static void mgos_mel_ac_svc_timer(void *arg) {
  struct mgos_mel_ac *mel = (struct mgos_mel_ac *) arg;

  if (!mel) return;

  if (mel->handler)
    mel->handler(mel, MGOS_MEL_AC_EV_TIMER, NULL, mel->handler_user_data);

  int64_t now = mgos_uptime_micros();

  if (now - mel->last_recv > MGOS_MEL_AC_PACKET_READ_TIMEOUT * 10 * 1e3) {
    // No packets for 10 loops? reconnecting
    mel->connected = false;
    if (mel->handler)
      mel->handler(mel, MGOS_MEL_AC_EV_CONNECTED, &mel->connected,
                   mel->handler_user_data);
  }

  if (!mel->connected) {
    if (now - mel->last_send < MGOS_MEL_AC_CONNECT_DELAY * 1e3) return;
    mgos_mel_ac_connect(mel);
  } else {
    if (mel->set_params) {
      if (now - mel->last_send < MGOS_MEL_AC_PACKET_SEND_DELAY * 1e3 / 2) return;
      mgos_mel_ac_params_update(mel);
    } else {
      if (now - mel->last_send < MGOS_MEL_AC_PACKET_SEND_DELAY * 1e3) return;
      if (mel->set_ext_temp) {
        mgos_mel_ac_ext_temp_update(mel);
        return;
      }

      memset((void *) &mel->packet.data, 0, MGOS_MEL_AC_PACKET_DATA_SIZE);
      mgos_mel_ac_packet_send(mel, MGOS_MEL_AC_PACKET_FLAGS_GET,
                           MGOS_MEL_AC_PACKETS_ORDER[mel->packet_index++],
                           MGOS_MEL_AC_PACKET_DATA_SIZE);
      if (mel->packet_index >= MGOS_MEL_AC_PACKETS_ORDER_LEN)
        mel->packet_index = 0;
    }
  }
  return;
}

bool mgos_mel_ac_svc_init(struct mgos_mel_ac *mel, uint16_t period_ms) {
  if (!mel) return false;

  if (mel->svc_timer_id > 0) {
    LOG(LL_ERROR, ("MEL-AC service already running"));
    return false;
  }
  mel->svc_period_ms = period_ms;
  mel->svc_timer_id =
      mgos_set_timer(mel->svc_period_ms, true, mgos_mel_ac_svc_timer, mel);

  LOG(LL_INFO, ("MEL-AC service running, period=%ums", mel->svc_period_ms));
  return true;
}