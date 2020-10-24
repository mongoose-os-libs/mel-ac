# MEL-AC lib for Mongoose OS

***Mitsubishi Electric*** AC (air condition unit) and ATW (air to water unit) control by UART using amasing IoT platform - ***Mongoose OS***

Inspired by [great research](https://nicegear.nz/blog/hacking-a-mitsubishi-heat-pump-air-conditioner/) made by [Hadley Rich](https://github.com/hadleyrich) from New Zealand and [SwiCago](https://github.com/SwiCago), who sorted all out and made an [Arduino implementation](https://github.com/SwiCago/HeatPump) and tutorial

## Hardware

The protocol was sniffed from original `Mitsubishi MAC-567IF-E` controller:

<img src="https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/MAC-567IF-E.png"/>

Mitsubishi indoor unit's control board has ***CN105*** (RED) connector for communication purposes.
The communication performed by UART @ 2400 8E1 (5V TTL). The ***CN105*** connector pinouts (1..5) = 12V, GND, 5V,TX, RX

### RobotDyn WiFi-NodeM

<img src="https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/mel-ac-nodem.png"/>

### ESP-01 + 5V adapter

<img src="https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/mel-ac-esp-01.png"/>

## Compatability

All model coming with ***CN105*** on an indoor unit's board, except PCA-RP71HAQ, PEA-RP400GAQ and PEA-RP500GAQ

More details available on [MELCloud](https://innovations.mitsubishi-les.com/en/controls/wifi-adapter) and [KumoCloud](https://www.mitsubishicomfort.com/kumocloud/compatibility) pages

## Mounting

```IMPORTANT:``` When mounting the ```MEC-AC``` unit inside an indoor unit, refer to the installation manual of the indoor unit. 
Do not mount the Interface unit inside the indoor unit, if not mentioned

```AC```'s indoor unit mount:

<img src="https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/ac_mount.png"/>


```ATW```'s indoor unit mount:

<img src="https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/atw_mount.png"/>

## Software

The `mgos_mel_ac_svc_init()` make a timer is set up at `period_ms` milliseconds intervals to handle the device.
The handler quering the HVAC params in a loop and sends new params when there are changes happen.

A callback handler in `struct mgos_mel_ac_cfg` receives event callbacks as follows:
*   `MGOS_MEL_AC_EV_INITIALIZED`: when the service initialized successfully
*   `MGOS_MEL_AC_EV_CONNECTED`: when `connected` state changed. New state goes to `*ev_data`
*   `MGOS_MEL_AC_EV_PACKET_WRITE`: after packet sent to HVAC
*   `MGOS_MEL_AC_EV_PACKET_READ`: after new packet has been read from HVAC
*   `MGOS_MEL_AC_EV_PACKET_READ_ERROR`: when there was a problem in packet from HVAC
*   `MGOS_MEL_AC_EV_PARAMS_CHANGED`: when new changed params loaded from HVAC
*   `MGOS_MEL_AC_EV_ROOMTEMP_CHANGED`: when room temperature change received from HVAC
*   `MGOS_MEL_AC_EV_TIMERS_CHANGED`: when timers changed on HVAC
*   `MGOS_MEL_AC_EV_OPERATING_CHANGED`: when operating state changed on HVAC
*   `MGOS_MEL_AC_EV_PARAMS_SET`: when new params successfully set to HVAC
*   `MGOS_MEL_AC_EV_TIMER`: before every service timer handler running
*   `MGOS_MEL_AC_EV_RX_COUNT`: when have new data to read in UART
*   `MGOS_MEL_AC_EV_CONNECT_ERROR`: when handshake packet returned error

To read params from HVAC:

```c
void mgos_mel_ac_get_params(struct mgos_mel_ac *mel, struct mgos_mel_ac_params *params);
```
To write params to HVAC:

```c
void mgos_mel_ac_set_power(struct mgos_mel_ac *mel, enum mgos_mel_ac_param_power power);
void mgos_mel_ac_set_mode(struct mgos_mel_ac *mel, enum mgos_mel_ac_param_mode mode);
void mgos_mel_ac_set_setpoint(struct mgos_mel_ac *mel, float setpoint);
void mgos_mel_ac_set_ext_temp(struct mgos_mel_ac *mel, float temp);
void mgos_mel_ac_set_fan(struct mgos_mel_ac *mel, enum mgos_mel_ac_param_fan fan);
void mgos_mel_ac_set_vane_vert(struct mgos_mel_ac *mel,
                            enum mgos_mel_ac_param_vane_vert vane_vert);
void mgos_mel_ac_set_vane_horiz(struct mgos_mel_ac *mel,
                             enum mgos_mel_ac_param_vane_horiz vane_horiz);
void mgos_mel_ac_set_params(struct mgos_mel_ac *mel, struct mgos_mel_ac_params *params);
```

It's currently tested on `ESP8266` platform. More is coming

For a complete demonstration of the driver, look at this [Mongoose App](https://github.com/mongoose-os-apps/mel-ac-demo)

## Extra

A sample case 3D design for WiFi-NodeM:

![](https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/wifi-nodem-case.gif)

Ready to print STL images available for [top](https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/nodem-top-case-v24.stl) and [bottom](https://github.com/mongoose-os-libs/mel-ac/blob/master/docs/nodem-bottom-case-v24.stl) part
