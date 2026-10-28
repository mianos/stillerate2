# Reflux Still Control

ESP32-C6 controller for the still: reads RTDs mid-column (above the plates,
below the packing) and at the head, and runs a PID loop on the middle probe
that drives the reflux (dephlegmator) coolant pump.

Commands are REST on `http://still2.mianos.com`. MQTT carries telemetry only:
`tele/still2/{init,temp,pid,pumps}` and a retained `Online`/`Offline` on
`tele/still2/LWT`. Every POST is logged with the caller's address.

Success is a 2xx whose body is the resulting state. Errors return
`{"error", "statusCode"}` with the same HTTP status: 400 bad input, 404/405
unknown path or method, 413 body too large.

## Loop

```sh
curl -X POST -d '{"sample_time": 5}' http://still2.mianos.com/pid/run   # start, 5 s period
curl -X POST -d '{"sample_time": 0}' http://still2.mianos.com/pid/run   # stop (pump off)
curl http://still2.mianos.com/pid                                       # params, running, last start/stop and who sent it
curl -X POST -d '{"set_point": 78, "kp": 2}' http://still2.mianos.com/pid
curl -X POST http://still2.mianos.com/pid/reset                         # clear integral and derivative history
```

`POST /pid` accepts any subset of `kp ki kd set_point output_min output_max
integral_limit wind_down_threshold integral_unwind_factor derivative_filter`.
Invalid values are rejected and the reply shows the parameters in force. A bad
request returns 400 with an error message.

## Emulation

```sh
curl -X POST -d '{"enabled": true, "temp": 72}' http://still2.mianos.com/emulation
curl -X POST -d '{"enabled": false}' http://still2.mianos.com/emulation
```

Feeds the loop a fixed middle temperature; the real probes keep publishing.

## Sensors

```sh
curl http://still2.mianos.com/sensors           # raw code, ohms, temperature, faults per RTD
curl -X POST http://still2.mianos.com/sensors/reset
```

## Settings

```sh
curl http://still2.mianos.com/settings
curl -X POST -d '{"refluxPump": "http://reflux.mianos.com/pump"}' http://still2.mianos.com/settings
curl -X POST -d '{"middleRref": 401.5, "middleOffset": 0}' http://still2.mianos.com/settings
```

RTD calibration (`middleRref headRref middleOffset headOffset`) applies at
once; broker, `sensorName` and pump URLs take effect after a reboot. Hostname:
`curl -X POST -d '{"host_name": "still2"}' http://still2.mianos.com/set_hostname`.

## Build and update

```sh
./build.sh
curl -X POST --data-binary @build/stillerate2.bin http://still2.mianos.com/firmware
curl http://still2.mianos.com/firmware
```

An update boots pending verification and rolls back unless it reaches Wi-Fi
within 120 s.
