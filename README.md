# Reflux Still Control

ESP32-C6 controller for the still: reads RTDs mid-column (above the plates,
below the packing) and at the head, and runs a PID loop on the head (top)
probe that drives the reflux (dephlegmator) coolant pump.

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
request returns 400 with an error message. Ranges: output limits are 0..100 with
`min <= max`, `integral_limit` and `wind_down_threshold` must be >= 0, the two
filter coefficients lie in [0, 1), and every value must be finite — nothing is
persisted to NVS unless it is valid.

## Emulation

```sh
curl -X POST -d '{"enabled": true, "temp": 72}' http://still2.mianos.com/emulation
curl -X POST -d '{"enabled": false}' http://still2.mianos.com/emulation
```

Feeds the loop a fixed head (top) temperature; the real probes keep publishing.

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

## Advisory tuner (Node-RED)

`nodered_flows/tuning_flows.json` is a self-contained flow tab plus a
`/dashboard/tuning` page on the **Node-RED host**. (`still2.mianos.com` is the
ESP32's own HTTP server - it answers `/pid` and `/settings` and returns its own
JSON 404 for everything else, dashboard paths included.) Import the file in the
Node-RED editor (Menu → Import → select a file); it reuses the existing
`ui-base`, theme and MQTT broker, and does not touch the existing nodes. The
model proposes, deterministic code disposes, and a human presses Apply.

```
tele/still2/pid ──► buffer ───┐
                              ├──► prompt ──► LLM ──► validate ──► table
GET /pid ─────────────────────┘                          │
                                                         └──► one repair retry
Apply    : GET /pid ──► stash ──► POST /pid ──► audit ──► tele/still2/tune
Revert   : last good ───────────► POST /pid ──► audit
Watchdog : every 60 s ──────────► POST /pid ──► audit
```

Rules the code enforces, not the model:

- **Authoritative state comes from `GET /pid`**, never from the MQTT stream.
  A retained `tele/still2/pid` can be hours old, so samples are stamped with
  arrival time and the digest is refused if the newest is older than 30 s,
  if the window is short, or if it contains a fault, emulation or pump stall.
- **Hard envelope** (a proposal outside it is dropped whole, not clamped):
  `kp 0..8`, `ki 0..1`, `kd 0..60`, `set_point 70..85`,
  `output_min 0..50`, `output_max 20..100`, `integral_limit 0..10000`,
  `wind_down_threshold 0..10`, `integral_unwind_factor 0..0.9`,
  `derivative_filter 0..0.9`.
- **Step limits**: at most +-30% per gain, +-0.5 degC on `set_point`, one
  proposal per 5 minutes.
- **Reply handling**: tolerant JSON extraction (fences and prose tolerated), one
  repair retry, then a rejection. `finish_reason: length` is a rejection;
  a tool call is a retry. The reply contract is
  `{diagnosis, proposal, expected_effect, confidence, watch_for, revert_after_min}`.
- **Apply** re-reads `GET /pid` first, so the revert target is the state that was
  actually in force, and refuses to post the same proposal twice. Only the
  validated subset is sent; fields the device silently ignored are reported.
- **Watchdog** compares the 15 minutes after an apply with the 10 minutes before
  it, shows its countdown and both figures on the dashboard every 60 s, and
  reverts on `mae` +25%, `|bias|` +0.3 degC, saturation over 40%, or a pump
  stall. It retires the watch after four clean horizons.

`cfg.dryRun` in the `CONFIG (edit me)` node is `true` by default: every button
works, shows what it would send, and writes nothing to the still. Audit records
go to `tele/still2/tune` (and the last 50 stay in `flow` context).

The dashboard page is a thin view: a state line, a proposal table with the
current/proposed/percent change, the diagnosis, the last action, the watchdog
line, and the Build digest / Apply / Revert buttons.

## Build and update

```sh
./build.sh
curl -X POST --data-binary @build/stillerate2.bin http://still2.mianos.com/firmware
curl http://still2.mianos.com/firmware
```

An update boots pending verification and rolls back unless it reaches Wi-Fi
within 120 s.
