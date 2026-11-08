# Reflux Still Control

ESP32-C6 controller for the still: reads RTDs mid-column (above the plates,
below the packing) and at the head, and runs a PID loop on the head (top)
probe that drives the reflux (dephlegmator) coolant pump.

Commands are REST on `http://still2.mianos.com`. MQTT carries telemetry only:
`tele/still2/{init,temp,pid,pumps,safety}` and a retained `Online`/`Offline` on
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
curl -X POST -d '{"pidSource": "middle"}' http://still2.mianos.com/settings
curl -X POST -d '{"dangerTemp": 98.5, "dangerHyst": 3}' http://still2.mianos.com/settings
curl -X POST -d '{"powerCutUrl": "http://iampwr.mianos.com/api/setpower?x=0"}' http://still2.mianos.com/settings
```

RTD calibration (`middleRref headRref middleOffset headOffset`), `pidSource`
and the safety knobs (`dangerTemp dangerHyst powerCutUrl`) apply at once;
broker, `sensorName` and the reflux/condenser pump URLs take effect after a
reboot. Hostname:
`curl -X POST -d '{"host_name": "still2"}' http://still2.mianos.com/set_hostname`.

## Safety cutoff

The firmware watches the boiler (head) probe and cuts the element if the
boiler runs away. Every 2 s it compares the head temperature against
`dangerTemp` (default 98.5 °C); when it is exceeded it GETs `powerCutUrl`
(`http://iampwr.mianos.com/api/setpower?x=0`) to force the element to 0 W,
and re-asserts that cut every 10 s while the probe stays over the limit, so a
manual re-apply on the Power page is overridden. The trip is logged and
published on `tele/still2/safety` as `{"tripped":true,"boiler_c":C}`; once the
boiler falls below `dangerTemp - dangerHyst` (default 95.5 °C) it re-arms and
publishes `{"tripped":false,"boiler_c":C}`. `dangerTemp`, `dangerHyst` and
`powerCutUrl` are NVS-persisted settings.

The PID loop has a separate sensor-fault fail-safe. A reading that is a fault,
non-finite, or outside −40..200 °C is never fed to the PID: the last output is
held for up to 3 consecutive samples, then the pump is driven to the fail-safe
output (`output_max`, i.e. full reflux) while the fault persists. Each fault
is published on `tele/still2/pid` as
`{"fault":true,"fail_safe":…,"held_output":…,"consecutive_faults":…}`.

## Control flows (Node-RED)

`nodered_flows/flows.json` is the live still-control flow running on the
Node-RED host (the same one that hosts the advisory tuner below). It provides
the dashboard: temperature charts, boiler power (0–2400 W via the iampwr
`/api/setpower?x=<W>` endpoint), the reflux/condenser pump duty sliders, PID
start/stop, and two heat-up helpers.

- **Heat-up mode** (Power page): a manual on/off that fires the boiler at
  2400 W, then drops to a run power (default 1200 W) once the vapour probe
  (`middle`) reaches a switch temperature (default 50 °C). It manages the
  element power only; it does not start the PID.
- **Stilling schedule** (Schedule group): an armed delayed start. At the armed
  time it verifies the condenser pump came up, fires the boiler at 2400 W, and
  then — when the vapour probe reaches the switch temperature (default 78 °C)
  — drops the boiler to the run power (default 1200 W) and sets the reflux
  pump to 100 % (full reflux). After the settle time (default 30 min) it
  starts the PID via `/pid/run`, which takes over the reflux pump. Run power
  and switch temperature are editable in the Schedule group; Cancel clears the
  heat-up state.

The schedule's heat-up stage runs in Node-RED rather than the firmware; the
ESP32's own over-temperature cutoff (see Safety cutoff) is the independent
backstop for the boiler.

## Advisory tuner (Node-RED)

`nodered_flows/tuning_flows.json` is a self-contained flow tab plus a
`/dashboard/tuning` page on the **Node-RED host**. (`still2.mianos.com` is the
ESP32's own HTTP server - it answers `/pid` and `/settings` and returns its own
JSON 404 for everything else, dashboard paths included.) Import the file in the
Node-RED editor (Menu → Import → select a file); it reuses the existing
`ui-base`, theme and MQTT broker, and does not touch the existing nodes. The
model proposes, deterministic code disposes, and a human presses Apply.

The flow is generated, not hand-edited: `nodered_flows/tuner/gen.py` builds
`tuning_flows.json` from the function code in `nodered_flows/tuner/js/`. Check
it with `nodered_flows/test/check_flow.py`, exercise the logic with
`nodered_flows/test/sim.py` (`pip install -r nodered_flows/test/requirements.txt`),
deploy with `nodered_flows/test/deploy.py`, and roll back with
`nodered_flows/test/rollback.py`.

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

`cfg.dryRun` in the `CONFIG (edit me)` node is `false` by default: Apply really
POSTs to the still, and the watchdog then watches for 15 minutes, reverting if
it gets worse. Set `dryRun` to `true` to rehearse dry runs. Audit records go to
`tele/still2/tune` (and the last 50 stay in `flow` context).

The dashboard page is a thin view: a state line, a proposal table with the
current/proposed/percent change, the diagnosis, the last action, the watchdog
line, and the Build digest / Apply / Revert buttons.

## Build and update

```sh
./build.sh
curl -X POST --data-binary @build/stillerate2.bin http://still2.mianos.com/firmware
curl http://still2.mianos.com/firmware
```

The build tree does not track header dependencies (this ESP-IDF build generates
no ninja depfiles), so a header-only change will not trigger a recompile. Touch
the sources that include the changed header first (e.g. `touch main/main.cpp
main/StillWebServer.cpp`), or do a full rebuild with `rm -rf build && ./build.sh`.

An update boots pending verification and rolls back unless it reaches Wi-Fi
within 120 s.
