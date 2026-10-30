// Build the digest and the LLM request. Authoritative state comes from GET /pid;
// the time series comes from the local buffer. Anything ambiguous (stale,
// emulated, faulted, stopped, rate-limited) is refused here rather than
// described to the model as if it were a tuning problem.
const cfg = flow.get('cfg');
const say = t => ({ payload: 'digest refused: ' + t });
if (!cfg) return [null, say('CONFIG has not run: redeploy the flow')];

const snap = msg.payload || {};
if (typeof snap.kp !== 'number') return [null, say('GET /pid returned no parameters - device down?')];
const gapMin = cfg.minProposalGapMin;
if (Date.now() - (flow.get('lastAskAt') || 0) < gapMin * 60000)
    return [null, say('rate limit: one proposal per ' + gapMin + ' min')];
if (!snap.running) return [null, say('PID loop is stopped (POST /pid/run first) - nothing to tune')];
if (snap.emulation) return [null, say('emulation is on: current_temp is synthetic, not tunable')];

const now = Date.now();
const all = (flow.get('buf') || []).filter(s => s.received >= now - cfg.windowMin * 60000);
if (all.some(s => s.fault)) return [null, say('sensor fault inside the window - fix the sensor first')];
const buf = all.filter(s => typeof s.temp === 'number');
if (buf.length < 5)
    return [null, say('only ' + buf.length + ' samples in the last ' + cfg.windowMin + ' min (need 5)')];
const newest = buf[buf.length - 1];
const ageS = (now - newest.received) / 1000;
if (ageS > cfg.maxSampleAgeSec)
    return [null, say('newest sample is ' + ageS.toFixed(0) + ' s old (limit ' + cfg.maxSampleAgeSec + ' s)')];
if (buf.some(s => s.emulation)) return [null, say('emulation appeared inside the window')];
if (buf.some(s => s.pump_stalled))
    return [null, say('pump stall inside the window - fix the pump before tuning')];

const mean = a => a.reduce((x, y) => x + y, 0) / a.length;
const sp = typeof snap.set_point === 'number' ? snap.set_point : newest.sp;
const errs = buf.map(s => s.temp - (typeof s.sp === 'number' ? s.sp : sp));
const outs = buf.map(s => (typeof s.out === 'number' ? s.out : 0));
const bias = mean(errs);
let reversals = 0;
for (let i = 2; i < buf.length; i++) {
    const a1 = buf[i - 1].temp - buf[i - 2].temp;
    const a2 = buf[i].temp - buf[i - 1].temp;
    if (a1 * a2 < 0) reversals++;
}
const m = {
    n: buf.length,
    window_min: cfg.windowMin,
    sample_time_s: snap.sample_time,
    set_point: sp,
    temp_mean: +mean(buf.map(s => s.temp)).toFixed(3),
    bias: +bias.toFixed(3),                 // mean(temp - sp): + means running hot
    mae: +mean(errs.map(e => Math.abs(e))).toFixed(3),
    stddev: +Math.sqrt(mean(errs.map(e => (e - bias) * (e - bias)))).toFixed(3),
    temp_reversals_per_min: +(reversals / cfg.windowMin).toFixed(1),
    duty_mean: +mean(outs).toFixed(1),
    duty_max: +Math.max.apply(null, outs).toFixed(1),
    saturation_pct: +(100 * buf.filter(s => s.saturated).length / buf.length).toFixed(1),
    integral_peak: +Math.max.apply(null, buf.map(s => Math.abs(s.integral || 0))).toFixed(1)
};

const step = Math.max(1, Math.floor(buf.length / cfg.csvRows));
const rows = [];
for (let i = 0; i < buf.length; i += step) {
    const s = buf[i];
    rows.push([
        ((s.received - now) / 60000).toFixed(1),
        s.temp.toFixed(2),
        (typeof s.out === 'number' ? s.out : 0).toFixed(1),
        (s.P || 0).toFixed(2),
        (s.I || 0).toFixed(2),
        (s.D || 0).toFixed(2),
        s.saturated ? 1 : 0
    ].join(','));
}

// Stable prefix: identical bytes on every call is what makes the wrapper's
// prefix cache hit (cold ~20-40 s, warm ~2-3 s). Keep it free of run data.
const SYSTEM = [
'You are a control engineer assisting with a reflux distillation still. You never',
'touch hardware: you return a tuning proposal that a human reviews and that a',
'deterministic validator checks against a hard envelope.',
'',
'PLANT',
'- A PID runs every `sample_time` seconds on `current_temp`: vapour temperature at',
'  the head (top) of the column, degC.',
'- Output is reflux pump duty, percent 0..100 = coolant flow to the dephlegmator.',
'  More coolant means a cooler column.',
'- Direct acting on a cooling actuator: error = measurement - set_point, so output',
'  rises when the column is hotter than the setpoint.',
'- Setpoint is about 78.4 degC, the ethanol/water azeotrope vapour temperature.',
'  Hotter pushes water and heads through (product quality loss); colder floods the',
'  column and stops take-off. A setpoint change needs a process reason, not tuning.',
'- Expect large dead time: the pump responds in about 1 s, the column lags by tens',
'  of seconds to minutes.',
'',
'PARAMETERS (duty-percent units)',
'- kp  proportional gain, duty per degC of error.',
'- ki  integral gain per second; the integral state is bounded to +-integral_limit.',
'- kd  derivative gain per (degC/s) on the MEASUREMENT (no setpoint kick),',
'  EMA-filtered by derivative_filter (0 = off, 0.3..0.6 damps a noisy signal).',
'- integral_limit  hard bound on |I|.',
'- wind_down_threshold  below this |error| the integral leaks at',
'  integral_unwind_factor per second (0 disables the leak).',
'- output_min/output_max  duty clamp; output_min > 0 keeps the pump turning.',
'- Anti-windup is back-calculation against the output rail. Known artefact: while',
'  the output is pinned at output_min the integrator is preloaded towards',
'  (u_sat - P - D)/ki, so a small residual bias just after a cold start is',
'  expected and is not evidence of windup.',
'',
'HOW TO REASON',
'- Running hot and not saturated: raise kp (or ki) modestly.',
'- Slow return to setpoint after a disturbance: ki too low.',
'- Oscillation or jittery duty: kd too high for the sensor noise, or',
'  derivative_filter too low. Check temp_reversals_per_min first.',
'- Pinned at a rail for much of the window: the loop is authority-limited, not',
'  gain-limited. Say so instead of raising gains.',
'- integral_peak near integral_limit: windup risk; consider lowering the limit.',
'- Under about 10 minutes of data, or a fault/emulation/stall flag: the evidence is',
'  insufficient. Set confidence "low" and change nothing else.',
'- bias is mean(temp - sp) (positive = running hot), mae is mean |error|,',
'  saturation_pct is the share of samples pinned at a rail.',
'',
'RULES',
'- Change as few parameters as possible: one change per proposal, the smallest',
'  step the evidence supports.',
'- Never propose anything outside the hard envelope; the validator drops the whole',
'  proposal and the human sees nothing.',
'- Numbers must be finite JSON numbers, never strings.',
'',
'OUTPUT: one JSON object, no prose, no markdown fence:',
'{"diagnosis":"1-2 sentences on what the data shows",',
' "proposal":{"kp":2.0},',
' "expected_effect":"what should measurably change, and by when",',
' "confidence":"low|medium|high",',
' "watch_for":"the symptom that would mean this was wrong",',
' "revert_after_min":15}'
].join('\n');

const params = {
    kp: snap.kp, ki: snap.ki, kd: snap.kd, set_point: snap.set_point,
    output_min: snap.output_min, output_max: snap.output_max,
    integral_limit: snap.integral_limit,
    wind_down_threshold: snap.wind_down_threshold,
    integral_unwind_factor: snap.integral_unwind_factor,
    derivative_filter: snap.derivative_filter
};
const env = Object.keys(cfg.limits).map(function (k) {
    const b = cfg.limits[k];
    return k + ' ' + b[0] + '..' + b[1];
}).join(', ');

const req = {
    model: cfg.llmModel,
    temperature: 0,
    seed: 42,
    max_tokens: cfg.maxTokens,
    messages: [
        { role: 'system', content: SYSTEM },
        { role: 'user', content:
            'HARD ENVELOPE (the validator rejects anything outside it): ' + env +
            '\n\nCURRENT PARAMETERS\n' + JSON.stringify(params, null, 1) +
            '\n\nMETRICS, LAST ' + m.window_min + ' MIN\n' + JSON.stringify(m, null, 1) +
            '\n\nTELEMETRY, one row every ' + step + ' loop ticks, newest row last\n' +
            't_min,temp_c,duty_pct,P,I,D,sat\n' + rows.join('\n') +
            '\n\nReply with the JSON object only.' }
    ]
};

flow.set('snapshot', snap);
flow.set('digest', { metrics: m, params: params, at: now });
flow.set('lastAskAt', now);
flow.set('lastRequest', req);
flow.set('retried', false);

msg.payload = req;
msg.headers = { 'Content-Type': 'application/json' };
msg.requestTimeout = cfg.llmTimeoutMs;
return [msg, null];
