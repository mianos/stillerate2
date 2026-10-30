// Keep a bounded ring of PID telemetry for the digest. `received` is our own
// clock: a retained tele message from a previous session can be days old, and
// the digest must reject it rather than describe it as current.
const buf = flow.get('buf') || [];
const p = msg.payload || {};
const rec = { received: Date.now() };

if (p.fault === true) {
    rec.fault = true;
    rec.fail_safe = p.fail_safe === true;
    rec.held_output = p.held_output;
} else if (typeof p.current_temp === 'number' && isFinite(p.current_temp)) {
    rec.temp = p.current_temp;
    rec.sp = typeof p.set_point === 'number' ? p.set_point : null;
    rec.out = typeof p.output === 'number' ? p.output : null;
    rec.P = p.P; rec.I = p.I; rec.D = p.D;
    rec.integral = typeof p.integral === 'number' ? p.integral : null;
    rec.saturated = p.saturated === true;
    rec.emulation = p.emulation === true;
    rec.pump_ok = p.pump_ok !== false;
    rec.pump_stalled = p.pump_stalled === true;
} else {
    return null;   // unrecognised frame
}

buf.push(rec);
while (buf.length > 2000) buf.shift();
flow.set('buf', buf);
return null;
