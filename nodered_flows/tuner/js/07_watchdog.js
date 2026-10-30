// Watchdog: after `revertAfterMin` of running with the new gains, compare the
// window against the pre-apply baseline and revert if it got worse.
// Outputs: 1 = POST /pid, 2 = alert, 3 = dashboard status line.
const cfg = flow.get('cfg');
const note = t => ({ payload: t });
if (!cfg) return [null, null, null];

const applied = flow.get('applied');
if (!applied) return [null, null, note('idle - nothing applied')];

const now = Date.now();
const elapsedMin = (now - applied.ts) / 60000;
const base = applied.baseline || {};
const show = v => (typeof v === 'number' ? v.toFixed(2) : '-');
const head = 'armed ' + elapsedMin.toFixed(0) + '/' + applied.revertAfterMin + ' min';

const buf = (flow.get('buf') || []).filter(function (s) {
    return !s.fault && typeof s.temp === 'number' && s.received >= applied.ts;
});
if (buf.length < 5) return [null, null, note(head + ' - waiting for samples')];

const e = buf.map(function (s) { return s.temp - (typeof s.sp === 'number' ? s.sp : 0); });
const bias = e.reduce(function (x, y) { return x + y; }, 0) / e.length;
const mae = e.reduce(function (x, y) { return x + Math.abs(y); }, 0) / e.length;
const satPct = 100 * buf.filter(function (s) { return s.saturated; }).length / buf.length;
const status = head + ', mae ' + show(mae) + ' vs ' + show(base.mae) +
               ', bias ' + show(bias) + ' vs ' + show(base.bias);
if (elapsedMin < applied.revertAfterMin) return [null, null, note(status)];

const reasons = [];
if (base.mae > 0 && mae > base.mae * (1 + cfg.watchdogWorsePct / 100))
    reasons.push('mae ' + base.mae + ' -> ' + mae.toFixed(2));
if (Math.abs(bias) > Math.abs(base.bias || 0) + cfg.watchdogBiasMargin)
    reasons.push('bias ' + base.bias + ' -> ' + bias.toFixed(2));
if (satPct > cfg.watchdogSatPct) reasons.push('saturation ' + satPct.toFixed(0) + '%');
if (buf.some(function (s) { return s.pump_stalled; })) reasons.push('pump stall');

if (!reasons.length) {
    // Healthy for several horizons: stop watching this apply.
    if (elapsedMin > applied.revertAfterMin * 4) flow.set('applied', null);
    return [null, null, note('ok at the horizon: ' + status)];
}

flow.set('applied', null);
const text = 'watchdog revert after ' + elapsedMin.toFixed(0) + ' min: ' + reasons.join(', ');
if (cfg.dryRun) return [null, note('DRY RUN: ' + text), note(text)];

msg.payload = applied.before;
msg.headers = { 'Content-Type': 'application/json' };
msg.action = 'revert';
msg.reason = text;
return [msg, null, note(text)];

