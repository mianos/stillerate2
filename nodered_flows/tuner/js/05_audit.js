// Post-POST bookkeeping for both apply and revert: confirm the device took the
// values, arm (or clear) the watchdog, and emit an audit record.
const cfg = flow.get('cfg');
const say = t => ({ payload: 'apply: ' + t });
if (!cfg) return [null, say('CONFIG has not run: redeploy the flow')];

const now = Date.now();
const isRevert = msg.action === 'revert';
const before = flow.get('lastgood') || {};
const after = msg.payload || {};
if (after.error) return [null, say('POST /pid failed: ' + after.error)];

const fields = ['kp', 'ki', 'kd', 'set_point', 'integral_limit', 'wind_down_threshold',
                'integral_unwind_factor', 'derivative_filter', 'output_min', 'output_max'];
const changed = [];
for (let i = 0; i < fields.length; i++) {
    const f = fields[i];
    if (typeof after[f] === 'number' && typeof before[f] === 'number' && after[f] !== before[f])
        changed.push(f + ' ' + before[f] + ' -> ' + after[f]);
}

const prop = flow.get('proposal') || {};
const refused = Object.keys(prop.clean || {}).filter(function (f) { return after[f] !== prop.clean[f]; });

const sp = typeof before.set_point === 'number' ? before.set_point : 0;
const win = (flow.get('buf') || []).filter(function (s) {
    return !s.fault && typeof s.temp === 'number' && s.received > now - cfg.windowMin * 60000;
});
let baseline = null;
if (win.length) {
    const e = win.map(function (s) { return s.temp - (typeof s.sp === 'number' ? s.sp : sp); });
    const bias = e.reduce(function (x, y) { return x + y; }, 0) / e.length;
    baseline = {
        n: win.length,
        bias: +bias.toFixed(2),
        mae: +(e.reduce(function (x, y) { return x + Math.abs(y); }, 0) / e.length).toFixed(2),
        satPct: +(100 * win.filter(function (s) { return s.saturated; }).length / win.length).toFixed(1)
    };
}

if (isRevert) {
    flow.set('applied', null);
} else {
    flow.set('applied', {
        ts: now,
        revertAfterMin: prop.revert_after_min > 0 ? prop.revert_after_min : cfg.revertAfterMin,
        before: before,
        baseline: baseline,
        clean: prop.clean || null
    });
}

const audit = {
    event: isRevert ? 'revert' : 'apply',
    at: new Date(now).toISOString(),
    reason: msg.reason || null,
    before: before, after: after,
    diagnosis: prop.diagnosis || null,
    confidence: prop.confidence || null,
    metrics: prop.metrics || null,
    model: cfg.llmModel
};
const log = flow.get('auditLog') || [];
log.push(audit);
flow.set('auditLog', log.slice(-50));

let text = (isRevert ? 'REVERTED' : 'APPLIED') + ': ' + (changed.length ? changed.join(', ') : 'no change');
if (!isRevert && refused.length) text += ' | device kept its old value for: ' + refused.join(', ');
if (!isRevert && baseline) text += ' | baseline mae ' + baseline.mae + ', bias ' + baseline.bias +
    ', sat ' + baseline.satPct + '% over ' + baseline.n + ' samples';

return [{ payload: audit, topic: 'tele/still2/tune' },
        { payload: text }];
