// Manual revert to the snapshot captured just before the last apply.
const cfg = flow.get('cfg');
const note = t => ({ payload: 'revert: ' + t });
if (!cfg) return [null, note('CONFIG has not run: redeploy the flow')];

const lastgood = flow.get('lastgood');
if (!lastgood || typeof lastgood.kp !== 'number') return [null, note('nothing to revert to')];

const body = {};
['kp', 'ki', 'kd', 'set_point', 'output_min', 'output_max', 'integral_limit',
 'wind_down_threshold', 'integral_unwind_factor', 'derivative_filter'].forEach(function (f) {
    if (typeof lastgood[f] === 'number') body[f] = lastgood[f];
});

if (cfg.dryRun) return [null, note('DRY RUN - would POST ' + JSON.stringify(body))];

msg.payload = body;
msg.headers = { 'Content-Type': 'application/json' };
msg.action = 'revert';
msg.reason = 'manual';
return [msg, null];
