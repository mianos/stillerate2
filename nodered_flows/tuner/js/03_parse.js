// Validate the model's reply. This is the trust boundary: the LLM proposes, this
// code disposes. Outputs: 1 = table, 2 = diagnosis, 3 = rejected, 4 = retry.
const cfg = flow.get('cfg');
const snap = flow.get('snapshot') || {};
const out2 = t => ({ payload: 'proposal rejected: ' + t });
const reject = why => {
    flow.set('proposal', { accepted: false, why: why, at: Date.now() });
    return [null, null, out2(why), null];
};
if (!cfg) return reject('CONFIG has not run: redeploy the flow');

const choice = (msg.payload && msg.payload.choices && msg.payload.choices[0]) || {};
const text = (choice.message && choice.message.content) || '';
const retryOrReject = why => {
    if (flow.get('retried')) return reject('after one retry: ' + why);
    const req = JSON.parse(JSON.stringify(flow.get('lastRequest') || {}));
    req.messages = (req.messages || []).concat([{ role: 'user', content:
        'That reply could not be used (' + why + '). Reply with the JSON object only, nothing else.' }]);
    flow.set('retried', true);
    msg.payload = req;
    msg.headers = { 'Content-Type': 'application/json' };
    msg.requestTimeout = cfg.llmTimeoutMs;
    return [null, null, null, msg];
};

if (!text) return retryOrReject('empty reply');
if (choice.finish_reason === 'length') return reject('model hit max_tokens before finishing');
if (choice.finish_reason === 'tool_calls') return retryOrReject('reply was a tool call, not a proposal');

const body = String(text).trim().replace(/^```[a-zA-Z]*\n?/, '').replace(/```$/, '').trim();
const a = body.indexOf('{');
const b = body.lastIndexOf('}');
if (a < 0 || b <= a) return retryOrReject('no JSON object in the reply');
let parsed;
try {
    parsed = JSON.parse(body.slice(a, b + 1));
} catch (e) {
    return retryOrReject('invalid JSON: ' + e.message);
}

const raw = (parsed && typeof parsed.proposal === 'object' && parsed.proposal !== null)
    ? parsed.proposal : parsed;
const fields = Object.keys(cfg.limits);
const problems = [];
const clean = {};
for (let i = 0; i < fields.length; i++) {
    const f = fields[i];
    if (raw[f] === undefined || raw[f] === null) continue;
    const v = Number(raw[f]);
    if (!isFinite(v)) { problems.push(f + ' is not a finite number'); continue; }
    const lo = cfg.limits[f][0];
    const hi = cfg.limits[f][1];
    if (v < lo || v > hi) {
        problems.push(f + '=' + v + ' outside the envelope ' + lo + '..' + hi);
        continue;
    }
    clean[f] = v;
}
if (!Object.keys(clean).length) problems.push('no usable parameter in the proposal');
if (clean.output_min !== undefined && clean.output_max !== undefined && clean.output_min > clean.output_max)
    problems.push('output_min > output_max');
for (let i = 0; i < 3; i++) {
    const f = ['kp', 'ki', 'kd'][i];
    if (clean[f] === undefined) continue;
    const old = Number(snap[f]);
    if (!isFinite(old)) continue;
    const pct = Math.abs(clean[f] - old) / Math.max(Math.abs(old), 0.01) * 100;
    if (pct > cfg.maxDeltaPct)
        problems.push(f + ' would move ' + pct.toFixed(0) + '% (cap ' + cfg.maxDeltaPct + '%)');
}
if (clean.set_point !== undefined) {
    const d = Math.abs(clean.set_point - Number(snap.set_point));
    if (!(d <= cfg.maxSetPointStep))
        problems.push('set_point would move ' + d.toFixed(2) + ' degC (cap ' + cfg.maxSetPointStep + ')');
}
if (problems.length) return reject(problems.join('; '));

const proposal = {
    accepted: true, at: Date.now(), clean: clean,
    diagnosis: String(parsed.diagnosis || ''),
    confidence: String(parsed.confidence || 'low'),
    expected_effect: String(parsed.expected_effect || ''),
    watch_for: String(parsed.watch_for || ''),
    revert_after_min: Number(parsed.revert_after_min) > 0 ? Number(parsed.revert_after_min) : cfg.revertAfterMin,
    before: snap, model: cfg.llmModel,
    metrics: (flow.get('digest') || {}).metrics || null
};
flow.set('proposal', proposal);

const rows = Object.keys(clean).map(function (f) {
    const cur = Number(snap[f]);
    const pct = isFinite(cur) && cur !== 0 ? +(100 * (clean[f] - cur) / Math.abs(cur)).toFixed(1) : null;
    return { field: f, current: isFinite(cur) ? cur : snap[f], proposed: clean[f], change_pct: pct };
});
msg.payload = rows;
// The dashboard ui-text widget renders msg.payload only (its `format` property is
// ignored in this version), so the diagnosis goes out as its own payload string.
const diag = [proposal.diagnosis,
    'confidence: ' + proposal.confidence,
    proposal.expected_effect ? 'expected: ' + proposal.expected_effect : null,
    proposal.watch_for ? 'watch for: ' + proposal.watch_for : null
].filter(Boolean).join('<br><br>');
return [msg, { payload: diag }, null, null];
