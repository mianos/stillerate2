"""Run the shipped function-node code against a synthetic still to prove the
digest -> validate -> apply -> watchdog chain behaves. Uses the exact strings
from nodered_flows/tuning_flows.json (not the source files)."""
import json
import os
import quickjs

TUN = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tuning_flows.json'))
nodes = json.load(open(TUN))
ctx = quickjs.Context()
ctx.eval("""
var _ctx = {};
var flow = { get: function (k) { return _ctx[k]; }, set: function (k, v) { _ctx[k] = v; } };
var node = { warn: function () {}, error: function () {}, log: function () {},
             status: function () {}, send: function () {} };
var FN = {};
""")

fns = {}
for nd in nodes:
    if nd.get('type') != 'function':
        continue
    fns[nd['name']] = nd['id']
    if nd.get('initialize'):
        ctx.eval('FN["%s__init"] = function () {\n%s\n};' % (nd['id'], nd['initialize']))
    ctx.eval('FN["%s"] = function (msg) {\n%s\n};' % (nd['id'], nd['func']))


def call(name, msg=None):
    ctx.eval('var __m = ' + json.dumps(msg) + ';')
    raw = ctx.eval('(function () { var o = FN[%s](__m);'
                   ' return JSON.stringify(o === undefined ? null : o); })()'
                   % json.dumps(fns[name]))
    return json.loads(raw)


def init(name):
    ctx.eval('FN[%s]();' % json.dumps(fns[name] + '__init'))


def ctxget(key):
    return json.loads(ctx.eval('JSON.stringify(_ctx[%s] === undefined ? null : _ctx[%s])'
                               % (json.dumps(key), json.dumps(key))))


def ctxset(key, value):
    ctx.eval('_ctx[%s] = %s;' % (json.dumps(key), json.dumps(value)))


fails = []


def ok(cond, what):
    print(('PASS  ' if cond else 'FAIL  ') + what)
    if not cond:
        fails.append(what)


# ---------------------------------------------------------------- config ----
init('CONFIG (edit me)')
cfg = ctxget('cfg')
ok(cfg['dryRun'] is False, 'CONFIG now defaults to dryRun=false (Apply really posts)')
ok(cfg['limits']['kp'] == [0, 8] and cfg['limits']['set_point'] == [70, 85],
   'CONFIG carries the hard envelope')

# ------------------------------------------------------------- telemetry ----
# 10 minutes of a still running 1.5 degC hot, drifting, with noise, 5 s loop.
frames = []
for i in range(121):
    temp = 79.9 + 0.6 * (i / 120.0) + (0.05 if i % 2 else -0.05)
    frames.append({"current_temp": round(temp, 3), "error": round(temp - 78.4, 3),
                   "output": min(100.0, 55 + i * 0.15), "P": 3.0, "I": 52.0, "D": -0.4,
                   "integral": 260.0 + i, "dt": 5.0, "saturated": False,
                   "kp": 2.0, "ki": 0.02, "kd": 10.0, "set_point": 78.4,
                   "integral_limit": 500.0, "wind_down_threshold": 0.5,
                   "integral_unwind_factor": 0.1, "derivative_filter": 0.3,
                   "output_min": 5.0, "output_max": 95.0,
                   "pump_ok": True, "pump_stalled": False,
                   "time": "2026-10-29T12:00:00Z"})
for i, f in enumerate(frames):
    call('buffer samples', {"topic": "tele/still2/pid", "payload": f})
    ctx.eval('_ctx.buf[_ctx.buf.length - 1].received = Date.now() - %d;' % ((len(frames) - 1 - i) * 5000))
ok(len(ctxget('buf')) == 121, 'buffer samples holds every frame (121)')

call('buffer samples', {"payload": {"fault": True, "fail_safe": False, "held_output": 55}})
ok(ctxget('buf')[-1].get('fault') is True, 'buffer samples keeps fault frames flagged')
ctx.eval('_ctx.buf.pop();')

SNAP = {"kp": 2.0, "ki": 0.02, "kd": 10.0, "set_point": 78.4, "output_min": 5.0,
        "output_max": 95.0, "integral_limit": 500.0, "wind_down_threshold": 0.5,
        "integral_unwind_factor": 0.1, "derivative_filter": 0.3,
        "running": True, "sample_time": 5.0, "emulation": False,
        "last_run_sample_time": 5.0, "last_run_from": "192.168.1.9"}

# ---------------------------------------------------------------- prompt ----
out = call('build prompt', {"payload": SNAP})
req = out[0]['payload']
user = req['messages'][1]['content']
ok(req['temperature'] == 0 and req['seed'] == 42, 'prompt asks for deterministic sampling')
ok('METRICS, LAST 10 MIN' in user and 't_min,temp_c,duty_pct' in user,
   'prompt carries metrics and a CSV block')
ok('HARD ENVELOPE' in user and 'kp 0..8' in user, 'prompt states the hard envelope')
ok(user.split('\n')[-1].startswith('Reply with the JSON object only'),
   'run data is last, stable prefix first (prefix cache)')
metrics = ctxget('digest')['metrics']
ok(metrics['n'] == 120 and 1.5 < metrics['bias'] < 2.1, 'digest sees 120 samples, bias about +1.8 degC')
ok(abs(metrics['mae'] - metrics['bias']) < 1e-6, 'mae equals bias when the error never changes sign')
ok(out[1] is None, 'healthy digest returns no error')

# -------------------------------------------------------------- validate ----
def llm(content, finish='stop'):
    return {"choices": [{"finish_reason": finish, "message": {"content": content}}],
            "usage": {"completion_tokens": 90}}


GOOD = {"diagnosis": "Sitting 1.8 degC hot at 64% duty with no saturation: kp is short.",
        "proposal": {"kp": 2.4}, "expected_effect": "error roughly halves within 10 min",
        "confidence": "medium", "watch_for": "duty oscillating at the sample rate",
        "revert_after_min": 15}

out = call('validate proposal', {"payload": llm("```json\n" + json.dumps(GOOD) + "\n```")})
rows = out[0]['payload']
ok(rows == [{"field": "kp", "current": 2.0, "proposed": 2.4, "change_pct": 20.0}],
   'accepted proposal becomes one table row with the % change')
ok('confidence: medium' in out[1]['payload'], 'diagnosis text carries the confidence')
ok(ctxget('proposal')['accepted'] is True and ctxget('proposal')['clean'] == {"kp": 2.4},
   'validated subset stored for Apply')


def reject_case(label, content, needle, finish='stop'):
    ctxset('retried', False)
    res = call('validate proposal', {"payload": llm(content, finish)})
    txt = (res[2] or {}).get('payload', '')
    ok(res[0] is None and needle in txt, label + '  [' + txt[:78] + ']')
    return res


reject_case('kp outside the envelope is refused', json.dumps({"proposal": {"kp": 12}}), 'outside the envelope')
reject_case('a 200% gain jump is refused', json.dumps({"proposal": {"kp": 6}}), 'would move')
reject_case('a 1.6 degC setpoint jump is refused', json.dumps({"proposal": {"set_point": 80}}), 'set_point would move')
reject_case('output_min > output_max is refused',
            json.dumps({"proposal": {"output_min": 40, "output_max": 30}}), 'output_min > output_max')
reject_case('a non-numeric gain is refused', json.dumps({"proposal": {"kp": "lots"}}), 'not a finite number')
reject_case('an empty proposal is refused', json.dumps({"proposal": {}}), 'no usable parameter')
reject_case('a truncated reply is refused', json.dumps({"answer": "raise kp"}), 'max_tokens', finish='length')

ctxset('retried', False)
res = call('validate proposal', {"payload": llm('Raise kp to 3.')})
ok(res[0] is None and res[1] is None and res[2] is None and res[3] is not None and
   len(res[3]['payload']['messages']) == 3,
   'prose instead of JSON triggers exactly one retry')
res = call('validate proposal', {"payload": llm('Raise kp to 3.')})
txt = (res[2] or {}).get('payload', '')
ok(res[3] is None and 'after one retry' in txt, 'a second unparseable reply is rejected  [' + txt + ']')

out = call('validate proposal', {"payload": llm(json.dumps({"proposal": {"ki": "0.024"}}))})
ok(out[0] and out[0]['payload'][0]['proposed'] == 0.024, 'numeric strings are coerced, not trusted blindly')


# ----------------------------------------------------------------- apply ----
# Default is now dryRun=false; prove the dry-run path explicitly first.
cfg = ctxget('cfg')
cfg['dryRun'] = True
ctxset('cfg', cfg)
call('validate proposal', {"payload": llm(json.dumps(GOOD))})
out = call('stash + build POST', {"payload": SNAP})
ok(out[0] is None and out[1]['payload'].startswith('DRY RUN: would POST'),
   'Apply is a no-op while dryRun is true')
ok(ctxget('lastgood')['kp'] == 2.0, 'the revert target is captured even in a dry run')

cfg = ctxget('cfg')
cfg['dryRun'] = False
ctxset('cfg', cfg)
ctxset('appliedProposalAt', None)   # the sim runs sub-millisecond, so Date.now() repeats
call('validate proposal', {"payload": llm(json.dumps(GOOD))})   # new proposal, new timestamp
out = call('stash + build POST', {"payload": SNAP})
ok(out[0]['payload'] == {"kp": 2.4} and out[0]['action'] == 'apply',
   'Apply posts only the validated subset')
ok(out[0]['headers']['Content-Type'] == 'application/json', 'POST carries a JSON content type')
res = call('stash + build POST', {"payload": SNAP})
ok('already applied' in (res[1] or {}).get('payload', ''),
   'a second click cannot re-post the same proposal')

after = dict(SNAP)
after['kp'] = 2.4
out = call('audit', {"action": "apply", "payload": after})
ok(out[0]['payload']['event'] == 'apply' and out[0]['topic'] == 'tele/still2/tune',
   'audit record is addressed to tele/still2/tune')
ok('APPLIED: kp 2 -> 2.4' in out[1]['payload'], 'status text: ' + out[1]['payload'])
ok(ctxget('applied') is not None and ctxget('applied')['baseline']['n'] > 0,
   'watchdog armed with a pre-apply baseline')
ok(len(ctxget('auditLog')) == 1, 'audit log keeps the record')

out = call('audit', {"action": "apply", "payload": dict(SNAP)})
ok('device kept its old value for: kp' in out[1]['payload'],
   'a silently ignored field is surfaced: ' + out[1]['payload'])

# --------------------------------------------------------------- watchdog ----
ctx.eval('_ctx.applied.ts = Date.now();')
out = call('watchdog', {})
ok(out[0] is None and out[1] is None and 'armed' in out[2]['payload'],
   'watchdog is quiet before the horizon and reports its clock: ' + out[2]['payload'])
ctx.eval('_ctx.applied.ts = Date.now() - 20 * 60000;')
for _ in range(20):
    call('buffer samples', {"payload": dict(frames[-1], current_temp=86.0)})
    ctx.eval('_ctx.buf[_ctx.buf.length - 1].received = Date.now() - 1000;')
out = call('watchdog', {})
ok(out[0] and out[0]['action'] == 'revert', 'watchdog reverts when bias and mae collapse')
ok(out[0]['payload']['kp'] == 2.0, 'watchdog reverts to the pre-apply snapshot')
ok('watchdog revert after 20 min' in out[2]['payload'],
   'watchdog tells the dashboard why: ' + out[2]['payload'])
ok(ctxget('applied') is None, 'watchdog disarms after reverting')

ctxset('applied', {"ts": 1, "revertAfterMin": 15, "before": SNAP,
                   "baseline": {"n": 120, "bias": 7.6, "mae": 7.6, "satPct": 0},
                   "clean": {"kp": 2.4}})
out = call('watchdog', {})
ok(out[0] is None and out[1] is None and 'ok at the horizon' in out[2]['payload']
   and ctxget('applied') is None,
   'watchdog leaves a healthy apply alone, then retires it after four horizons')

# ----------------------------------------------------------------- revert ----
ctxset('lastgood', SNAP)
out = call('revert now', {})
ok(out[0]['payload']['kp'] == 2.0 and out[0]['action'] == 'revert',
   'manual revert posts the last good snapshot')
ok(sorted(out[0]['payload']) == sorted(['kp', 'ki', 'kd', 'set_point', 'output_min', 'output_max',
                                        'integral_limit', 'wind_down_threshold',
                                        'integral_unwind_factor', 'derivative_filter']),
   'revert sends the whole parameter set (no half-restored loop)')

# ------------------------------------------------- digest refusal gates ----
# The retained tele/still2/pid message that misled the model earlier is exactly
# what these gates exist for.
def refusal(label, needle):
    res = call('build prompt', {"payload": SNAP})
    txt = (res[1] or {}).get('payload', '')
    ok(res[0] is None and needle in txt, label + '  [' + txt + ']')


ctxset('lastAskAt', 0)
ctx.eval("""
var t = Date.now();
_ctx.buf = [];
for (var i = 0; i < 6; i++) _ctx.buf.push({received: t - 60000 + i * 5000, temp: 79.0,
    sp: 78.4, out: 50, P: 2, I: 48, D: 0, integral: 240, saturated: false,
    emulation: false, pump_ok: true, pump_stalled: false});
""")
refusal('a stale sample is refused', 's old (limit 30 s)')

ctx.eval('_ctx.buf.forEach(function (s) { s.received = Date.now() - 1000; });')
ctx.eval('_ctx.buf[5].emulation = true;')
refusal('emulation inside the window is refused', 'emulation appeared')

ctx.eval('_ctx.buf[5].emulation = false;')
ctx.eval('_ctx.buf.push({received: Date.now() - 5000, fault: true, fail_safe: false});')
refusal('a sensor fault inside the window is refused', 'sensor fault')

ctx.eval('_ctx.buf.pop();')
ctx.eval('_ctx.buf.push({received: Date.now() - 5000, temp: 79, sp: 78.4, out: 50, ' +
         'P: 2, I: 48, D: 0, integral: 240, saturated: false, emulation: false, ' +
         'pump_ok: true, pump_stalled: true});')
refusal('a pump stall inside the window is refused', 'pump stall')

ctx.eval('_ctx.buf.pop();')
ctxset('lastAskAt', 9999999999999)
refusal('the rate limit refuses a second call within 5 min', 'rate limit')

ctxset('lastAskAt', 0)
stopped = dict(SNAP)
stopped['running'] = False
res = call('build prompt', {"payload": stopped})
ok(res[0] is None and 'stopped' in res[1]['payload'], 'a stopped loop is refused')

emu = dict(SNAP)
emu['emulation'] = True
res = call('build prompt', {"payload": emu})
ok(res[0] is None and 'emulation is on' in res[1]['payload'], 'emulation mode is refused')

res = call('build prompt', {"payload": {"error": "not found"}})
ok(res[0] is None and 'device down' in res[1]['payload'], 'a dead device is refused, not digested')

# A snapshot with too few samples is refused rather than described to the model.
ctx.eval('_ctx.buf = _ctx.buf.slice(0, 3);')
ctxset('lastAskAt', 0)
res = call('build prompt', {"payload": SNAP})
ok(res[0] is None and 'only 3 samples' in res[1]['payload'], 'a short window is refused')

print()
if fails:
    print('%d FAILED' % len(fails))
    raise SystemExit(1)
print('all checks passed')


