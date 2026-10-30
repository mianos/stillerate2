// Apply: GET /pid gave us the authoritative pre-apply state, stash it as the
// revert target, then hand the validated subset to POST /pid.
const cfg = flow.get('cfg');
const note = t => ({ payload: 'apply: ' + t });
if (!cfg) return [null, note('CONFIG has not run: redeploy the flow')];

const prop = flow.get('proposal') || {};
if (!prop.accepted) return [null, note('no accepted proposal to apply')];
if (typeof msg.payload !== 'object' || msg.payload === null || typeof msg.payload.kp !== 'number')
    return [null, note('GET /pid before apply returned no parameters')];
// One proposal, one apply: a second click must not re-post stale values.
if (flow.get('appliedProposalAt') === prop.at) return [null, note('this proposal was already applied')];

flow.set('lastgood', msg.payload);
if (cfg.dryRun) {
    flow.set('appliedProposalAt', prop.at);
    return [null, { payload: 'DRY RUN: would POST ' + JSON.stringify(prop.clean) }];
}

flow.set('appliedProposalAt', prop.at);
msg.payload = prop.clean;
msg.headers = { 'Content-Type': 'application/json' };
msg.action = 'apply';
return [msg, null];
