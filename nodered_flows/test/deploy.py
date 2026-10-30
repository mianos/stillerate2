#!/usr/bin/env python3
"""Merge nodered_flows/tuning_flows.json into the live Node-RED instance and deploy.

Fresh GET (so the rev is current), merge by id, validate, POST, verify.
"""
import json
import os
import sys
import time
import urllib.error
import urllib.request

BASE = 'http://nodered.mianos.com:1880/admin'
HERE = os.path.dirname(os.path.abspath(__file__))
MINE = os.path.abspath(os.path.join(HERE, '..', 'tuning_flows.json'))
BACKUP = os.path.join(HERE, 'backup')


def request(path, method='GET', body=None, headers=None):
    data = None if body is None else json.dumps(body).encode()
    h = {'Content-Type': 'application/json'}
    if headers:
        h.update(headers)
    req = urllib.request.Request(BASE + path, data=data, headers=h, method=method)
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status, dict(r.headers), r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read().decode()


def get_flows():
    # v2: the rev is in the body as {rev, flows}. (The ETag is just Express's
    # hash of the response body and is not accepted as a rev.)
    status, headers, text = request('/flows', headers={'Node-RED-API-Version': 'v2'})
    if status != 200:
        sys.exit('GET /flows failed: %s %s' % (status, text[:200]))
    doc = json.loads(text)
    return doc['flows'], doc['rev']


def validate(merged, mine):
    ids = [n['id'] for n in merged]
    dupes = {i for i in ids if ids.count(i) > 1}
    assert not dupes, 'duplicate ids after merge: %s' % dupes
    known = set(ids)
    # In this dashboard version these node types are global config, not tab
    # members: they are referenced by `ui`/`page`/`group`, and carry no `z`.
    no_z = {'tab', 'ui-base', 'ui-theme', 'ui-page', 'ui-group', 'mqtt-broker'}
    for n in mine:
        for out in n.get('wires', []):
            for target in out:
                assert target in known, '%s wires to unknown %s' % (n['id'], target)
        if n.get('group'):
            assert n['group'] in known, '%s has unknown group %s' % (n['id'], n['group'])
        if n.get('page'):
            assert n['page'] in known, '%s has unknown page %s' % (n['id'], n['page'])
        if n.get('ui'):
            assert n['ui'] in known, '%s has unknown ui-base %s' % (n['id'], n['ui'])
        if n.get('broker'):
            assert n['broker'] in known, '%s has unknown broker %s' % (n['id'], n['broker'])
        if n['type'] not in no_z:
            assert n.get('z') in known, '%s has no valid tab %s' % (n['id'], n.get('z'))


def deploy(attempt=1):
    live, rev = get_flows()
    mine = json.load(open(MINE))
    live_ids = {n['id'] for n in live}
    mine_ids = {n['id'] for n in mine}

    stale = [n for n in live if n['id'] in mine_ids]
    if stale:
        print('replacing %d tuner nodes already on the instance' % len(stale))
        live = [n for n in live if n['id'] not in mine_ids]

    print('live %d nodes (rev %s), adding %d' % (len(live), rev, len(mine)))
    merged = live + mine
    validate(merged, mine)
    print('merge validated: %d nodes, ids unique, every reference resolves' % len(merged))

    os.makedirs(BACKUP, exist_ok=True)
    stamp = time.strftime('%Y%m%d-%H%M%S')
    json.dump(merged, open('%s/merged_%s.json' % (BACKUP, stamp), 'w'), indent=1)
    json.dump(live, open('%s/admin_flows_predeploy_%s.json' % (BACKUP, stamp), 'w'), indent=1)

    status, headers, text = request('/flows', 'POST',
                                    {'flows': merged, 'rev': rev, 'type': 'flows'},
                                    {'Node-RED-API-Version': 'v2',
                                     'Node-RED-Deployment-Type': 'flows'})
    print('POST /flows -> %s %s' % (status, text[:200]))
    if status == 409 and attempt == 1:
        print('rev conflict, retrying with the server rev')
        return deploy(attempt=2)
    if status not in (200, 204):
        sys.exit('deploy failed')

    after, rev2 = get_flows()
    tuner = [n for n in after if n['id'].startswith('tuner_')]
    print('after: %d nodes (rev %s), %d tuner nodes' % (len(after), rev2, len(tuner)))
    missing = mine_ids - {n['id'] for n in after}
    print('missing after deploy:', sorted(missing) or 'none')
    return rev2


if __name__ == '__main__':
    deploy()
