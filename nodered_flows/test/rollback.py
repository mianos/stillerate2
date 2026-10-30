#!/usr/bin/env python3
"""Put the Node-RED instance back the way it was before the tuner deploy.

Usage: python3 rollback.py [/tmp/tuner/backup/admin_flows_predeploy_<stamp>.json]
Default is the snapshot taken immediately before the tuner was added.
"""
import json
import os
import sys
import urllib.request

BASE = 'http://nodered.mianos.com:1880/admin'
HERE = os.path.dirname(os.path.abspath(__file__))
SNAPSHOT = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(HERE, 'backup', 'admin_flows_predeploy_20261029-165644.json')
HEADERS = {'Content-Type': 'application/json', 'Node-RED-API-Version': 'v2'}


def call(path, body=None, extra=None):
    headers = dict(HEADERS)
    if extra:
        headers.update(extra)
    req = urllib.request.Request(
        BASE + path, method='POST' if body is not None else 'GET', headers=headers,
        data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(req, timeout=30) as r:
        return r.status, r.read().decode()


_, text = call('/flows')
current = json.loads(text)
flows = json.load(open(SNAPSHOT))
print('rolling back to %s: %d nodes (current %d)' % (SNAPSHOT, len(flows), len(current['flows'])))
print('removing:', sorted(n['id'] for n in current['flows'] if n['id'].startswith('tuner_')))
status, body = call('/flows', {'flows': flows, 'rev': current['rev'], 'type': 'full'},
                    {'Node-RED-Deployment-Type': 'full'})
print('POST /flows ->', status, body)
_, text = call('/flows')
after = json.loads(text)
print('now: %d nodes, tuner nodes left: %d' %
      (len(after['flows']), sum(1 for n in after['flows'] if n['id'].startswith('tuner_'))))
