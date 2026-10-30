import json, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
TUN = os.path.abspath(os.path.join(HERE, '..', 'tuning_flows.json'))
EXI = os.path.abspath(os.path.join(HERE, '..', 'flows.json'))

tun = json.load(open(TUN))
exi = json.load(open(EXI))
errs = []

ids = [x.get('id') for x in tun]
if len(ids) != len(set(ids)):
    errs.append('duplicate ids inside the import')
clash = set(ids) & set(x.get('id') for x in exi)
if clash:
    errs.append('id clash with existing flows.json: %s' % clash)

known = set(ids) | set(x.get('id') for x in exi)
tab = [x for x in tun if x.get('type') == 'tab'][0]['id']

for x in tun:
    for outs in x.get('wires', []) or []:
        for t in outs or []:
            if t not in known:
                errs.append('%s (%s) wires to unknown id %s' % (x['id'], x.get('type'), t))
            elif t not in ids:
                errs.append('%s (%s) wires into the existing flow (%s) - cross-tab link'
                            % (x['id'], x.get('type'), t))
    if x.get('type') not in ('tab', 'ui-page', 'ui-group') and x.get('z') != tab:
        errs.append('%s (%s) has z=%s' % (x['id'], x.get('type'), x.get('z')))

refs = {'ui-base': 'ui', 'ui-page': 'ui', 'ui-group': 'page', 'ui-theme': None}
for x in tun:
    for field, want in (('ui', 'ui-base'), ('page', 'ui-page'), ('group', 'ui-group'),
                        ('broker', 'mqtt-broker'), ('theme', 'ui-theme')):
        if field in x:
            target = [y for y in exi + tun if y.get('id') == x[field]]
            if not target:
                errs.append('%s (%s) references missing %s %s'
                            % (x['id'], x.get('type'), field, x[field]))
            elif want and target[0].get('type') != want:
                errs.append('%s references %s which is a %s, not a %s'
                            % (x['id'], x[field], target[0].get('type'), want))

# widgets must land in a group that exists on the tuner page
groups = {x['id']: x for x in tun if x.get('type') == 'ui-group'}
for x in tun:
    if 'group' in x:
        g = groups.get(x['group'])
        if not g:
            errs.append('%s is in group %s which is not in this import' % (x['id'], x['group']))
        elif g.get('page') != 'tuner_page00001':
            errs.append('%s group %s is not on the tuner page' % (x['id'], x['group']))

# every function node has code, and every http node a url
for x in tun:
    if x.get('type') == 'function' and not (x.get('func') or '').strip() \
            and not (x.get('initialize') or '').strip():
        errs.append('function %s has no code' % x['name'])
    if x.get('type') == 'http request' and not x.get('url'):
        errs.append('http request %s has no url' % x['name'])

from collections import Counter
print('nodes:', len(tun), dict(Counter(x['type'] for x in tun)))
if errs:
    print('FAIL')
    for e in errs:
        print('  -', e)
    sys.exit(1)
print('OK: ids unique, no clashes, every wire/group/broker/page reference resolves'
      ' inside the import or to the existing dashboard')
