import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
JS = os.path.join(HERE, 'js', '')          # trailing sep so JS + name works
OUT = os.path.abspath(os.path.join(HERE, '..', 'tuning_flows.json'))

def js(name):
    with open(JS + name) as f:
        return f.read().rstrip('\n')

TAB = 'tuner_tab000001'
BROKER = '3fe3ac59af05955a'      # existing mqtt-broker "MQTT2" in flows.json
UIBASE = 'c776e0d19c314f0d'      # existing ui-base "Stillerate 2"
THEME = 'ff6476dcafc85310'       # existing ui-theme "Default Theme"
STILL = 'http://still2.mianos.com'
LLM = 'http://docker-host.mianos.com:8082/v1/chat/completions'

n = []

n.append({
    "id": TAB, "type": "tab", "label": "PID tuning", "disabled": False, "env": [],
    "info": "Advisory PID tuner.\n\nMQTT telemetry -> digest (GET /pid + sample ring) -> LLM proposal -> "
            "deterministic validator (envelope, step caps, rate limit) -> human Apply/Reject -> POST /pid.\n\n"
            "cfg.dryRun = true in the CONFIG node means nothing is ever written to the device; every action is "
            "logged instead. Flip it to false only once you trust the proposals."
})

n.append({
    "id": "tuner_page00001", "type": "ui-page", "name": "tuning", "ui": UIBASE, "path": "/tuning",
    "icon": "tune", "layout": "grid", "theme": THEME, "order": 4, "className": "",
    "visible": "true", "disabled": "false"
})

def group(gid, name, order, h):
    return {"id": gid, "type": "ui-group", "name": name, "page": "tuner_page00001",
            "width": "6", "height": str(h), "order": order, "showTitle": True,
            "className": "", "visible": "true", "disabled": "false"}

n.append(group('tuner_grp_digest', '1 digest', 1, 3))
n.append(group('tuner_grp_prop', '2 proposal', 2, 4))
n.append(group('tuner_grp_apply', '3 apply', 3, 3))
n.append(group('tuner_grp_watch', '4 watchdog', 4, 2))

def utext(nid, grp, order, label, fmt):
    return {"id": nid, "type": "ui-text", "z": TAB, "group": grp, "order": order,
            "width": 0, "height": 0, "name": "", "label": label, "format": fmt,
            "layout": "row-left", "style": False, "font": "", "fontSize": 16, "color": "",
            "className": "", "x": 0, "y": 0, "wires": []}

def ubutton(nid, grp, order, label, tooltip):
    return {"id": nid, "type": "ui-button", "z": TAB, "group": grp, "name": "", "label": label,
            "order": order, "width": 0, "height": 0, "emulateClick": False, "tooltip": tooltip,
            "color": "", "bgcolor": "", "className": "", "icon": "", "iconPosition": "left",
            "payload": "", "payloadType": "date", "topic": "topic", "topicType": "msg",
            "x": 0, "y": 0, "wires": [[]]}

n.append(utext('tuner_txt_state0', 'tuner_grp_digest', 1, 'state', '{{msg.payload}}'))
n.append(ubutton('tuner_btn_digest', 'tuner_grp_digest', 2, 'Build digest',
                 'Fetch GET /pid, digest the last 10 min, ask the model'))
n.append({
    "id": "tuner_tbl_prop00", "type": "ui-table", "z": TAB, "group": "tuner_grp_prop",
    "name": "proposal", "label": "text", "order": 1, "width": 0, "height": 0, "maxrows": "0",
    "passthru": False, "autocols": True, "showSearch": False, "selectionType": "none",
    "columns": [], "mobileBreakpoint": "sm", "mobileBreakpointType": "defaults",
    "action": "replace", "x": 0, "y": 0, "wires": [[]]
})
n.append(utext('tuner_txt_diag00', 'tuner_grp_prop', 2, 'diagnosis', '{{msg.payload}}'))
n.append(ubutton('tuner_btn_apply0', 'tuner_grp_apply', 1, 'Apply proposal',
                 'Re-read GET /pid, then POST the validated subset (no-op while dryRun)'))
n.append(ubutton('tuner_btn_revert', 'tuner_grp_apply', 2, 'Revert to last good',
                 'POST the parameters captured just before the last apply'))
n.append(utext('tuner_txt_last00', 'tuner_grp_apply', 3, 'last action', '{{msg.payload}}'))
n.append(utext('tuner_txt_wd0000', 'tuner_grp_watch', 1, 'watchdog', '{{msg.payload}}'))
n.append({
    "id": "tuner_note000000", "type": "ui-notification", "z": TAB, "ui": UIBASE,
    "name": "tuner alerts", "position": "top right", "colorDefault": True, "color": "#000000",
    "displayTime": "10", "showCountdown": True, "outputs": 1, "allowDismiss": True,
    "dismissText": "Close", "raw": False, "className": "", "x": 0, "y": 0, "wires": [[]]
})

def fn(nid, name, code, outputs, wires, init=''):
    return {"id": nid, "type": "function", "z": TAB, "name": name, "func": code,
            "outputs": outputs, "timeout": 0, "noerr": 0, "initialize": init,
            "finalize": "", "libs": [], "x": 0, "y": 0, "wires": wires}

def http(nid, name, method, url, wires):
    return {"id": nid, "type": "http request", "z": TAB, "name": name, "method": method,
            "ret": "obj", "paytoqs": "ignore", "url": url, "tls": "", "persist": False,
            "proxy": "", "insecureHTTPParser": False, "authType": "", "senderr": False,
            "headers": [], "x": 0, "y": 0, "wires": wires}

n.append(fn('tuner_f_cfg00000', 'CONFIG (edit me)', 'return null;', 0, [], init=js('00_config.js')))
n.append({"id": "tuner_mqtt_in000", "type": "mqtt in", "z": TAB, "name": "PID telemetry",
          "topic": "tele/still2/pid", "qos": "2", "datatype": "auto-detect", "broker": BROKER,
          "nl": False, "rap": True, "rh": 0, "inputs": 0, "x": 0, "y": 0,
          "wires": [["tuner_f_buf00000"]]})
n.append(fn('tuner_f_buf00000', 'buffer samples', js('01_buffer.js'), 0, []))
n.append(http('tuner_http_get_dg', 'GET /pid (digest)', 'GET', STILL + '/pid', [["tuner_f_prompt00"]]))
n.append(fn('tuner_f_prompt00', 'build prompt', js('02_prompt.js'), 2,
            [["tuner_http_llm00"], ["tuner_txt_state0", "tuner_note000000"]]))
n.append(http('tuner_http_llm00', 'LLM chat completions', 'POST', LLM, [["tuner_f_parse000"]]))
n.append(fn('tuner_f_parse000', 'validate proposal', js('03_parse.js'), 4,
            [["tuner_tbl_prop00"], ["tuner_txt_diag00"], ["tuner_txt_state0", "tuner_note000000"],
             ["tuner_http_llm00"]]))
n.append(http('tuner_http_get_ap', 'GET /pid (before apply)', 'GET', STILL + '/pid',
              [["tuner_f_stash000"]]))
n.append(fn('tuner_f_stash000', 'stash + build POST', js('04_stash.js'), 2,
            [["tuner_http_post00"], ["tuner_txt_state0", "tuner_note000000"]]))
n.append(http('tuner_http_post00', 'POST /pid', 'POST', STILL + '/pid', [["tuner_f_audit000"]]))
n.append(fn('tuner_f_audit000', 'audit', js('05_audit.js'), 2,
            [["tuner_mqtt_out000"], ["tuner_txt_last00", "tuner_note000000"]]))
n.append(fn('tuner_f_revert00', 'revert now', js('06_revert.js'), 2,
            [["tuner_http_post00"], ["tuner_txt_state0", "tuner_note000000"]]))
n.append({"id": "tuner_inj_wd00000", "type": "inject", "z": TAB, "name": "every 60 s",
          "props": [{"p": "payload"}], "repeat": "60", "crontab": "", "once": True,
          "onceDelay": "20", "topic": "", "payload": "", "payloadType": "date",
          "x": 0, "y": 0, "wires": [["tuner_f_watchdog"]]})
n.append(fn('tuner_f_watchdog', 'watchdog', js('07_watchdog.js'), 3,
            [["tuner_http_post00"], ["tuner_note000000"], ["tuner_txt_wd0000"]]))
n.append({"id": "tuner_mqtt_out000", "type": "mqtt out", "z": TAB, "name": "audit trail",
          "topic": "", "qos": "", "retain": "", "respTopic": "", "contentType": "",
          "userProps": "", "correl": "", "expiry": "", "broker": BROKER, "x": 0, "y": 0,
          "wires": []})

by = {x['id']: x for x in n}
by['tuner_btn_digest']['wires'] = [["tuner_http_get_dg"]]
by['tuner_btn_apply0']['wires'] = [["tuner_http_get_ap"]]
by['tuner_btn_revert']['wires'] = [["tuner_f_revert00"]]

LAYOUT = {
    'tuner_f_cfg00000': (140, 80), 'tuner_mqtt_in000': (140, 240), 'tuner_f_buf00000': (360, 240),
    'tuner_http_get_dg': (340, 340), 'tuner_f_prompt00': (560, 340), 'tuner_http_llm00': (780, 340),
    'tuner_f_parse000': (980, 340),
    'tuner_http_get_ap': (340, 460), 'tuner_f_stash000': (560, 460), 'tuner_http_post00': (780, 460),
    'tuner_f_audit000': (980, 460), 'tuner_mqtt_out000': (1220, 460),
    'tuner_f_revert00': (340, 580), 'tuner_inj_wd00000': (140, 680), 'tuner_f_watchdog': (340, 680),
    'tuner_txt_state0': (1520, 80), 'tuner_btn_digest': (1520, 140), 'tuner_tbl_prop00': (1520, 220),
    'tuner_txt_diag00': (1520, 320), 'tuner_btn_apply0': (1520, 400), 'tuner_btn_revert': (1520, 460),
    'tuner_txt_last00': (1520, 520), 'tuner_txt_wd0000': (1520, 580), 'tuner_note000000': (1520, 650),
}
for nid, (x, y) in LAYOUT.items():
    by[nid]['x'], by[nid]['y'] = x, y

with open(OUT, 'w') as f:
    json.dump(n, f, indent=4)
    f.write('\n')
print('wrote', OUT, len(n), 'nodes')

