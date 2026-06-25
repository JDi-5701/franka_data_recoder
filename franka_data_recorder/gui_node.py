"""Adaptive web dashboard for the recorder.

Reads the SAME recorder config, then live-visualizes exactly what that config records:
- every image feature (observation.images.*) -> an MJPEG camera panel (N cameras adaptive),
- every low-dim source (TCP pose, joints, gripper, wrench, ...) -> a numeric readout row,
plus Start / Stop / Discard / Reset buttons (call the recorder Trigger services).

Everything is config-driven, so when you change what the recorder records, the GUI adapts.
Topics the controller does not publish yet (e.g. joint_states) simply show "waiting..."
until they exist.

Run (conda ros_ml): ros2 run franka_data_recorder gui   ->  http://localhost:8088
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from .extractors import get_extractor
from .recorder_node import _resolve_type

ACTIONS = {'start': 'start_recording', 'stop': 'stop_recording',
           'discard': 'discard_episode', 'reset': 'reset'}


def _label(topic):
    return topic.rstrip('/').split('/')[-1] or topic


class GuiNode(Node):
    def __init__(self):
        super().__init__('franka_recorder_gui')
        share = get_package_share_directory('franka_data_recorder')
        cfg_path = self.declare_parameter(
            'config_file', os.path.join(share, 'config', 'recorder.yaml')).value
        ns = self.declare_parameter('recorder_node', '/franka_data_recorder').value
        self.port = int(self.declare_parameter('port', 8088).value)
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        # discover what to show from the SAME config the recorder uses
        self.cams = []     # [{id, topic, label}]
        self.fields = []   # [{topic, label, extractor_name}]
        seen = set()
        for name, spec in cfg.get('features', {}).items():
            is_img = name.startswith('observation.images.')
            srcs = spec.get('concat', [spec]) if isinstance(spec, dict) else [spec]
            for s in srcs:
                topic = s['topic']
                if topic in seen:
                    continue
                seen.add(topic)
                if is_img:
                    self.cams.append({'id': len(self.cams), 'topic': topic, 'label': name})
                else:
                    self.fields.append({'topic': topic, 'label': _label(topic),
                                        'extractor': s['extractor']})
                self._subscribe(topic, s['type'], s['extractor'], is_img)

        self._state = {}   # topic -> list[float]
        self._jpeg = {}    # topic -> bytes (latest encoded frame)
        self._bridge = None

        self._cli = {a: self.create_client(Trigger, f'{ns}/{srv}')
                     for a, srv in ACTIONS.items()}
        self.get_logger().info(
            f'GUI: {len(self.cams)} camera(s), {len(self.fields)} state field(s) '
            f'-> http://localhost:{self.port}')

    # ---- ROS side ------------------------------------------------------
    def _subscribe(self, topic, type_str, extractor_name, is_img):
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        msg_cls = _resolve_type(type_str)
        if is_img:
            self.create_subscription(msg_cls, topic,
                                     lambda m, t=topic: self._on_image(t, m), qos)
        else:
            fn = get_extractor(extractor_name)
            self.create_subscription(
                msg_cls, topic,
                lambda m, t=topic, f=fn: self._state.__setitem__(t, f(m).tolist()), qos)

    def _on_image(self, topic, msg):
        try:
            import cv2  # noqa
            if self._bridge is None:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
            if msg.__class__.__name__ == 'CompressedImage':
                bgr = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            else:
                bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self._jpeg[topic] = buf.tobytes()
        except Exception as e:  # noqa - missing cv_bridge/opencv -> images just stay blank
            self.get_logger().warn(f'image decode failed ({topic}): {e}', once=True)

    def call(self, action):
        cli = self._cli.get(action)
        if cli is None:
            return False, f'unknown action: {action}'
        if not cli.wait_for_service(timeout_sec=2.0):
            return False, f'{action}: service unavailable'
        fut = cli.call_async(Trigger.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=30.0):
            return False, f'{action}: timeout'
        r = fut.result()
        return r.success, r.message

    def layout(self):
        return {'cameras': self.cams, 'fields': self.fields}

    def state(self):
        return {f['topic']: self._state.get(f['topic']) for f in self.fields}


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Franka Recorder</title>
<meta name="viewport" content="width=device-width,initial-scale=1"><style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:18px}
 h1{font-weight:500;font-size:19px;margin:0 0 12px}
 .bar{display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-bottom:16px}
 button{font-size:16px;padding:12px 20px;border:0;border-radius:10px;color:#fff;cursor:pointer}
 .start{background:#2e7d32}.stop{background:#c62828}.discard{background:#616161}.reset{background:#1565c0}
 #rec{font-weight:600;margin-left:8px}
 #status{margin-left:auto;color:#9fd;font-size:14px}.err{color:#f99}
 .cams{display:flex;gap:12px;flex-wrap:wrap;margin-bottom:16px}
 .cam{background:#000;border:1px solid #333;border-radius:8px;overflow:hidden}
 .cam img{display:block;max-width:420px;height:auto}.cam .cap{font-size:12px;color:#aaa;padding:4px 8px}
 table{border-collapse:collapse;font-variant-numeric:tabular-nums}
 td{padding:5px 12px;border-bottom:1px solid #222;font-size:14px}
 td.k{color:#8cf}td.v{font-family:ui-monospace,monospace;color:#cfc;white-space:nowrap}
</style></head><body>
<h1>Franka Data Recorder <span id="rec"></span></h1>
<div class="bar">
 <button class="start" onclick="call('start',1)">● Start</button>
 <button class="stop"  onclick="call('stop',0)">■ Stop</button>
 <button class="discard" onclick="call('discard',0)">Discard</button>
 <button class="reset" onclick="call('reset',0)">Reset robot</button>
 <span id="status">ready</span>
</div>
<div class="cams" id="cams"></div>
<table id="tbl"></table>
<script>
let fields=[];
async function init(){
 const L=await (await fetch('/layout')).json();
 const cd=document.getElementById('cams');
 L.cameras.forEach(c=>{cd.insertAdjacentHTML('beforeend',
   `<div class="cam"><img src="/stream/${c.id}"><div class="cap">${c.label} (${c.topic})</div></div>`);});
 fields=L.fields;
 document.getElementById('tbl').innerHTML=fields.map(f=>
   `<tr><td class="k">${f.label}</td><td class="v" id="v_${f.topic}">waiting…</td></tr>`).join('');
 setInterval(poll,200);
}
async function poll(){
 try{const s=await (await fetch('/state')).json();
  fields.forEach(f=>{const e=document.getElementById('v_'+f.topic);
   const a=s[f.topic]; e.textContent=a?a.map(x=>x.toFixed(3)).join('  '):'waiting…';});
 }catch(e){}
}
async function call(a,rec){
 const s=document.getElementById('status'); s.className=''; s.textContent=a+'…';
 try{const j=await (await fetch('/api/'+a,{method:'POST'})).json();
  s.textContent=j.message; s.className=j.success?'':'err';
  if(j.success&&a=='start')document.getElementById('rec').textContent='● REC';
  if(j.success&&(a=='stop'||a=='discard'))document.getElementById('rec').textContent='';
 }catch(e){s.textContent='request failed';s.className='err';}
}
init();
</script></body></html>"""


def _make_handler(node):
    class H(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype='text/html; charset=utf-8'):
            data = body if isinstance(body, bytes) else body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._send(200, PAGE)
            elif self.path == '/layout':
                self._send(200, json.dumps(node.layout()), 'application/json')
            elif self.path == '/state':
                self._send(200, json.dumps(node.state()), 'application/json')
            elif self.path.startswith('/stream/'):
                self._mjpeg(self.path[len('/stream/'):])
            else:
                self._send(404, 'not found')

        def _mjpeg(self, cam_id):
            try:
                topic = node.cams[int(cam_id)]['topic']
            except Exception:  # noqa
                self._send(404, 'no such camera'); return
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    jpg = node._jpeg.get(topic)
                    if jpg:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: ' + str(len(jpg)).encode()
                                         + b'\r\n\r\n' + jpg + b'\r\n')
                    time.sleep(1.0 / 15)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path.startswith('/api/'):
                ok, msg = node.call(self.path[len('/api/'):])
                self._send(200, json.dumps({'success': ok, 'message': msg}),
                           'application/json')
            else:
                self._send(404, 'not found')

        def log_message(self, *_a):
            pass

    return H


def main():
    rclpy.init()
    node = GuiNode()
    httpd = ThreadingHTTPServer(('0.0.0.0', node.port), _make_handler(node))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    node.get_logger().info(f'recorder GUI -> http://localhost:{node.port}')
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
