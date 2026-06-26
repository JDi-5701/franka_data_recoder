"""Adaptive web dashboard for the recorder (live view + control only).

Reads the SAME recorder config, then live-visualizes exactly what that config records:
- every image feature (observation.images.*) -> an MJPEG camera panel (N cameras adaptive),
- every low-dim source (TCP pose, joints, gripper, wrench, ...) -> a live value row AND a
  rolling real-time curve plot (one plot per topic, one line per component),
- a dataset panel showing the output path + how many episodes/frames are stored.

Control buttons: Start / Stop / Discard (recording) and Go Home / Go Pose (homing). All call
the recorder's std_srvs/Trigger services.

Dataset *replay* is intentionally NOT here -- use the official `lerobot-dataset-viz` tool to
inspect recorded datasets.

Config-driven: change what the recorder records and the GUI adapts. Topics not published
yet show "waiting...".

Run (conda ros_ml): ros2 run franka_data_recorder gui   ->  http://localhost:8088
"""
import glob
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from .extractors import get_extractor
from .recorder_node import _resolve_type, resolve_data_root

ACTIONS = {'start': 'start_recording', 'stop': 'stop_recording',
           'discard': 'discard_episode', 'go_home': 'go_home', 'go_pose': 'go_pose'}


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
        self.ds_root = resolve_data_root((cfg.get('dataset') or {}).get('root'), cfg_path)

        self.cams = []     # [{id, topic, label}]
        self.fields = []   # [{topic, label}]
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
                    self.fields.append({'topic': topic, 'label': _label(topic)})
                self._subscribe(topic, s['type'], s.get('extractor'), is_img)

        self._state = {}   # topic -> list[float]
        self._jpeg = {}    # topic -> bytes
        self._bridge = None

        self._cli = {a: self.create_client(Trigger, f'{ns}/{srv}')
                     for a, srv in ACTIONS.items()}
        self.get_logger().info(
            f'GUI: {len(self.cams)} camera(s), {len(self.fields)} field(s) '
            f'-> http://localhost:{self.port}')

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
        except Exception as e:  # noqa
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

    def jpeg(self, topic):
        return self._jpeg.get(topic)

    def dataset(self):
        # The recorder writes to a per-run timestamped dir <ds_root>_<stamp>; show the most
        # recent one that actually has a dataset in it. Fall back to ds_root itself.
        root = self._latest_dataset_dir()
        info = {'root': root, 'exists': False, 'episodes': 0, 'frames': 0, 'fps': None}
        try:
            p = os.path.join(root, 'meta', 'info.json')
            if os.path.exists(p):
                with open(p) as f:
                    j = json.load(f)
                info.update(exists=True, episodes=j.get('total_episodes', 0),
                            frames=j.get('total_frames', 0), fps=j.get('fps'))
        except Exception:  # noqa
            pass
        return info

    def _latest_dataset_dir(self):
        # timestamp suffix sorts lexically, so the max glob match is the newest run
        candidates = [d for d in glob.glob(self.ds_root + '_*')
                      if os.path.exists(os.path.join(d, 'meta', 'info.json'))]
        return max(candidates) if candidates else self.ds_root


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Franka Recorder</title>
<meta name="viewport" content="width=device-width,initial-scale=1"><style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
 h1{font-weight:500;font-size:19px;margin:0 0 10px}#rec{color:#f55;font-weight:600;margin-left:8px}
 .bar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-bottom:12px}
 button{font-size:15px;padding:11px 18px;border:0;border-radius:9px;color:#fff;cursor:pointer}
 .start{background:#2e7d32}.stop{background:#c62828}.discard{background:#616161}
 .gohome{background:#1565c0}.gopose{background:#00695c}
 #status{margin-left:auto;color:#9fd;font-size:14px}.err{color:#f99}
 #ds{background:#181818;border:1px solid #333;border-radius:8px;padding:8px 12px;margin-bottom:14px;font-size:13px;color:#bcd}
 .cams{display:flex;gap:12px;flex-wrap:wrap;margin-bottom:14px}
 .cam{background:#000;border:1px solid #333;border-radius:8px;overflow:hidden}
 .cam img{display:block;max-width:380px;height:auto}.cam .cap{font-size:12px;color:#aaa;padding:3px 8px}
 .grid{display:flex;flex-wrap:wrap;gap:12px}
 .fld{background:#181818;border:1px solid #2a2a2a;border-radius:8px;padding:8px 10px;width:400px}
 .hd{display:flex;justify-content:space-between;font-size:13px;margin-bottom:4px}
 .hd .k{color:#8cf}.hd .v{font-family:ui-monospace,monospace;color:#cfc}
 canvas{display:block;width:100%;height:90px;background:#0d0d0d;border-radius:4px}
</style></head><body>
<h1>Franka Data Recorder <span id="rec"></span></h1>
<div class="bar">
 <button class="start" onclick="call('start')">● Start</button>
 <button class="stop"  onclick="call('stop')">■ Stop</button>
 <button class="discard" onclick="call('discard')">Discard</button>
 <button class="gohome" onclick="call('go_home')">⌂ Go Home</button>
 <button class="gopose" onclick="call('go_pose')">Go Pose</button>
 <span id="status">ready</span>
</div>
<div id="ds">dataset: …</div>
<div class="cams" id="cams"></div>
<div class="grid" id="grid"></div>
<script>
const N=240, COLS=['#6cf','#fc6','#6f9','#f69','#9cf','#fc9','#c9f','#ff8'];
let fields=[], P={};
async function init(){
 const L=await (await fetch('/layout')).json();
 const cd=document.getElementById('cams');
 L.cameras.forEach(c=>cd.insertAdjacentHTML('beforeend',
   `<div class="cam"><img src="/stream/${c.id}"><div class="cap">${c.label}</div></div>`));
 fields=L.fields;
 const g=document.getElementById('grid');
 fields.forEach(f=>{g.insertAdjacentHTML('beforeend',
   `<div class="fld"><div class="hd"><span class="k">${f.label}</span><span class="v" id="v_${f.topic}">waiting…</span></div>`+
   `<canvas id="c_${f.topic}" width="380" height="90"></canvas></div>`);
   P[f.topic]={cv:document.getElementById('c_'+f.topic),buf:[]};});
 setInterval(poll,150); setInterval(loadDs,2000); loadDs();
}
function draw(p){
 const cv=p.cv,ctx=cv.getContext('2d'),W=cv.width,H=cv.height; ctx.clearRect(0,0,W,H);
 let mn=1e9,mx=-1e9; p.buf.forEach(b=>b.forEach(v=>{if(v<mn)mn=v;if(v>mx)mx=v;}));
 if(!(mx>mn)){mn-=1;mx+=1;} const pad=(mx-mn)*0.1; mn-=pad; mx+=pad;
 p.buf.forEach((b,ci)=>{ctx.strokeStyle=COLS[ci%COLS.length];ctx.lineWidth=1.2;ctx.beginPath();
  b.forEach((v,j)=>{const x=j/(N-1)*W, y=H-(v-mn)/(mx-mn)*H; j?ctx.lineTo(x,y):ctx.moveTo(x,y);});
  ctx.stroke();});
}
async function poll(){
 try{const s=await (await fetch('/state')).json();
  fields.forEach(f=>{const a=s[f.topic], p=P[f.topic];
   const e=document.getElementById('v_'+f.topic);
   if(!a){e.textContent='waiting…';return;}
   e.textContent=a.map(x=>x.toFixed(3)).join('  ');
   if(p.buf.length!=a.length)p.buf=a.map(()=>[]);
   a.forEach((v,i)=>{p.buf[i].push(v); if(p.buf[i].length>N)p.buf[i].shift();});
   draw(p);});
 }catch(e){}
}
async function loadDs(){
 try{const d=await (await fetch('/dataset')).json();
  document.getElementById('ds').textContent= d.exists
   ? `dataset: ${d.root}  —  ${d.episodes} episode(s), ${d.frames} frame(s) @ ${d.fps} fps`
   : `dataset: ${d.root}  —  (empty / not created yet)`;
 }catch(e){}
}
async function call(a){
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
            elif self.path == '/dataset':
                self._send(200, json.dumps(node.dataset()), 'application/json')
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
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    jpg = node.jpeg(topic)
                    if jpg:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                    time.sleep(1.0 / 15)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path.startswith('/api/'):
                ok, msg = node.call(self.path[len('/api/'):])
                self._send(200, json.dumps({'success': ok, 'message': msg}), 'application/json')
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
