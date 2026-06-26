"""Adaptive web dashboard for the recorder (live view + control only).

Reads the SAME recorder config, then live-visualizes exactly what that config records:
- every image feature (observation.images.*) -> an MJPEG camera panel (N cameras adaptive),
- every low-dim source (TCP pose, joints, gripper, wrench, ...) -> a live value row AND a
  rolling real-time curve plot (one plot per topic, one line per component),
- a big colored CONTROL-MODE banner from the controller's ~/control_state (TOPIC/HOMING/GUARD),
- a recorded-episodes / frames counter and the dataset path.

Control buttons: Start / Stop / Discard (recording) and Go Home / Go Pose (homing). All call
the recorder's std_srvs/Trigger services; the last result is shown in the status bar.
Button gating:
- Start is enabled ONLY while the controller is in TOPIC and not already recording.
- Stop / Discard are enabled only while recording.
- Go Home / Go Pose are disabled while a homing is in progress (HOMING).
- Pressing Go Home / Go Pose WHILE recording makes the recorder discard the in-progress episode.

Dataset replay is intentionally NOT here -- use `lerobot-dataset-viz` to inspect datasets.

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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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
        self.control_state_topic = self.declare_parameter(
            'control_state_topic', '/cartesian_impedance_node/control_state').value
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

        self._ctrl = {'available': False, 'state': 'N/A',
                      'position_error': 0.0, 'orientation_error': 0.0}
        self._sub_control_state()

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

    def _sub_control_state(self):
        try:
            from franka_cartesian_impedance_msgs.msg import ControlState
        except Exception as e:  # noqa
            self.get_logger().warn(f'ControlState unavailable ({e}); banner = N/A, Start ungated')
            return
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ControlState, self.control_state_topic,
                                 self._control_state_cb, qos)
        self._ctrl['available'] = True

    def _control_state_cb(self, msg):
        self._ctrl.update(available=True, state=msg.state,
                          position_error=msg.position_error,
                          orientation_error=msg.orientation_error)

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

    def control_state(self):
        return self._ctrl

    def jpeg(self, topic):
        return self._jpeg.get(topic)

    def dataset(self):
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
        candidates = [d for d in glob.glob(self.ds_root + '_*')
                      if os.path.exists(os.path.join(d, 'meta', 'info.json'))]
        return max(candidates) if candidates else self.ds_root


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Franka Recorder</title>
<meta name="viewport" content="width=device-width,initial-scale=1"><style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:20px}
 h1{font-weight:500;font-size:22px;margin:0 0 12px}#rec{color:#f55;font-weight:700;margin-left:10px}
 #mode{font-size:36px;font-weight:800;letter-spacing:2px;padding:22px 26px;border-radius:16px;
   margin-bottom:16px;text-align:center;transition:background .2s}
 .m-topic{background:#1b5e20;color:#b9f6ca}.m-homing{background:#e65100;color:#ffe0b2}
 .m-guard{background:#b71c1c;color:#ffcdd2}.m-na{background:#2a2a2a;color:#999}
 #mode .sub{display:block;font-size:17px;font-weight:500;letter-spacing:0;margin-top:8px;opacity:.85}
 .bar{display:flex;gap:14px;flex-wrap:wrap;align-items:center;margin-bottom:14px}
 button{font-size:18px;padding:15px 26px;border:0;border-radius:11px;color:#fff;cursor:pointer}
 .start{background:#2e7d32}.stop{background:#c62828}.discard{background:#616161}
 .gohome{background:#1565c0}.gopose{background:#00695c}
 button:disabled{opacity:.3;cursor:not-allowed;filter:grayscale(.7)}
 .stats{display:flex;gap:16px;margin-bottom:14px;flex-wrap:wrap}
 .stat{background:#181818;border:1px solid #333;border-radius:12px;padding:14px 22px;min-width:160px;text-align:center}
 .stat .n{font-size:40px;font-weight:800;color:#8cf;line-height:1}
 .stat .l{font-size:14px;color:#9ab;margin-top:6px}
 #status{font-size:16px;padding:12px 16px;border-radius:9px;background:#181818;border:1px solid #333;
   margin-bottom:14px;color:#9fd}#status.err{color:#f99;border-color:#822}
 #ds{background:#181818;border:1px solid #333;border-radius:9px;padding:11px 15px;margin-bottom:18px;font-size:14px;color:#bcd}
 .cams{display:flex;gap:16px;flex-wrap:wrap;margin-bottom:18px}
 .cam{background:#000;border:1px solid #333;border-radius:12px;overflow:hidden}
 .cam img{display:block;max-width:860px;width:100%;height:auto}.cam .cap{font-size:14px;color:#aaa;padding:6px 12px}
 .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(780px,1fr));gap:18px}
 .fld{background:#181818;border:1px solid #2a2a2a;border-radius:12px;padding:14px 16px}
 .hd{display:flex;justify-content:space-between;font-size:17px;margin-bottom:8px}
 .hd .k{color:#8cf;font-weight:700}.hd .v{font-family:ui-monospace,monospace;color:#cfc;font-size:16px}
 canvas{display:block;width:100%;height:240px;background:#0d0d0d;border-radius:7px}
</style></head><body>
<h1>Franka Data Recorder <span id="rec"></span></h1>
<div id="mode" class="m-na">—</div>
<div class="bar">
 <button class="start" id="btnStart" onclick="call('start')">● Start</button>
 <button class="stop"  id="btnStop" onclick="call('stop')">■ Stop</button>
 <button class="discard" id="btnDiscard" onclick="call('discard')">Discard</button>
 <button class="gohome" id="btnHome" onclick="call('go_home')">⌂ Go Home</button>
 <button class="gopose" id="btnPose" onclick="call('go_pose')">Go Pose</button>
</div>
<div class="stats">
 <div class="stat"><div class="n" id="nEp">0</div><div class="l">episodes recorded</div></div>
 <div class="stat"><div class="n" id="nFr">0</div><div class="l">frames</div></div>
</div>
<div id="status">ready</div>
<div id="ds">dataset: …</div>
<div class="cams" id="cams"></div>
<div class="grid" id="grid"></div>
<script>
const N=400, COLS=['#6cf','#fc6','#6f9','#f69','#9cf','#fc9','#c9f','#ff8'];
let fields=[], P={}, recording=false, ctrlAvail=false, ctrlState='N/A';
async function init(){
 const L=await (await fetch('/layout')).json();
 const cd=document.getElementById('cams');
 L.cameras.forEach(c=>cd.insertAdjacentHTML('beforeend',
   `<div class="cam"><img src="/stream/${c.id}"><div class="cap">${c.label}</div></div>`));
 fields=L.fields;
 const g=document.getElementById('grid');
 fields.forEach(f=>{g.insertAdjacentHTML('beforeend',
   `<div class="fld"><div class="hd"><span class="k">${f.label}</span><span class="v" id="v_${f.topic}">waiting…</span></div>`+
   `<canvas id="c_${f.topic}" width="760" height="240"></canvas></div>`);
   P[f.topic]={cv:document.getElementById('c_'+f.topic),buf:[]};});
 setInterval(poll,150); setInterval(loadDs,2000); loadDs();
 setInterval(pollMode,250); pollMode(); updateButtons();
}
function updateButtons(){
 const set=(id,v)=>{const e=document.getElementById(id); if(e)e.disabled=v;};
 const notTopic = ctrlAvail && ctrlState!='TOPIC';
 const homing   = ctrlAvail && ctrlState=='HOMING';
 set('btnStart', recording || notTopic);   // record only in TOPIC, not already recording
 set('btnStop', !recording);
 set('btnDiscard', !recording);
 set('btnHome', homing);                    // no re-trigger while homing
 set('btnPose', homing);
 document.getElementById('rec').textContent = recording ? '● REC' : '';
}
async function pollMode(){
 try{const m=await (await fetch('/control_state')).json();
  ctrlAvail=!!m.available; ctrlState=m.state||'N/A';
  const el=document.getElementById('mode');
  let cls='m-na', txt='CONTROL STATE: N/A', sub='(ControlState msg not available)';
  if(ctrlAvail){
   cls = ctrlState=='TOPIC'?'m-topic': ctrlState=='HOMING'?'m-homing': ctrlState=='GUARD'?'m-guard':'m-na';
   txt = ctrlState;
   sub = `pos err ${(m.position_error*1000).toFixed(0)} mm · rot err ${m.orientation_error.toFixed(1)}°`;
  }
  el.className=cls; el.innerHTML=`${txt}<span class="sub">${sub}</span>`;
  updateButtons();
 }catch(e){}
}
function draw(p){
 const cv=p.cv,ctx=cv.getContext('2d'),W=cv.width,H=cv.height; ctx.clearRect(0,0,W,H);
 let mn=1e9,mx=-1e9; p.buf.forEach(b=>b.forEach(v=>{if(v<mn)mn=v;if(v>mx)mx=v;}));
 if(!(mx>mn)){mn-=1;mx+=1;} const pad=(mx-mn)*0.1; mn-=pad; mx+=pad;
 p.buf.forEach((b,ci)=>{ctx.strokeStyle=COLS[ci%COLS.length];ctx.lineWidth=1.6;ctx.beginPath();
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
  document.getElementById('nEp').textContent=d.episodes||0;
  document.getElementById('nFr').textContent=d.frames||0;
  document.getElementById('ds').textContent= d.exists
   ? `dataset: ${d.root}  @ ${d.fps} fps`
   : `dataset: ${d.root}  —  (empty / not created yet)`;
 }catch(e){}
}
async function call(a){
 const s=document.getElementById('status'); s.className=''; s.textContent=a+'…';
 try{const j=await (await fetch('/api/'+a,{method:'POST'})).json();
  s.textContent=`${a}: ${j.message}`; s.className=j.success?'':'err';
  if(j.success){
   if(a=='start') recording=true;
   if(a=='stop'||a=='discard'||a=='go_home'||a=='go_pose') recording=false;
   updateButtons(); loadDs();
  }
 }catch(e){s.textContent=a+': request failed';s.className='err';}
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
            elif self.path == '/control_state':
                self._send(200, json.dumps(node.control_state()), 'application/json')
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
