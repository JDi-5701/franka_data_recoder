"""Adaptive web dashboard for the recorder.

Reads the SAME recorder config, then live-visualizes exactly what that config records:
- every image feature (observation.images.*) -> an MJPEG camera panel (N cameras adaptive),
- every low-dim source (TCP pose, joints, gripper, wrench, ...) -> a live value row AND a
  rolling real-time curve plot (one plot per topic, one line per component),
- a dataset panel showing the output path + how many episodes/frames are stored,
- a DATA PLAYER: pick any recorded dataset + episode and replay it through the same camera
  and curve panels (recorded features are sliced back onto their source topics so playback
  reuses every live widget). Play / Pause / back-to-Live.
plus Start / Stop / Discard / Reset buttons (call the recorder Trigger services).

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
from urllib.parse import parse_qs, urlparse

# Read existing LeRobot datasets back for the player WITHOUT any Hub access (mirror the
# writer). Must be set before lerobot is imported (done lazily in _load_episode).
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")

import numpy as np
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

        self._state = {}   # topic -> list[float]  (live)
        self._jpeg = {}    # topic -> bytes        (live)
        self._bridge = None

        # ---- data player (replay recorded episodes through the same panels) ----------
        # Map each recorded feature back onto the live topics so replay reuses the existing
        # curve/camera widgets: a concat low-dim feature (e.g. observation.state, 22-dim) is
        # sliced back into its per-topic components using the config `dim`s; an image feature
        # maps to its camera topic.
        self._data_dir = os.path.dirname(self.ds_root)
        self._replay_map = []
        for name, spec in cfg.get('features', {}).items():
            if name.startswith('observation.images.'):
                self._replay_map.append({'name': name, 'image': True, 'topic': spec['topic']})
            else:
                srcs = spec.get('concat', [spec]) if isinstance(spec, dict) else [spec]
                parts, off = [], 0
                for s in srcs:
                    d = int(s.get('dim', 0))
                    parts.append({'topic': s['topic'], 'start': off, 'dim': d})
                    off += d
                self._replay_map.append({'name': name, 'image': False, 'parts': parts})
        self._mode = 'live'                # 'live' or 'replay'
        self._rstate = {}                  # topic -> list[float]  (replay)
        self._rjpeg = {}                   # topic -> bytes        (replay)
        self._rp = {'playing': False, 'paused': False, 'i': 0, 'total': 0,
                    'dataset': None, 'episode': None, 'msg': ''}
        self._rp_lock = threading.Lock()
        self._rp_thread = None
        self._loaded_ds = None
        self._loaded_key = None

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
        src = self._rstate if self._mode == 'replay' else self._state
        return {f['topic']: src.get(f['topic']) for f in self.fields}

    def jpeg(self, topic):
        return self._rjpeg.get(topic) if self._mode == 'replay' else self._jpeg.get(topic)

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

    # ---- data player ----------------------------------------------------
    def list_datasets(self):
        """Every LeRobot dataset under the data dir (each <name>/meta/info.json), newest first."""
        out = []
        for info_path in sorted(glob.glob(os.path.join(self._data_dir, '*', 'meta', 'info.json')),
                                reverse=True):
            path = os.path.dirname(os.path.dirname(info_path))
            try:
                with open(info_path) as f:
                    j = json.load(f)
                out.append({'name': os.path.basename(path), 'path': path,
                            'episodes': j.get('total_episodes', 0),
                            'frames': j.get('total_frames', 0), 'fps': j.get('fps')})
            except Exception:  # noqa
                continue
        return out

    def list_episodes(self, path):
        """Episode indices in a dataset (read straight from info.json — no dataset load)."""
        try:
            with open(os.path.join(path, 'meta', 'info.json')) as f:
                j = json.load(f)
            n = int(j.get('total_episodes', 0))
            return {'fps': j.get('fps'), 'episodes': list(range(n))}
        except Exception as e:  # noqa
            return {'fps': None, 'episodes': [], 'error': str(e)}

    def _import_lerobot(self):
        for p in ('lerobot.datasets.lerobot_dataset',
                  'lerobot.common.datasets.lerobot_dataset'):
            try:
                return __import__(p, fromlist=['LeRobotDataset']).LeRobotDataset
            except Exception:  # noqa
                continue
        raise ImportError('lerobot not importable in this env')

    def _load_episode(self, path, ep):
        key = (path, ep)
        if self._loaded_key == key and self._loaded_ds is not None:
            return self._loaded_ds
        LeRobotDataset = self._import_lerobot()
        repo_id = os.path.basename(path)
        try:
            ds = LeRobotDataset(repo_id, root=path, episodes=[int(ep)], download_videos=False)
        except TypeError:
            ds = LeRobotDataset(repo_id, root=path, episodes=[int(ep)])
        self._loaded_ds, self._loaded_key = ds, key
        return ds

    @staticmethod
    def _to_np(v):
        try:
            import torch  # noqa
            if isinstance(v, torch.Tensor):
                return v.detach().cpu().numpy()
        except Exception:  # noqa
            pass
        return np.asarray(v)

    def _to_jpeg(self, val):
        import cv2  # noqa
        arr = self._to_np(val)
        if arr.ndim == 3 and arr.shape[0] in (1, 3) and arr.shape[0] < arr.shape[-1]:
            arr = np.transpose(arr, (1, 2, 0))        # CHW -> HWC
        if arr.dtype != np.uint8:                      # lerobot returns float [0,1]
            arr = np.clip(arr * (255.0 if arr.max() <= 1.0 + 1e-3 else 1.0), 0, 255).astype(np.uint8)
        if arr.ndim == 3 and arr.shape[2] == 3:
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)  # lerobot stores RGB
        ok, buf = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None

    def _apply_replay_frame(self, item):
        for spec in self._replay_map:
            val = item.get(spec['name'])
            if val is None:
                continue
            if spec['image']:
                jpg = self._to_jpeg(val)
                if jpg:
                    self._rjpeg[spec['topic']] = jpg
            else:
                arr = self._to_np(val).reshape(-1)
                for p in spec['parts']:
                    self._rstate[p['topic']] = arr[p['start']:p['start'] + p['dim']].tolist()

    def replay_play(self, dataset, episode):
        if not dataset or not os.path.isdir(dataset):
            return False, 'pick a dataset first'
        # already playing this episode? just resume.
        with self._rp_lock:
            same = (self._rp['dataset'] == dataset and self._rp['episode'] == episode
                    and self._rp_thread is not None and self._rp_thread.is_alive())
            if same:
                self._rp['paused'] = False
                self._mode = 'replay'
                return True, 'resumed'
        try:
            ds = self._load_episode(dataset, episode)
            total = int(ds.num_frames)
        except Exception as e:  # noqa
            return False, f'load failed: {e}'
        if total <= 0:
            return False, 'episode has no frames'
        self._stop_replay_thread()
        with self._rp_lock:
            self._rp.update(playing=True, paused=False, i=0, total=total,
                            dataset=dataset, episode=episode, msg='playing')
            self._mode = 'replay'
        self._rp_thread = threading.Thread(target=self._replay_loop, args=(ds, total), daemon=True)
        self._rp_thread.start()
        return True, f'playing episode {episode} ({total} frames)'

    def _replay_loop(self, ds, total):
        fps = float(getattr(getattr(ds, 'meta', None), 'fps', 0) or self.fps_fallback())
        period = 1.0 / max(fps, 1.0)
        while True:
            with self._rp_lock:
                if not self._rp['playing']:
                    return
                paused, i = self._rp['paused'], self._rp['i']
            if paused:
                time.sleep(0.05)
                continue
            if i >= total:
                with self._rp_lock:
                    self._rp['playing'] = False
                    self._rp['msg'] = 'finished'
                return
            try:
                self._apply_replay_frame(ds[i])
            except Exception as e:  # noqa
                self.get_logger().warn(f'replay frame {i} failed: {e}', throttle_duration_sec=2.0)
            with self._rp_lock:
                self._rp['i'] = i + 1
            time.sleep(period)

    def fps_fallback(self):
        return 30.0

    def _stop_replay_thread(self):
        with self._rp_lock:
            self._rp['playing'] = False
        t = self._rp_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)

    def replay_pause(self):
        with self._rp_lock:
            self._rp['paused'] = True
            self._rp['msg'] = 'paused'
        return True, 'paused'

    def replay_stop(self):
        self._stop_replay_thread()
        with self._rp_lock:
            self._rp.update(playing=False, paused=False, i=0, msg='stopped')
            self._mode = 'live'
        return True, 'back to live'

    def replay_status(self):
        with self._rp_lock:
            return dict(self._rp, mode=self._mode)


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Franka Recorder</title>
<meta name="viewport" content="width=device-width,initial-scale=1"><style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
 h1{font-weight:500;font-size:19px;margin:0 0 10px}#rec{color:#f55;font-weight:600;margin-left:8px}
 .bar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-bottom:12px}
 button{font-size:15px;padding:11px 18px;border:0;border-radius:9px;color:#fff;cursor:pointer}
 .start{background:#2e7d32}.stop{background:#c62828}.discard{background:#616161}.reset{background:#1565c0}
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
 select{font-size:14px;padding:8px;border-radius:7px;background:#222;color:#eee;border:1px solid #333;max-width:46vw}
 #player{background:#161616;border:1px solid #333;border-radius:8px;padding:10px 12px;margin-bottom:14px}
 #player .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
 #rp{margin-left:6px;color:#fc6;font-size:13px;font-family:ui-monospace,monospace}
 .play{background:#00897b}.pause{background:#8d6e00}.pstop{background:#5d4037}
</style></head><body>
<h1>Franka Data Recorder <span id="rec"></span></h1>
<div class="bar">
 <button class="start" onclick="call('start')">● Start</button>
 <button class="stop"  onclick="call('stop')">■ Stop</button>
 <button class="discard" onclick="call('discard')">Discard</button>
 <button class="reset" onclick="call('reset')">Reset robot</button>
 <span id="status">ready</span>
</div>
<div id="ds">dataset: …</div>
<div id="player">
 <div class="row">
  <strong style="color:#8cf">Data player</strong>
  <select id="dsel" onchange="loadEps()"></select>
  <select id="esel"></select>
  <button class="play"  onclick="rplay()">▶ Play</button>
  <button class="pause" onclick="rpost('pause')">⏸ Pause</button>
  <button class="pstop" onclick="rpost('stop')">⏹ Live</button>
  <button style="background:#37474f" onclick="loadDatasets()">⟳ Refresh</button>
  <span id="rp"></span>
 </div>
</div>
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
 setInterval(pollReplay,300); loadDatasets();
}
async function loadDatasets(){
 try{const ds=await (await fetch('/datasets')).json();
  const s=document.getElementById('dsel'), cur=s.value;
  s.innerHTML=ds.map(d=>`<option value="${d.path}">${d.name} — ${d.episodes} ep, ${d.frames} fr</option>`).join('');
  if(cur&&ds.some(d=>d.path==cur))s.value=cur;
  if(ds.length)loadEps();
  else document.getElementById('esel').innerHTML='';
 }catch(e){}
}
async function loadEps(){
 const p=document.getElementById('dsel').value; if(!p)return;
 try{const j=await (await fetch('/episodes?dataset='+encodeURIComponent(p))).json();
  document.getElementById('esel').innerHTML=(j.episodes||[]).map(i=>`<option value="${i}">episode ${i}</option>`).join('');
 }catch(e){}
}
function rplay(){
 const dataset=document.getElementById('dsel').value;
 const episode=parseInt(document.getElementById('esel').value);
 if(isNaN(episode)){const s=document.getElementById('status');s.textContent='no episode selected';s.className='err';return;}
 rpost('play',{dataset,episode});
}
async function rpost(action,body){
 try{const j=await (await fetch('/replay/'+action,{method:'POST',
   headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})})).json();
  const s=document.getElementById('status'); s.textContent=j.message; s.className=j.success?'':'err';
 }catch(e){}
}
async function pollReplay(){
 try{const r=await (await fetch('/replay/status')).json();
  document.getElementById('rp').textContent = r.mode=='replay'
   ? `▶ REPLAY ep ${r.episode} — frame ${r.i}/${r.total}${r.paused?' (paused)':''}`
   : '';
 }catch(e){}
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
            elif self.path == '/datasets':
                self._send(200, json.dumps(node.list_datasets()), 'application/json')
            elif self.path.startswith('/episodes'):
                q = parse_qs(urlparse(self.path).query)
                self._send(200, json.dumps(node.list_episodes(q.get('dataset', [''])[0])),
                           'application/json')
            elif self.path == '/replay/status':
                self._send(200, json.dumps(node.replay_status()), 'application/json')
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
            elif self.path.startswith('/replay/'):
                action = self.path[len('/replay/'):]
                body = {}
                try:
                    n = int(self.headers.get('Content-Length', 0))
                    if n:
                        body = json.loads(self.rfile.read(n) or b'{}')
                except Exception:  # noqa
                    body = {}
                if action == 'play':
                    ok, msg = node.replay_play(body.get('dataset'), body.get('episode'))
                elif action == 'pause':
                    ok, msg = node.replay_pause()
                elif action == 'stop':
                    ok, msg = node.replay_stop()
                else:
                    ok, msg = False, f'unknown replay action: {action}'
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
