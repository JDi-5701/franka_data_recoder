"""Minimal web GUI to drive the recorder from a browser.

Serves a small page with Start / Stop / Discard / Reset buttons; each button calls the
corresponding std_srvs/Trigger service on the recorder. Self-contained (no rosbridge):
an embedded HTTP server runs in a daemon thread, the rclpy node is spun by a
MultiThreadedExecutor so the cross-thread service calls are safe.

Run (on the GPU):  ros2 run franka_data_recorder gui
Then open:          http://localhost:8088
Later this page is the place to add live state + camera visualization.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

# button id -> service short name (under the recorder node namespace)
ACTIONS = {
    'start': 'start_recording',
    'stop': 'stop_recording',
    'discard': 'discard_episode',
    'reset': 'reset',
}

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>Franka Recorder</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:24px}
 h1{font-weight:500;font-size:20px}
 .row{display:flex;gap:14px;justify-content:center;flex-wrap:wrap;margin-top:24px}
 button{font-size:18px;padding:18px 26px;border:0;border-radius:12px;color:#fff;cursor:pointer;min-width:150px}
 .start{background:#2e7d32}.stop{background:#c62828}.discard{background:#616161}.reset{background:#1565c0}
 button:active{filter:brightness(1.2)}
 #status{margin-top:26px;font-size:15px;min-height:22px;color:#9fd}
 .err{color:#f99 !important}
</style></head><body>
<h1>Franka Data Recorder</h1>
<div class="row">
 <button class="start"   onclick="call('start')">● Start</button>
 <button class="stop"    onclick="call('stop')">■ Stop</button>
 <button class="discard" onclick="call('discard')">Discard</button>
 <button class="reset"   onclick="call('reset')">Reset robot</button>
</div>
<div id="status">ready</div>
<script>
async function call(a){
 const s=document.getElementById('status'); s.className=''; s.textContent=a+'...';
 try{ const r=await fetch('/api/'+a,{method:'POST'}); const j=await r.json();
      s.textContent=j.message; s.className=j.success?'':'err'; }
 catch(e){ s.textContent='request failed: '+e; s.className='err'; }
}
</script></body></html>"""


class GuiNode(Node):
    def __init__(self):
        super().__init__('franka_recorder_gui')
        ns = self.declare_parameter('recorder_node', '/franka_data_recorder').value
        self.port = int(self.declare_parameter('port', 8088).value)
        self._cli = {a: self.create_client(Trigger, f'{ns}/{srv}')
                     for a, srv in ACTIONS.items()}

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
        res = fut.result()
        return res.success, res.message


def _make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype='text/html; charset=utf-8'):
            data = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._send(200, PAGE)
            else:
                self._send(404, 'not found')

        def do_POST(self):
            if self.path.startswith('/api/'):
                ok, msg = node.call(self.path[len('/api/'):])
                self._send(200, json.dumps({'success': ok, 'message': msg}),
                           'application/json')
            else:
                self._send(404, 'not found')

        def log_message(self, *_a):  # silence default request logging
            pass

    return Handler


def main():
    rclpy.init()
    node = GuiNode()
    httpd = ThreadingHTTPServer(('0.0.0.0', node.port), _make_handler(node))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    node.get_logger().info(f'recorder GUI -> http://localhost:{node.port}')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
