#!/usr/bin/env python3
# dashboard_server: FastAPI + ROS bridge. Stage 1 - live sensor value to browser.
from sensor_msgs.msg import CompressedImage
from fastapi.responses import StreamingResponse
import time
import threading
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import uvicorn
import asyncio

# ---- shared state (updated by ROS, read by web) ----
state = {
    'gas': False,
    'instability': False,
    'tilt': [0.0, 0.0],
}
latest_frame = {'data': None}

# ---- ROS node ----
class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.create_subscription(Bool, '/gas_detected', self.gas_cb, 10)
        self.create_subscription(Bool, '/instability', self.instab_cb, 10)
        self.create_subscription(Float32MultiArray, '/tilt', self.tilt_cb, 10)
        self.create_subscription(CompressedImage, '/camera/annotated', self.frame_cb, 10)

    def frame_cb(self, msg):
        latest_frame['data'] = bytes(msg.data)

    def gas_cb(self, msg):
        state['gas'] = msg.data

    def instab_cb(self, msg):
        state['instability'] = msg.data

    def tilt_cb(self, msg):
        state['tilt'] = list(msg.data)


# ---- ROS spins in a background thread ----
def ros_thread():
    rclpy.init()
    node = DashboardNode()
    rclpy.spin(node)

# ---- FastAPI app ----
app = FastAPI()

HTML = """
<!DOCTYPE html>
<html>
<head><title>Rescue Rover Dashboard</title>
<style>
  body { font-family: sans-serif; background:#1a1a1a; color:#eee; text-align:center; padding:40px; }
  .card { display:inline-block; background:#2a2a2a; border-radius:12px; padding:30px; margin:15px; min-width:180px; }
  .label { color:#888; font-size:14px; }
  .value { font-size:32px; font-weight:bold; margin-top:8px; }
  .alert { color:#ff5555; }
  .ok { color:#55ff55; }
</style>
</head>
<body>
  <h1>🤖 Rescue Rover</h1>
  <div class="card"><div class="label">GAS</div><div id="gas" class="value">-</div></div>
  <div class="card"><div class="label">STABILITY</div><div id="instab" class="value">-</div></div>
  <div class="card"><div class="label">TILT (pitch, roll)</div><div id="tilt" class="value">-</div></div>
  <div style="margin-top:30px;">
    <div class="label">LIVE CAMERA</div>
    <img src="/video" style="max-width:640px; width:90%; border-radius:12px; margin-top:10px;">
  </div>
<script>
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    const gas = document.getElementById('gas');
    gas.textContent = d.gas ? 'DETECTED' : 'clear';
    gas.className = 'value ' + (d.gas ? 'alert' : 'ok');
    const ins = document.getElementById('instab');
    ins.textContent = d.instability ? 'UNSTABLE' : 'stable';
    ins.className = 'value ' + (d.instability ? 'alert' : 'ok');
    document.getElementById('tilt').textContent =
      d.tilt[0].toFixed(1) + ', ' + d.tilt[1].toFixed(1);
  };
</script>
</body>
</html>
"""

@app.get("/")
async def index():
    return HTMLResponse(HTML)

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_text(json.dumps(state))
            await asyncio.sleep(0.2)   # 5 Hz updates
    except Exception:
        pass

@app.get("/video")
def video():
    def gen():
        while True:
            frame = latest_frame['data']
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.1)
    return StreamingResponse(gen(),
        media_type='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    threading.Thread(target=ros_thread, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8080)
