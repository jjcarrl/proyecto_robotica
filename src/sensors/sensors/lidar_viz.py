#!/usr/bin/env python3
"""
Lidar visualizer — subscribes to /scan and serves a live polar-plot web UI.
Open http://<robot-ip>:5002 in any browser on the same network.
"""
import json
import math
import threading
import time

from flask import Flask, Response, render_template_string
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PORT = 5002

HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LiDAR Viewer</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #1e1e2e; color: #cdd6f4;
      font-family: 'Segoe UI', sans-serif;
      display: flex; flex-direction: column; align-items: center;
      padding: 20px; gap: 16px; min-height: 100vh;
    }
    h1 { color: #89b4fa; letter-spacing: 3px; font-size: 1.3rem; }
    #canvas { border-radius: 50%; background: #11111b; cursor: crosshair; }
    #status {
      display: flex; gap: 24px; background: #313244;
      border-radius: 10px; padding: 10px 20px;
      border: 1px solid #45475a; font-size: 0.85rem;
    }
    .stat { text-align: center; }
    .stat .lbl { color: #a6adc8; font-size: 0.7rem; text-transform: uppercase; letter-spacing: 1px; }
    .stat .val { color: #89b4fa; font-size: 1.1rem; font-weight: bold; }
    #dot { width: 9px; height: 9px; border-radius: 50%; background: #a6e3a1;
           display: inline-block; animation: pulse 1.5s infinite; }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.2} }
    #range-slider { display: flex; align-items: center; gap: 10px; color: #a6adc8; font-size: 0.8rem; }
    input[type=range] { accent-color: #89b4fa; width: 140px; }
  </style>
</head>
<body>
  <h1>&#x25CF; LIDAR SCAN</h1>

  <div id="range-slider">
    <span>Max range</span>
    <input type="range" id="maxRange" min="1" max="10" step="0.5" value="5">
    <span id="rangeLabel">5.0 m</span>
  </div>

  <canvas id="canvas" width="580" height="580"></canvas>

  <div id="status">
    <div class="stat"><div class="lbl">Status</div>
      <div class="val"><span id="dot"></span> Live</div></div>
    <div class="stat"><div class="lbl">Rate</div>
      <div class="val"><span id="rate">--</span> Hz</div></div>
    <div class="stat"><div class="lbl">Points</div>
      <div class="val"><span id="pts">--</span></div></div>
    <div class="stat"><div class="lbl">Nearest</div>
      <div class="val"><span id="near">--</span> m</div></div>
    <div class="stat"><div class="lbl">Cursor</div>
      <div class="val"><span id="cursor">--</span></div></div>
  </div>

<script>
const canvas   = document.getElementById('canvas');
const ctx      = canvas.getContext('2d');
const W = canvas.width, H = canvas.height;
const CX = W / 2, CY = H / 2;

let maxRange = 5.0;
let lastRanges = null, lastAngleMin = 0, lastAngleInc = 0;
let frameCount = 0, lastRateTime = performance.now(), displayRate = 0;

// ── range slider ─────────────────────────────────────────────────────────────
const slider = document.getElementById('maxRange');
const rangeLabel = document.getElementById('rangeLabel');
slider.addEventListener('input', () => {
  maxRange = parseFloat(slider.value);
  rangeLabel.textContent = maxRange.toFixed(1) + ' m';
  if (lastRanges) draw(lastRanges, lastAngleMin, lastAngleInc);
});

// ── cursor tooltip ────────────────────────────────────────────────────────────
canvas.addEventListener('mousemove', e => {
  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left - CX;
  const my = -(e.clientY - rect.top - CY);
  const scale = (W / 2 - 30) / maxRange;
  const rm = Math.sqrt(mx*mx + my*my) / scale;
  const ang = Math.atan2(mx, my) * 180 / Math.PI;
  const angPos = (ang + 360) % 360;
  document.getElementById('cursor').textContent =
    rm.toFixed(2) + ' m, ' + angPos.toFixed(1) + '°';
});

// ── grid ──────────────────────────────────────────────────────────────────────
function drawGrid(scale) {
  const rings = Math.floor(maxRange);
  // range rings
  for (let r = 1; r <= rings; r++) {
    const px = r * scale;
    ctx.beginPath();
    ctx.arc(CX, CY, px, 0, 2 * Math.PI);
    ctx.strokeStyle = r === rings ? '#45475a' : '#2a2a3e';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = '#585b70';
    ctx.font = '10px monospace';
    ctx.fillText(r + 'm', CX + px + 3, CY - 3);
  }
  // half-ring at 0.5m intervals
  for (let r = 0.5; r < maxRange; r += 1.0) {
    ctx.beginPath();
    ctx.arc(CX, CY, r * scale, 0, 2 * Math.PI);
    ctx.strokeStyle = '#1f1f30';
    ctx.lineWidth = 0.5;
    ctx.stroke();
  }
  // angle lines every 30°
  ctx.strokeStyle = '#2a2a3e';
  ctx.lineWidth = 0.5;
  for (let a = 0; a < 360; a += 30) {
    const rad = a * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(CX, CY);
    ctx.lineTo(CX + Math.sin(rad) * (W/2-10), CY - Math.cos(rad) * (H/2-10));
    ctx.stroke();
  }
  // cardinal labels
  const labels = { '0°':'N', '90°':'E', '180°':'S', '270°':'W' };
  ctx.fillStyle = '#585b70';
  ctx.font = 'bold 11px monospace';
  const R = W/2 - 14;
  [0, 90, 180, 270].forEach(a => {
    const rad = a * Math.PI / 180;
    const lbl = Object.values(labels)[a/90];
    ctx.fillText(lbl, CX + Math.sin(rad)*R - 5, CY - Math.cos(rad)*R + 4);
  });
  // center dot (robot)
  ctx.beginPath();
  ctx.arc(CX, CY, 4, 0, 2 * Math.PI);
  ctx.fillStyle = '#f38ba8';
  ctx.fill();
  // forward arrow
  ctx.beginPath();
  ctx.moveTo(CX, CY);
  ctx.lineTo(CX, CY - 18);
  ctx.strokeStyle = '#f38ba8';
  ctx.lineWidth = 2;
  ctx.stroke();
}

// ── scan render ───────────────────────────────────────────────────────────────
function draw(ranges, angleMin, angleInc) {
  const scale = (W / 2 - 30) / maxRange;

  ctx.clearRect(0, 0, W, H);

  // outer circle clip
  ctx.save();
  ctx.beginPath();
  ctx.arc(CX, CY, W/2 - 8, 0, 2*Math.PI);
  ctx.clip();

  drawGrid(scale);

  let pts = 0, nearest = Infinity;

  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    if (r === null || r <= 0 || r > maxRange) continue;
    const angle = angleMin + i * angleInc;
    const x = CX + Math.sin(angle) * r * scale;
    const y = CY - Math.cos(angle) * r * scale;
    // hue: 150° (teal) close → 60° (yellow) → 0° (red) far
    const t = r / maxRange;
    const hue = Math.round(150 * (1 - t));
    ctx.fillStyle = `hsl(${hue},90%,60%)`;
    ctx.fillRect(x - 1, y - 1, 2.5, 2.5);
    pts++;
    if (r < nearest) nearest = r;
  }

  ctx.restore();

  // border ring
  ctx.beginPath();
  ctx.arc(CX, CY, W/2 - 8, 0, 2*Math.PI);
  ctx.strokeStyle = '#45475a';
  ctx.lineWidth = 2;
  ctx.stroke();

  document.getElementById('pts').textContent  = pts;
  document.getElementById('near').textContent = nearest === Infinity ? '--' : nearest.toFixed(2);
}

// ── SSE ───────────────────────────────────────────────────────────────────────
const es = new EventSource('/stream');
es.onmessage = e => {
  const d = JSON.parse(e.data);
  lastRanges   = d.ranges;
  lastAngleMin = d.angle_min;
  lastAngleInc = d.angle_inc;
  draw(d.ranges, d.angle_min, d.angle_inc);

  frameCount++;
  const now = performance.now();
  if (now - lastRateTime >= 1000) {
    displayRate = (frameCount * 1000 / (now - lastRateTime)).toFixed(1);
    document.getElementById('rate').textContent = displayRate;
    frameCount = 0;
    lastRateTime = now;
  }
};
es.onerror = () => {
  document.getElementById('dot').style.background = '#f38ba8';
  document.getElementById('dot').style.animation = 'none';
};
</script>
</body>
</html>"""


class LidarVizNode(Node):
    def __init__(self):
        super().__init__('lidar_viz_node')

        self._lock      = threading.Lock()
        self._scan_data = None
        self._seq       = 0

        self.create_subscription(LaserScan, '/scan', self._cb, 10)

        app = Flask(__name__)
        app.logger.disabled = True
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        @app.route('/')
        def index():
            return render_template_string(HTML)

        @app.route('/stream')
        def stream():
            def generate():
                last = -1
                while True:
                    with self._lock:
                        seq, data = self._seq, self._scan_data
                    if seq != last and data is not None:
                        last = seq
                        yield f'data: {json.dumps(data)}\n\n'
                    else:
                        time.sleep(0.02)
            return Response(generate(),
                            mimetype='text/event-stream',
                            headers={'Cache-Control': 'no-cache',
                                     'X-Accel-Buffering': 'no'})

        t = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=PORT, threaded=True),
            daemon=True)
        t.start()

        import socket
        ip = socket.gethostbyname(socket.gethostname())
        self.get_logger().info(f'Lidar viz at http://{ip}:{PORT}')

    def _cb(self, msg: LaserScan):
        # Replace inf/nan with None (JSON-safe) and downsample to reduce payload
        ranges = [None if (math.isinf(r) or math.isnan(r)) else round(r, 3)
                  for r in msg.ranges]
        data = {
            'ranges':    ranges,
            'angle_min': round(msg.angle_min, 5),
            'angle_inc': round(msg.angle_increment, 6),
        }
        with self._lock:
            self._scan_data = data
            self._seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = LidarVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
