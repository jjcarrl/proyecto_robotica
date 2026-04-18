#!/usr/bin/env python3
"""
Servidor de trayectoria standalone — sin ROS.

Endpoints:
  POST /push   {"x", "y", "yaw_deg", "vel", "timestamp"}  → actualiza estado
  POST /reset                                              → limpia trayectoria
  GET  /data                                              → devuelve estado actual (JSON)
  GET  /                                                  → dashboard HTML
"""

import socket
import json
import threading
from flask import Flask, jsonify, render_template_string


HTML = """
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <title>Robot Dashboard</title>
  <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { background: #1e1e2e; color: #cdd6f4; font-family: 'Segoe UI', sans-serif; padding: 20px; }
    h1 { text-align: center; font-size: 1.5rem; margin-bottom: 20px; color: #89b4fa; letter-spacing: 2px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; max-width: 1200px; margin: 0 auto; }
    .card { background: #313244; border-radius: 12px; padding: 16px; border: 1px solid #45475a; }
    .card h2 { font-size: 0.85rem; color: #a6adc8; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 12px; }
    .card.full { grid-column: 1 / 3; }
    .stats-grid { display: grid; grid-template-columns: 1fr 1fr 1fr 1fr; gap: 10px; }
    .stat { background: #1e1e2e; border-radius: 8px; padding: 12px; text-align: center; }
    .stat .label { font-size: 0.75rem; color: #a6adc8; margin-bottom: 4px; }
    .stat .value { font-size: 1.4rem; font-weight: bold; color: #89b4fa; }
    .stat .unit  { font-size: 0.7rem; color: #6c7086; }
    .status-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%;
                  background: #a6e3a1; margin-right: 6px; animation: pulse 1.5s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.3; } }
  </style>
</head>
<body>

<h1>🤖 Robot Dashboard</h1>

<div class="grid">

  <div class="card full">
    <h2><span class="status-dot"></span>Trayectoria en tiempo real</h2>
    <div id="plot-traj" style="height:350px;"></div>
  </div>

  <div class="card full">
    <h2>Estado actual</h2>
    <div class="stats-grid">
      <div class="stat"><div class="label">X</div><div class="value" id="val-x">0.000</div><div class="unit">metros</div></div>
      <div class="stat"><div class="label">Y</div><div class="value" id="val-y">0.000</div><div class="unit">metros</div></div>
      <div class="stat"><div class="label">Yaw</div><div class="value" id="val-yaw">0.0</div><div class="unit">grados</div></div>
      <div class="stat"><div class="label">Velocidad</div><div class="value" id="val-vel">0.000</div><div class="unit">m/s</div></div>
    </div>
  </div>

  <div class="card full">
    <h2>Yaw a lo largo del tiempo</h2>
    <div id="plot-yaw" style="height:280px;"></div>
  </div>

</div>

<script>
  let xs = [0], ys = [0], times = [0], yaws_deg = [0];
  let t0 = null, lastReset = 0;

  const BASE = {
    paper_bgcolor: '#313244', plot_bgcolor: '#1e1e2e',
    font: { color: '#cdd6f4' },
    margin: { t: 10, b: 40, l: 55, r: 10 },
    legend: { bgcolor: '#1e1e2e', bordercolor: '#45475a', borderwidth: 1 },
  };

  Plotly.newPlot('plot-traj', [
    { x: xs, y: ys, mode: 'lines', line: { color: '#89b4fa', width: 2 }, name: 'Trayectoria' },
    { x: [0], y: [0], mode: 'markers', marker: { color: '#a6e3a1', size: 12 }, name: 'Inicio' },
    { x: [0], y: [0], mode: 'markers', marker: { color: '#f38ba8', size: 12 }, name: 'Robot' },
  ], { ...BASE,
    xaxis: { title: 'X (m)', gridcolor: '#45475a', zeroline: false },
    yaxis: { title: 'Y (m)', gridcolor: '#45475a', zeroline: false, scaleanchor: 'x' },
    showlegend: true,
  }, { responsive: true });

  Plotly.newPlot('plot-yaw', [
    { x: times, y: yaws_deg, mode: 'lines', line: { color: '#cba6f7', width: 2 }, name: 'Yaw' },
  ], { ...BASE,
    xaxis: { title: 'Tiempo (s)', gridcolor: '#45475a' },
    yaxis: { title: 'Yaw (°)',    gridcolor: '#45475a' },
  }, { responsive: true });

  async function poll() {
    try {
      const d = await fetch('/data').then(r => r.json());

      if (d.reset_count !== lastReset) {
        lastReset = d.reset_count;
        xs=[0]; ys=[0]; times=[0]; yaws_deg=[0]; t0=null;
        Plotly.update('plot-traj', { x:[[0],[0],[0]], y:[[0],[0],[0]] }, {}, [0,1,2]);
        Plotly.update('plot-yaw',  { x:[[0]], y:[[0]] }, {}, [0]);
      }

      if (t0 === null) t0 = d.timestamp;
      const t = d.timestamp - t0;

      xs.push(d.x);
      ys.push(d.y);
      times.push(t);
      yaws_deg.push(d.yaw_deg);

      Plotly.update('plot-traj',
        { x: [xs, [xs[0]], [d.x]], y: [ys, [ys[0]], [d.y]] },
        {}, [0, 1, 2]
      );
      Plotly.update('plot-yaw', { x: [times], y: [yaws_deg] }, {}, [0]);

      document.getElementById('val-x').textContent   = d.x.toFixed(3);
      document.getElementById('val-y').textContent   = d.y.toFixed(3);
      document.getElementById('val-yaw').textContent = d.yaw_deg.toFixed(1);
      document.getElementById('val-vel').textContent = d.vel.toFixed(3);
    } catch (_) {}
  }

  setInterval(poll, 100);
</script>
</body>
</html>
"""


app = Flask(__name__)

_state = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "vel": 0.0, "timestamp": 0.0, "reset_count": 0}
_lock  = threading.Lock()


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/data")
def data():
    with _lock:
        return jsonify(dict(_state))


def _udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5001))
    while True:
        data, _ = sock.recvfrom(1024)
        try:
            d = json.loads(data.decode())
            with _lock:
                for key in ("x", "y", "yaw_deg", "vel", "timestamp"):
                    if key in d:
                        _state[key] = d[key]
        except Exception:
            pass


def main():
    threading.Thread(target=_udp_listener, daemon=True).start()
    print("Dashboard → http://0.0.0.0:5000  |  UDP → 5001")
    app.run(host="0.0.0.0", port=5000, threaded=True)


if __name__ == "__main__":
    main()
