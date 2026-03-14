#!/usr/bin/env python3
"""
Nodo ROS2 que sirve un dashboard web en tiempo real
suscribiéndose a /odom.
Abre http://<IP_DE_LA_PI>:5000 en tu navegador.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from flask import Flask, render_template_string
from flask_socketio import SocketIO


# ─────────────────────────────────────────────────────────────────
# HTML + JS
# ─────────────────────────────────────────────────────────────────
HTML = """
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <title>Robot Dashboard</title>
  <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
  <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #1e1e2e;
      color: #cdd6f4;
      font-family: 'Segoe UI', sans-serif;
      padding: 20px;
    }
    h1 {
      text-align: center;
      font-size: 1.5rem;
      margin-bottom: 20px;
      color: #89b4fa;
      letter-spacing: 2px;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
      max-width: 1200px;
      margin: 0 auto;
    }
    .card {
      background: #313244;
      border-radius: 12px;
      padding: 16px;
      border: 1px solid #45475a;
    }
    .card h2 {
      font-size: 0.85rem;
      color: #a6adc8;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 12px;
    }
    .card.full { grid-column: 1 / 3; }
    .stats-grid {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 10px;
    }
    .stat {
      background: #1e1e2e;
      border-radius: 8px;
      padding: 12px;
      text-align: center;
    }
    .stat .label { font-size: 0.75rem; color: #a6adc8; margin-bottom: 4px; }
    .stat .value { font-size: 1.4rem; font-weight: bold; color: #89b4fa; }
    .stat .unit  { font-size: 0.7rem; color: #6c7086; }
    .status-dot {
      display: inline-block;
      width: 10px; height: 10px;
      border-radius: 50%;
      background: #a6e3a1;
      margin-right: 6px;
      animation: pulse 1.5s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50%       { opacity: 0.3; }
    }
  </style>
</head>
<body>

<h1>🤖 Robot Dashboard</h1>

<div class="grid">

  <!-- Trayectoria -->
  <div class="card full">
    <h2><span class="status-dot"></span>Trayectoria en tiempo real</h2>
    <div id="plot-traj" style="height:350px;"></div>
  </div>

  <!-- Stats -->
  <div class="card full">
    <h2>Estado actual</h2>
    <div class="stats-grid">
      <div class="stat">
        <div class="label">X</div>
        <div class="value" id="val-x">0.000</div>
        <div class="unit">metros</div>
      </div>
      <div class="stat">
        <div class="label">Y</div>
        <div class="value" id="val-y">0.000</div>
        <div class="unit">metros</div>
      </div>
      <div class="stat">
        <div class="label">Yaw</div>
        <div class="value" id="val-yaw">0.0</div>
        <div class="unit">grados</div>
      </div>
      <div class="stat">
        <div class="label">Velocidad</div>
        <div class="value" id="val-vel">0.000</div>
        <div class="unit">m/s</div>
      </div>
    </div>
  </div>



  <!-- Yaw -->
  <div class="card">
    <h2>Yaw a lo largo del tiempo</h2>
    <div id="plot-yaw" style="height:280px;"></div>
  </div>

</div>

<script>
  const socket = io();

  let xs = [0], ys = [0];
  let times = [0], enc_l = [0], enc_r = [0], yaws_deg = [0];
  let t0 = null;

  const LAYOUT_BASE = {
    paper_bgcolor: '#313244',
    plot_bgcolor:  '#1e1e2e',
    font:   { color: '#cdd6f4' },
    margin: { t: 10, b: 40, l: 55, r: 10 },
    legend: { bgcolor: '#1e1e2e', bordercolor: '#45475a', borderwidth: 1 },
  };

  // ── Trayectoria ───────────────────────────────────────────────
  Plotly.newPlot('plot-traj', [
    { x: xs, y: ys,
      mode: 'lines', line: { color: '#89b4fa', width: 2 }, name: 'Trayectoria' },
    { x: [0], y: [0],
      mode: 'markers', marker: { color: '#a6e3a1', size: 12 }, name: 'Inicio' },
    { x: [0], y: [0],
      mode: 'markers', marker: { color: '#f38ba8', size: 12 }, name: 'Robot' },
  ], { ...LAYOUT_BASE,
    xaxis: { title: 'X (m)', gridcolor: '#45475a', zeroline: false },
    yaxis: { title: 'Y (m)', gridcolor: '#45475a', zeroline: false, scaleanchor: 'x' },
    showlegend: true,
  }, { responsive: true });

  // ── Encoders ──────────────────────────────────────────────────
  Plotly.newPlot('plot-enc', [
    { x: times, y: enc_l,
      mode: 'lines', line: { color: '#a6e3a1', width: 2 }, name: 'Izquierdo' },
    { x: times, y: enc_r,
      mode: 'lines', line: { color: '#fab387', width: 2 }, name: 'Derecho' },
  ], { ...LAYOUT_BASE,
    xaxis: { title: 'Tiempo (s)', gridcolor: '#45475a' },
    yaxis: { title: 'Ticks',      gridcolor: '#45475a' },
  }, { responsive: true });

  // ── Yaw ───────────────────────────────────────────────────────
  Plotly.newPlot('plot-yaw', [
    { x: times, y: yaws_deg,
      mode: 'lines', line: { color: '#cba6f7', width: 2 }, name: 'Yaw' },
  ], { ...LAYOUT_BASE,
    xaxis: { title: 'Tiempo (s)', gridcolor: '#45475a' },
    yaxis: { title: 'Yaw (°)',    gridcolor: '#45475a' },
  }, { responsive: true });

  // ── Recibir datos ─────────────────────────────────────────────
  socket.on('odom_data', (d) => {
    if (t0 === null) t0 = d.timestamp;
    const t = d.timestamp - t0;

    xs.push(d.x);         ys.push(d.y);
    times.push(t);        enc_l.push(d.enc_l);
    enc_r.push(d.enc_r);  yaws_deg.push(d.yaw_deg);

    Plotly.update('plot-traj',
      { x: [xs, [xs[0]], [d.x]], y: [ys, [ys[0]], [d.y]] },
      {}, [0, 1, 2]
    );
    Plotly.update('plot-enc',
      { x: [times, times], y: [enc_l, enc_r] }, {}, [0, 1]
    );
    Plotly.update('plot-yaw',
      { x: [times], y: [yaws_deg] }, {}, [0]
    );

    document.getElementById('val-x').textContent    = d.x.toFixed(3);
    document.getElementById('val-y').textContent    = d.y.toFixed(3);
    document.getElementById('val-yaw').textContent  = d.yaw_deg.toFixed(1);
    document.getElementById('val-enc-l').textContent = d.enc_l;
    document.getElementById('val-enc-r').textContent = d.enc_r;
    document.getElementById('val-vel').textContent  = d.vel.toFixed(3);
  });

  // ── Reset al iniciar nueva sesión ─────────────────────────────
  socket.on('reset', () => {
    xs=[0]; ys=[0]; times=[0]; enc_l=[0]; enc_r=[0]; yaws_deg=[0]; t0=null;
    Plotly.update('plot-traj', { x:[[0],[0],[0]], y:[[0],[0],[0]] }, {}, [0,1,2]);
    Plotly.update('plot-enc',  { x:[[0],[0]], y:[[0],[0]] }, {}, [0,1]);
    Plotly.update('plot-yaw',  { x:[[0]], y:[[0]] }, {}, [0]);
  });
</script>
</body>
</html>
"""


# ─────────────────────────────────────────────────────────────────
# Flask + SocketIO
# ─────────────────────────────────────────────────────────────────
app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


@app.route("/")
def index():
    return render_template_string(HTML)


# ─────────────────────────────────────────────────────────────────
# Nodo ROS2
# ─────────────────────────────────────────────────────────────────
class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_plotter")

        self.xs_    = [0.0]
        self.ys_    = [0.0]
        self.yaws_  = [0.0]
        self.prev_x_ = 0.0
        self.prev_y_ = 0.0
        self.prev_t_ = None
        self.lock_   = threading.Lock()

        self.odom_sub_ = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.mode_sub_ = self.create_subscription(
            String, "/motor_mode", self.mode_callback, 10
        )

        self.get_logger().info(
            "TrajectoryPlotter listo.\n"
            "  Abre en tu navegador: http://<IP_DE_LA_PI>:5000"
        )

    def odom_callback(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Velocidad
        if self.prev_t_ is not None and (now - self.prev_t_) > 0:
            vel = math.hypot(x - self.prev_x_, y - self.prev_y_) / (now - self.prev_t_)
        else:
            vel = 0.0

        self.prev_x_ = x
        self.prev_y_ = y
        self.prev_t_ = now

        with self.lock_:
            if abs(x - self.xs_[-1]) > 0.001 or abs(y - self.ys_[-1]) > 0.001:
                self.xs_.append(x)
                self.ys_.append(y)
                self.yaws_.append(yaw)

        sio.emit("odom_data", {
            "x":        round(x,   4),
            "y":        round(y,   4),
            "yaw_deg":  round(math.degrees(yaw), 2),
            "vel":      round(vel, 4),
            "timestamp": now,
        })

    def mode_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ("record", "play"):
            sio.emit("reset")
            with self.lock_:
                self.xs_   = [0.0]
                self.ys_   = [0.0]
                self.yaws_ = [0.0]


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()

    # Flask en hilo separado
    flask_thread = threading.Thread(
        target=lambda: sio.run(
            app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True
        ),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()