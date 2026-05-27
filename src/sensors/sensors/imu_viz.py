#!/usr/bin/env python3
"""
IMU visualizer — suscribe a /imu y sirve un dashboard web en el puerto 5003.
Abrir http://<ip-del-robot>:5003 desde cualquier navegador en la red.
"""
import json
import socket
import threading
import time

from flask import Flask, Response, render_template_string
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

PORT = 5003

HTML = """<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>IMU — BNO055</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <style>
    *{margin:0;padding:0;box-sizing:border-box}
    body{background:#1e1e2e;color:#cdd6f4;font-family:'Segoe UI',sans-serif;
         display:flex;flex-direction:column;align-items:center;padding:16px;gap:12px}
    h1{color:#89b4fa;letter-spacing:3px;font-size:1.2rem}

    .row{display:flex;gap:12px;width:100%;max-width:1040px}

    /* ── 3D ── */
    #three-wrap{flex:0 0 300px;height:300px;background:#11111b;
                border-radius:12px;border:1px solid #45475a;overflow:hidden}

    /* ── ángulos ── */
    .angles{flex:1;display:flex;flex-direction:column;gap:8px}
    .angle-row{display:flex;gap:8px}
    .acard{flex:1;background:#313244;border:1px solid #45475a;border-radius:10px;
           padding:10px 14px}
    .acard .lbl{font-size:.68rem;color:#a6adc8;text-transform:uppercase;
                letter-spacing:1px;margin-bottom:2px}
    .acard .val{font-size:1.9rem;font-weight:bold;font-variant-numeric:tabular-nums}
    .acard .bar-bg{height:5px;background:#181825;border-radius:3px;margin-top:6px}
    .acard .bar{height:5px;border-radius:3px;transition:width .08s linear}
    .roll-c{color:#f38ba8} .roll-b{background:#f38ba8}
    .pitch-c{color:#a6e3a1} .pitch-b{background:#a6e3a1}
    .yaw-c{color:#89b4fa}   .yaw-b{background:#89b4fa}

    .kv-row{display:flex;gap:8px;margin-top:auto}
    .kv{flex:1;background:#1e1e2e;border-radius:8px;padding:8px 12px}
    .kv .lbl{font-size:.68rem;color:#a6adc8;text-transform:uppercase;letter-spacing:1px}
    .kv .val{font-size:1.1rem;font-weight:bold;color:#cba6f7;font-variant-numeric:tabular-nums}

    /* ── calibración ── */
    .cal-row{display:flex;gap:8px;width:100%;max-width:1040px}
    .cal-card{flex:1;background:#313244;border:1px solid #45475a;border-radius:10px;
              padding:10px;text-align:center}
    .cal-card .lbl{font-size:.68rem;color:#a6adc8;text-transform:uppercase;
                   letter-spacing:1px;margin-bottom:6px}
    .dots{display:flex;justify-content:center;gap:5px}
    .dot{width:13px;height:13px;border-radius:50%;background:#181825;
         border:1px solid #45475a;transition:all .3s}
    .dot.on{border:none;box-shadow:0 0 7px currentColor}
    .d0{color:#f38ba8;background:#f38ba8}
    .d1{color:#fab387;background:#fab387}
    .d2{color:#f9e2af;background:#f9e2af}
    .d3{color:#a6e3a1;background:#a6e3a1}

    /* ── gráficas ── */
    .graph-row{display:flex;gap:12px;width:100%;max-width:1040px}
    .gcard{flex:1;background:#313244;border:1px solid #45475a;border-radius:10px;padding:10px}
    .gcard h2{font-size:.72rem;color:#a6adc8;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px}
    canvas.g{display:block;width:100%;height:110px;background:#11111b;border-radius:6px}

    /* ── status ── */
    #sbar{display:flex;gap:20px;background:#313244;border:1px solid #45475a;
          border-radius:10px;padding:7px 16px;font-size:.82rem;width:100%;max-width:1040px}
    #dot{width:8px;height:8px;border-radius:50%;background:#a6e3a1;
         display:inline-block;animation:pulse 1.5s infinite}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:.2}}
    .sv .lbl{color:#585b70;font-size:.68rem;text-transform:uppercase}
    .sv .val{color:#89b4fa;font-weight:bold}
  </style>
</head>
<body>
<h1>&#9632; IMU VIEWER — BNO055</h1>

<div class="row">
  <div id="three-wrap"></div>

  <div class="angles">
    <div class="angle-row">
      <div class="acard">
        <div class="lbl">Roll</div>
        <div class="val roll-c"  id="roll">+0.0°</div>
        <div class="bar-bg"><div class="bar roll-b"  id="b-roll"  style="width:50%"></div></div>
      </div>
      <div class="acard">
        <div class="lbl">Pitch</div>
        <div class="val pitch-c" id="pitch">+0.0°</div>
        <div class="bar-bg"><div class="bar pitch-b" id="b-pitch" style="width:50%"></div></div>
      </div>
      <div class="acard">
        <div class="lbl">Yaw</div>
        <div class="val yaw-c"   id="yaw">+0.0°</div>
        <div class="bar-bg"><div class="bar yaw-b"   id="b-yaw"   style="width:50%"></div></div>
      </div>
    </div>
    <div class="kv-row">
      <div class="kv">
        <div class="lbl">ω z (yaw rate)</div>
        <div class="val" id="wz">0.0000 rad/s</div>
      </div>
      <div class="kv">
        <div class="lbl">a x (forward)</div>
        <div class="val" id="ax">0.000 m/s²</div>
      </div>
      <div class="kv">
        <div class="lbl">Estado cal.</div>
        <div class="val" id="cal-badge" style="color:#cba6f7">Calibrando…</div>
      </div>
    </div>
  </div>
</div>

<!-- calibración -->
<div class="cal-row">
  <div class="cal-card"><div class="lbl">Sistema</div>
    <div class="dots" id="d-sys"></div></div>
  <div class="cal-card"><div class="lbl">Giroscopio</div>
    <div class="dots" id="d-gyro"></div></div>
  <div class="cal-card"><div class="lbl">Acelerómetro</div>
    <div class="dots" id="d-acc"></div></div>
  <div class="cal-card"><div class="lbl">Magnetómetro</div>
    <div class="dots" id="d-mag"></div></div>
</div>

<!-- gráficas -->
<div class="graph-row">
  <div class="gcard">
    <h2>Velocidad angular (rad/s) — X Y Z</h2>
    <canvas class="g" id="cg"></canvas>
  </div>
  <div class="gcard">
    <h2>Aceleración lineal (m/s²) — X Y Z</h2>
    <canvas class="g" id="ca"></canvas>
  </div>
</div>

<!-- status -->
<div id="sbar">
  <div class="sv"><div class="lbl">Estado</div>
    <div class="val"><span id="dot"></span> Live</div></div>
  <div class="sv"><div class="lbl">Rate</div>
    <div class="val"><span id="hz">--</span> Hz</div></div>
  <div class="sv"><div class="lbl">Quaternion w</div>
    <div class="val"><span id="qw">--</span></div></div>
</div>

<script>
// ── Three.js ──────────────────────────────────────────────────────────────────
const wrap = document.getElementById('three-wrap');
const TW = wrap.clientWidth, TH = wrap.clientHeight;
const renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
renderer.setSize(TW, TH);
renderer.setPixelRatio(window.devicePixelRatio);
wrap.appendChild(renderer.domElement);

const scene  = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, TW/TH, 0.1, 100);
camera.position.set(0, 1.4, 3.8);
camera.lookAt(0, 0, 0);

// robot body (diferencial: ancho, largo, bajo)
const geo  = new THREE.BoxGeometry(1.8, 0.48, 2.2);
const mats = [
  new THREE.MeshLambertMaterial({color:0x89b4fa}), // +x right
  new THREE.MeshLambertMaterial({color:0x89b4fa}), // -x left
  new THREE.MeshLambertMaterial({color:0xa6e3a1}), // +y top  (verde)
  new THREE.MeshLambertMaterial({color:0x313244}), // -y bottom
  new THREE.MeshLambertMaterial({color:0xf38ba8}), // +z front (rosa = adelante)
  new THREE.MeshLambertMaterial({color:0x45475a}), // -z back
];
const box = new THREE.Mesh(geo, mats);
scene.add(box);

// flecha adelante
const cone = new THREE.Mesh(
  new THREE.ConeGeometry(0.11, 0.38, 8),
  new THREE.MeshLambertMaterial({color:0xf38ba8}));
cone.rotation.x = -Math.PI/2;
cone.position.set(0, 0.3, -1.3);
box.add(cone);

// ejes y piso
scene.add(new THREE.AxesHelper(1.8));
scene.add(new THREE.GridHelper(6, 12, 0x313244, 0x313244));

// luz
scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dl = new THREE.DirectionalLight(0xffffff, 0.85);
dl.position.set(4, 6, 3); scene.add(dl);

const targetQ = new THREE.Quaternion();
(function animate(){
  requestAnimationFrame(animate);
  box.quaternion.slerp(targetQ, 0.14);
  renderer.render(scene, camera);
})();

// ── Rolling graph ─────────────────────────────────────────────────────────────
const N = 220;
const COLORS = ['#f38ba8','#a6e3a1','#89b4fa'];

function makeGraph(id) {
  const cv  = document.getElementById(id);
  cv.width  = cv.offsetWidth; cv.height = cv.offsetHeight;
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  const bufs = [new Float32Array(N), new Float32Array(N), new Float32Array(N)];
  let ptr = 0;

  return function push(x, y, z) {
    bufs[0][ptr]=x; bufs[1][ptr]=y; bufs[2][ptr]=z;
    ptr = (ptr+1) % N;

    let mx = 0.05;
    for (const b of bufs) for (const v of b) if (Math.abs(v)>mx) mx=Math.abs(v);
    mx *= 1.15;

    ctx.fillStyle = '#11111b'; ctx.fillRect(0,0,W,H);
    ctx.strokeStyle='#313244'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(0,H/2); ctx.lineTo(W,H/2); ctx.stroke();

    for (let ci=0; ci<3; ci++) {
      ctx.strokeStyle = COLORS[ci]; ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i=0; i<N; i++) {
        const si = (ptr+i) % N;
        const px = i*W/(N-1);
        const py = H/2 - (bufs[ci][si]/mx)*(H/2-5);
        i===0 ? ctx.moveTo(px,py) : ctx.lineTo(px,py);
      }
      ctx.stroke();
      const last = bufs[ci][(ptr-1+N)%N];
      ctx.fillStyle=COLORS[ci]; ctx.font='9px monospace';
      ctx.fillText(['X','Y','Z'][ci]+' '+last.toFixed(3), 4+ci*50, 11);
    }
  };
}

const pushGyro = makeGraph('cg');
const pushAcc  = makeGraph('ca');

// ── calibración dots ──────────────────────────────────────────────────────────
['sys','gyro','acc','mag'].forEach(k => {
  const el = document.getElementById('d-'+k);
  el.innerHTML = [0,1,2,3].map(i=>`<div class="dot" id="dot-${k}-${i}"></div>`).join('');
});
function setCal(key, n) {
  [0,1,2,3].forEach(i => {
    const d = document.getElementById(`dot-${key}-${i}`);
    d.className = i < n ? `dot on d${n}` : 'dot';
  });
}

// ── Quaternion → Euler ────────────────────────────────────────────────────────
function qToEuler(w,x,y,z) {
  const roll  = Math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
  const sinp  = 2*(w*y-z*x);
  const pitch = Math.abs(sinp)>=1 ? Math.sign(sinp)*Math.PI/2 : Math.asin(sinp);
  const yaw   = Math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
  return [roll,pitch,yaw].map(v=>v*180/Math.PI);
}
function fmt(v){ return (v>=0?'+':'')+v.toFixed(1)+'°'; }
function bar(deg){ return Math.min(100,Math.max(0,50+deg/3.6)).toFixed(1)+'%'; }

// ── Hz ────────────────────────────────────────────────────────────────────────
let frames=0, lastT=performance.now();

// ── SSE ───────────────────────────────────────────────────────────────────────
const es = new EventSource('/stream');
es.onmessage = ev => {
  const d = JSON.parse(ev.data);

  // 3D
  targetQ.set(d.ox, d.oy, d.oz, d.ow);

  // euler
  const [roll,pitch,yaw] = qToEuler(d.ow,d.ox,d.oy,d.oz);
  document.getElementById('roll').textContent  = fmt(roll);
  document.getElementById('pitch').textContent = fmt(pitch);
  document.getElementById('yaw').textContent   = fmt(yaw);
  document.getElementById('b-roll').style.width  = bar(roll);
  document.getElementById('b-pitch').style.width = bar(pitch);
  document.getElementById('b-yaw').style.width   = bar(yaw);

  // kv
  document.getElementById('wz').textContent = d.gz.toFixed(4)+' rad/s';
  document.getElementById('ax').textContent = d.ax.toFixed(3)+' m/s²';
  document.getElementById('qw').textContent = d.ow.toFixed(4);

  // graphs
  pushGyro(d.gx, d.gy, d.gz);
  pushAcc(d.ax, d.ay, d.az);

  // cal
  const [cs,cg,ca,cm] = d.cal;
  setCal('sys',cs); setCal('gyro',cg); setCal('acc',ca); setCal('mag',cm);
  const ready = cs===3 && cg===3 && ca===3;
  const badge = document.getElementById('cal-badge');
  badge.textContent = ready ? '✓ Listo' : 'Calibrando…';
  badge.style.color = ready ? '#a6e3a1' : '#cba6f7';

  // hz
  frames++;
  const now = performance.now();
  if (now-lastT >= 1000){
    document.getElementById('hz').textContent = (frames*1000/(now-lastT)).toFixed(1);
    frames=0; lastT=now;
  }
};
es.onerror = () => {
  const d = document.getElementById('dot');
  d.style.background='#f38ba8'; d.style.animation='none';
};
</script>
</body>
</html>"""


class ImuVizNode(Node):
    def __init__(self):
        super().__init__('imu_viz_node')

        self._lock = threading.Lock()
        self._data = None
        self._seq  = 0

        self.create_subscription(Imu, '/imu', self._cb, 10)

        app = Flask(__name__)
        import logging
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        app.logger.disabled = True

        @app.route('/')
        def index():
            return render_template_string(HTML)

        @app.route('/stream')
        def stream():
            def generate():
                last = -1
                while True:
                    with self._lock:
                        seq, data = self._seq, self._data
                    if seq != last and data is not None:
                        last = seq
                        yield f'data: {json.dumps(data)}\n\n'
                    else:
                        time.sleep(0.018)
            return Response(generate(),
                            mimetype='text/event-stream',
                            headers={'Cache-Control': 'no-cache',
                                     'X-Accel-Buffering': 'no'})

        threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=PORT, threaded=True),
            daemon=True).start()

        ip = socket.gethostbyname(socket.gethostname())
        self.get_logger().info(f'IMU viz en http://{ip}:{PORT}')

    def _cb(self, msg: Imu):
        o  = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration

        data = {
            'ow': round(o.w,  6), 'ox': round(o.x, 6),
            'oy': round(o.y,  6), 'oz': round(o.z, 6),
            'gx': round(av.x, 5), 'gy': round(av.y, 5), 'gz': round(av.z, 5),
            'ax': round(la.x, 4), 'ay': round(la.y, 4), 'az': round(la.z, 4),
            # cal no viene en el mensaje Imu estándar — el mpu_node lo loguea
            'cal': [0, 0, 0, 0],
        }
        with self._lock:
            self._data = data
            self._seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImuVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
