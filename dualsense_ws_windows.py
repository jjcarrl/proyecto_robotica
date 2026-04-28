#!/usr/bin/env python
"""
Control DualSense del brazo RRR — version WebSocket.
Los comandos se envian a la Raspberry Pi (que tiene la ESP32 conectada por USB)
en lugar de al puerto serial local.

Dependencias Windows:
    pip install websockets PyQt5 numpy

Uso:
    python dualsense_ws_windows.py --ip 192.168.x.x
"""

import argparse
import asyncio
import queue
import sys
import threading
import types

import websockets

# Bloquear mujoco antes de importar dms
if "mujoco" not in sys.modules:
    sys.modules["mujoco"] = types.ModuleType("mujoco")

# ─────────────────────────────── ejes y botones DualSense ────────────────────

_DS_LEFT_X  = 0
_DS_LEFT_Y  = 1
_DS_RIGHT_Y = 3

_BTN_CROSS    = 0
_BTN_CIRCLE   = 1
_BTN_SQUARE   = 2
_BTN_TRIANGLE = 3


# ────────────────────────────────── WebSocket sender ─────────────────────────

class _WsSender:
    """
    Cliente WebSocket no bloqueante que corre en su propio hilo.
    Reconecta automáticamente si se cae la conexión.
    """

    def __init__(self, ip: str, port: int):
        self._uri   = f"ws://{ip}:{port}"
        self._queue = queue.Queue(maxsize=5)  # no acumular frames viejos
        threading.Thread(target=self._run, daemon=True).start()
        print(f"[WsSender] Conectando a {self._uri} ...")

    def send(self, cmd: str):
        try:
            self._queue.put_nowait(cmd)
        except queue.Full:
            self._queue.get_nowait()   # descartar frame viejo
            self._queue.put_nowait(cmd)

    def _run(self):
        asyncio.run(self._async_run())

    async def _async_run(self):
        loop = asyncio.get_event_loop()
        while True:
            try:
                async with websockets.connect(self._uri) as ws:
                    print(f"[WsSender] Conectado a {self._uri}")
                    while True:
                        cmd = await loop.run_in_executor(None, self._queue.get)
                        await ws.send(cmd)
            except Exception as e:
                print(f"[WsSender] Reconectando en 2 s... ({e})")
                await asyncio.sleep(2)


# ──────────────────────────────── robot RRR físico ───────────────────────────

def _make_rrr_physical(sender: _WsSender, L1=19.5, L2=11.5, L3=20.5):
    from dms.robots import RRR
    from dms.robots.model import JointInfo

    class RRRPhysical(RRR):
        def joint_info(self):
            return [
                JointInfo("Joint 1",  0,  180,  90),
                JointInfo("Joint 2", -90,  90,   0),
                JointInfo("Joint 3", -90,  90, -90),
            ]

        def serial_config(self):
            # La ESP32 está en la Pi, no aquí: devolver None para que la
            # librería no intente abrir un puerto serial local.
            return None

        def serial_command(self, joint_angles_deg):
            t1, t2 = joint_angles_deg[0], joint_angles_deg[1]
            d23 = max(0, min(180, int(round(t1))))
            d15 = 180 - d23
            d13 = max(0, min(180, int(round(t2 + 90))))
            d25 = max(0, min(180, int(round(90 + t1 + t2))))
            cmd = f"{d23},{d15},{d13},{d25 - 15}\n"
            sender.send(cmd)
            return b""   # sin serial local

    return RRRPhysical(params={"L1": L1, "L2": L2, "L3": L3})


# ────────────────────────────── parche del viewer ────────────────────────────

def _patch_viewer(viewer_cls):
    viewer_cls._FK_AXIS_MAP = [_DS_LEFT_Y, _DS_RIGHT_Y]
    viewer_cls._FK_SPEED    = 1.5
    viewer_cls._IK_SPEED    = 1.5e-2

    _orig_update_fk = viewer_cls._update_fk

    def _update_fk_horizontal(self):
        t1 = self.sliders[0].get_deg()
        t2 = self.sliders[1].get_deg()
        t3 = -(t1 + t2)
        s3 = self.sliders[2]
        t3 = max(s3.joint.min_deg, min(s3.joint.max_deg, t3))
        s3.set_value(t3)
        _orig_update_fk(self)

    def _gp_axes_ds(self, axes: dict):
        if self.fk_btn.isChecked():
            moved = False
            for i, slider in enumerate(self.sliders[:2]):
                val = axes.get(self._FK_AXIS_MAP[i], 0.0)
                if val == 0.0:
                    continue
                delta = -val * self._FK_SPEED
                new   = max(slider.joint.min_deg,
                            min(slider.joint.max_deg, slider.get_deg() + delta))
                slider.set_value(new)
                moved = True
            if moved:
                self._update_fk()
        else:
            vx = axes.get(_DS_LEFT_X, 0.0)
            vy = -axes.get(_DS_LEFT_Y, 0.0)
            if (vx or vy) and self._last_eef:
                tx = self._last_eef[0] + vx * self._IK_SPEED
                ty = self._last_eef[1] + vy * self._IK_SPEED
                self._on_ik_request(tx, ty)

    def _gp_button_ds(self, btn: int):
        if   btn == _BTN_CROSS:    self.ik_btn.setChecked(True) if self.fk_btn.isChecked() else self.fk_btn.setChecked(True)
        elif btn == _BTN_CIRCLE:   self._reset()
        elif btn == _BTN_SQUARE:   self.trail_cb.setChecked(not self.trail_cb.isChecked())
        elif btn == _BTN_TRIANGLE: self.canvas.clear_trail()

    def _init_gamepad_ds(self, parent_layout):
        try:
            from dms.robots.robot_viewer.gamepad import GamepadManager
        except ImportError:
            return
        if not GamepadManager.available():
            return

        from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QLabel
        gp_box = QGroupBox("DualSense")
        gp_lay = QVBoxLayout(gp_box)
        self.gp_status = QLabel("Buscando controlador ...")
        self.gp_status.setStyleSheet("color: #6c7086;")
        gp_lay.addWidget(self.gp_status)

        hint = QLabel(
            "Stick izq Y:   Joint 1   [FK]\n"
            "Stick der Y:   Joint 2   [FK]\n"
            "Stick izq X/Y: EEF       [IK]\n"
            "Joint 3:       auto-horizontal\n\n"
            "×  alternar IK/FK\n"
            "○  reset   □  rastro   △  limpiar"
        )
        hint.setStyleSheet("color: #585b70; font-size: 11px;")
        gp_lay.addWidget(hint)
        parent_layout.addWidget(gp_box)

        self._gamepad = GamepadManager(parent=self)
        self._gamepad.connected.connect(self._gp_connected)
        self._gamepad.disconnected.connect(self._gp_disconnected)
        self._gamepad.axes_updated.connect(self._on_gp_axes)
        self._gamepad.button_pressed.connect(self._on_gp_button)

    viewer_cls._update_fk    = _update_fk_horizontal
    viewer_cls._on_gp_axes   = _gp_axes_ds
    viewer_cls._on_gp_button = _gp_button_ds
    viewer_cls._init_gamepad = _init_gamepad_ds
    return viewer_cls


def _setup_window(win):
    win.sliders[2].slider.setEnabled(False)
    win.sliders[2].name_label.setText("Joint 3  [auto-horiz]")
    win.ik_btn.setChecked(True)


# ──────────────────────────────────── main ───────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control DualSense via WebSocket")
    parser.add_argument("--ip",   required=True,        help="IP de la Raspberry Pi")
    parser.add_argument("--port", default=8766, type=int, help="Puerto WebSocket del arm_bridge")
    parser.add_argument("--L1",   default=19.5, type=float)
    parser.add_argument("--L2",   default=11.5, type=float)
    parser.add_argument("--L3",   default=20.5, type=float)
    args = parser.parse_args()

    print(f"Brazo RRR — DualSense WS   L1={args.L1}  L2={args.L2}  L3={args.L3}")
    print(f"Enviando a ws://{args.ip}:{args.port}")
    print("=" * 55)

    sender = _WsSender(args.ip, args.port)
    robot  = _make_rrr_physical(sender, args.L1, args.L2, args.L3)

    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QApplication
    from dms.robots.robot_viewer.gui import RobotViewer, DARK_STYLE

    _patch_viewer(RobotViewer)

    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)

    win = RobotViewer(robot, title=f"Brazo RRR — DualSense WS  ({args.ip}:{args.port})")
    win.show()

    QTimer.singleShot(100, lambda: _setup_window(win))

    sys.exit(app.exec_())
