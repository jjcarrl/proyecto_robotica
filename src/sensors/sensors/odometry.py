#!/usr/bin/env python3

import os
import csv
import time
import math
import threading
import socket
import json

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, RotaryEncoder
Device.pin_factory = LGPIOFactory(chip=0)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf2_ros

# =========================
# Pines GPIO encoders (BCM)
# =========================
PIN_ENC_L_A = 20
PIN_ENC_L_B = 21
PIN_ENC_R_A = 23
PIN_ENC_R_B = 24

# =========================
# Parámetros del robot
# =========================
ENCODER_PPR  = 562
WHEEL_DIAM_M = 0.08
WHEEL_BASE   = 0.20
DIST_PER_TICK = (math.pi * WHEEL_DIAM_M) / ENCODER_PPR


class RecorderPlayer(Node):
    def __init__(self):
        super().__init__("recorder_player")

        # ── Encoders ─────────────────────────────────────────────
        self.enc_l = RotaryEncoder(PIN_ENC_L_A, PIN_ENC_L_B, max_steps=0)
        self.enc_r = RotaryEncoder(PIN_ENC_R_A, PIN_ENC_R_B, max_steps=0)

        # ── Estado ───────────────────────────────────────────────
        self.latest_cmd_   = Twist()
        self.mode_         = "idle"
        self.record_data_  = []
        self.play_index_   = 0
        self.play_start_t_ = 0.0

        # ── Odometría ────────────────────────────────────────────
        self.x_   = 0.0
        self.y_   = 0.0
        self.omega_r = 0.0
        self.omega_l = 0.0
        self.yaw_ = 0.0
        self.prev_ticks_l_ = 0
        self.prev_ticks_r_ = 0

        # ── Suscripciones ────────────────────────────────────────
        self.cmd_sub_ = self.create_subscription(
            Twist, "/motor_vel", self.cmd_callback, 10
        )
        self.mode_sub_ = self.create_subscription(
            String, "/motor_mode", self.mode_callback, 10
        )

        # ── Publicadores ─────────────────────────────────────────
        self.cmd_pub_  = self.create_publisher(Twist,    "/motor_vel", 10)
        self.speed_pub_ = self.create_publisher(Twist,    "/omega", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "/odom",      10)

        # ── TF broadcaster ───────────────────────────────────────
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        # ── Timer principal 50 Hz ────────────────────────────────
        self.timer_period_ = 0.02  # 20 ms -> 50 Hz
        self.timer_ = self.create_timer(self.timer_period_, self.main_loop)

        # ── Socket UDP → dashboard ───────────────────────────────
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── Hilo de teclado ──────────────────────────────────────
        self.keyboard_thread_ = threading.Thread(
            target=self._keyboard_loop, daemon=True
        )
        self.keyboard_thread_.start()

        self._print_status()

    # ─────────────────────────────────────────────────────────────
    # Teclado
    # ─────────────────────────────────────────────────────────────

    def _keyboard_loop(self):
        commands = {
            "record":      "Iniciar grabación",
            "stop_record": "Detener grabación y guardar CSV",
            "play":        "Reproducir grabación",
            "stop_play":   "Detener reproducción",
            "idle":        "Modo inactivo",
            "status":      "Ver modo actual",
            "reset_odom":  "Resetear odometría a cero",
        }
        while rclpy.ok():
            try:
                raw = input()
                cmd = raw.strip().lower()

                if cmd == "":
                    continue
                elif cmd == "status":
                    self._print_status()
                elif cmd == "reset_odom":
                    self._reset_odometry()
                elif cmd in commands:
                    msg = String()
                    msg.data = cmd
                    self.mode_callback(msg)
                else:
                    print(f"  Comando desconocido: '{cmd}'")
                    print(f"  Comandos disponibles: {', '.join(commands.keys())}")

            except EOFError:
                break

    def _print_status(self):
        icons = {
            "idle":      "⏸",
            "recording": "🔴",
            "playing":   "▶",
        }
        icon = icons.get(self.mode_, "?")
        print(f"\n{'─'*40}")
        print(f"  Modo actual: {icon}  {self.mode_.upper()}")
        print(f"  Muestras grabadas: {len(self.record_data_)}")
        print(f"  Posición: x={self.x_:.3f}m  y={self.y_:.3f}m  yaw={math.degrees(self.yaw_):.1f}°")
        print(f"{'─'*40}")
        print(f"  Comandos: record | stop_record | play | stop_play | idle | status | reset_odom")
        print(f"{'─'*40}\n")

    # ─────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────

    def cmd_callback(self, msg: Twist):
        self.latest_cmd_ = msg

    def mode_callback(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == "record":
            self._start_recording()
        elif cmd == "stop_record":
            self._stop_recording()
        elif cmd == "play":
            self._start_playback()
        elif cmd == "stop_play":
            self._stop_playback()
        elif cmd == "idle":
            self.mode_ = "idle"
            print(f"  ⏸  Modo: IDLE")
        else:
            self.get_logger().warn(f"Modo desconocido: '{cmd}'")

    # ─────────────────────────────────────────────────────────────
    # Modos
    # ─────────────────────────────────────────────────────────────

    def _start_recording(self):
        self.record_data_  = []
        self.enc_l.steps   = 0
        self.enc_r.steps   = 0
        self.prev_ticks_l_ = 0
        self.prev_ticks_r_ = 0
        self.x_   = 0.0
        self.y_   = 0.0
        self.yaw_ = 0.0
        self.mode_ = "recording"
        print("  🔴  Modo: RECORDING — grabando comandos y encoders...")

    def _stop_recording(self):
        if self.mode_ != "recording":
            print("  ⚠️  No estaba grabando.")
            return
        self.mode_ = "idle"
        self._save_csv()
        print(f"  ⏹  Grabación detenida. {len(self.record_data_)} muestras guardadas.")

    def _start_playback(self):
        if not self.record_data_:
            print("  ⚠️  No hay datos grabados para reproducir.")
            return
        self.play_index_   = 0
        self.play_start_t_ = self.get_clock().now().nanoseconds * 1e-9
        self.x_   = 0.0
        self.y_   = 0.0
        self.yaw_ = 0.0
        self.mode_ = "playing"
        print(f"  ▶  Modo: PLAYING — reproduciendo {len(self.record_data_)} muestras...")

    def _stop_playback(self):
        self.mode_ = "idle"
        self.cmd_pub_.publish(Twist())
        print("  ⏹  Reproducción detenida.")

    def _reset_odometry(self):
        self.x_            = 0.0
        self.y_            = 0.0
        self.omega_r       = 0.0
        self.omega_l       = 0.0
        self.yaw_          = 0.0
        self.prev_ticks_l_ = 0
        self.prev_ticks_r_ = 0
        print("  🔄  Odometría reseteada a (0, 0, 0°)")

    # ─────────────────────────────────────────────────────────────
    # Loop principal
    # ─────────────────────────────────────────────────────────────

    def main_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._update_odometry()

        if self.mode_ == "recording":
            self._record_sample(now)
        elif self.mode_ == "playing":
            self._playback_step(now)

        self._publish_odometry()

    # ─────────────────────────────────────────────────────────────
    # Odometría
    # ─────────────────────────────────────────────────────────────

    def _update_odometry(self):
        ticks_l = self.enc_l.steps
        ticks_r = self.enc_r.steps

        d_l = (ticks_l - self.prev_ticks_l_) * DIST_PER_TICK
        d_r = (ticks_r - self.prev_ticks_r_) * DIST_PER_TICK

        speed_r = d_r / self.timer_period_
        speed_l = d_l / self.timer_period_

        self.omega_r = speed_r / 2 * WHEEL_DIAM_M  # rad/s
        self.omega_l = speed_l / 2 * WHEEL_DIAM_M  # rad/s

        self.prev_ticks_l_ = ticks_l
        self.prev_ticks_r_ = ticks_r

        d_center = (d_l + d_r) / 2.0
        d_yaw    = (d_r - d_l) / WHEEL_BASE

        self.x_   += d_center * math.cos(self.yaw_ + d_yaw / 2.0)
        self.y_   += d_center * math.sin(self.yaw_ + d_yaw / 2.0)
        self.yaw_ += d_yaw

    def _publish_odometry(self):
        now_stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = now_stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.orientation.z = math.sin(self.yaw_ / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw_ / 2.0)
        self.odom_pub_.publish(odom)
        omega = Twist()
        omega.linear.x  = self.omega_r
        omega.angular.z = self.omega_l
        self.speed_pub_.publish(omega)

        t = TransformStamped()
        t.header.stamp            = now_stamp
        t.header.frame_id         = "odom"
        t.child_frame_id          = "base_link"
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.rotation.z    = math.sin(self.yaw_ / 2.0)
        t.transform.rotation.w    = math.cos(self.yaw_ / 2.0)
        self.tf_broadcaster_.sendTransform(t)

        self._push_to_dashboard(self.x_, self.y_, math.degrees(self.yaw_), (self.omega_r + self.omega_l)/2, now_stamp.sec + now_stamp.nanosec * 1e-9)   

    # ─────────────────────────────────────────────────────────────
    # Grabación
    # ─────────────────────────────────────────────────────────────
    def _push_to_dashboard(self, x, y, yaw_deg, vel, timestamp):
        payload = json.dumps({"x": x, "y": y, "yaw_deg": yaw_deg,
                              "vel": vel, "timestamp": timestamp}).encode()
        self._udp_sock.sendto(payload, ("127.0.0.1", 5001))

    def _record_sample(self, timestamp: float):
        self.record_data_.append({
            "timestamp": timestamp,
            "linear_x":  self.latest_cmd_.linear.x,
            "angular_z": self.latest_cmd_.angular.z,
            "enc_left":  self.enc_l.steps,
            "enc_right": self.enc_r.steps,
            "odom_x":    self.x_,
            "odom_y":    self.y_,
            "odom_yaw":  self.yaw_,
        })

    def _save_csv(self):
        filename   = f"motor_record_{int(time.time())}.csv"
        fieldnames = [
            "timestamp", "linear_x", "angular_z",
            "enc_left",  "enc_right",
            "odom_x",    "odom_y",   "odom_yaw"
        ]
        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.record_data_)
        print(f"  💾  CSV guardado: {filename}")

    # ─────────────────────────────────────────────────────────────
    # Reproducción
    # ─────────────────────────────────────────────────────────────

    def _playback_step(self, now: float):
        if self.play_index_ >= len(self.record_data_):
            print("  ✅  Reproducción completada.")
            self._stop_playback()
            return

        row       = self.record_data_[self.play_index_]
        t_origin  = self.record_data_[0]["timestamp"]
        t_sample  = row["timestamp"] - t_origin
        t_elapsed = now - self.play_start_t_

        if t_elapsed >= t_sample:
            cmd           = Twist()
            cmd.linear.x  = float(row["linear_x"])
            cmd.angular.z = float(row["angular_z"])
            self.cmd_pub_.publish(cmd)
            self.play_index_ += 1


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RecorderPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()