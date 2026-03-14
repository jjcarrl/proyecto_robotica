#!/usr/bin/env python3

import os
import csv
import time
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gpiozero import RotaryEncoder


# =========================
# Pines GPIO encoders (BCM) — cámbialos por los tuyos
# =========================
PIN_ENC_L_A = 20
PIN_ENC_L_B = 21
PIN_ENC_R_A = 23
PIN_ENC_R_B = 24

# Pulsos por revolución de tu JGA25-370 (ajusta según reducción)
ENCODER_PPR = 341.2


class RecorderPlayer(Node):
    def __init__(self):
        super().__init__("recorder_player")

        # ── Encoders ─────────────────────────────────────────────
        self.enc_l = RotaryEncoder(PIN_ENC_L_A, PIN_ENC_L_B, max_steps=0)
        self.enc_r = RotaryEncoder(PIN_ENC_R_A, PIN_ENC_R_B, max_steps=0)

        # ── Estado interno ────────────────────────────────────────
        self.latest_cmd_   = Twist()
        self.mode_         = "idle"   # "idle" | "recording" | "playing"
        self.record_data_  = []
        self.play_index_   = 0
        self.play_start_t_ = 0.0

        # ── Suscripciones ─────────────────────────────────────────
        self.cmd_sub_ = self.create_subscription(
            Twist, "/motor_vel", self.cmd_callback, 10
        )

        # Modo:
        #   "record"      → empieza a grabar
        #   "stop_record" → detiene y guarda CSV
        #   "play"        → reproduce lo grabado publicando en /motor_vel
        #   "stop_play"   → detiene reproducción
        #   "idle"        → no hace nada
        self.mode_sub_ = self.create_subscription(
            String, "/motor_mode", self.mode_callback, 10
        )

        # ── Publicador (solo activo durante playback) ─────────────
        self.cmd_pub_ = self.create_publisher(Twist, "/motor_vel", 10)

        # ── Timer principal 50 Hz ─────────────────────────────────
        self.timer_ = self.create_timer(0.02, self.main_loop)

        self.get_logger().info(
            "RecorderPlayer listo.\n"
            "  Modos disponibles en /motor_mode:\n"
            "    record | stop_record | play | stop_play | idle"
        )

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
        else:
            self.get_logger().warn(f"Modo desconocido: '{cmd}'")

    # ─────────────────────────────────────────────────────────────
    # Control de modos
    # ─────────────────────────────────────────────────────────────

    def _start_recording(self):
        self.record_data_ = []
        self.enc_l.steps  = 0
        self.enc_r.steps  = 0
        self.mode_        = "recording"
        self.get_logger().info("▶ Grabación iniciada.")

    def _stop_recording(self):
        if self.mode_ != "recording":
            self.get_logger().warn("No estaba grabando.")
            return

        self.mode_ = "idle"
        self._save_csv()
        self.get_logger().info(
            f"⏹ Grabación detenida. {len(self.record_data_)} muestras guardadas."
        )

    def _start_playback(self):
        if not self.record_data_:
            self.get_logger().warn("No hay datos grabados para reproducir.")
            return

        self.play_index_   = 0
        self.play_start_t_ = self.get_clock().now().nanoseconds * 1e-9
        self.mode_         = "playing"
        self.get_logger().info(
            f"▶ Reproducción iniciada ({len(self.record_data_)} muestras)."
        )

    def _stop_playback(self):
        self.mode_ = "idle"
        # Publica un Twist vacío para detener el robot
        self.cmd_pub_.publish(Twist())
        self.get_logger().info("⏹ Reproducción detenida.")

    # ─────────────────────────────────────────────────────────────
    # Loop principal
    # ─────────────────────────────────────────────────────────────

    def main_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.mode_ == "recording":
            self._record_sample(now)

        elif self.mode_ == "playing":
            self._playback_step(now)

    # ─────────────────────────────────────────────────────────────
    # Grabación
    # ─────────────────────────────────────────────────────────────

    def _record_sample(self, timestamp: float):
        self.record_data_.append({
            "timestamp": timestamp,
            "linear_x":  self.latest_cmd_.linear.x,
            "angular_z": self.latest_cmd_.angular.z,
            "enc_left":  self.enc_l.steps,
            "enc_right": self.enc_r.steps,
        })

    def _save_csv(self):
        filename   = f"motor_record_{int(time.time())}.csv"
        fieldnames = ["timestamp", "linear_x", "angular_z", "enc_left", "enc_right"]

        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.record_data_)

        self.get_logger().info(f"CSV guardado: {filename}")

    # ─────────────────────────────────────────────────────────────
    # Reproducción
    # ─────────────────────────────────────────────────────────────

    def _playback_step(self, now: float):
        if self.play_index_ >= len(self.record_data_):
            self.get_logger().info("✅ Reproducción completada.")
            self._stop_playback()
            return

        row      = self.record_data_[self.play_index_]
        t_origin = self.record_data_[0]["timestamp"]
        t_sample = row["timestamp"] - t_origin          # tiempo relativo del sample
        t_elapsed= now - self.play_start_t_             # tiempo transcurrido en reproducción

        if t_elapsed >= t_sample:
            cmd           = Twist()
            cmd.linear.x  = float(row["linear_x"])
            cmd.angular_z = float(row["angular_z"])
            self.cmd_pub_.publish(cmd)
            self.play_index_ += 1


# ─────────────────────────────────────────────────────────────────
# Main
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()