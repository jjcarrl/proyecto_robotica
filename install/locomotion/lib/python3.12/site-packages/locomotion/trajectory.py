#!/usr/bin/env python3
"""
Nodo ROS2 que grafica la trayectoria del robot en tiempo real
suscribiéndose a /odom.
"""

import os
os.environ["DISPLAY"] = os.environ.get("DISPLAY", "localhost:0.0")

import math
import time
import threading

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_plotter")

        self.xs_   = [0.0]
        self.ys_   = [0.0]
        self.yaws_ = [0.0]
        self.lock_ = threading.Lock()

        self.odom_sub_ = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.mode_sub_ = self.create_subscription(
            String, "/motor_mode", self.mode_callback, 10
        )

        self.get_logger().info(
            "TrajectoryPlotter listo.\n"
            "  Escuchando /odom ...\n"
            "  Ctrl+C o 'stop_record'/'stop_play' en /motor_mode para guardar imagen."
        )

    # ─────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)

        with self.lock_:
            if len(self.xs_) == 0 or \
               abs(x - self.xs_[-1]) > 0.001 or \
               abs(y - self.ys_[-1]) > 0.001:
                self.xs_.append(x)
                self.ys_.append(y)
                self.yaws_.append(yaw)

    def mode_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ("stop_record", "stop_play"):
            self._save_plot()

    # ─────────────────────────────────────────────────────────────
    # Guardar imagen
    # ─────────────────────────────────────────────────────────────

    def _save_plot(self):
        with self.lock_:
            xs   = list(self.xs_)
            ys   = list(self.ys_)
            yaws = list(self.yaws_)

        if len(xs) < 2:
            self.get_logger().warn("No hay suficientes datos para graficar.")
            return

        fig, ax = plt.subplots(figsize=(7, 7))
        fig.patch.set_facecolor("#1e1e2e")
        ax.set_facecolor("#1e1e2e")
        ax.set_title("Trayectoria del robot", color="white", fontsize=14, pad=12)
        ax.set_xlabel("X (m)", color="#cdd6f4")
        ax.set_ylabel("Y (m)", color="#cdd6f4")
        ax.tick_params(colors="#cdd6f4")
        for spine in ax.spines.values():
            spine.set_edgecolor("#45475a")
        ax.grid(True, color="#313244", linestyle="--", linewidth=0.5)
        ax.set_aspect("equal")

        ax.plot(xs, ys, color="#89b4fa", linewidth=1.8, zorder=2)
        ax.plot(xs[0],  ys[0],  "o", color="#a6e3a1", markersize=9, zorder=3)
        ax.plot(xs[-1], ys[-1], "o", color="#f38ba8", markersize=9, zorder=4)

        arrow_len = max(0.05, (max(xs) - min(xs) + max(ys) - min(ys)) * 0.03)
        dx = arrow_len * math.cos(yaws[-1])
        dy = arrow_len * math.sin(yaws[-1])
        ax.annotate(
            "", xy=(xs[-1] + dx, ys[-1] + dy),
            xytext=(xs[-1], ys[-1]),
            arrowprops=dict(arrowstyle="->", color="#fab387", lw=2),
            zorder=5
        )

        legend = [
            mpatches.Patch(color="#a6e3a1", label="Inicio"),
            mpatches.Patch(color="#f38ba8", label="Fin"),
            mpatches.Patch(color="#89b4fa", label="Trayectoria"),
        ]
        ax.legend(handles=legend, facecolor="#313244", labelcolor="#cdd6f4",
                  loc="upper left", fontsize=9)

        margin = max(0.1, arrow_len * 2)
        x_min, x_max = min(xs) - margin, max(xs) + margin
        y_min, y_max = min(ys) - margin, max(ys) + margin
        rng = max(x_max - x_min, y_max - y_min) / 2.0
        cx  = (x_min + x_max) / 2.0
        cy  = (y_min + y_max) / 2.0
        ax.set_xlim(cx - rng, cx + rng)
        ax.set_ylim(cy - rng, cy + rng)

        filename = f"trayectoria_{int(time.time())}.png"
        plt.tight_layout()
        fig.savefig(filename, dpi=150)
        plt.close(fig)

        self.get_logger().info(f"📸 Imagen guardada: {filename}")

        with self.lock_:
            self.xs_   = [0.0]
            self.ys_   = [0.0]
            self.yaws_ = [0.0]

    def get_data(self):
        with self.lock_:
            return list(self.xs_), list(self.ys_), list(self.yaws_)


# ─────────────────────────────────────────────────────────────────
# Figura en tiempo real
# ─────────────────────────────────────────────────────────────────

def start_plot(node: TrajectoryPlotter):
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.patch.set_facecolor("#1e1e2e")
    ax.set_facecolor("#1e1e2e")
    ax.set_title("Trayectoria del robot — Tiempo real", color="white", fontsize=14, pad=12)
    ax.set_xlabel("X (m)", color="#cdd6f4")
    ax.set_ylabel("Y (m)", color="#cdd6f4")
    ax.tick_params(colors="#cdd6f4")
    for spine in ax.spines.values():
        spine.set_edgecolor("#45475a")
    ax.grid(True, color="#313244", linestyle="--", linewidth=0.5)
    ax.set_aspect("equal")

    line,      = ax.plot([], [], color="#89b4fa", linewidth=1.8, zorder=2)
    start_dot, = ax.plot([], [], "o", color="#a6e3a1", markersize=8, zorder=3)
    robot_dot, = ax.plot([], [], "o", color="#f38ba8", markersize=10, zorder=4)
    arrow_container = [None]

    legend = [
        mpatches.Patch(color="#a6e3a1", label="Inicio"),
        mpatches.Patch(color="#f38ba8", label="Posición actual"),
        mpatches.Patch(color="#89b4fa", label="Trayectoria"),
    ]
    ax.legend(handles=legend, facecolor="#313244", labelcolor="#cdd6f4",
              loc="upper left", fontsize=9)

    def update(_frame):
        xs, ys, yaws = node.get_data()

        if len(xs) < 1:
            return line, start_dot, robot_dot

        line.set_data(xs, ys)
        start_dot.set_data([xs[0]], [ys[0]])
        robot_dot.set_data([xs[-1]], [ys[-1]])

        if arrow_container[0] is not None:
            arrow_container[0].remove()

        arrow_len = max(0.05, (max(xs) - min(xs) + max(ys) - min(ys)) * 0.03)
        dx = arrow_len * math.cos(yaws[-1])
        dy = arrow_len * math.sin(yaws[-1])
        arrow_container[0] = ax.annotate(
            "", xy=(xs[-1] + dx, ys[-1] + dy),
            xytext=(xs[-1], ys[-1]),
            arrowprops=dict(arrowstyle="->", color="#fab387", lw=2),
            zorder=5
        )

        margin = max(0.3, arrow_len * 2)
        x_min, x_max = min(xs) - margin, max(xs) + margin
        y_min, y_max = min(ys) - margin, max(ys) + margin
        rng = max(x_max - x_min, y_max - y_min) / 2.0
        cx  = (x_min + x_max) / 2.0
        cy  = (y_min + y_max) / 2.0
        ax.set_xlim(cx - rng, cx + rng)
        ax.set_ylim(cy - rng, cy + rng)

        return line, start_dot, robot_dot

    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    return ani


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()

    # ROS corre en hilo separado para no bloquear matplotlib
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        # matplotlib debe correr en el hilo principal
        ani = start_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save_plot()
        node.destroy_node()


if __name__ == "__main__":
    main()