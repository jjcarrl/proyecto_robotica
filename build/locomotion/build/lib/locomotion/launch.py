# robot_manager.py
import subprocess
import threading
import os
import signal
import rclpy
from rclpy.node import Node


# ── Comandos de cada nodo ─────────────────────────────────────────
NODES = {
    "motors":   ["ros2", "run", "locomotion", "motor_com"],
    "rc":       ["ros2", "run", "locomotion", "rc_control"],
    "DualSense": ["ros2", "run", "joy", "joy_node"],
}


class RobotManager(Node):
    def __init__(self):
        super().__init__("robot_manager")
        self.processes_ = {}   # nombre → subprocess.Popen

        # Hilo de teclado
        self.keyboard_thread_ = threading.Thread(
            target=self._keyboard_loop, daemon=True
        )
        self.keyboard_thread_.start()
        self._print_help()

    # ─────────────────────────────────────────────────────────────
    # Teclado
    # ─────────────────────────────────────────────────────────────

    def _keyboard_loop(self):
        while rclpy.ok():
            try:
                raw = input()
                cmd = raw.strip().lower()

                if cmd == "":
                    continue

                # ── Comandos compuestos primero ───────────────────────
                if cmd == "start all":
                    for name in NODES:
                        self._start_node(name)

                elif cmd == "stop all":
                    for name in list(self.processes_.keys()):
                        self._stop_node(name)

                # ── Comandos simples después ──────────────────────────
                else:
                    parts = cmd.split()

                    if parts[0] == "start" and len(parts) == 2:
                        self._start_node(parts[1])

                    elif parts[0] == "stop" and len(parts) == 2:
                        self._stop_node(parts[1])

                    elif parts[0] == "restart" and len(parts) == 2:
                        self._stop_node(parts[1])
                        self._start_node(parts[1])

                    elif cmd == "status":
                        self._print_status()

                    elif cmd in ("help", "h"):
                        self._print_help()

                    elif cmd in ("exit", "quit"):
                        self._stop_all_and_exit()

                    else:
                        print(f"  ⚠️  Comando desconocido: '{cmd}'")
                        print(f"       Escribe 'help' para ver los comandos.")

            except EOFError:
                break

    # ─────────────────────────────────────────────────────────────
    # Control de nodos
    # ─────────────────────────────────────────────────────────────

    def _start_node(self, name: str):
        if name not in NODES:
            print(f"  ⚠️  Nodo desconocido: '{name}'. Disponibles: {', '.join(NODES.keys())}")
            return

        if name in self.processes_ and self.processes_[name].poll() is None:
            print(f"  ⚠️  '{name}' ya está corriendo.")
            return

        env = os.environ.copy()
        proc = subprocess.Popen(
            NODES[name],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,   # ← crea grupo de procesos propio
        )
        self.processes_[name] = proc
        print(f"  ▶  '{name}' iniciado (PID {proc.pid})")

        # Hilo para imprimir la salida del nodo
        t = threading.Thread(
            target=self._stream_output, args=(name, proc), daemon=True
        )
        t.start()

    def _stop_node(self, name: str):
        if name not in self.processes_:
            print(f"  ⚠️  '{name}' no está corriendo.")
            return

        proc = self.processes_[name]
        if proc.poll() is not None:
            print(f"  ⚠️  '{name}' ya estaba detenido.")
            del self.processes_[name]
            return

        # Mata todo el grupo de procesos (padre + hijos)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

        proc.wait()
        del self.processes_[name]
        print(f"  ⏹  '{name}' detenido.")

    def _stop_all_and_exit(self):
        print("  Cerrando todos los nodos...")
        for name in list(self.processes_.keys()):
            self._stop_node(name)
        print("  👋  Hasta luego.")
        raise SystemExit

    # ─────────────────────────────────────────────────────────────
    # Output de subprocesos
    # ─────────────────────────────────────────────────────────────

    def _stream_output(self, name: str, proc: subprocess.Popen):
        prefix = f"  [{name}]"
        for line in proc.stdout:
            print(f"{prefix} {line}", end="")
        # Cuando termina el proceso
        ret = proc.poll()
        if ret is not None and ret != 0:
            print(f"  ❌  '{name}' terminó con error (código {ret})")
        elif ret == 0:
            print(f"  ✅  '{name}' terminó correctamente.")

    # ─────────────────────────────────────────────────────────────
    # Status y ayuda
    # ─────────────────────────────────────────────────────────────

    def _print_status(self):
        print(f"\n{'─'*45}")
        print(f"  {'NODO':<12} {'ESTADO':<12} {'PID'}")
        print(f"{'─'*45}")
        for name in NODES:
            if name in self.processes_ and self.processes_[name].poll() is None:
                pid    = self.processes_[name].pid
                estado = "🟢  corriendo"
            else:
                pid    = "─"
                estado = "🔴  detenido"
            print(f"  {name:<12} {estado:<20} {pid}")
        print(f"{'─'*45}\n")

    def _print_help(self):
        print(f"\n{'─'*45}")
        print(f"  🤖  Robot Manager")
        print(f"{'─'*45}")
        print(f"  start <nodo>   →  iniciar nodo")
        print(f"  stop <nodo>    →  detener nodo")
        print(f"  restart <nodo> →  reiniciar nodo")
        print(f"  start all      →  iniciar todos")
        print(f"  stop all       →  detener todos")
        print(f"  status         →  ver estado de nodos")
        print(f"  exit           →  cerrar todo y salir")
        print(f"{'─'*45}")
        print(f"  Nodos: {', '.join(NODES.keys())}")
        print(f"{'─'*45}\n")


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()