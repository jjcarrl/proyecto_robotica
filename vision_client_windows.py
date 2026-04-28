"""
Cliente de vision para PC Windows.
Recibe frames JPEG del robot via WebSocket, corre reconocimiento de
color y forma, y devuelve los resultados al robot.

Instalar dependencias (Windows):
    pip install websockets opencv-python numpy

Uso:
    python vision_client_windows.py --ip 192.168.x.x
"""

import argparse
import asyncio
import json

import cv2
import numpy as np
import websockets


# ─────────────────────────────────────────────── reconocimiento (editar aqui)

def detect_squares(frame: np.ndarray) -> tuple[list[dict], np.ndarray]:
    """
    Detecta cuadrados de cualquier color en el frame.
    Retorna (lista de detecciones, imagen B&N con contornos detectados).
    Factor de forma: aspect ratio entre 0.8 y 1.25 (80 % de tolerancia).
    """
    results = []
    contour_canvas = np.zeros(frame.shape[:2], dtype=np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- rangos de color (ajustar segun iluminacion del ambiente) ---
    color_ranges = {
        "red":    [(np.array([0,   120,  70]), np.array([10,  255, 255])),
                   (np.array([170, 120,  70]), np.array([180, 255, 255]))],
        "green":  [(np.array([36,   50,  70]), np.array([89,  255, 255]))],
        "blue":   [(np.array([90,   50,  70]), np.array([128, 255, 255]))],
        "yellow": [(np.array([20,  100, 100]), np.array([35,  255, 255]))],
    }

    for color_name, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < 1000:
                continue
            if not _is_square(cnt):
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            results.append({
                "color": color_name,
                "shape": "square",
                "area":  int(cv2.contourArea(cnt)),
                "cx":    cx,
                "cy":    cy,
            })

            # frame a color: contorno verde + etiqueta
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            cv2.putText(frame, f"{color_name} square", (cx - 50, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # canvas B&N: contorno blanco
            cv2.drawContours(contour_canvas, [cnt], -1, 255, 2)

    return results, contour_canvas


def _is_square(contour) -> bool:
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    if len(approx) != 4:
        return False
    x, y, w, h = cv2.boundingRect(approx)
    aspect = w / float(h)
    return 0.8 <= aspect <= 1.25


# ────────────────────────────────────────────────────────── loop WebSocket

async def run(ip: str, port: int):
    uri = f"ws://{ip}:{port}"
    print(f"Conectando a {uri} ...")

    async with websockets.connect(uri, max_size=10 * 1024 * 1024) as ws:
        print("Conectado. Presiona 'q' en la ventana para salir.\n")
        async for message in ws:
            buf = np.frombuffer(message, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            detections, contour_canvas = detect_squares(frame)

            cv2.imshow("Robot Vision", frame)
            cv2.imshow("Contornos B&N", contour_canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if detections:
                payload = json.dumps({"detections": detections})
                await ws.send(payload)
                print(f"Enviado: {detections}")

    cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────── main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Cliente de vision del robot")
    parser.add_argument("--ip",   default="192.168.1.100", help="IP de la Raspberry Pi")
    parser.add_argument("--port", default=8765, type=int,  help="Puerto WebSocket")
    args = parser.parse_args()

    asyncio.run(run(args.ip, args.port))
