import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# ──────────────────────────────────────────────────────────────────────────────
# Parámetros ajustables
# ──────────────────────────────────────────────────────────────────────────────

MAX_OBJETOS = 5
AREA_MINIMA = 700
AREA_MAXIMA_RELATIVA = 0.90

EPSILON_APROX = 0.038

UMBRAL_CUADRADO_MIN = 0.70
UMBRAL_CUADRADO_MAX = 1.45

RECTANGULARIDAD_MIN = 0.54
SOLIDEZ_MIN = 0.68
ASPECTO_MAXIMO = 4.2
ANGULO_MIN = 52
ANGULO_MAX = 128

MAX_ANCHO_RELATIVO_OBJETO = 0.45
MAX_ALTO_RELATIVO_OBJETO = 0.70
RECHAZAR_CONTORNOS_EN_BORDE = True
MARGEN_BORDE = 4

FRACCION_COLOR_MIN = 0.04
PIXELS_COLOR_MIN = 35

DETECTAR_POR_COLOR_SEPARADO = True

FILTRAR_SOLO_CUADRADOS_RECTANGULOS = True

USAR_BORDES_COMO_APOYO = False

MOSTRAR_VIDEO_LOCAL = True

MOSTRAR_CONTORNO_REAL = True
MOSTRAR_CAJA_RECTANGULAR = True

PUBLICAR_VISUALIZACION = True
TOPICO_VISUALIZACION = '/shape_color_detector/debug_image'

VISUALIZACION_EN_TIEMPO_REAL = True
PERIODO_VISUALIZACION = 0.05
ESCALA_VISUALIZACION = 0.70


# ──────────────────────────────────────────────────────────────────────────────
# Preprocesamiento
# ──────────────────────────────────────────────────────────────────────────────

def normalizar_iluminacion(frame):
    suavizado = cv2.bilateralFilter(frame, d=7, sigmaColor=60, sigmaSpace=60)
    suavizado = cv2.GaussianBlur(suavizado, (5, 5), 0)

    hsv = cv2.cvtColor(suavizado, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v = clahe.apply(v)

    hsv_norm = cv2.merge([h, s, v])
    return cv2.cvtColor(hsv_norm, cv2.COLOR_HSV2BGR)


def limpiar_mascara(mascara):
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel_open, iterations=1)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel_close, iterations=2)
    mascara = cv2.dilate(mascara, kernel_open, iterations=1)
    return mascara


def obtener_mascaras_colores(frame_norm):
    hsv = cv2.cvtColor(frame_norm, cv2.COLOR_BGR2HSV)

    rojo_1 = cv2.inRange(hsv, np.array([0, 45, 35]), np.array([22, 255, 255]))
    rojo_2 = cv2.inRange(hsv, np.array([155, 45, 35]), np.array([180, 255, 255]))
    rojo = cv2.bitwise_or(rojo_1, rojo_2)

    verde = cv2.inRange(hsv, np.array([28, 38, 30]), np.array([100, 255, 255]))

    azul_hsv = cv2.inRange(hsv, np.array([88, 35, 25]), np.array([145, 255, 255]))

    h, s, _ = cv2.split(hsv)
    b, g, r = cv2.split(frame_norm)
    b_int = b.astype(np.int16)
    g_int = g.astype(np.int16)
    r_int = r.astype(np.int16)

    azul_dominante = np.zeros(frame_norm.shape[:2], dtype=np.uint8)
    condicion_azul = (
        (b_int > r_int + 25)
        & (b_int > g_int + 12)
        & (b_int > 45)
        & (s > 35)
    )
    azul_dominante[condicion_azul] = 255

    azul = cv2.bitwise_or(azul_hsv, azul_dominante)

    return {
        'Rojo': limpiar_mascara(rojo),
        'Verde': limpiar_mascara(verde),
        'Azul': limpiar_mascara(azul),
    }


def preprocesar_para_contornos(frame):
    frame_norm = normalizar_iluminacion(frame)
    mascaras = obtener_mascaras_colores(frame_norm)

    mascara_color = np.zeros(frame.shape[:2], dtype=np.uint8)
    for mascara in mascaras.values():
        mascara_color = cv2.bitwise_or(mascara_color, mascara)

    if USAR_BORDES_COMO_APOYO:
        gris = cv2.cvtColor(frame_norm, cv2.COLOR_BGR2GRAY)
        gris = cv2.GaussianBlur(gris, (5, 5), 0)

        mediana = np.median(gris)
        bajo = int(max(20, 0.66 * mediana))
        alto = int(min(180, 1.33 * mediana))
        bordes = cv2.Canny(gris, bajo, alto)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        zona_color = cv2.dilate(mascara_color, kernel, iterations=1)
        bordes = cv2.bitwise_and(bordes, bordes, mask=zona_color)

        mascara_objetos = cv2.bitwise_or(mascara_color, bordes)
    else:
        mascara_objetos = mascara_color

    mascara_objetos = limpiar_mascara(mascara_objetos)

    contornos, _ = cv2.findContours(mascara_objetos, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rellena = np.zeros_like(mascara_objetos)
    cv2.drawContours(rellena, contornos, -1, 255, -1)
    rellena = limpiar_mascara(rellena)

    return rellena, frame_norm, mascaras


# ──────────────────────────────────────────────────────────────────────────────
# Clasificación de forma y color
# ──────────────────────────────────────────────────────────────────────────────

def calcular_angulo(p0, p1, p2):
    v1 = p0 - p1
    v2 = p2 - p1

    norma_1 = np.linalg.norm(v1)
    norma_2 = np.linalg.norm(v2)

    if norma_1 == 0 or norma_2 == 0:
        return 0

    coseno = np.clip(np.dot(v1, v2) / (norma_1 * norma_2), -1.0, 1.0)
    return np.degrees(np.arccos(coseno))


def tiene_angulos_rectangulares(aproximacion):
    puntos = aproximacion.reshape(4, 2)
    for i in range(4):
        angulo = calcular_angulo(
            puntos[(i - 1) % 4],
            puntos[i],
            puntos[(i + 1) % 4],
        )
        if angulo < ANGULO_MIN or angulo > ANGULO_MAX:
            return False
    return True


def clasificar_figura(contorno):
    area = cv2.contourArea(contorno)
    perimetro = cv2.arcLength(contorno, True)

    if perimetro == 0 or area <= 0:
        return 'Otro'

    aproximacion = cv2.approxPolyDP(contorno, EPSILON_APROX * perimetro, True)
    vertices = len(aproximacion)

    if vertices < 4 or vertices > 8:
        return 'Otro'

    if vertices == 4 and not tiene_angulos_rectangulares(aproximacion):
        return 'Otro'

    rect_rotado = cv2.minAreaRect(contorno)
    ancho, alto = rect_rotado[1]

    if ancho <= 0 or alto <= 0:
        return 'Otro'

    lado_mayor = max(ancho, alto)
    lado_menor = min(ancho, alto)
    relacion = lado_mayor / lado_menor

    if relacion > ASPECTO_MAXIMO:
        return 'Otro'

    area_rect_rotado = ancho * alto
    rectangularidad = area / area_rect_rotado if area_rect_rotado > 0 else 0

    hull = cv2.convexHull(contorno)
    area_hull = cv2.contourArea(hull)
    solidez = area / area_hull if area_hull > 0 else 0

    if rectangularidad < RECTANGULARIDAD_MIN:
        return 'Otro'

    if solidez < SOLIDEZ_MIN:
        return 'Otro'

    if vertices > 4 and rectangularidad < 0.60:
        return 'Otro'

    if UMBRAL_CUADRADO_MIN <= relacion <= UMBRAL_CUADRADO_MAX:
        return 'Cuadrado'

    return 'Rectangulo'


def detectar_color(frame_norm, contorno, mascaras_colores):
    mascara_objeto = np.zeros(frame_norm.shape[:2], dtype=np.uint8)
    cv2.drawContours(mascara_objeto, [contorno], -1, 255, -1)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mascara_interior = cv2.erode(mascara_objeto, kernel, iterations=1)

    area_interior = cv2.countNonZero(mascara_interior)
    if area_interior == 0:
        mascara_interior = mascara_objeto
        area_interior = cv2.countNonZero(mascara_interior)

    puntajes = {}
    for nombre, mascara_color in mascaras_colores.items():
        coincidencia = cv2.bitwise_and(mascara_color, mascara_color, mask=mascara_interior)
        pixeles = cv2.countNonZero(coincidencia)
        fraccion = pixeles / float(area_interior) if area_interior > 0 else 0
        puntajes[nombre] = (pixeles, fraccion)

    color_detectado = max(puntajes, key=lambda c: puntajes[c][1])
    pixeles, fraccion = puntajes[color_detectado]

    if fraccion >= FRACCION_COLOR_MIN or pixeles >= PIXELS_COLOR_MIN:
        return color_detectado

    return 'Desconocido'


COLOR_BGR = {
    'Rojo': (0, 0, 220),
    'Verde': (0, 180, 0),
    'Azul': (220, 0, 0),
    'Desconocido': (128, 128, 128),
}


# ──────────────────────────────────────────────────────────────────────────────
# Análisis principal
# ──────────────────────────────────────────────────────────────────────────────

def analizar_frame(frame):
    mascara_objetos, frame_norm, mascaras_colores = preprocesar_para_contornos(frame)

    alto_frame, ancho_frame = frame.shape[:2]
    area_frame = alto_frame * ancho_frame

    candidatos = []

    if DETECTAR_POR_COLOR_SEPARADO:
        fuentes_contornos = []
        for nombre_color, mascara_color in mascaras_colores.items():
            contornos_color, _ = cv2.findContours(
                mascara_color,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            for c in contornos_color:
                fuentes_contornos.append((c, nombre_color))
    else:
        contornos, _ = cv2.findContours(
            mascara_objetos,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        fuentes_contornos = [(c, None) for c in contornos]

    for c, color_preasignado in fuentes_contornos:
        area = cv2.contourArea(c)

        if area < AREA_MINIMA or area > AREA_MAXIMA_RELATIVA * area_frame:
            continue

        x, y, w, h = cv2.boundingRect(c)
        if w <= 10 or h <= 10:
            continue

        if w > MAX_ANCHO_RELATIVO_OBJETO * ancho_frame:
            continue

        if h > MAX_ALTO_RELATIVO_OBJETO * alto_frame:
            continue

        if RECHAZAR_CONTORNOS_EN_BORDE:
            toca_borde = (
                x <= MARGEN_BORDE
                or y <= MARGEN_BORDE
                or x + w >= ancho_frame - MARGEN_BORDE
                or y + h >= alto_frame - MARGEN_BORDE
            )
            if toca_borde:
                continue

        figura = clasificar_figura(c)

        if FILTRAR_SOLO_CUADRADOS_RECTANGULOS and figura not in ('Cuadrado', 'Rectangulo'):
            continue

        if color_preasignado is None:
            color = detectar_color(frame_norm, c, mascaras_colores)
        else:
            color = color_preasignado

        if color == 'Desconocido':
            continue

        candidatos.append({'color': color, 'figura': figura, 'contorno': c, 'area': area})

    candidatos = sorted(candidatos, key=lambda r: r['area'], reverse=True)[:MAX_OBJETOS]

    return [{'color': r['color'], 'figura': r['figura'], 'contorno': r['contorno']}
            for r in candidatos], mascara_objetos


# ──────────────────────────────────────────────────────────────────────────────
# Visualización
# ──────────────────────────────────────────────────────────────────────────────

def dibujar_detecciones(frame, resultados):
    visualizacion = frame.copy()

    for r in resultados:
        contorno = r['contorno']
        etiqueta = f"{r['color']} - {r['figura']}"
        color_dibujo = COLOR_BGR.get(r['color'], (128, 128, 128))

        rect_rotado = cv2.minAreaRect(contorno)
        caja = np.intp(cv2.boxPoints(rect_rotado))

        if MOSTRAR_CONTORNO_REAL:
            cv2.drawContours(visualizacion, [contorno], -1, color_dibujo, 2)

        if MOSTRAR_CAJA_RECTANGULAR:
            cv2.drawContours(visualizacion, [caja], -1, color_dibujo, 2)

        x, y, w, h = cv2.boundingRect(caja)
        cv2.rectangle(visualizacion, (x, y), (x + w, y + h), color_dibujo, 2)

        (tw, th), _ = cv2.getTextSize(etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        y_texto = max(y - 6, th + 10)

        cv2.rectangle(
            visualizacion,
            (x, y_texto - th - 8),
            (x + tw + 8, y_texto + 4),
            (0, 0, 0),
            -1,
        )
        cv2.putText(
            visualizacion,
            etiqueta,
            (x + 4, y_texto),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color_dibujo,
            2,
        )

    return visualizacion


# ──────────────────────────────────────────────────────────────────────────────
# Nodo ROS2
# ──────────────────────────────────────────────────────────────────────────────

class ShapeColorDetectorNode(Node):
    def __init__(self):
        super().__init__('shape_color_detector')

        self.bridge = CvBridge()
        self.frame_cam0 = None
        self.ultimos_resultados = []
        self.contador = 0
        self.imprimir_cada = 10

        self.sub0 = self.create_subscription(
            Image,
            '/camera/image_raw/img0',
            self.cb_cam0,
            10,
        )

        self.pub_cubos = self.create_publisher(String, '/cubosCol', 10)
        self.pub_visualizacion = self.create_publisher(Image, TOPICO_VISUALIZACION, 10)

        self.timer = self.create_timer(3.0, self.detectar)

        self.timer_visualizacion = None
        if PUBLICAR_VISUALIZACION and VISUALIZACION_EN_TIEMPO_REAL:
            self.timer_visualizacion = self.create_timer(
                PERIODO_VISUALIZACION,
                self.publicar_visualizacion,
            )

        self.get_logger().info('shape_color_detector iniciado.')

    def cb_cam0(self, msg):
        try:
            self.frame_cam0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo frame cam0: {e}')

    def publicar_visualizacion(self):
        if self.frame_cam0 is None:
            return

        visualizacion = dibujar_detecciones(self.frame_cam0, self.ultimos_resultados)

        if ESCALA_VISUALIZACION != 1.0:
            visualizacion = cv2.resize(
                visualizacion,
                None,
                fx=ESCALA_VISUALIZACION,
                fy=ESCALA_VISUALIZACION,
                interpolation=cv2.INTER_AREA,
            )

        if MOSTRAR_VIDEO_LOCAL:
            cv2.imshow('Deteccion - cam0', visualizacion)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Cerrando ventana de visualización.')
                cv2.destroyAllWindows()

        if PUBLICAR_VISUALIZACION:
            msg_img = self.bridge.cv2_to_imgmsg(visualizacion, encoding='bgr8')
            msg_img.header.stamp = self.get_clock().now().to_msg()
            msg_img.header.frame_id = 'cam0'
            self.pub_visualizacion.publish(msg_img)

    def detectar(self):
        self.contador += 1
        imprimir_ahora = self.contador % self.imprimir_cada == 0

        if self.frame_cam0 is None:
            if imprimir_ahora:
                self.get_logger().info('Esperando frame de cam0...')
            return

        resultados, _ = analizar_frame(self.frame_cam0)
        self.ultimos_resultados = resultados

        if not VISUALIZACION_EN_TIEMPO_REAL:
            self.publicar_visualizacion()

        detecciones = [
            {'fuente': 'cam0', 'color': r['color'], 'figura': r['figura']}
            for r in resultados
        ]

        if detecciones:
            self.pub_cubos.publish(String(data=json.dumps(detecciones, ensure_ascii=False)))
            if imprimir_ahora:
                for d in detecciones:
                    self.get_logger().info(f"[cam0] Detectado: {d['color']} - {d['figura']}")
        else:
            if imprimir_ahora:
                self.get_logger().info('Sin detección en este ciclo.')


def main(args=None):
    rclpy.init(args=args)
    node = ShapeColorDetectorNode()

    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
