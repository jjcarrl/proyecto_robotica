# Proyecto Final Robótica — IELE3338L

Robot autónomo diferencial para **mapeo, navegación y manipulación de objetos** en pista logística tipo Sokoban.  
Curso IELE3338L — Universidad de los Andes, 2026-10.

---

## Descripción general

El robot es un móvil diferencial que integra LiDAR, IMU, encoders, cámara y un brazo manipulador. Opera de forma completamente autónoma: construye un mapa de la pista, se localiza, planea rutas y manipula bloques de colores para ubicarlos en zonas objetivo.

La arquitectura de software usa **ROS2** y combina:
- **SLAM Toolbox** para mapeo simultáneo con LiDAR
- **Nav2** para planificación de rutas y control de navegación
- **robot_localization (EKF)** para fusión de odometría + IMU
- Visión artificial procesada en PC externo para detección cenital de bloques y localización

---

## Estructura del repositorio

```
src/
├── proyecto_final_grupo5/ # Paquete de entrega oficial — launch unificado del sistema
├── sensors/               # Percepción: IMU, LiDAR, cámara, encoders, brazo, stepper
├── locomotion/            # Control de motores y dashboard de trayectoria
├── my_robot_description/  # URDF del robot y configuración EKF
├── nav2_config/           # Parámetros Nav2 + SLAM Toolbox
├── m-explore-ros2/        # Exploración por fronteras (librería de terceros)
└── tests/                 # Identificación de sistema
```

---

## Paquetes ROS2

### `sensors`

| Nodo | Ejecutable | Descripción |
|------|-----------|-------------|
| `CameraNode` | `camera_node` | Pi Camera 2 → `/camera/image_raw` (50 Hz) |
| `YDLidarNode` | `lidar_node` | YDLidar X4 Pro serial → `/scan` |
| `MPUNode` | `mpu_node` | BNO055 I2C (modo NDOF) → `/imu` (50 Hz) |
| `RecorderPlayer` | `encoders_node` | Encoders + odometría → `/odom`, `/omega` (50 Hz) |
| `VisionBridgeNode` | `vision_bridge_node` | Stream JPEG via WebSocket (Pi→PC) + recibe detecciones JSON → `/vision/detections` |
| `ArmBridgeNode` | `arm_bridge_node` | Puente serial ESP32 para control del brazo (modos: live/record/replay/stop) |
| `StepperNode` | `stepper_node` | Puente serial Arduino para eje Z → `/z_axis/position` |

### `locomotion`

| Nodo | Ejecutable | Descripción |
|------|-----------|-------------|
| `MotorControlNode` | `motor_control` | Control cinemático con PID de velocidad de rueda → GPIO PWM |
| `MotorCommand` | `motor_com` | Convierte `/motor_cmd` (Twist) directamente a GPIO PWM |
| `RC Control` | `rc_control` | Control remoto manual |
| Dashboard Flask | `graphical_odom` | Trayectoria en tiempo real en el navegador (UDP→Flask→Plotly) |

### `my_robot_description`

URDF del robot diferencial y configuración EKF (`robot_localization`) que fusiona `/odom` + `/imu` → `/odometry/filtered`.

### `nav2_config`

Pila completa de navegación: SLAM Toolbox (online-async) + Nav2 (BT navigator, planner, controller). Launch principal: `nav2_bringup.launch.py`.

---

## Tópicos principales

| Tópico | Tipo | Descripción |
|--------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Odometría de encoders |
| `/imu` | `sensor_msgs/Imu` | IMU BNO055 (quat + gyro + accel lineal) |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF fusionado (odom + imu) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 360° |
| `/camera/image_raw` | `sensor_msgs/Image` | Cámara Pi (robot) |
| `/vision/detections` | `std_msgs/String` (JSON) | Detecciones desde PC externo |
| `/map` | `nav_msgs/OccupancyGrid` | Mapa SLAM Toolbox |
| `/cmd_vel` | `geometry_msgs/Twist` | Comando de velocidad Nav2 → motores |
| `/motor_cmd` | `geometry_msgs/Twist` | Comando directo a `motor_com` |
| `/motor_vel` | `geometry_msgs/Twist` | Comando a encoders node |
| `/ref` | `nav_msgs/Odometry` | Referencia de posición para `motor_control` |
| `/omega` | `geometry_msgs/Twist` | Velocidades angulares de ruedas (feedback PID) |
| `/arm/mode` | `std_msgs/String` | Modo del brazo (`live`/`record`/`replay`/`stop`) |
| `/arm/status` | `std_msgs/String` (JSON) | Estado del brazo |
| `/z_axis/move` | `std_msgs/Float32` | Movimiento relativo eje Z (mm) |
| `/z_axis/position` | `std_msgs/Float32` | Posición actual eje Z (mm) |

---

## Hardware

| Componente | Interfaz | Notas |
|-----------|----------|-------|
| Raspberry Pi 4B | — | Computador principal |
| YDLidar X4 Pro | Serial `/dev/lidar` (128 000 baud) | Rango 0.12–10 m |
| BNO055 IMU | I2C bus 1, addr `0x28`/`0x29` | Modo NDOF (fusión completa) |
| Encoders (×2) | GPIO 20, 21 (izq.) / 23, 24 (der.) | 562 PPR |
| Motores DC | GPIO 5, 6 (izq.) / 17, 27 (der.) | Driver L298N, PWM |
| Pi Camera 2 | CSI | 820×640, 50 Hz |
| ESP32 (brazo) | Serial `/dev/ttyUSB0` (115 200 baud) | 4 servos, formato `d23,d15,d13,d25\n` |
| Arduino Uno (stepper) | Serial `/dev/arduino` (115 200 baud) | AccelStepper, eje Z |
| PC externo | WebSocket `:8765` (visión) / `:8766` (brazo) | Procesa visión cenital |

---

## Instalación y compilación

```bash
# Clonar el repositorio
git clone <url> ~/proyecto_robotica
cd ~/proyecto_robotica

# Instalar dependencias de ROS2
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --symlink-install

# Sourcear
source install/setup.bash
```

---

## Arranque del sistema

### Sistema completo — entrega oficial (paquete `proyecto_final_grupo5`)
```bash
ros2 launch proyecto_final_grupo5 bringup.launch.py
# Cambiar puerto LiDAR si es necesario:
ros2 launch proyecto_final_grupo5 bringup.launch.py lidar_port:=/dev/ttyUSB1
```

### Pila de navegación (SLAM + Nav2 + sensores + motores) — alternativa
```bash
ros2 launch nav2_config nav2_bringup.launch.py
```

### Solo sensores
```bash
ros2 launch sensors sensors.py
```

### Control de posición con PID (referencia por tópico `/ref`)
```bash
ros2 run locomotion motor_control
```

### Dashboard de trayectoria
```bash
ros2 run locomotion graphical_odom
# Abrir en navegador: http://<ip-raspberry>:5000
```

### Puente de visión (streaming Pi → PC)
```bash
ros2 run sensors vision_bridge_node
# En PC Windows: conectar ws://<ip-raspberry>:8765
```

### Control del brazo
```bash
ros2 run sensors arm_bridge_node
# Cambiar modo:
ros2 topic pub --once /arm/mode std_msgs/msg/String "data: 'live'"
```

### Eje Z (stepper)
```bash
ros2 run sensors stepper_node
# Mover +10 mm:
ros2 topic pub --once /z_axis/move std_msgs/msg/Float32 "data: 10.0"
```

---

## Parámetros de configuración relevantes

| Archivo | Parámetro | Valor | Descripción |
|---------|-----------|-------|-------------|
| `ekf.yaml` | `frequency` | 50 Hz | Frecuencia del filtro EKF |
| `motor_control.py` | `Ks` / `Kth` | 0.3 / 2.4 | Ganancias de control cinemático |
| `motor_control.py` | `P, I, D` | 0.775 / 28.0 / 0.002 | Ganancias PID de velocidad de rueda |
| `odometry.py` | `ENCODER_PPR` | 562 | Pulsos por revolución |
| `odometry.py` | `WHEEL_DIAM_M` | 0.08 m | Diámetro de rueda |
| `odometry.py` | `WHEEL_BASE` | 0.20 m | Distancia entre ruedas |

---

## Equipo

**Universidad de los Andes — Ingeniería Eléctrica y Electrónica**  
Curso: IELE3338L — Robótica — Periodo 2026-10  
**Grupo 5**  
Profesor: Jorge López Jiménez  
Asistente: Diego Santiago Jiménez Beltrán
