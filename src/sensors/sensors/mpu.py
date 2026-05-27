#!/usr/bin/env python3
import math
import struct
import time

from smbus2 import SMBus, i2c_msg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ── BNO055 registros ──────────────────────────────────────────────────────────
_CHIP_ID    = 0x00   # debe leer 0xA0
_PAGE_ID    = 0x07
_QUA_W_LSB  = 0x20   # quaternion  W X Y Z  (4 × int16 LE)
_GYR_X_LSB  = 0x14   # giroscopio  X Y Z    (3 × int16 LE)
_LIA_X_LSB  = 0x28   # acel lineal X Y Z    (3 × int16 LE, sin gravedad)
_CALIB_STAT = 0x36
_OPR_MODE   = 0x3D
_PWR_MODE   = 0x3E
_SYS_TRIG   = 0x3F
_UNIT_SEL   = 0x3B

_MODE_CONFIG = 0x00
_MODE_NDOF   = 0x0C   # fusión completa: accel + gyro + mag

# escala → unidades SI
_Q_SCALE   = 1.0 / 16384.0          # quaternion LSB → [-1, 1]
_G_SCALE   = (math.pi / 180.0) / 16.0   # dps LSB → rad/s
_A_SCALE   = 1.0 / 100.0            # m/s² LSB → m/s²

# covariances (NDOF completamente calibrado, datasheet BNO055)
_ORI_VAR = 1.1e-6
_GYR_VAR = 3.0e-6
_ACC_VAR = 8.7e-4
_ORI_COV = [_ORI_VAR, 0, 0,  0, _ORI_VAR, 0,  0, 0, _ORI_VAR]
_GYR_COV = [_GYR_VAR, 0, 0,  0, _GYR_VAR, 0,  0, 0, _GYR_VAR]
_ACC_COV = [_ACC_VAR, 0, 0,  0, _ACC_VAR, 0,  0, 0, _ACC_VAR]


class _BNO055:
    """Driver mínimo BNO055 usando smbus2.i2c_rdwr (I2C puro, sin SMBus)."""

    def __init__(self, bus: SMBus, addr: int = 0x28):
        self._bus  = bus
        self._addr = addr

    # ── I2C raw ───────────────────────────────────────────────────────────────

    def _read(self, reg: int, n: int) -> bytes:
        w = i2c_msg.write(self._addr, [reg])
        r = i2c_msg.read(self._addr, n)
        self._bus.i2c_rdwr(w, r)
        return bytes(r)

    def _write(self, reg: int, val: int):
        self._bus.i2c_rdwr(i2c_msg.write(self._addr, [reg, val]))

    # ── inicialización ────────────────────────────────────────────────────────

    def begin(self):
        chip = self._read(_CHIP_ID, 1)[0]
        if chip != 0xA0:
            raise RuntimeError(f'CHIP_ID inesperado: {hex(chip)} (esperado 0xa0)')

        # reset hardware
        self._write(_SYS_TRIG, 0x20)
        # esperar a que el BNO055 vuelva — puede tardar hasta ~1s
        for _ in range(20):
            time.sleep(0.1)
            try:
                if self._read(_CHIP_ID, 1)[0] == 0xA0:
                    break
            except OSError:
                pass  # normal mientras el chip está reseteando

        self._write(_PWR_MODE, 0x00)   # normal power
        self._write(_PAGE_ID,  0x00)   # página 0
        self._write(_SYS_TRIG, 0x00)   # sin trigger externo
        time.sleep(0.02)

        # unidades: m/s², dps, grados, Celsius (todos default = 0x00)
        self._write(_UNIT_SEL, 0x00)

        # modo fusión completa
        self._write(_OPR_MODE, _MODE_NDOF)
        time.sleep(0.02)

    # ── lecturas ──────────────────────────────────────────────────────────────

    @property
    def quaternion(self):
        raw = self._read(_QUA_W_LSB, 8)
        w, x, y, z = struct.unpack_from('<hhhh', raw)
        return w * _Q_SCALE, x * _Q_SCALE, y * _Q_SCALE, z * _Q_SCALE

    @property
    def gyro(self):
        raw = self._read(_GYR_X_LSB, 6)
        x, y, z = struct.unpack_from('<hhh', raw)
        return x * _G_SCALE, y * _G_SCALE, z * _G_SCALE

    @property
    def linear_acceleration(self):
        raw = self._read(_LIA_X_LSB, 6)
        x, y, z = struct.unpack_from('<hhh', raw)
        return x * _A_SCALE, y * _A_SCALE, z * _A_SCALE

    @property
    def calibration_status(self):
        b = self._read(_CALIB_STAT, 1)[0]
        return (b >> 6) & 3, (b >> 4) & 3, (b >> 2) & 3, b & 3  # sys, gyro, acc, mag


class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu_node')

        bus  = SMBus(1)
        addr = self._detect_addr(bus)
        self._sensor = _BNO055(bus, addr)

        for attempt in range(5):
            try:
                self._sensor.begin()
                break
            except Exception as e:
                self.get_logger().warn(f'BNO055 begin intento {attempt+1}/5: {e}')
                time.sleep(0.5)
        else:
            raise RuntimeError('BNO055 no inicializa. Verifica conexiones.')

        self.get_logger().info(f'BNO055 encontrado en {hex(addr)}')
        self._imu_pub = self.create_publisher(Imu, '/imu', 10)
        self._timer   = self.create_timer(0.02, self._loop)   # 50 Hz

        self._last_cal_ns  = 0
        self._last_cal_log = (-1, -1, -1, -1)
        self.get_logger().info('BNO055 OK — esperando calibración...')

    # ── loop 50 Hz ────────────────────────────────────────────────────────────

    def _loop(self):
        try:
            quat    = self._sensor.quaternion
            gyro    = self._sensor.gyro
            lin_acc = self._sensor.linear_acceleration
        except OSError:
            return   # lectura perdida transitoria

        self._log_cal()

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # orientación — BNO055 devuelve (w, x, y, z)
        # si el robot gira al revés: negar z e y del quat, y negar gyro[2]
        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]
        msg.orientation_covariance = _ORI_COV

        # vel angular (rad/s) — el EKF usa .z para yaw rate
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance = _GYR_COV

        # aceleración lineal sin gravedad — ekf.yaml: remove_gravitational_acceleration: false
        msg.linear_acceleration.x = lin_acc[0]
        msg.linear_acceleration.y = lin_acc[1]
        msg.linear_acceleration.z = lin_acc[2]
        msg.linear_acceleration_covariance = _ACC_COV

        self._imu_pub.publish(msg)

    @staticmethod
    def _detect_addr(bus: SMBus) -> int:
        for addr in (0x28, 0x29):
            try:
                w = i2c_msg.write(addr, [0x00])
                r = i2c_msg.read(addr, 1)
                bus.i2c_rdwr(w, r)
                if list(r)[0] == 0xA0:
                    return addr
            except OSError:
                pass
        raise RuntimeError('BNO055 no detectado en 0x28 ni 0x29')

    def _log_cal(self):
        now = self.get_clock().now().nanoseconds
        if now - self._last_cal_ns < 5_000_000_000:
            return
        self._last_cal_ns = now
        cal = self._sensor.calibration_status
        if cal != self._last_cal_log:
            self._last_cal_log = cal
            s, g, a, m = cal
            tag = '✓ LISTO   ' if s == 3 else '— calibrando'
            self.get_logger().info(f'BNO055 {tag} | sys:{s} gyro:{g} acc:{a} mag:{m}')


def main(args=None):
    rclpy.init(args=args)
    node = MPUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
