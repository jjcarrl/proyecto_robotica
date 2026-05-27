#!/usr/bin/env python3
import math
import struct
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── YDLidar X4 Pro serial protocol ───────────────────────────────────────────
BAUD       = 128000
START_CMD  = bytes([0xA5, 0x60])
STOP_CMD   = bytes([0xA5, 0x65])
SYNC_0     = 0xAA
SYNC_1     = 0x55

# ── LaserScan output grid ─────────────────────────────────────────────────────
RANGE_MIN   = 0.12          # metres  (spec: 0.12 m min)
RANGE_MAX   = 10.0          # metres  (spec: 10 m max)
RESOLUTION  = 0.5           # degrees per bin → 720 bins for 360 °
N_BINS      = int(360 / RESOLUTION)


class YDLidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_node')

        self.declare_parameter('port',     '/dev/lidar')
        self.declare_parameter('frame_id', 'laser')

        port         = self.get_parameter('port').value
        self._frame  = self.get_parameter('frame_id').value

        self._pub = self.create_publisher(LaserScan, '/scan', 10)

        self._ser = serial.Serial(port, BAUD, timeout=0.1)
        self._ser.reset_input_buffer()
        self._ser.write(START_CMD)
        # The device responds with a 7-byte command-response header before
        # sending scan packets.  Waiting 150 ms then flushing is the most
        # reliable way to skip it without parsing the header format.
        time.sleep(0.15)
        self._ser.reset_input_buffer()

        self._pts        = []    # [(angle_deg, dist_m)] for the current revolution
        self._last_pub_t = None  # monotonic time of the last publish (for scan_time)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f'YDLidar node started on {port}')

    # ── serial layer ──────────────────────────────────────────────────────────

    def _sync(self):
        """Advance stream to the next 0xAA 0x55 sync pair."""
        while rclpy.ok():
            b = self._ser.read(1)
            if b and b[0] == SYNC_0:
                b2 = self._ser.read(1)
                if b2 and b2[0] == SYNC_1:
                    return True
        return False

    def _read_packet(self):
        """
        Read and validate one scan packet.

        Returns (new_scan: bool, points: list[(angle_deg, dist_m)]) or None
        on checksum failure / short read.
        """
        if not self._sync():
            return None

        hdr = self._ser.read(8)
        if len(hdr) < 8:
            return None

        ct      = hdr[0]
        lsn     = hdr[1]
        fsa     = struct.unpack_from('<H', hdr, 2)[0]
        lsa     = struct.unpack_from('<H', hdr, 4)[0]
        cs      = struct.unpack_from('<H', hdr, 6)[0]

        raw = self._ser.read(lsn * 2)
        if len(raw) < lsn * 2:
            return None

        samples = struct.unpack_from(f'<{lsn}H', raw)

        # Verify checksum (XOR across header fields and all samples)
        cs_calc = 0x55AA ^ (ct | (lsn << 8)) ^ fsa ^ lsa
        for s in samples:
            cs_calc ^= s
        cs_calc &= 0xFFFF
        if cs_calc != cs:
            return None

        start_deg = (fsa >> 1) / 64.0
        end_deg   = (lsa >> 1) / 64.0

        pts = []
        for i, s in enumerate(samples):
            dist_m = (s >> 2) / 1000.0
            if lsn == 1:
                angle = start_deg
            else:
                diff  = end_deg - start_deg
                if diff < 0:
                    diff += 360.0
                angle = (start_deg + diff * i / (lsn - 1)) % 360.0
            pts.append((angle, dist_m))

        return bool(ct & 0x01), pts

    def _read_loop(self):
        while rclpy.ok():
            result = self._read_packet()
            if result is None:
                continue
            new_scan, pts = result
            # CT bit-0 = 1 marks the zero-degree reference (start of a new
            # full revolution).  Publish whatever we accumulated, then reset.
            if new_scan and self._pts:
                self._publish_scan(self._pts)
                self._pts = []
            self._pts.extend(pts)

    # ── LaserScan publisher ───────────────────────────────────────────────────

    def _publish_scan(self, pts):
        now = time.monotonic()
        scan_time = (now - self._last_pub_t) if self._last_pub_t else 0.264
        self._last_pub_t = now

        ranges = [float('inf')] * N_BINS

        for angle_deg, dist_m in pts:
            if not (RANGE_MIN <= dist_m <= RANGE_MAX):
                continue
            idx = int(angle_deg % 360.0 / RESOLUTION) % N_BINS
            if dist_m < ranges[idx]:
                ranges[idx] = dist_m

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.angle_min       = 0.0
        msg.angle_max       = 2.0 * math.pi - math.radians(RESOLUTION)
        msg.angle_increment = math.radians(RESOLUTION)
        msg.time_increment  = scan_time / N_BINS
        msg.scan_time       = scan_time
        msg.range_min       = RANGE_MIN
        msg.range_max       = RANGE_MAX
        msg.ranges          = ranges

        self._pub.publish(msg)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._ser.is_open:
            self._ser.write(STOP_CMD)
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
