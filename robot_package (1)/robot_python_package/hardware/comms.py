"""
Stm32 serial communication - binary protocol.

Downlink (Pi -> Stm32), 8 bytes, no header:
    v_target   : float32 little-endian (linear, m/s)
    omega_target: float32 little-endian (angular, rad/s)

Uplink (Stm32 -> Pi), 9 bytes:
    0xAA header (1B)
    RPM_L : float32
    RPM_R : float32
"""

import struct
import serial

from config import SERIAL_PORT, SERIAL_BAUD

UPLINK_HEADER = 0xAA
UPLINK_SIZE = 9   # header(1) + RPM_L(4) + RPM_R(4)
DOWNLINK_SIZE = 8  # v_target(4) + omega_target(4)


class STM32Comms:

    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.001)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self._last_rpm_l = 0.0
        self._last_rpm_r = 0.0
        self._packet_count = 0
        self._rx_buf = bytearray()

    def send_command(self, linear: float, angular: float):
        # Send velocity command to STM32 - raw binary, no header
        try:
            packet = struct.pack('<ff', float(linear), float(angular))
            self.ser.write(packet)
        except Exception as e:
            print(f"[COMMS ERROR] Send failed: {e}")

    def read_odometry(self) -> tuple:
        """
        Read wheel RPMs from STM32.
        Returns: (rpm_l, rpm_r), or last known values if nothing new,
        or (None, None) if nothing has ever been received.
        """
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self._rx_buf += self.ser.read(waiting)

            # Resync: scan for header byte, then check we have a full packet
            while len(self._rx_buf) >= UPLINK_SIZE:
                if self._rx_buf[0] != UPLINK_HEADER:
                    # Drop one byte and try again (byte-shifted stream)
                    del self._rx_buf[0]
                    continue

                packet = bytes(self._rx_buf[:UPLINK_SIZE])
                del self._rx_buf[:UPLINK_SIZE]

                rpm_l, rpm_r = struct.unpack('<ff', packet[1:])
                self._last_rpm_l = rpm_l
                self._last_rpm_r = rpm_r
                self._packet_count += 1
                return rpm_l, rpm_r

        except Exception as e:
            print(f"[COMMS ERROR] Read failed: {e}")

        # No new complete packet - return last known values
        if self._packet_count > 0:
            return self._last_rpm_l, self._last_rpm_r
        return None, None

    def flush(self):
        self.ser.reset_input_buffer()
        self._rx_buf.clear()

    def close(self):
        self.ser.close()

    def emergency_stop(self):
        for _ in range(5):
            try:
                self.ser.write(struct.pack('<ff', 0.0, 0.0))
            except Exception:
                pass