import sys
import smbus2


class ForceBus1(smbus2.SMBus):
    # Forces all SMBus access to I2C Bus 1
    def __init__(self, bus=1, force=False):
        super().__init__(1, force)


# Monkey-patch smbus to use smbus2 on Bus 1

sys.modules['smbus'] = sys.modules['smbus2']
sys.modules['smbus'].SMBus = ForceBus1

from mpu6050 import MPU6050
import math

from config import GYRO_SENSITIVITY, GYRO_CORRECTION_FACTOR


class IMU:
    #  MPU6050 wrapper

    def __init__(self, address=0x68):
        self.address = address
        self.sensor = MPU6050(address)
        if hasattr(self.sensor, 'wake_up'):
            self.sensor.wake_up()

        bus = smbus2.SMBus(1)
        bus.write_byte_data(address, 0x1B, 0x08)
        bus.close()

        self._gyro_z_bias = 0.0
        self._calibrated = False

    def calibrate(self, samples=50, delay=0.01):
        # Calibrate gyroscope Z bias
        print("Calibrating gyroscope... Keep robot perfectly still.")
        bias = 0.0
        for _ in range(samples):
            try:
                bias += self.sensor.get_rotation().z / GYRO_SENSITIVITY
            except Exception:
                pass
            import time
            time.sleep(delay)
        self._gyro_z_bias = bias / samples
        self._calibrated = True
        print(f"Gyro bias Z: {self._gyro_z_bias:.4f} deg/s")
        return self._gyro_z_bias

    def read_gyro_z_rad(self) -> float:
        # Read Z-axis gyroscope in rad/s 
        try:
            raw_z = self.sensor.get_rotation().z
            gyro_dps = (raw_z / GYRO_SENSITIVITY) - self._gyro_z_bias
            return math.radians(gyro_dps) * GYRO_CORRECTION_FACTOR
        except Exception:
            return 0.0
