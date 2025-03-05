from ..devices import ICM20600, AK09918
from .fusion import *

class IMU:
    """
    Implements a combination class of accelerometer + gyro + compass and sensor fusion for
    a complete inertial measurement unit, using the ICM20600 and AK09918 modules.

    - When heading is at 0 degrees, X-direction is pointing to the North
    - For more accurate heading readings, pitch and roll should be close to 0
    """
    def __init__(self, i2c):
        self.i2c = i2c
        self.F = Fusion()
        self.m = AK09918(self.i2c)
        self.m.setup()
        self.ag = ICM20600(i2c, int1=8, int2=9)
        self.ag.setup()
        self.ag.set_handler(self.update)
        self.temperature = None
        self.delta = None

    def calibrate(self, wait=1000):
        """
        Run calibration.
        """
        self.F.calibrate(self.m._read_data, wait=wait)

    def update(self, td, axyz, gxyz, T, *args, **kwargs):
        """
        Recomputes sensor fusion.
        """
        try:
            # Discard first value
            if self.delta is None:
                self.delta = 0
                return False
            # Delta to seconds (returned in microseconds)
            self.delta += td / 1_000_000
            # Read magnetometer immediately, units irrelevant, will be normalized
            mxyz = self.m._read_data()
            # Accel units irrelevant, will be normalized
            axyz = axyz
            # Convert gyro to deg/sec
            gxyz = tuple(gxyz[i] / self.ag.Gyro_Sensitivity for i in range(3))
            # Recompute fusion
            self.F.update(axyz, gxyz, mxyz, delta=self.delta)
            # Set current temperature
            self.temperature = T
            # Reset delta
            self.delta = 0
        except (TypeError, ValueError):
            pass
        return True

    def get_orientation(self):
        """
        Returns latest known orientation as 3-tuple of heading, pitch, roll in degrees
        """
        # Adjust heading to natural range N=0, S=180
        heading_adj = (360 - (self.F.heading + 360) % 360)
        return (heading_adj, self.F.pitch, self.F.roll)

    def get_temperature(self):
        """
        Returns current temperature reading.
        """
        return self.temperature

def get_default_imu():
    """
    Returns IMU instance with default configuration.
    """
    from machine import I2C

    i2c = I2C(0, scl=21, sda=20, freq=400_000)
    imu = IMU(i2c)

    return imu

if __name__ == '__main__':
    import time
    imu = get_default_imu()

    print("Calibrating...")
    imu.calibrate(1000)

    print("Found mXYZ bias (unitless): ", imu.F.mbias)
    time.sleep_ms(1000)
    
    while True:
        print(imu.get_orientation())
        time.sleep_ms(50)
